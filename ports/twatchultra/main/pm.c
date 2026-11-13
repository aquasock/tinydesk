/*
 * pm.c - Pokemon Mini on the watch, as a shell command.
 *
 * The core is components/pokemini, vendored from tinytama. This file is the port:
 * a 72 Hz frame loop, the dot buffer turned into panel bytes, the margins turned
 * into a Sega-style pad, and the cartridge in PSRAM.
 *
 * THE FRAME IS PACED BY A TIMER, NOT BY A TASK DELAY. The Pokemon Mini's frame is
 * 13,889 us and FreeRTOS is configured at 100 Hz, so the closest a
 * vTaskDelay()-based loop can get is one 10 ms tick: pacing with ticks would
 * either run the machine 39 percent slow or force sub-tick spinning that burns
 * most of the remaining budget. A periodic esp_timer at 72 Hz wakes the task
 * instead, so emulated time and wall time stay married without a busy wait.
 *
 * WHY PACING MATTERS AT ALL: PokeMini_EmulateCycles() advances EMULATED time and
 * does not care how long the host took. Free-running would make games play fast,
 * so a lagging frame has to be DROPPED (published as late) rather than compressed.
 * The binary semaphore dropping a give that arrives while one is pending is
 * exactly that behaviour, and the count of those drops is the honest lag figure.
 *
 * THE CARTRIDGE IS NOT COPIED. PokeMini_SetMINMem() stores the caller's pointer
 * and the S1C88 fetches instructions out of the cartridge, so where it lives
 * decides the frame rate. pmbench measured that a maximally sized cartridge
 * cannot be internal -- the largest free internal block was 31,744 bytes against
 * a 32,768-byte request -- so this reads the cartridge into PSRAM and says so if
 * it cannot.
 *
 * AUDIO IS ABSENT, deliberately. It is last for every platform in this project,
 * and the machine runs correctly silent.
 */
#include "pm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "display.h"
#include "font8x16.h"
#include "edges.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "tdsh.h"
#include "touch.h"

/* The core's own headers, through the component's exported include path. The
 * SAR #undef is required for the same reason pmbench.c needs it: MinxIO.h and
 * the Xtensa special-register namespace both define SAR. */
#undef SAR
#include "Hardware.h"
#include "MinxLCD.h"
#include "MinxIO.h"
#include "PokeMini.h"

static const char *TAG = "pm";

/* The instruction counter added by the component's patches/0001-exec-counter.patch.
 * Only `pm bench` reads it. */
extern unsigned long g_exec_count;

/* --- the emulated machine -------------------------------------------------- */

#define PM_CLOCK_HZ       4000000

/*
 * THE MACHINE'S FRAME IS NOT 4 MHz / 72. It is the PRC counter's own period:
 * the counter runs from 0x01000000 to 0x42000000, a span of 65 * 2^24, and
 * advances by 2^24/855 per CPU cycle, so one frame is exactly 65 * 855 = 55,575
 * cycles. That is 71.97 Hz at 4 MHz, which is where the core's own
 * `MinxPRC_On72HzRefresh()` name comes from -- the 72 is a rounded label on a
 * 71.97 Hz event, not a design figure.
 *
 * Using 4e6/72 = 55,555 instead would run the machine 0.036 percent fast, which
 * is small enough to hide and large enough to matter over a long session, so the
 * core's own constant is used here and the frame timer is derived from it rather
 * than the other way round.
 */
#define PM_FRAME_CYCLES   55575                           /* 65 * 855 */
/* A double, because every use of it is in a rate or a percentage. */
#define PM_FRAME_HZ       ((double)PM_CLOCK_HZ / PM_FRAME_CYCLES) /* 71.97 */
#define PM_FRAME_US       (((int64_t)PM_FRAME_CYCLES * 1000000) / PM_CLOCK_HZ)  /* 13,894 */

#define PM_W              96
#define PM_H              64

#define PM_STACK          10240
#define PM_BLIT_STACK     4096

/* How many tick credits may accumulate. One is enough to never lose a tick that
 * the emulator was merely late for; eight lets it catch up after a stall of
 * about 110 ms without letting a long one turn into a burst of frames. */
#define PM_TICK_BACKLOG   8
#define PM_PRIO           4

/* --- state ----------------------------------------------------------------- */

static bool s_run;
static TaskHandle_t s_task;
static SemaphoreHandle_t s_frame;
static esp_timer_handle_t s_tick;
static uint8_t *s_rom;
static char s_rom_path[128];

/* The pad, shared with the touch task: one aligned int, read whole or not at all,
 * for the reason nes.c records -- a multi-field update would need a lock and the
 * Tamagotchi's button state raced the emulated CPU when it was not one integer. */
static volatile int s_pad;
static int s_pad_applied;

static volatile uint32_t s_frames, s_misses;
static int64_t s_started_us;

/* Where a frame's time goes, accumulated. The NES port was already burned by a
 * timing story that was inferred rather than measured (core-log 61), so the two
 * halves are counted separately and `pm status` reports them.
 *
 * These are diagnostic only and are read from a different task than the one that
 * writes them: a 64-bit read on a 32-bit core can tear, so a figure taken while
 * the emulator runs is approximate. Nothing depends on them. */
static volatile int64_t s_emu_us, s_blit_us;
static char s_bios[48];

/* --- the frame pool --------------------------------------------------------
 *
 * OVERLAP, AND WHY IT NEEDS THREE BUFFERS.
 *
 * With the blit on the emulator's own task the frame cost was emulation 12.8 ms
 * plus blit 5.4 ms against a 13.89 ms budget, so the machine ran at 0.76x of
 * real time -- slow motion, which on this platform is audible as a beat in the
 * dither rather than visible as dropped frames. Moving the blit to core 1 makes
 * the frame max(12.8, 5.4) instead of their sum.
 *
 * Three buffers and not two, because with two the emulator would have to wait
 * for the blitter to release the other one and there would be nothing to
 * overlap. With three the emulator always has one to write while one is being
 * pushed and one is in flight.
 *
 * The dots are 6,144 bytes each, so the pool is 18 KB -- small enough to ask for
 * internal RAM first, which keeps the blitter's per-frame read off the PSRAM bus
 * that the 512 KB cartridge is also using.
 */
#define PM_VBUF_N 3
#define PM_DOTS   (PM_W * PM_H)

static uint8_t *s_dot[PM_VBUF_N];
static QueueHandle_t s_free_q;      /* indices: available to render */
static QueueHandle_t s_filled_q;    /* indices: rendered, waiting to be sent */
static TaskHandle_t s_blit_task;
static volatile bool s_blit_run;
static int s_cur;                   /* the buffer the emulator is writing */
static volatile uint32_t s_blits, s_blit_skipped, s_blit_same;
static bool s_pool_internal;

/*
 * Diagnostic: run the paced loop with no blitter at all.
 *
 * This exists to settle one question with a measurement instead of a story.
 * Puzzle Collection measures 12.28 ms of emulation per frame when nothing else
 * is running (`pm bench`) and 14.5 ms in the paced loop, and the loop's own
 * copy, compare and handoff account for only 0.15 ms of that difference. The two
 * remaining candidates are the core-1 blitter contending for the memory bus, and
 * the pacing itself. With `pm start <rom> noblit` the semaphore, the snapshot and
 * the timing all still happen but no push does, so the difference between it and
 * a normal run is the blitter's cost to the emulator rather than to the display.
 */
static bool s_nobl_diag;

/*
 * THE LAST PICTURE THAT WAS ACTUALLY SENT.
 *
 * A game draws at the PRC's rate, not at the machine's tick rate, and for
 * Pinball that is once every 2 ticks (rate register 0x81, divisor 2) -- 36 Hz
 * against a 71.97 Hz tick. Pushing on every tick therefore sends the same
 * 288x192 frame twice, which costs 5.7 ms of SPI per duplicate and, more
 * importantly, puts that traffic on the bus while the emulator is fetching a
 * 512 KB cartridge from PSRAM.
 *
 * Skipping a frame that is byte-identical to the one already on the glass is
 * also the more faithful behaviour: the real panel only changes when the PRC
 * copies, so this makes the host push at the rate the game actually draws.
 *
 * The comparison is 6,144 bytes of internal RAM, about 20 us, against 5.7 ms of
 * SPI that it avoids.
 */
static uint8_t *s_prev;
static bool s_have_prev;

/*
 * A LIT DOT IS DARK, WHICH IS THE OPPOSITE OF WHAT THE FIRST VERSION DID.
 *
 * The Pokemon Mini's panel is a monochrome LCD: what a game calls a set pixel is
 * a dark dot on a pale background, not a bright dot on black. PokeMini's
 * LCDPixelsD holds one byte per dot with non-zero meaning "this dot is lit", and
 * the reference front end renders exactly that -- pmgui.c maps a non-zero dot to
 * 0xFF16161A, near black, against 0xFFC8C8BE for an unlit one.
 *
 * This port mapped non-zero to white and shipped an inverted picture. The fault
 * was invisible in every counter: the frame rate, the speed ratio, the push count
 * and the identical-frame skip were all correct, because they do not care what a
 * pixel means. It took the operator putting the watch beside the desktop
 * emulator, on the same screen of the same cartridge, to see it.
 *
 * NOTE THIS IS NOT THE HARDWARE'S OWN INVERSION. MinxLCD has InvAllPix and
 * SetAllPix registers that a game can set, and MinxLCD_Render() applies them into
 * LCDPixelsD before this code ever sees it. So the polarity here is fixed and
 * absolute: nothing a cartridge does should change it.
 *
 * Both values are byte-order palindromic (0xFFFF, 0x0000), so no per-pixel swap
 * is needed -- the panel wants high byte first and these read the same either
 * way.
 */
#define PM_DOT_LIT    0x0000u   /* a set dot is dark */
#define PM_DOT_UNLIT  0xFFFFu   /* clear is pale */

/* --- the frame, as bytes for the panel ------------------------------------- */

/*
 * One destination row of the 3x-scaled picture.
 *
 * display_picture_rows() drives the panel's transmit band and asks for a row at a
 * time, so the scale happens straight into the band: one read of the dot buffer
 * per destination pixel and no intermediate image. `col` is the band-relative
 * starting column, which is why the source x is derived from it rather than from
 * a row-local counter -- a band that starts part-way across must still sample the
 * correct dots.
 */
/* A synthesised picture, for checking the scale and the centring on the glass
 * without a cartridge. A 1-pixel border plus an 8-pixel grid makes a wrong
 * origin, scale or byte order visible on the glass rather than inferred from a
 * counter -- and single-dot lines make a dropped column obvious in a way a
 * game's artwork does not. */
static void pm_pattern_dots(uint8_t *dots)
{
    for (int y = 0; y < PM_H; y++)
        for (int x = 0; x < PM_W; x++)
        {
            const bool edge = (x == 0 || x == PM_W - 1 || y == 0 || y == PM_H - 1);
            const bool grid = (x % 8 == 0) || (y % 8 == 0);
            dots[y * PM_W + x] = (edge || grid) ? 1 : 0;
        }
}

/*
 * The expanded destination row, cached per SOURCE row.
 *
 * THE FIRST VERSION EXPANDED PER DESTINATION ROW AND COST 9.9 ms. A 3x vertical
 * scale reads each source row three times, and the naive callback also divided
 * per pixel to recover the source column, so a 96x64 picture cost 55,296
 * divisions and 55,296 table decisions per frame for 6,144 distinct dots. The
 * band loop calls rows in order within a band, so one row of cache removes
 * two-thirds of that work: the row is expanded once and the two repeats become a
 * 576-byte copy.
 *
 * This is why the picture is built at PM_W * PM_SCALE wide rather than being
 * scaled at push time -- the scale now happens 64 times a frame instead of
 * 55,296.
 */
static uint16_t s_xrow[PM_W * PM_SCALE];
static int s_xrow_sy = -1;

static void pm_expand_row(const uint8_t *dots, int sy)
{
    const uint8_t *src = &dots[sy * PM_W];

    for (int x = 0; x < PM_W; x++)
    {
        const uint16_t v = src[x] ? PM_DOT_LIT : PM_DOT_UNLIT;
        uint16_t *d = &s_xrow[x * PM_SCALE];

        for (int k = 0; k < PM_SCALE; k++)
            d[k] = v;
    }
}

static void pm_fill_row(void *ctx, int row, int col, uint16_t *dst, int n)
{
    const uint8_t *dots = (const uint8_t *)ctx;

    const int sy = row / PM_SCALE;

    if (sy >= PM_H || col < 0)
    {
        for (int i = 0; i < n; i++)
            dst[i] = PM_DOT_UNLIT;
        return;
    }

    if (col + n > PM_W * PM_SCALE)
        n = PM_W * PM_SCALE - col;
    if (n <= 0)
        return;

    if (sy != s_xrow_sy)
    {
        pm_expand_row(dots, sy);
        s_xrow_sy = sy;
    }

    /* The destination is the panel's transmit band and the values are already in
     * wire order, so this is a straight copy of 16-bit elements. */
    memcpy(dst, &s_xrow[col], (size_t)n * sizeof(uint16_t));
}

/* One push, on whichever task is doing the pushing. */
static void blit_one(const uint8_t *dots)
{
    const int64_t b0 = esp_timer_get_time();

    /* The row cache holds one source row and is only valid within a frame: a band
     * that starts on the same source row the previous frame ended on would
     * otherwise emit that frame's dots. Cheap to clear, and the failure it
     * prevents is a one-line tear that would be blamed on the panel. */
    s_xrow_sy = -1;

    const bool ok = display_pixels_claimed() &&
                    display_picture_rows((void *)dots, PM_OX, PM_OY,
                                         PM_W * PM_SCALE, PM_H * PM_SCALE, pm_fill_row);
    if (!ok)
    {
        /* A refused push means ownership was lost, which the counters would
         * otherwise report as a healthy frame rate over a black screen. */
        static bool warned;
        if (!warned)
        {
            ESP_LOGW(TAG, "picture push refused -- not claimed?");
            warned = true;
        }
    }

    s_blit_us += esp_timer_get_time() - b0;
    s_blits++;
}

/*
 * THE HANDOFF, AND WHY ITS TWO STEPS ARE IN THIS ORDER.
 *
 * A free buffer is taken BEFORE the finished one is published. If the pool is
 * dry the frame is not queued and the emulator keeps writing the buffer it
 * already holds. Done the other way round, a dry pool would publish a buffer and
 * then keep rendering into it, and the blitter would push a half-written frame --
 * tearing that appears only under load and reads as a panel fault. This is
 * DSP-008, and the NES port learned it the same way.
 */
static void pm_handoff(void)
{
    int nxt;

    if (s_nobl_diag)
        return;

    if (xQueueReceive(s_free_q, &nxt, 0) != pdTRUE)
    {
        s_blit_skipped++;
        return;
    }

    if (xQueueSend(s_filled_q, &s_cur, 0) != pdTRUE)
    {
        s_blit_skipped++;
        xQueueSend(s_free_q, &nxt, 0);
        return;
    }

    s_cur = nxt;
}

/* Core 1. Pushes finished frames and gives each buffer back afterwards. */
static void pm_blit_task(void *arg)
{
    (void)arg;

    while (s_blit_run)
    {
        int idx;

        if (xQueueReceive(s_filled_q, &idx, pdMS_TO_TICKS(50)) != pdTRUE)
            continue;

        blit_one(s_dot[idx]);

        /* Return the buffer even if the loop is about to end, so a stop cannot
         * strand one. */
        xQueueSend(s_free_q, &idx, 0);
    }

    s_blit_task = NULL;
    vTaskDelete(NULL);
}

/* --- input ----------------------------------------------------------------- */

/* Margin buttons as a Pokemon Mini pad, following the layout nes.c uses so the
 * two emulators are not a second set of muscle memory:
 *
 *   bottom 0 -> B    bottom 1 -> A    bottom 2 -> C
 *   right 1 -> LEFT  right 2 -> RIGHT right 3 -> UP  right 4 -> DOWN
 */
#define PMK_A      (1 << 0)
#define PMK_B      (1 << 1)
#define PMK_C      (1 << 2)
#define PMK_UP     (1 << 3)
#define PMK_DOWN   (1 << 4)
#define PMK_LEFT   (1 << 5)
#define PMK_RIGHT  (1 << 6)
#define PMK_POWER  (1 << 7)

static int pad_from_frame(const touch_frame_t *f)
{
    int pad = 0;
    if (!f)
        return 0;

    for (int i = 0; i < f->n && i < TOUCH_POINTS; i++)
    {
        if (f->p[i].region == EDGE_BOTTOM)
        {
            if (f->p[i].index == 0) pad |= PMK_B;
            if (f->p[i].index == 1) pad |= PMK_A;
            if (f->p[i].index == 2) pad |= PMK_C;
        }
        else if (f->p[i].region == EDGE_RIGHT)
        {
            if (f->p[i].index == 1) pad |= PMK_LEFT;
            if (f->p[i].index == 2) pad |= PMK_RIGHT;
            if (f->p[i].index == 3) pad |= PMK_UP;
            if (f->p[i].index == 4) pad |= PMK_DOWN;
        }
    }
    return pad;
}

static void pm_app_touch(void *user, const touch_frame_t *f)
{
    (void)user;
    if (!s_run)
        return;
    s_pad = pad_from_frame(f);
}

/*
 * Keys go to the core as EVENTS, not as a state the core polls.
 *
 * PokeMini_KeypadEvent() takes a MINX_KEY_* enum value (1..9), never a bit mask;
 * passing masks is the bug that made the desktop front end send B when Z was
 * pressed, nothing for C, and POWER for Down -- which blanks the screen. So this
 * diffs the mask and emits one event per changed button, with the enum name
 * resolved by the compiler rather than by an index.
 */
static void apply_pad(int now)
{
    const int changed = now ^ s_pad_applied;
    if (!changed)
        return;

    static const struct { int bit; uint8_t key; } map[] = {
        { PMK_A,     MINX_KEY_A     },
        { PMK_B,     MINX_KEY_B     },
        { PMK_C,     MINX_KEY_C     },
        { PMK_UP,    MINX_KEY_UP    },
        { PMK_DOWN,  MINX_KEY_DOWN  },
        { PMK_LEFT,  MINX_KEY_LEFT  },
        { PMK_RIGHT, MINX_KEY_RIGHT },
        { PMK_POWER, MINX_KEY_POWER },
    };

    for (size_t i = 0; i < sizeof(map) / sizeof(map[0]); i++)
        if (changed & map[i].bit)
            PokeMini_KeypadEvent(map[i].key, (now & map[i].bit) ? 1 : 0);

    s_pad_applied = now;
}

/* --- the frame loop -------------------------------------------------------- */

/*
 * THE TICK MUST BE A COUNT, NOT A FLAG.
 *
 * This was a binary semaphore and it capped the machine at 0.981x however fast
 * the emulator got. A binary semaphore holds one pending give: when the
 * emulator overran by more than a frame the next two ticks collapsed into one,
 * and the second was gone permanently -- so every overrun converted into
 * emulated time that never happened. The counter said so exactly (33 missed of
 * 1,837 ticks, 1.8 percent, against a measured 0.981x) and the figure moved
 * with throughput in a way that looked like the emulator being one percent too
 * slow rather than the pacer throwing ticks away.
 *
 * A counting semaphore remembers them instead, so the task drains the backlog
 * back-to-back and catches up. If the emulator is persistently slower than the
 * tick the count simply saturates and the loop runs flat out, which is the
 * honest behaviour: slow motion rather than lost frames.
 */
static void pm_tick(void *arg)
{
    (void)arg;

    if (xSemaphoreGive(s_frame) != pdTRUE)
        s_misses++;      /* backlog full: genuinely behind, not merely late */
}

static void pm_task(void *arg)
{
    (void)arg;

    for (;;)
    {
        if (!s_run)
            break;

        if (xSemaphoreTake(s_frame, pdMS_TO_TICKS(200)) != pdTRUE)
            continue;

        if (!s_run)
            break;

        apply_pad(s_pad);

        /* Fixed cycle slices, so emulated time advances at the machine's own
         * frame rate whatever the host manages. One frame per wake. */
        const int64_t te0 = esp_timer_get_time();
        PokeMini_EmulateCycles(PM_FRAME_CYCLES);
        const int64_t te1 = esp_timer_get_time();
        s_emu_us += te1 - te0;

        /*
         * COMPARE FIRST, AND COPY ONLY WHAT DIFFERS.
         *
         * LCDPixelsD is compared directly against the last picture that was
         * pushed. Fifty percent of frames are identical -- a cartridge draws at
         * its PRC rate, once every 2 ticks for most of this set -- so on those
         * frames this now copies nothing at all, where it used to copy 6 KB and
         * then discover it was unnecessary.
         *
         * The copy is still required on a frame that DOES differ, because the
         * core writes LCDPixelsD in place and the blitter reads the snapshot
         * while the next frame is being emulated. Comparing the live buffer
         * first is safe: the emulator is the only writer and this is that task.
         */
        memcpy(s_dot[s_cur], LCDPixelsD, PM_DOTS);
        if (s_have_prev && memcmp(s_dot[s_cur], s_prev, PM_DOTS) == 0)
        {
            s_blit_same++;
        }
        else
        {
            memcpy(s_prev, s_dot[s_cur], PM_DOTS);
            s_have_prev = true;
            pm_handoff();
        }

        s_frames++;
    }

    s_task = NULL;
    vTaskDelete(NULL);
}

/* --- lifecycle ------------------------------------------------------------- */

static uint8_t *load_rom(const char *path, int *out_len)
{
    FILE *f = fopen(path, "rb");
    if (!f)
    {
        printf("pm: %s: cannot open\n", path);
        return NULL;
    }

    if (fseek(f, 0, SEEK_END) != 0)
    {
        fclose(f);
        return NULL;
    }
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (len <= 0 || len > (2 * 1024 * 1024))
    {
        printf("pm: %s: %ld bytes is not a cartridge size\n", path, len);
        fclose(f);
        return NULL;
    }

    /* PSRAM, because a maximally sized cartridge does not fit internal DRAM once
     * the emulator's own 14.5 KB of state has taken its share (core-log 66). */
    uint8_t *buf = heap_caps_malloc((size_t)len, MALLOC_CAP_SPIRAM);
    if (!buf)
        buf = heap_caps_malloc((size_t)len, MALLOC_CAP_8BIT);

    if (!buf)
    {
        printf("pm: no %ld bytes for the cartridge\n", len);
        fclose(f);
        return NULL;
    }

    size_t got = fread(buf, 1, (size_t)len, f);
    fclose(f);

    if (got != (size_t)len)
    {
        printf("pm: %s: short read\n", path);
        heap_caps_free(buf);
        return NULL;
    }

    *out_len = (int)len;
    return buf;
}

/* --- the d-pad, drawn into the rows the picture does not use -----------------
 *
 * The picture is 288x192 and top-aligned, which leaves 224 rows of framebuffer
 * below it. That is where the d-pad goes: the Pokemon Mini has one and the
 * watch has no hardware equivalent, so it has to exist on the glass for the
 * console to be playable at all.
 *
 * DRAWN ONCE, NOT PER FRAME. Nothing about it changes while a cartridge runs, so
 * it is pushed once at start rather than with every frame; the picture is 55,296
 * pixels a frame and adding a static 28,224 would have cost about half again in
 * SPI traffic for an image that never moves. That is why this is a separate
 * picture push and not part of the blit callback.
 *
 * SINGLE-PIXEL WHITE OUTLINE, as asked for, so the shape can be judged and
 * adjusted before anything depends on it. It is deliberately not filled: a solid
 * cross would be a large white block on a black screen and would hide the
 * boundary mistakes that a line drawing makes obvious.
 *
 * THE SHAPE IS A UNION OF TWO BARS with its twelve corners rounded, and the
 * outline falls out of that rather than from hand-placed segments: a pixel is
 * drawn when it is inside the rounded cross and at least one of its four
 * neighbours is not. Every edge comes from that one rule, so the outline cannot
 * disagree with itself at a corner.
 *
 * Geometry is a square of PM_DPAD_S with arms one third of it, vertically
 * centred in the leftover rows and inset from the LEFT by the same amount as it
 * is inset from the bottom. A 56-pixel arm is comfortably above the ~40 px a
 * touch target wants.
 *
 * THE LEFT INSET IS DERIVED FROM THE BOTTOM ONE rather than written as 28. That
 * is the operator's instruction -- the margin to the left edge should match the
 * margin to the bottom edge -- and deriving it means the two stay matched if the
 * d-pad is ever resized or the picture's height changes. Written as a literal it
 * would silently stop being true the first time either moved.
 *
 * The cost is a large empty margin on the right, 124 px, which is intentional:
 * the d-pad lives in the bottom-left corner and the space to its right is where
 * the A, B and C cluster belongs once it is drawn.
 */
/* POSITION: CENTRED ON THE GLASS, NOT ON THE FRAMEBUFFER.
 *
 * These are two different rectangles and the difference is 41 rows. The
 * framebuffer is 320x416 and stops at fb row 415, but the AMOLED's glass runs 41
 * rows further down -- DISPLAY_PANEL_H 502 less the framebuffer's 45-row origin
 * and its 416 height -- and that strip carries the bottom margin buttons, which
 * is the visible proof it exists. An app cannot DRAW there; display_picture_rows()
 * clips to the framebuffer. But it can be SEEN there, and the eye centres on what
 * it can see.
 *
 * So the d-pad is centred in the band from the picture's bottom edge to the
 * glass's bottom edge, not in the band from the picture to the framebuffer's last
 * row. That puts 48 px above it and 49 px below, against a 1 px asymmetry that
 * comes from the band being an odd 265 rows and cannot be removed at an integer
 * position. Centring on the framebuffer instead would have left 16 px below
 * inside the fb and then a further 41 px of visible glass, so the d-pad would
 * have looked 57 px off-centre on the watch while reading as centred in the
 * source.
 *
 * The horizontal inset stays 16, matching the picture's own PM_OX, so the d-pad
 * lines up with the left edge of the game above it.
 */
#define PM_DPAD_S      168
#define PM_DPAD_ARM    (PM_DPAD_S / 3)
#define PM_DPAD_MLFT   16                                                 /* 16 */

/* The glass's bottom edge expressed in framebuffer rows: one past the last
 * visible row. Derived, so a panel or origin change moves it. */
#define PM_GLASS_BOT   (DISPLAY_PANEL_H - DISPLAY_ORIGIN_Y)               /* 457 */

/* The centring above puts the d-pad's 168 px box at y 240..407 and x 16..183.
 * From there the operator nudges it down and left to open thumb room.
 *
 * THE BOUND IS THE PANEL, NOT THE FRAMEBUFFER. An earlier version of this
 * comment computed the largest safe nudge as 10 px, because the rotated box needs
 * 238 rows and the framebuffer has only 224 below the picture. That was correct
 * about display_picture_rows() and wrong about what is visible: the framebuffer is
 * a 320x416 window, the glass is 410x502, and the 41 rows below the window are
 * addressable and visible -- the bottom margin buttons are already drawn there.
 * Drawing through display_panel_rows() instead removes the artificial edge, which
 * is what the operator asked for by pointing out the unused glass.
 *
 * So the real limits are the panel's own: y 0..501 and x 0..409 in panel
 * coordinates. At a 20 px nudge the cluster's lowest row is framebuffer 427,
 * which is panel 472, and the leftmost is framebuffer 6, which is panel 73 --
 * both comfortably on the glass.
 */
#define PM_DPAD_NUDGE_DOWN  20
#define PM_DPAD_NUDGE_LEFT  20

#define PM_DPAD_Y      (PM_H * PM_SCALE +                                        \
                        ((PM_GLASS_BOT - PM_H * PM_SCALE) - PM_DPAD_S) / 2 \
                        + PM_DPAD_NUDGE_DOWN)                                /* 260 */
#define PM_DPAD_X      (PM_DPAD_MLFT - PM_DPAD_NUDGE_LEFT)                /* -4 */

/* Kept as a name for the circle layout below, which still measures its gap from
 * the d-pad's old relationship rather than from the new margins. */
#define PM_DPAD_MARGIN PM_DPAD_MLFT

/*
 * COLOURS, TAKEN FROM THE MARGIN BUTTONS RATHER THAN CHOSEN HERE.
 *
 * The d-pad is furniture on the same glass as the twelve margin buttons, so it
 * uses their palette: violet border, dark grey face, green arrows. The values are
 * read from display_margin_colours() at draw time instead of being written as
 * constants, so tuning the buttons moves the d-pad with them.
 *
 * All three are byte-order palindromic once byte-swapped by hand below, which is
 * why the swap is done once per push rather than per pixel.
 */
static uint16_t s_dpad_edge;   /* outline  -- the buttons' border */
static uint16_t s_dpad_face;   /* interior -- the buttons' face   */
static uint16_t s_dpad_ink;    /* arrows   -- the buttons' label  */

/*
 * THE BACKGROUND IS BLACK, and the grid is kept behind a switch.
 *
 * The grid was drawn as a diagnostic -- a line every 16 px, matching what
 * `pattern 1` puts on the glass -- so the controls' alignment could be judged by
 * eye against a known scale. It has served that purpose: three rounds of
 * positioning were specified in grid cells rather than pixels precisely because
 * the grid made them countable off the watch.
 *
 * The operator now wants the controls seen against their normal background, so
 * PM_BG selects black. PM_GRID_ON is the switch rather than a comment, because
 * the grid will be wanted again the next time something is positioned by eye and
 * a deleted block would be written a third time.
 *
 * PM_GRID stays 16 because that is the increment `pattern 1` uses, and it is what
 * "one cell" has meant throughout this layout's history -- the cell offsets in
 * s_pad_off[] are still measured in it, so the grid's increment and the layout's
 * unit must not drift apart even while the lines are invisible.
 */
#define PM_GRID      16
#define PM_GRID_LINE 0x18C3u
#define PM_GRID_ON   0

#define PM_BG        0x0000u

static inline uint16_t pm_grid(int x, int y)
{
#if PM_GRID_ON
    return (x % PM_GRID == 0 || y % PM_GRID == 0) ? PM_GRID_LINE : PM_BG;
#else
    (void)x;
    (void)y;
    return PM_BG;
#endif
}

/*
 * THE EIGHT ARM TIPS ARE ROUNDED AND THE FOUR INNER CORNERS ARE SHARP.
 *
 * That asymmetry is deliberate rather than unfinished. Rounding the inner
 * corners was tried three ways and none of them looked right on the glass: an
 * arc centred on the corner and added bulges into the notch; centred in the void
 * and subtracted is tangent and removes almost nothing; centred on the corner
 * and subtracted is the true mirror of a tip but eats three quarters of its disc
 * from the reflex angle, so at any radius that reads well on the tips the inner
 * corners look over-rounded. The operator's call was to leave them sharp, which
 * is also the conventional d-pad shape -- the tips are what a thumb meets and
 * the inner corners are where the eye wants a clean right angle.
 *
 * A TIP is rounded by putting an arc's centre r inside the material on both axes
 * and cutting away everything in the r-by-r corner square that falls outside it.
 *
 * The table is what makes a wrong direction one entry rather than a wrong
 * triangle in one corner, and it is the same table the inner corners will need
 * if they are ever revisited. The direction fields point INTO the material.
 *
 * At r = 5 on a 168 px shape the effect is deliberately small: it takes the hard
 * corner off without changing the silhouette.
 */
#define PM_DPAD_R 5

typedef struct { int x, y, dx, dy; } dpad_corner_t;

/* Arm tips. Four arms, two corners each. Cut: outside the arc. */
static const dpad_corner_t s_tip[] = {
    { 0,                    PM_DPAD_ARM,         +1, +1 },
    { 0,                    2 * PM_DPAD_ARM - 1, +1, -1 },
    { PM_DPAD_S - 1,        PM_DPAD_ARM,         -1, +1 },
    { PM_DPAD_S - 1,        2 * PM_DPAD_ARM - 1, -1, -1 },
    { PM_DPAD_ARM,          0,                   +1, +1 },
    { 2 * PM_DPAD_ARM - 1,  0,                   -1, +1 },
    { PM_DPAD_ARM,          PM_DPAD_S - 1,       +1, -1 },
    { 2 * PM_DPAD_ARM - 1,  PM_DPAD_S - 1,       -1, -1 },
};

/* Is this point inside the sharp cross, before rounding? */
static bool dpad_cross(int x, int y)
{
    if (x < 0 || y < 0 || x >= PM_DPAD_S || y >= PM_DPAD_S)
        return false;

    /* Horizontal bar spans the full width of the two middle thirds; vertical bar
     * spans the full height of the two middle thirds. Their union is the cross. */
    return (y >= PM_DPAD_ARM && y < 2 * PM_DPAD_ARM) ||
           (x >= PM_DPAD_ARM && x < 2 * PM_DPAD_ARM);
}

static int dpad_lo(int c, int d) { return d > 0 ? c : c - PM_DPAD_R; }
static int dpad_hi(int c, int d) { return d > 0 ? c + PM_DPAD_R : c; }

static bool dpad_inside(int x, int y)
{
    bool in = dpad_cross(x, y);

    for (size_t i = 0; i < sizeof(s_tip) / sizeof(s_tip[0]); i++)
    {
        const dpad_corner_t *c = &s_tip[i];

        if (x < dpad_lo(c->x, c->dx) || x > dpad_hi(c->x, c->dx) ||
            y < dpad_lo(c->y, c->dy) || y > dpad_hi(c->y, c->dy))
            continue;

        const int ox = c->x + c->dx * PM_DPAD_R;
        const int oy = c->y + c->dy * PM_DPAD_R;
        const int dx = x - ox, dy = y - oy;

        if (dx * dx + dy * dy > PM_DPAD_R * PM_DPAD_R)
            in = false;
    }

    return in;
}

/*
 * THE FOUR ARROWS, at double size and placed in the arms.
 *
 * These are the same four characters the desktop's right column uses -- U+2190,
 * U+2191, U+2192, U+2193, taken from the same 8x16 font -- so the watch's d-pad
 * and its margin arrows read as one piece of furniture rather than two. Each is
 * doubled to 16x32 by sampling every source pixel as a 2x2 block, which is the
 * only scale that keeps an 8x16 glyph legible without inventing pixels: a
 * non-integer scale would drop or double stems unevenly and a stem is one pixel
 * wide at this size.
 *
 * Positions are centred in each arm, computed from PM_DPAD_ARM so they follow the
 * arms if the d-pad is ever resized. The arms are PM_DPAD_ARM wide and this glyph
 * is 32 tall at double size, which leaves 12 px of arm either side -- comfortably
 * clear of the outline even with the 5 px corner rounding taking a bite.
 */
#define PM_ARROW_W 16
#define PM_ARROW_H 32

typedef struct { uint16_t cp; int x, y; } dpad_arrow_t;

/* Arm centres. The arms are the OUTER thirds -- [0, A) and [2A, S) -- so their
 * centres are A/2 and 5A/2, not A/2 and 3A/2. Using 3A/2 puts the glyph in the
 * middle square instead of the arm, which is what the first version did: Up and
 * Left landed correctly and Down and Right drew on top of each other in the
 * centre. Both are computed from PM_DPAD_ARM so they follow the arms if the
 * d-pad is resized. */
#define PM_ARM_NEAR  (PM_DPAD_ARM / 2)
#define PM_ARM_FAR   (5 * PM_DPAD_ARM / 2)

static const dpad_arrow_t s_arrow[] = {
    /* up */    { 0x2191, (PM_DPAD_S - PM_ARROW_W) / 2,        PM_ARM_NEAR - PM_ARROW_H / 2 },
    /* down */  { 0x2193, (PM_DPAD_S - PM_ARROW_W) / 2,        PM_ARM_FAR  - PM_ARROW_H / 2 },
    /* left */  { 0x2190, PM_ARM_NEAR - PM_ARROW_W / 2,        (PM_DPAD_S - PM_ARROW_H) / 2 },
    /* right */ { 0x2192, PM_ARM_FAR  - PM_ARROW_W / 2,        (PM_DPAD_S - PM_ARROW_H) / 2 },
};

/* Is (x, y) ink in one of the arrows, in d-pad local coordinates? */
static bool dpad_arrow_ink(int x, int y)
{
    for (size_t i = 0; i < sizeof(s_arrow) / sizeof(s_arrow[0]); i++)
    {
        const dpad_arrow_t *a = &s_arrow[i];
        const int gx = x - a->x, gy = y - a->y;

        if (gx < 0 || gy < 0 || gx >= PM_ARROW_W || gy >= PM_ARROW_H)
            continue;

        const uint8_t *rows = font8x16_glyph(a->cp);
        if (!rows)
            continue;

        /* Doubling: every source pixel becomes a 2x2 block. */
        const uint8_t bits = rows[gy / 2];

        if (bits & (0x80 >> (gx / 2)))
            return true;
    }

    return false;
}

/*
 * THE OUTLINE IS TWO PIXELS, THE SAME AS THE MARGIN BUTTONS.
 *
 * `display.c` builds a button's border as the shape minus a copy of itself inset
 * by BTN_BORDER, and BTN_BORDER is 2. The same construction is used here rather
 * than a thicker one-pixel test, so the d-pad's edge has the same weight as the
 * twelve buttons around it: the outline is every inside pixel whose 5x5
 * neighbourhood is not entirely inside, which is a Chebyshev erosion by 2.
 *
 * A one-pixel test repeated twice would have been the obvious alternative and is
 * not the same thing -- it thickens diagonals unevenly and leaves a notch at each
 * of the rounded tips, which is exactly where the eye goes.
 */
#define PM_DPAD_LINE 2

static bool dpad_core(int x, int y)
{
    for (int dy = -PM_DPAD_LINE; dy <= PM_DPAD_LINE; dy++)
        for (int dx = -PM_DPAD_LINE; dx <= PM_DPAD_LINE; dx++)
            if (!dpad_inside(x + dx, y + dy))
                return false;

    return true;
}

/* --- the two round action buttons -------------------------------------------
 *
 * Right of the d-pad, stacked vertically, sized to fill what is left with the
 * same 28 px margin the d-pad keeps from the edges. The width is what binds: the
 * gap between the d-pad's right edge and the framebuffer's right margin is 124 px,
 * so a 28 px margin on each side leaves 68 for a circle, and 68 is the diameter.
 *
 * Vertically the pair occupies the d-pad's own band, 220..388. Two 68 px circles
 * leave 32 px between them, which is wider than the outer margins -- unavoidable
 * at this diameter, and it reads as two buttons rather than as a column.
 *
 * THEY CARRY A AND B, which is an assumption worth stating: the labels were not
 * specified, and A and B are the two action buttons the console's own layout puts
 * beside the d-pad. `C` is already a margin button on the right. If these are
 * meant to be something else, the label table is the only thing to change.
 */
/* cos(45 degrees) in 16.16 fixed point, shared by both rotations -- the d-pad's
 * clockwise one and the A/B labels' counter-clockwise one. Exactly representable
 * and identical run to run, which matters because these shapes are rasterised
 * once and any drift would show as a different edge. */
#define PM_ROT_K     46341

/*
 * THE TWO CIRCLES ARE PLACED INDEPENDENTLY, in grid cells.
 *
 * They began as one column -- same x, y separated by PM_PAD_GAP -- which was
 * right while they were a pair beside the d-pad. The operator now positions each
 * one separately, and positions them by counting GRID SQUARES on the glass rather
 * than in pixels, because the grid is the coordinate system a person can actually
 * read off the watch. So each circle carries its own cell offset from the
 * original column, and one cell is PM_GRID (16 px).
 *
 * The base is the pair's original position: x 224, A at the d-pad's top row --
 * 260, which is the centred-and-nudged value -- and B one diameter plus one gap
 * below that. Offsets are in cells and added to that base, so the
 * numbers below read as "how far did it move on the grid" rather than as
 * absolute pixels that have to be recomputed whenever anything else shifts.
 *
 * PM_PAD_X0/PM_PAD_Y0 are the BASE, in framebuffer coordinates. A and B are index
 * 0 and 1 everywhere -- in the label table, in pad_inside(), in pad_cy() -- so the
 * offset tables are indexed the same way and cannot get out of step.
 */
#define PM_PAD_D0    68                       /* the diameter both started at */
#define PM_PAD_D     84                       /* grown by one cell           */
#define PM_PAD_XBASE 224
#define PM_PAD_YBASE (PM_DPAD_Y)                                      /* 260 */
#define PM_PAD_GAP   ((PM_DPAD_S - 2 * PM_PAD_D0))                    /* 32  */

/* Cells. One cell is PM_GRID. Positive x is right, positive y is down. */
#define PM_PAD_CELL   PM_GRID

typedef struct { int cx, cy; } pad_off_t;

/*
 * PLACEMENT AND SIZE ARE PER CIRCLE, and the diameter is now independent of the
 * font. Both circles grew by one cell -- 68 to 84 -- while the labels stay at the
 * d-pad arrows' doubled 16x32, because the instruction was to resize the circles
 * and explicitly not the text.
 *
 * OFF SETS ARE FROM THE BASE COLUMN, NOT FROM EACH OTHER, so either circle can be
 * moved without disturbing the other. A shared an x offset with B for one cycle,
 * when the operator wanted them on one vertical axis; the two values are separate
 * now that A can be placed independently.
 *
 * HORIZONTAL HEADROOM: THE RIGHT COLUMN IS THE WALL, NOT THE SCREEN. The push
 * above the column bottom is framebuffer-wide -- panel 67..386 -- because the
 * column occupies 387..431, so a circle that reaches past fb column 319 is simply
 * not drawn there and the black under it shows as a rectangular notch. Measured
 * against that wall, with the circle 84 px wide from a base of x 224:
 *
 *   xoff 0 -> panel 291..374   clipped 0 px
 *   xoff 1 -> panel 307..390   clipped 4 px
 *   xoff 2 -> panel 323..406   clipped 20 px
 *   xoff 3 -> panel 339..422   clipped 36 px
 *
 * xoff 3 was tried on the operator's instruction and produced a 36 px notch in A's
 * upper-right, which is what this value used to be. 1 is the compromise: one cell
 * right as asked, with a 4 px cut that reads as a slightly flat edge rather than a
 * step. Only 0 is exactly clean.
 *
 * Shrinking the circle does not fix this -- at 68 px, xoff 2 still clips 4 px and
 * xoff 3 still clips 20 -- because the wall is a column boundary and not a size
 * limit. The vertical direction has more room: B's bottom is 443 against a glass
 * bottom of 456.
 */
#define PM_PAD_A_XOFF   1                     /* 1 cell right of the base */
#define PM_PAD_B_XOFF  (-2)   /* 2 cells left of the base */
#define PM_PAD_A_YOFF   1                     /* cells below the base row */
#define PM_PAD_B_YOFF   0                     /* up one from where it was */

static const pad_off_t s_pad_off[2] = {
    /* A: base column, 1 cell down */
    { .cx = PM_PAD_A_XOFF, .cy = PM_PAD_A_YOFF },
    /* B: base column, on the base row -- up one cell from where it was */
    { .cx = PM_PAD_B_XOFF, .cy = PM_PAD_B_YOFF },
};

/* Top-left of circle i, in framebuffer coordinates. B starts one OLD diameter plus
 * one gap below A's base, which is the column the pair was built as; the offsets
 * then move each from there. Using the original 68 and not the new 84 matters:
 * the base row is a fixed reference, so growing a circle does not silently shift
 * the other one. */
static int pad_x(int i) { return PM_PAD_XBASE + s_pad_off[i].cx * PM_PAD_CELL; }

static int pad_y(int i)
{
    const int base = PM_PAD_YBASE + (i ? PM_PAD_D0 + PM_PAD_GAP : 0);

    return base + s_pad_off[i].cy * PM_PAD_CELL;
}

static const uint16_t s_pad_label[2] = { 'A', 'B' };

static int pad_cy(int i) { return pad_y(i) + PM_PAD_D / 2; }
static int pad_cx(int i) { return pad_x(i) + PM_PAD_D / 2; }

/* Inside a circle of radius (D-1)/2, measured in half-pixels so the disc is
 * symmetric -- an even diameter has no centre pixel and an integer radius test
 * would put the bias on one side. */
static bool pad_inside(int i, int x, int y)
{
    const int dx2 = 2 * (x - pad_cx(i));
    const int dy2 = 2 * (y - pad_cy(i));
    const int rr = PM_PAD_D - 1;

    return dx2 * dx2 + dy2 * dy2 <= rr * rr;
}

/*
 * The letter, doubled like the d-pad arrows and UPRIGHT.
 *
 * It was rotated 45 degrees counter-clockwise for one cycle and the operator
 * asked for it back at 0. Upright is also the better answer for a glyph this
 * small: a 16x32 bitmap resampled onto a diagonal staircases badly, and at 84 px
 * the circle has room to hold it square. The d-pad's arrows stay rotated because
 * they are part of a rotated shape; these two are marks on their own circles and
 * do not have to agree with it.
 */
static bool pad_label_ink(int i, int x, int y)
{
    const int cx = pad_cx(i), cy = pad_cy(i);

    const int gx = (x - cx) + PM_ARROW_W / 2;
    const int gy = (y - cy) + PM_ARROW_H / 2;

    if (gx < 0 || gy < 0 || gx >= PM_ARROW_W || gy >= PM_ARROW_H)
        return false;

    const uint8_t *rows = font8x16_glyph(s_pad_label[i]);
    if (!rows)
        return false;

    return (rows[gy / 2] & (0x80 >> (gx / 2))) != 0;
}

/*
 * One destination row of the d-pad. A pixel is on the outline when it is inside
 * the rounded cross and at least one of its neighbours is not.
 *
 * EIGHT NEIGHBOURS, NOT FOUR, and the reason is the inner corners. An arc that
 * merely grazes an edge -- which is exactly what a fillet tangent to two edges
 * does -- leaves one pixel of the material edge removed and the pixel beyond it
 * drawn, with nothing drawn beside it. On the four-neighbour rule that is an
 * isolated white speck floating next to the outline: eight of them, two at each
 * inner corner. Counting the diagonals merges each speck into the arc it belongs
 * to. Measured on the shape: 640 pixels drawn and 8 specks on four neighbours,
 * 684 drawn and none on eight. The line is a little thicker where it steps
 * diagonally, which on a 168 px figure is not visible.
 */
/*
 * THE D-PAD IS ROTATED 45 DEGREES CLOCKWISE, and the whole thing rotates with it
 * -- arms and arrows both.
 *
 * The implementation is an inverse transform rather than a rotated rasteriser:
 * every pixel is mapped back into the unrotated frame and tested there, so
 * dpad_inside() and the arrow sampler are unchanged and cannot drift apart from
 * the shape. One rotation, one place.
 *
 * SCREEN -> SHAPE is the inverse of the rotation, which is why the sign looks
 * backwards: a point in the upper-right of the screen is mapped to the TOP arm.
 * With dx, dy measured from the d-pad's centre and k = cos45:
 *
 *     u = (dx + dy) * k        v = (dy - dx) * k
 *
 * and the result is tested at (u + S/2, v + S/2) in the shape's own 0..S frame.
 *
 * THE ARROWS ROTATE TOO, and they are the reason this is done as a transform of
 * the input rather than as a dance of pre-rotated tables: the four glyphs are
 * sampled in shape space, so they come out pointing into the rotated arms'
 * directions without a second copy of the font or a rotated bitmap to maintain.
 * At 45 degrees a 16x32 arrow is resampled on a diagonal, which is expected and
 * is what "rotate the entire dpad" means.
 *
 * PM_ROT_HALF is the rotated shape's bounding half-extent: the cross's extremes
 * are its arm tips, and the largest |u| among them is 84 * sqrt(2) / 1.5 =
 * 79.2, plus one row for the 2 px border's outer edge. The region below is built
 * from it, so the rotated d-pad cannot be clipped.
 *
 * Fixed point, not float: k is 46341 in 16.16, which is cos45 to within one
 * part in 65536. The whole shape is rasterised once at start, so the arithmetic
 * cost is irrelevant and an exactly-representable integer keeps the rendering
 * identical run to run.
 */
#define PM_ROT_HALF  81               /* rotated bounding half-extent */

static void dpad_unrotate(int x, int y, int *u, int *v)
{
    /* x, y are relative to the d-pad's centre, in framebuffer pixels. */
    *u = ((x + y) * PM_ROT_K) >> 16;
    *v = ((y - x) * PM_ROT_K) >> 16;
}

/* The whole control cluster as one region: the d-pad plus the two circles to its
 * right. One push, not two, because they are adjacent, they never change, and two
 * windows would be two bus acquisitions and two transfers for one static image. */
#define PM_DPAD_CX  (PM_DPAD_X + PM_DPAD_S / 2)                    /* 100 */
#define PM_DPAD_CY  (PM_DPAD_Y + PM_DPAD_S / 2)                    /* 324 */

#define PM_ROT_X0   (PM_DPAD_CX - PM_ROT_HALF)                     /* 19  */
#define PM_ROT_Y0   (PM_DPAD_CY - PM_ROT_HALF)                     /* 243 */
#define PM_ROT_X1   (PM_DPAD_CX + PM_ROT_HALF)                     /* 181 */
#define PM_ROT_Y1   (PM_DPAD_CY + PM_ROT_HALF)                     /* 405 */

/* THE REGION IS THE WHOLE AREA BELOW THE PICTURE, not just the controls' bounding
 * box, because the background is now the grid and a grid has to reach the edges
 * it divides. Previously this was the union of the d-pad's rotated box and the
 * circles' box, which would have painted a 273x166 patch of graph paper with
 * black all round it -- a harder thing to read than either extreme.
 *
 * It spans the full framebuffer width and runs from the picture's bottom edge to
 * the bottom of the glass, which is 41 rows past the framebuffer and is only
 * reachable at all because this goes through display_panel_rows(). */
/*
 * THE PUSH IS TWO ZONES, NOT ONE RECTANGLE, and that is what the column resize
 * bought.
 *
 * Above the columns' last button the only safe width is the framebuffer's: the
 * side bars occupy panel 22..66 and 387..431, and a full-width rectangle there
 * erases them -- which it did, cutting the third button in half.
 *
 * Below their last button the whole glass is free, because this app draws only
 * three slots per column. That space is where the d-pad and the action buttons
 * live, and an earlier version could not use it: the region was one rectangle at
 * the framebuffer's width, so anything to the right of panel 386 was clipped
 * whatever the glass had room for. That is what made "move A a few cells right"
 * look impossible.
 *
 * So: an upper band at framebuffer width, and a lower band at full glass width
 * from the column bottom down.
 */
/*
 * WHERE THE BAND BEING DRAWN STARTS, in framebuffer coordinates.
 *
 * BOTH axes, and the y one is not decoration. The push is two bands with
 * different origins, and the callback's `row` is counted from the region's own
 * top -- so a band that starts at fb y 294 has to be told that, or every shape is
 * offset by the difference. The first version passed only x and added the fixed
 * PM_CTL_Y for y, which drew the lower band 102 rows too low: the d-pad was cut
 * in half by the bottom of the glass, B fell off it entirely, and A clipped. All
 * three symptoms, one missing offset.
 */
static int s_row_x0;
static int s_row_y0;

#define PM_COL_SLOTS  3

/* Bottom row of the last drawn column slot, in framebuffer coordinates. Derived
 * from the same formula edges.h uses, so it follows a slot-count change. */
#define PM_COL_BOT    (COL_Y0 + (PM_COL_SLOTS * (COL_Y1 - COL_Y0 + 1)) / COL_N - 1)

#define PM_CTL_Y      (PM_H * PM_SCALE)                                /* 192 */
#define PM_CTL_Y1     (PM_PAD_YBASE + PM_PAD_D0 + PM_PAD_GAP +                     \
                       PM_PAD_B_YOFF * PM_PAD_CELL + PM_PAD_D)         /* 443 */

/* Upper band: framebuffer width, from the picture down to the column bottom. */
#define PM_UP_X       0
#define PM_UP_W       DISPLAY_W                                    /* 320 */
#define PM_UP_H       (PM_COL_BOT - PM_CTL_Y + 1)                  /* 102 */

/* Lower band: full glass width, from just below the column bottom. */
#define PM_LO_X       (-DISPLAY_ORIGIN_X)                          /* -45 */
#define PM_LO_W       DISPLAY_PANEL_W                              /* 410 */
#define PM_LO_H       (PM_CTL_Y1 - (PM_COL_BOT + 1) + 1)           /* 150 */

static void pm_dpad_row(void *ctx, int row, int col, uint16_t *dst, int n)
{
    (void)ctx;

    for (int i = 0; i < n; i++)
    {
        /* Region-local, because that is what the row callback is given: `col`
         * starts at the region's left edge, not the framebuffer's. */
        const int x = col + i;
        const int y = row;

        /* Both shapes are tested in framebuffer coordinates, and the grid is
         * drawn in those too, so the background lines up with the framebuffer's
         * own origin rather than with the region's. */
        const int fx = x + s_row_x0;
        const int fy = y + s_row_y0;

        uint16_t c = pm_grid(fx, fy);

        /* --- the d-pad, rotated ------------------------------------------- */
        int u, v;
        dpad_unrotate(fx - PM_DPAD_CX, fy - PM_DPAD_CY, &u, &v);

        const int dx0 = u + PM_DPAD_S / 2;
        const int dy0 = v + PM_DPAD_S / 2;

        if (dpad_inside(dx0, dy0))
        {
            /* Two-pixel outline: inside the shape but not inside its erosion. */
            c = dpad_core(dx0, dy0) ? s_dpad_face : s_dpad_edge;

            if (c == s_dpad_face && dpad_arrow_ink(dx0, dy0))
                c = s_dpad_ink;
        }
        else
        {
            /* --- the two circles -----------------------------------------
             * Only tested where the d-pad is absent: they do not overlap, and
             * skipping the test keeps the grid where neither shape is. */
            for (int b = 0; b < 2; b++)
            {
                if (!pad_inside(b, fx, fy))
                    continue;

                bool core = true;

                for (int ey = -PM_DPAD_LINE; ey <= PM_DPAD_LINE && core; ey++)
                    for (int ex = -PM_DPAD_LINE; ex <= PM_DPAD_LINE; ex++)
                        if (!pad_inside(b, fx + ex, fy + ey))
                        {
                            core = false;
                            break;
                        }

                c = core ? s_dpad_face : s_dpad_edge;

                if (c == s_dpad_face && pad_label_ink(b, fx, fy))
                    c = s_dpad_ink;

                break;
            }
        }

        dst[i] = c;
    }
}

/* Push the d-pad. Safe to call more than once -- it overwrites the same region --
 * which is what makes `pm dpad` useful for adjusting the shape without a
 * restart. Requires the pixels to be claimed. */
static bool pm_draw_dpad(void)
{
    if (!display_pixels_claimed())
        return false;

    /* Fetched per draw, not cached at init: display_margin_colours() reads the
     * terminal palette, which is populated during display_init() and could move
     * if that ever gains themes. */
    uint16_t fill, edge, ink;
    display_margin_colours(&fill, &edge, &ink);

    /* The panel wants each 16-bit pixel high byte first, and this port's other
     * picture paths hand over pre-swapped values. The margin painters build their
     * own through display_swap16(); these three are swapped here, once. */
    s_dpad_face = display_swap16(fill);
    s_dpad_edge = display_swap16(edge);
    s_dpad_ink  = display_swap16(ink);

    /*
     * Two pushes. `x_origin` reaches the row callback through the ctx pointer,
     * because the callback needs to know which framebuffer column it is drawing
     * and the two bands start at different ones: the upper at fb 0, the lower at
     * fb -45, since the lower band spans the whole glass.
     */
    bool ok = true;

    s_row_x0 = PM_UP_X;
    s_row_y0 = PM_CTL_Y;
    ok = display_panel_rows(PM_UP_X + DISPLAY_ORIGIN_X + PANEL_COL_OFFSET,
                            PM_CTL_Y + DISPLAY_ORIGIN_Y,
                            PM_UP_W, PM_UP_H, pm_dpad_row) && ok;

    s_row_x0 = PM_LO_X;
    s_row_y0 = PM_COL_BOT + 1;
    ok = display_panel_rows(PM_LO_X + DISPLAY_ORIGIN_X + PANEL_COL_OFFSET,
                            PM_COL_BOT + 1 + DISPLAY_ORIGIN_Y,
                            PM_LO_W, PM_LO_H, pm_dpad_row) && ok;

    return ok;
}

/* --- the margin buttons ----------------------------------------------------
 *
 * WHAT THE GLASS SAYS WHILE A POKEMON MINI IS RUNNING.
 *
 * The layout is the operator's, and it follows the console's own control set
 * rather than the desktop's keys:
 *
 *   left column,  top to bottom   SS, Save, Load, Pause, Reset
 *   bottom strip, left to right   Shake, A, B
 *   right column, top to bottom   Hold, Sound, Speed, (blank), C
 *
 * The blank right-column slot is deliberate and is not a mistake: four of the
 * five right-hand slots are taken and the fifth is where a later control goes,
 * so leaving it visibly empty keeps the button positions stable when it arrives.
 *
 * THE LABELS ARE APPLIED AS OVERRIDES, NOT BY EDITING edges.h. Those tables are
 * the Tamagotchi's and are pinned against its action table by
 * tests/test_tama_ui.c; adding a second app's names to them would either break
 * that test or force it to know about two apps. `display_margin_label()` and
 * `display_margin_btn_label()` exist for exactly this and fall back to the
 * shared default when a slot is cleared.
 *
 * NO BUTTON DOES ANYTHING YET. This cycle is the labels only; the actions are
 * the next one. That is worth stating in the source because edges.h's rule is
 * that a label must not claim a job the button is not doing -- so these are
 * currently a statement of intent, and the first thing the next cycle owes is
 * making them true. `SS` is the one that needs explaining: it is the
 * screenshot/save-state slot, and it is the name the operator asked for.
 */
/* THE SEVEN BARE SLOTS CARRY NO TEXT EITHER.
 *
 * They have no box and no label: Pause, Reset, Shake, A, B, C and the reserved
 * slot are simply black glass. Their duties are drawn on the framebuffer instead
 * -- the d-pad and the A and B circles -- so a legend beside them would be a
 * second name for a control the player can already see.
 *
 * The six that keep a label keep their box too, so a slot is either a drawn
 * button with its name or nothing at all, and the two never mix. */
/* A SINGLE SPACE, NOT AN EMPTY STRING, and that is not a trick.
 * display_margin_label() treats "" as "clear the override so the shared table
 * applies again", which would restore the Tamagotchi's words -- "Sound",
 * "Reset", the degree circle -- on exactly the slots meant to be blank. A space
 * is a real label that happens to be invisible, so the override sticks and
 * nothing is drawn. */
#define PM_BLANK " "

static const char *const PM_LEFT[COL_N]  = { "SS", "Save", "Load", PM_BLANK, PM_BLANK };
static const char *const PM_RIGHT[COL_N] = { "Hold", "Sound", "Speed", PM_BLANK, PM_BLANK };
static const char *const PM_BOTTOM[BTN_N] = { PM_BLANK, PM_BLANK, PM_BLANK };

/*
 * WHICH SLOTS ARE BARE GLASS rather than drawn buttons.
 *
 * The controls the app supplies itself -- the d-pad and the A and B circles on
 * the framebuffer -- must not also appear as drawn buttons in the margins, or
 * the glass carries two controls for one duty. So Pause and Reset on the left,
 * all three bottom bars, and the blank slot with C on the right lose their box
 * and keep only their label; SS, Save, Load, Hold, Sound and Speed keep theirs.
 *
 * Expressed as three masks in the label tables' index order, so reading them
 * beside the tables above shows exactly which slots are affected.
 */
static const bool PM_BARE_LEFT[COL_N]  = { false, false, false, true,  true  };
static const bool PM_BARE_RIGHT[COL_N] = { false, false, false, true,  true  };
static const bool PM_BARE_BOTTOM[BTN_N] = { true, true, true };

/* Store the overrides. Safe to call before the app owns the glass: the setters
 * record the label and simply skip the repaint, so the single paint that
 * display_margins_set() then does is the one that shows all twelve at once. */
static void pm_label_margins(void)
{
    for (int i = 0; i < COL_N; i++)
    {
        display_margin_label(0, i, PM_LEFT[i]);
        display_margin_label(1, i, PM_RIGHT[i]);
    }

    for (int i = 0; i < BTN_N; i++)
        display_margin_btn_label(i, PM_BOTTOM[i]);

    /* Chrome after labels, so the one repaint each setter does already carries
     * the right text. */
    for (int i = 0; i < COL_N; i++)
    {
        display_margin_chrome(0, i, !PM_BARE_LEFT[i]);
        display_margin_chrome(1, i, !PM_BARE_RIGHT[i]);
    }

    for (int i = 0; i < BTN_N; i++)
        display_margin_chrome(2, i, !PM_BARE_BOTTOM[i]);
}

/* Clearing every slot hands the next app the shared defaults back. Without this
 * the Tamagotchi would inherit "Shake A B" on the strip. */
static void pm_unlabel_margins(void)
{
    for (int i = 0; i < COL_N; i++)
    {
        display_margin_label(0, i, "");
        display_margin_label(1, i, "");
    }

    for (int i = 0; i < BTN_N; i++)
        display_margin_btn_label(i, "");

    /* The next app gets the desktop's five back. */
    display_margin_column_slots(COL_N);

    /* Restore every box, or the next app inherits bare slots. */
    for (int i = 0; i < COL_N; i++)
    {
        display_margin_chrome(0, i, true);
        display_margin_chrome(1, i, true);
    }

    for (int i = 0; i < BTN_N; i++)
        display_margin_chrome(2, i, true);
}

bool pmini_start(const char *rom_path)
{
    if (s_run)
    {
        printf("pm: already running -- pm stop first\n");
        return false;
    }

    const char *path = rom_path ? rom_path : PM_ROM_DEFAULT;

    int len = 0;
    s_rom = load_rom(path, &len);
    if (!s_rom)
        return false;

    /* Create fills PM_BIOS with 0xFF and loads the GPL free BIOS. A RETAIL
     * CARTRIDGE NEEDS THE RETAIL BIOS: the free one idles at $0040 and does not
     * hand over, and the two diverge at byte 6 (`2F 02` against `CF 01`). So a
     * dump at PM_BIOS_DEFAULT is loaded over the top when it is there, and which
     * one ran is reported rather than assumed. No BIOS dump is in this
     * repository; it is placed on the device by whoever owns it. */
    if (!PokeMini_Create(0, 0))
    {
        printf("pm: PokeMini_Create failed\n");
        heap_caps_free(s_rom);
        s_rom = NULL;
        return false;
    }

    /*
     * THE FREE BIOS IS NOT A FALLBACK HERE, IT IS A REFUSAL.
     *
     * The GPL free BIOS is compiled into the component and PokeMini_Create()
     * memcpy's it into PM_BIOS on every start, so there is no file to delete that
     * would stop it being used -- the only thing that stops it is checking.
     * PokeMini_LoadBIOSFile() clears PokeMini_FreeBIOS when it reads a 4096-byte
     * dump, so that flag is the authoritative answer to "which BIOS is in memory
     * right now", and it is read after the load rather than trusting the fopen.
     *
     * Why refusing is right rather than harsh: the free BIOS idles at $0040 and
     * never hands over to a retail cartridge. Booting a commercial cartridge on
     * it does not fail loudly -- it produces a running emulator that executes the
     * BIOS's idle loop and reports a healthy frame rate over a static or blank
     * picture, which is the precise failure this project has already paid for
     * twice (core-log 66 and EMU-018). A refusal names the missing file.
     */
    if (PokeMini_LoadBIOSFile(PM_BIOS_DEFAULT) <= 0 || PokeMini_FreeBIOS)
    {
        printf("pm: no retail BIOS at %s -- refusing to start\n", PM_BIOS_DEFAULT);
        printf("pm: the free BIOS boots no retail cartridge; put a 4096-byte\n");
        printf("pm: dump there (it is not part of this repository)\n");
        PokeMini_Destroy();
        heap_caps_free(s_rom);
        s_rom = NULL;
        return false;
    }

    snprintf(s_bios, sizeof(s_bios), "retail %s", PM_BIOS_DEFAULT);

    if (!PokeMini_SetMINMem(s_rom, len))
    {
        printf("pm: %s: not a cartridge image\n", path);
        PokeMini_Destroy();
        heap_caps_free(s_rom);
        s_rom = NULL;
        return false;
    }

    PokeMini_Reset(0);

    /* The screen, claimed before the first push. A failed claim must not leave
     * the emulator running against a framebuffer it does not own. */
    display_pixels_claim(true);
    display_fill(0x0000);

    /* Three slots per column, not five: this app's own d-pad and action buttons
     * own everything below the third button, and the columns must stop there for
     * that claim to mean anything. Set before the margins are painted. */
    display_margin_column_slots(3);

    /* Labels first, then the single paint that shows them. */
    pm_label_margins();
    display_margins_set(DISPLAY_MARGINS_APP);

    /* Once. Nothing else is writing yet -- the blitter task does not exist. */
    if (!pm_draw_dpad())
        ESP_LOGW(TAG, "d-pad push refused");

    s_frame = xSemaphoreCreateCounting(PM_TICK_BACKLOG, 0);
    if (!s_frame)
    {
        display_pixels_claim(false);
        PokeMini_Destroy();
        heap_caps_free(s_rom);
        s_rom = NULL;
        printf("pm: no semaphore\n");
        return false;
    }

    /*
     * THE POOL GOES IN INTERNAL RAM IF IT WILL, because the blitter reads it
     * every frame and the 512 KB cartridge is already saturating PSRAM from core
     * 0. 18 KB is not always available -- the largest free internal block has
     * measured 31,744 bytes with the SSH server up -- so PSRAM is the fallback
     * and which one was used is reported rather than assumed.
     */
    s_pool_internal = true;
    for (int i = 0; i < PM_VBUF_N; i++)
    {
        s_dot[i] = heap_caps_malloc(PM_DOTS, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_dot[i])
        {
            s_pool_internal = false;
            break;
        }
    }

    if (!s_pool_internal)
    {
        for (int i = 0; i < PM_VBUF_N; i++)
        {
            if (s_dot[i])
            {
                heap_caps_free(s_dot[i]);
                s_dot[i] = NULL;
            }
        }
        for (int i = 0; i < PM_VBUF_N; i++)
            s_dot[i] = heap_caps_malloc(PM_DOTS, MALLOC_CAP_SPIRAM);
    }

    for (int i = 0; i < PM_VBUF_N && !s_dot[i]; i++)
    {
        printf("pm: no memory for frame buffer %d\n", i);
        goto fail;
    }

    s_prev = heap_caps_malloc(PM_DOTS, s_pool_internal
                                          ? (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
                                          : MALLOC_CAP_SPIRAM);
    if (!s_prev)
    {
        printf("pm: no memory for the compare buffer\n");
        goto fail;
    }
    s_have_prev = false;
    s_blit_same = 0;

    s_free_q = xQueueCreate(PM_VBUF_N, sizeof(int));
    s_filled_q = xQueueCreate(PM_VBUF_N, sizeof(int));
    if (!s_free_q || !s_filled_q)
    {
        printf("pm: no queues\n");
        goto fail;
    }

    for (int i = 0; i < PM_VBUF_N; i++)
        xQueueSend(s_free_q, &i, 0);

    /* The emulator starts with one buffer already claimed. */
    xQueueReceive(s_free_q, &s_cur, 0);

    s_blits = 0;
    s_blit_skipped = 0;
    s_blit_us = 0;
    s_emu_us = 0;
    s_blit_run = true;

    if (!s_nobl_diag && xTaskCreatePinnedToCore(pm_blit_task, "pmblit", PM_BLIT_STACK, NULL,
                                PM_PRIO, &s_blit_task, 1) != pdPASS)
    {
        s_blit_run = false;
        s_blit_task = NULL;
        printf("pm: no blit task\n");
        goto fail;
    }

    s_pad = 0;
    s_pad_applied = 0;
    s_frames = 0;
    s_misses = 0;
    s_started_us = esp_timer_get_time();
    s_run = true;

    const esp_timer_create_args_t args = {
        .callback = pm_tick,
        .arg = NULL,
        .name = "pmtick",
    };

    if (esp_timer_create(&args, &s_tick) != ESP_OK ||
        esp_timer_start_periodic(s_tick, PM_FRAME_US) != ESP_OK)
    {
        printf("pm: no frame timer\n");
        goto fail;
    }

    if (xTaskCreatePinnedToCore(pm_task, "pm", PM_STACK, NULL,
                                PM_PRIO, &s_task, 0) != pdPASS)
    {
        printf("pm: no task\n");
        s_task = NULL;
        goto fail;
    }

    snprintf(s_rom_path, sizeof(s_rom_path), "%s", path);
    ESP_LOGI(TAG, "running %s (%d bytes)", path, len);
    return true;

fail:
    s_run = false;
    s_blit_run = false;

    for (int i = 0; i < 100 && s_blit_task; i++)
        vTaskDelay(pdMS_TO_TICKS(10));

    if (s_tick)
    {
        esp_timer_stop(s_tick);
        esp_timer_delete(s_tick);
        s_tick = NULL;
    }
    if (s_free_q) { vQueueDelete(s_free_q); s_free_q = NULL; }
    if (s_filled_q) { vQueueDelete(s_filled_q); s_filled_q = NULL; }
    for (int i = 0; i < PM_VBUF_N; i++)
    {
        if (s_dot[i]) { heap_caps_free(s_dot[i]); s_dot[i] = NULL; }
    }
    if (s_prev) { heap_caps_free(s_prev); s_prev = NULL; }
    if (s_frame) { vSemaphoreDelete(s_frame); s_frame = NULL; }
    display_pixels_claim(false);
    PokeMini_Destroy();
    heap_caps_free(s_rom);
    s_rom = NULL;
    return false;
}

void pmini_stop(void)
{
    if (!s_run)
        return;

    s_run = false;

    if (s_tick)
    {
        esp_timer_stop(s_tick);
        esp_timer_delete(s_tick);
        s_tick = NULL;
    }

    /* Wake the task so it leaves rather than waiting out its timeout. */
    if (s_frame)
        xSemaphoreGive(s_frame);

    for (int i = 0; i < 100 && s_task; i++)
        vTaskDelay(pdMS_TO_TICKS(10));

    if (s_task)
        ESP_LOGE(TAG, "emulator task did not stop");

    /*
     * THE BLITTER GOES NEXT, BEFORE THE POOL IT READS.
     *
     * It exits within one queue timeout (50 ms) once s_blit_run is clear, and it
     * returns its buffer before doing so. Waiting for it here rather than
     * assuming it noticed is what stops a stop from freeing memory a push is
     * still reading -- the same ordering the NES port uses, and for the same
     * reason.
     */
    s_blit_run = false;

    for (int i = 0; i < 100 && s_blit_task; i++)
        vTaskDelay(pdMS_TO_TICKS(10));

    if (s_blit_task)
        ESP_LOGE(TAG, "blit task did not stop; not freeing");

    if (s_free_q) { vQueueDelete(s_free_q); s_free_q = NULL; }
    if (s_filled_q) { vQueueDelete(s_filled_q); s_filled_q = NULL; }
    if (s_frame) { vSemaphoreDelete(s_frame); s_frame = NULL; }

    PokeMini_Destroy();

    if (s_rom)
    {
        heap_caps_free(s_rom);
        s_rom = NULL;
    }

    for (int i = 0; i < PM_VBUF_N; i++)
    {
        if (s_dot[i])
        {
            heap_caps_free(s_dot[i]);
            s_dot[i] = NULL;
        }
    }

    if (s_prev)
    {
        heap_caps_free(s_prev);
        s_prev = NULL;
    }

    /* Order matters and follows nes_stop(): the margins are repainted while the
     * framebuffer is still owned, and ownership is released last, so the painter
     * never comes back to a desktop whose margins are still in app mode. */
    pm_unlabel_margins();
    display_margins_set(DISPLAY_MARGINS_DESKTOP);
    display_pixels_claim(false);
}

bool pmini_running(void)
{
    return s_run;
}

/* --- the shell command ----------------------------------------------------- */

static int cmd_pm(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    const char *sub = argc >= 2 ? argv[1] : "status";

    if (strcmp(sub, "start") == 0)
    {
        const char *rom = argc >= 3 ? argv[2] : NULL;

        if (argc >= 4 && strcmp(argv[3], "noblit") == 0)
        {
            s_nobl_diag = true;
            printf("pm: DIAGNOSTIC -- no blitter, display will not update\n");
        }
        else
        {
            s_nobl_diag = false;
        }

        if (!pmini_start(rom))
            return 1;
        printf("pm running %s\n", s_rom_path);
        printf("pm stop to leave\n");
        return 0;
    }

    if (strcmp(sub, "stop") == 0)
    {
        pmini_stop();
        printf("pm stopped\n");
        return 0;
    }

    if (strcmp(sub, "status") == 0)
    {
        if (!s_run)
        {
            printf("pm not running\n");
            return 0;
        }
        const double secs = (double)(esp_timer_get_time() - s_started_us) / 1e6;
        const double fps = secs > 0.0 ? s_frames / secs : 0.0;
        const double emu_ms = s_frames ? (double)s_emu_us / s_frames / 1000.0 : 0.0;
        const double blit_ms = s_blits ? (double)s_blit_us / s_blits / 1000.0 : 0.0;
        const double slowest = emu_ms > blit_ms ? emu_ms : blit_ms;
        printf("pm  %s  %.1f fps (machine %.2f Hz)  frames=%u missed=%u  %.1fs\n",
               s_rom_path, fps, PM_FRAME_HZ, (unsigned)s_frames,
               (unsigned)s_misses, secs);
        printf("    bios %s, pool %s\n", s_bios,
               s_pool_internal ? "internal" : "psram");
        /* Overlapped, the frame costs the SLOWER of the two, not their sum --
         * which is the entire point of the two tasks. Reporting the sum here
         * would describe the design this replaced. */
        printf("    per frame: emu %.1f ms | blit %.1f ms -> %.1f of %.1f (core 1 push)\n",
               emu_ms, blit_ms, slowest, 1000.0 / PM_FRAME_HZ);
        printf("    pushes=%u identical=%u skipped=%u\n",
               (unsigned)s_blits, (unsigned)s_blit_same, (unsigned)s_blit_skipped);

        /* WHAT RATE THE CARTRIDGE ACTUALLY DRAWS AT.
         *
         * The 71.97 Hz counter is the machine's tick, not the picture rate. The
         * PRC renders once every `divisor` ticks, chosen by the cartridge through
         * rate control (register 0x81): the match value is (divisor-1)<<4, so the
         * divisor is match/16+1 and runs 2..12. So a game's drawn frame rate is
         * 71.97/divisor -- 36, 24, 18, 14.4, 10.3, 9 or 6 Hz -- and there is no
         * divisor of 1. Reporting it is the only way "what rate is this game
         * running at" can be answered for a specific cartridge instead of
         * assumed from the base tick. */
        const int match = MinxPRC.PRCRateMatch;
        const int divisor = (match >> 4) + 1;
        printf("    prc rate reg 0x%02X, match 0x%02X -> draws every %d ticks"
               " = %.1f Hz\n",
               (unsigned)PMR_PRC_RATE, (unsigned)match, divisor,
               (double)PM_FRAME_HZ / divisor);
        return 0;
    }

    if (strcmp(sub, "dpad") == 0)
    {
        /* Redraw the d-pad in place. Safe while a cartridge runs: it overwrites
         * only its own region, and the display layer holds the bus per band and
         * programs each window inside that hold, so two pushers interleave
         * without either landing in the wrong rectangle. */
        const bool ran = s_run;

        if (!ran)
        {
            display_pixels_claim(true);
            display_fill(0x0000);
            display_margins_set(DISPLAY_MARGINS_APP);
        }

        const bool ok = pm_draw_dpad();
        printf("pm dpad %s -- band y%d..%d fb-width, y%d..%d full glass\n",
               ok ? "drawn" : "PUSH REFUSED",
               PM_CTL_Y, PM_COL_BOT, PM_COL_BOT + 1, PM_CTL_Y1);

        if (!ran)
        {
            vTaskDelay(pdMS_TO_TICKS(5000));
            display_margins_set(DISPLAY_MARGINS_DESKTOP);
            display_pixels_claim(false);
        }
        return ok ? 0 : 1;
    }

    if (strcmp(sub, "pattern") == 0)
    {
        /* Nothing here touches the emulator: this draws a 1-pixel border and an
         * 8-pixel grid at 3x through the same blit the cartridge uses, so a
         * wrong origin, scale or byte order is visible on the glass rather than
         * inferred from a counter. */
        if (s_run)
        {
            printf("pm: stop the emulator first\n");
            return 1;
        }

        /* A static buffer: the pattern is drawn once, from this task, with no
         * emulator running, so it does not need the pool. */
        static uint8_t dots[PM_DOTS];
        pm_pattern_dots(dots);

        display_pixels_claim(true);
        display_fill(0x0000);
        display_margins_set(DISPLAY_MARGINS_APP);

        const bool claimed = display_pixels_claimed();
        blit_one(dots);
        const bool ok = claimed && s_blits > 0;

        printf("pm pattern %s -- 288x192 at (%d,%d), 3x\n",
               ok ? "drawn" : "PUSH REFUSED", PM_OX, PM_OY);

        vTaskDelay(pdMS_TO_TICKS(6000));

        display_margins_set(DISPLAY_MARGINS_DESKTOP);
        display_pixels_claim(false);
        return ok ? 0 : 1;
    }

    if (strcmp(sub, "bench") == 0)
    {
        /*
         * Unpaced measurement of a cartridge, with no display and no task.
         *
         * This replaces an earlier `pm ff` that advanced the core from the shell
         * while the frame task was still doing the same thing, and the only thing
         * it produced was the core's own crash diagnostic as two threads raced
         * one emulated CPU. It is a separate path now: it refuses while the
         * emulator is running, and it owns the core for its whole duration.
         */
        if (s_run)
        {
            printf("pm: stop the emulator first\n");
            return 1;
        }

        const char *path = argc >= 3 ? argv[2] : PM_ROM_DEFAULT;
        const int n = argc >= 4 ? atoi(argv[3]) : 400;

        int len = 0;
        uint8_t *rom = load_rom(path, &len);
        if (!rom)
            return 1;

        if (!PokeMini_Create(0, 0))
        {
            heap_caps_free(rom);
            printf("pm: PokeMini_Create failed\n");
            return 1;
        }
        if (PokeMini_LoadBIOSFile(PM_BIOS_DEFAULT) <= 0 || PokeMini_FreeBIOS)
        {
            printf("pm: no retail BIOS at %s -- refusing to measure\n", PM_BIOS_DEFAULT);
            PokeMini_Destroy();
            heap_caps_free(rom);
            return 1;
        }

        snprintf(s_bios, sizeof(s_bios), "retail");
        PokeMini_SetMINMem(rom, len);
        PokeMini_Reset(0);

        /* BIOS handover first, so its cost is not counted as per-frame. */
        PokeMini_EmulateCycles(PM_FRAME_CYCLES * 8);

        const unsigned long s0 = g_exec_count;
        const int64_t t0 = esp_timer_get_time();
        for (int i = 0; i < n; i++)
            PokeMini_EmulateCycles(PM_FRAME_CYCLES);
        const int64_t dt = esp_timer_get_time() - t0;
        const unsigned long steps = g_exec_count - s0;

        const double ms = (double)dt / 1000.0;
        const double fps = ms > 0.0 ? n * 1000.0 / ms : 0.0;

        printf("pm bench %s, bios %s\n", path, s_bios);
        printf("    %d frames in %.1f ms = %.1f fps unpaced (%.0f%% of 72)\n",
               n, ms, fps, fps / PM_FRAME_HZ * 100.0);
        printf("    %.0f instruct/sec, %.0f per frame, %.2f us each\n",
               steps / (ms / 1000.0), (double)steps / n,
               (double)steps ? (double)dt / steps : 0.0);

        PokeMini_Destroy();
        heap_caps_free(rom);
        return 0;
    }

    printf("usage: pm start [rom] | pm stop | pm status | pm dpad | pm pattern | pm bench [rom] [frames]\n");
    return 2;
}

static const tdsh_command_t s_pm_cmd = {
    "pm",
    "pm start [rom] | pm stop | pm status | pm dpad | pm pattern | pm bench [rom] [frames]",
    "Run a Pokemon Mini cartridge on the screen",
    cmd_pm,
    TDSH_CMD_INTERACTIVE,
};

bool pmini_register_command(void)
{
    if (!touch_add_app_touch(pm_app_touch, NULL))
        ESP_LOGW(TAG, "pm touch consumer did not register");

    return tdsh_register_command(&s_pm_cmd) == 0;
}
