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
    display_margins_set(DISPLAY_MARGINS_APP);

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

    printf("usage: pm start [rom] | pm stop | pm status | pm pattern | pm bench [rom] [frames]\n");
    return 2;
}

static const tdsh_command_t s_pm_cmd = {
    "pm",
    "pm start [rom] | pm stop | pm status | pm pattern | pm bench [rom] [frames]",
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
