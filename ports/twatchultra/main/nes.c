/*
 * nes.c - the NES emulator, on the watch, as a shell command.
 *
 * The emulator is components/nofrendo, vendored unmodified. This file is the
 * whole of what a board adds, and it is short because nofrendo asks for very
 * little: a frame buffer, a blit callback, an input callback and a vsync hook.
 *
 * THE FRAME BUFFER IS THE PORT'S, WHICH IS THE POINT. `nes_setvidbuf()` hands the
 * core a 272x240 byte-per-pixel PALETTE-INDEXED buffer, and the core writes into
 * it rather than allocating one. So the port decides where it lives, and it lives
 * in PSRAM: 65 KB cannot come from internal DRAM on this board, where 4 KB once
 * pushed the SSH server below its floor and it refused to start (EMU-018).
 *
 * THE CONVERSION IS THE PORT'S TOO. The core outputs palette indices; the panel
 * wants RGB565. That is a 256-entry table built once from nofrendo's own palette
 * data and applied with a straight LUT copy -- no arithmetic per pixel, and no
 * need to vendor retro-go's front end just for `nofrendo_buildpalette()`.
 */
#include "nes.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "display.h"
#include "edges.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "tdsh.h"
#include "hid_input.h"
#include "touch.h"

/* nofrendo's own headers. Included through the component's exported include
 * path, which is why the component lists "." and "nes" in INCLUDE_DIRS. */
#include "nes/nes.h"
#include "nes/apu.h"
#include "sound.h"
#include "palettes.h"

static const char *TAG = "nes";

/* --- state ---------------------------------------------------------------- */

#define NES_STACK       8192
/* The blit task needs less: it converts and pushes, and its deepest call is the
 * row callback. It gets its own constant rather than sharing, because a stack
 * sized for the 6502 interpreter would be 8 KB of internal DRAM held for a task
 * that only ever runs the palette loop. */
#define NES_BLIT_STACK  4096
#define NES_PRIO        3
/* The NES runs at 60.0988 Hz on NTSC. Pacing to the core's own refresh rate
 * rather than to a round 60 keeps audio pitch and game speed right -- the
 * difference is 0.16 percent, inaudible on its own, but it accumulates: a
 * minute of play would drift about a tenth of a second against anything else
 * keeping time. */
#define NES_FRAME_US    (1000000.0 / 60.0988)

static nes_t *s_nes;
/* Where the NES frame buffer comes from: see the allocation in nes_start().
 * 0 is the shipped arrangement (heap, internal if it fits, else PSRAM); 1 is the
 * internal-DRAM experiment, which requires a rebuild to change.
 *
 * MEASURED, 2026-11-12, Zelda title screen, 25-second runs:
 *
 *   0 (heap -> PSRAM)   42 fps   blit 8,647 us/frame   internal free 143,275
 *   1 (static internal) 45 fps   blit 8,070 us/frame   internal free  77,475
 *
 * So the whole prize is 3 fps for 65,800 bytes of internal DRAM held for the
 * life of the firmware -- and with the radio initialised that puts internal free
 * below the 65,536 tdsh's SSH server demands, so SSH stops starting. Not worth
 * it, which is why 0 is the default and this is kept only so the experiment is
 * not run twice.
 *
 * The number that made it not worth it is also the useful finding: moving the
 * frame buffer into SRAM saved only 577 us of an 8,647 us blit. The prediction
 * was several milliseconds, and it was wrong because the blit is dominated by
 * the SPI push and not by reading the buffer. */
#define NES_VIDBUF_STATIC 0

/*
 * THREE FRAME BUFFERS, SO THE BLIT CAN RUN WHILE THE NEXT FRAME IS EMULATED.
 *
 * The emulator and the blit used to run one after the other on one task: 15,242 us
 * of interpreter and PPU, then 8,409 us of conversion and SPI. Nothing about
 * either half is wasteful -- the SPI runs at 1.10x its arithmetic floor and the
 * conversion reads four source bytes per lookup -- so the only thing left to win
 * was running them at the same time, and both core 1 and the QSPI peripheral
 * were idle for 8 of every 23 ms.
 *
 * Three buffers, not two, because with two the renderer would have to wait for
 * the blitter to finish before starting the next frame, and there would be
 * nothing to overlap. Three lets the blitter hold one while the renderer fills
 * another and the third is free, so a slow blit costs latency rather than
 * throughput.
 *
 * The handoff uses two queues rather than flags: `s_free_q` holds the indices of
 * buffers nobody is using, `s_filled_q` the ones waiting to be pushed. That
 * ordering is the whole safety argument -- a buffer is only handed to the
 * blitter after a free one has been taken for the renderer, so the renderer can
 * never be writing the buffer the blitter is reading, even if it fails to get a
 * free one and has to skip a frame.
 */
#define NES_VBUF_N 3

static uint8_t *s_vb[NES_VBUF_N];   /* PSRAM, 272x240 palette indices each */
/* Which pool the buffers actually came from: true for internal DRAM, false for
 * PSRAM. Reported by `nes status`, because the two produce very different frame
 * times and a measurement is worthless if it does not say which it had. */
static bool s_vidbuf_internal;

static QueueHandle_t s_free_q;      /* indices: available to render          */
static QueueHandle_t s_filled_q;    /* indices: rendered, waiting to be sent */
static TaskHandle_t s_blit_task;
static volatile bool s_blit_run;

/* Counters for the handoff itself. A skip means the blitter was still busy when
 * the renderer finished a frame -- the pool ran dry, which on the glass is a
 * repeated frame rather than a delay. Visible rather than inferred. */
static volatile uint32_t s_blit_done, s_blit_skipped;
static TaskHandle_t s_task;
static volatile bool s_run;
static volatile bool s_paused;
static char s_rom_path[64] = NES_ROM_DEFAULT;

/* Counters, for `nes status` and the card report. A frame count alone cannot
 * distinguish "running well" from "running but every frame is a repeat", and
 * after the NES harness produced three false passes on the host (NES-001) the
 * rule here is that a number has to be able to be wrong. */
static uint32_t s_frames;
static uint32_t s_fps;             /* measured over the last second */
static uint32_t s_dropped;         /* frames the loop could not keep up with */

/* WHERE THE FRAME GOES. A single frames-per-second figure cannot say whether the
 * interpreter or the display path is the limit, and those have completely
 * different fixes -- the first is optimisation, the second is fewer pixels or a
 * cheaper conversion. So the two are timed separately and reported apart. Cost
 * is two esp_timer reads per frame, which is 2 us against a 16,667 us budget. */
static volatile uint32_t s_us_emu;     /* last second: time inside nes_emulate */
static volatile uint32_t s_us_blit;    /* last second: time inside the blit     */

/* --- the palette ---------------------------------------------------------- */

/*
 * The 256-entry RGB565 table.
 *
 * nofrendo's palette data is RGB888 in `nes_palettes[][]` and its own builder
 * lives in `nofrendo.c`, the retro-go front end that is deliberately NOT
 * vendored. Rebuilding the table here is four lines and keeps the component
 * free of that dependency.
 *
 * THE LAYOUT IS THE CORE'S, NOT A CHOICE: entries 0..63 are the NES palette,
 * repeated three times (192..255 are the GUI colours) so that sprite priority
 * and background transparency can be done by flipping bits in the index the
 * PPU produces. A port that built only 64 entries would find the top two thirds
 * of its sprites rendered in the wrong colours, and it would look like a
 * palette bug rather than an indexing one.
 */
static uint16_t s_pal[256];

/* The same 256 colours with each pixel's bytes exchanged into the order the
 * panel wants on the wire (display.h: display_swap16). It exists so the blit
 * can emit panel-ready values straight from a table lookup: the interchange is
 * then done 256 times at start instead of 61,440 times per frame, and -- more
 * to the point -- it lets the row callback write into the transmit band
 * directly, which is what removes the full-frame copy through PSRAM. */
static uint16_t s_pal_tx[256];

/* The blit's own duration, picked up by the task loop after nes_emulate returns.
 * A file-scope variable rather than a return value because the core calls the
 * blit through a function pointer. */
static volatile uint32_t s_us_blit_last;

/*
 * THE CONTENTION DIAGNOSTIC. `nes start <rom> noblit` runs the emulator with no
 * blit task at all.
 *
 * The question it answers is whether the blitter is costing the interpreter
 * anything. The blit runs on core 1 and is therefore overlapped, so its own
 * duration should not appear in the frame time -- but both cores read PSRAM, and
 * the frame buffer, the paletted source and the converted band all live there.
 * If suppressing the blit makes the INTERPRETER faster, the blit is stealing
 * memory bandwidth even though it never shares a core; if the interpreter is
 * unchanged, the blit is free and optimising it is pointless.
 *
 * Same shape and same reason as `pm start <rom> noblit` in pm.c.
 */
static bool s_nobl_diag;

/* --- audio -----------------------------------------------------------------
 *
 * THE APU WAS ALWAYS SYNTHESISING; NOTHING WAS COLLECTING ITS OUTPUT.
 * `nes_emulate()` ends with `apu_emulate()`, which calls
 * `apu_process(apu.buffer, samples_per_frame, stereo)` and fills the APU's own
 * buffer once per frame. The port has been doing that since its first boot.
 * What was missing was only the step that hands those samples to a DAC.
 *
 * THE FIRST VERSION OF THIS CALLED `apu_process()` AGAIN, AND THAT IS THE BUG
 * THAT WAS HEARD RATHER THAN SEEN. `apu_process()` does not merely copy: it
 * advances every channel -- volume decay, length counters, envelope phase,
 * sweep. Calling it a second time per frame ran the whole sound engine at
 * twice its intended rate against a buffer nobody read, and the result was
 * reported by ear as "scratchy and tiny" before any counter showed it. Nothing
 * in `nes status` could have caught it: frames were clean, the blit was clean,
 * and no sample was dropped.
 *
 * SO THE SAMPLES ARE TAKEN, NOT MADE. `nes_getptr()` returns the machine
 * struct, which carries `apu_t *apu`, which carries the buffer and its size --
 * none of which needs an edit to the vendored core, unlike `apu_getcontext()`,
 * which is a stub that prints "Not implemented". One channel is copied out
 * because `sound_stream_write()` duplicates a mono sample into both slots.
 *
 * THE 0.16 PERCENT PITCH ERROR IS INHERITED AND DELIBERATE. nofrendo computes
 * `samples_per_frame = sample_rate / refresh_rate` with refresh_rate as the
 * integer 60, giving 735, while the port paces frames at 60.0988 Hz where the
 * true figure is 733.79. Playing all 735 at 44.1 kHz makes the music 2.8 cents
 * sharp -- about a fortieth of a semitone. The alternative is dropping 1.2
 * samples a frame, which is an audible click every frame, so the pitch error
 * is the better trade by a wide margin. Revisit only if the DECLARED rate is
 * changed to match: 735 samples at 60.0988 Hz is 44,172 Hz.
 */
static bool s_audio;
static int  s_aud_samples;      /* samples the APU produces per frame */
static bool s_aud_stereo;       /* how apu_process was told to emit */
static const int16_t *s_aud_buf;/* the APU's own output buffer, once found */
/* THE RATE WE ACTUALLY PRODUCE, AND WHAT IT DID AND DID NOT FIX.
 *
 * nofrendo computes `samples_per_frame = sample_rate / refresh_rate` with
 * refresh_rate as the integer 60, giving 735. The port paces frames at
 * 60.0988 Hz, so the true output rate is 735 x 60.0988 = 44,172.6 Hz. Telling
 * the DAC 44,100 made us produce 0.165 percent faster than it drained.
 *
 * THAT WAS NOT THE CAUSE OF THE DROPS, AND THE MEASUREMENT SAID SO. The drop
 * count read 1,341 at 44,100 Hz and 1,341 again at 44,172 -- identical, so the
 * hypothesis was wrong. What the log then showed is that every drop happens at
 * frame 0 and frame 3 and never again: 735 refused, then 159, then silence for
 * the rest of the run. That is the DMA ring not being ready at the instant the
 * channel is opened, not a rate mismatch, and it is why the operator heard
 * "the same underruns in the same place" -- the same place was the start.
 *
 * 44,172 IS STILL THE RIGHT NUMBER and is kept for a reason that survives:
 * it is the correct pitch. The NES really does run at 60.0988 Hz, so playing
 * its 735 samples at 44.1 kHz put the music 2.8 cents sharp. Matching the
 * declared rate to production also means no drift accumulates over a long
 * session, which is worth having whether or not it fixed anything audible.
 *
 * Files are the fact: /sd/nes_audio.log records every refused write with its
 * frame number, because a total in `nes status` cannot distinguish a startup
 * transient from a recurring stall and the two want opposite fixes. */
#define NES_AUDIO_HZ   44172
#define NES_AUD_MAX    736

static int16_t s_pcm[NES_AUD_MAX];

/*
 * CUMULATIVE counters, and the last-second figures above are not enough.
 *
 * This cycle wasted a measurement on exactly that: the conversion was changed
 * and the "last second" blit figure moved from 9,218 to 9,731 us, which reads as
 * a 5 percent regression but is a one-second sample of a loop whose frame time
 * wanders. A per-run average over ~900 frames is the number to compare, so the
 * totals are kept as well as the sample.
 */
static volatile uint64_t s_blit_us_total;
/* Push counters that predate the frame counter, so the first frame's average is
 * not skewed by the margins being painted before nes_start() zeroed them. */
static int64_t s_push_baseline_us;
static volatile uint32_t s_blit_frames_total;

static void build_palette(void)
{
    const uint8_t *rgb = nes_palettes[0];   /* NES_PALETTE_NOFRENDO */

    for (int i = 0; i < 64; i++)
    {
        uint8_t r = rgb[i * 3 + 0];
        uint8_t g = rgb[i * 3 + 1];
        uint8_t b = rgb[i * 3 + 2];

        /* RGB565, and green gets the extra bit because it has the extra bit. */
        uint16_t c = (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));

        s_pal[i] = s_pal[i + 64] = s_pal[i + 128] = c;
    }

    /* The eight GUI colours, which the core uses for its own overlays. */
    for (int i = 0; i < 8; i++)
    {
        uint8_t r = gui_pal[i * 3 + 0];
        uint8_t g = gui_pal[i * 3 + 1];
        uint8_t b = gui_pal[i * 3 + 2];

        s_pal[192 + i] = (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
    }

    for (int i = 0; i < 256; i++)
        s_pal_tx[i] = display_swap16(s_pal[i]);
}

/*
 * Give the frame buffer back, if it was ever the heap's to give.
 *
 * With NES_VIDBUF_STATIC the buffer is a linker-placed array and freeing it
 * would be a heap corruption bug rather than a no-op, so the release goes
 * through here and the static case does nothing. Every teardown path calls this
 * instead of heap_caps_free() so that turning the experiment on or off cannot
 * leave a stray free behind.
 */
static void vidbuf_release(void)
{
    for (int i = 0; i < NES_VBUF_N; i++)
    {
        if (s_vb[i])
            heap_caps_free(s_vb[i]);

        s_vb[i] = NULL;
    }
}

/* --- the blit ------------------------------------------------------------- */

/*
 * Called once per frame by the core with the finished 272x240 index buffer.
 *
 * THE OVERDRAW IS SKIPPED, not presented. `NES_SCREEN_PITCH` is 8 + 256 + 8
 * because the PPU writes eight pixels of slop on each side for a sprite that
 * starts there; showing that would put the game eight pixels off centre. So the
 * row fetch starts `NES_SCREEN_OVERDRAW` into each row and only 256 pixels are
 * converted.
 *
 * IT NO LONGER BUILDS A FRAME. This used to convert all 61,440 pixels into a
 * 120 KB PSRAM scratch buffer and hand that to display_picture_rgb(), which
 * copied it into the 260 KB PSRAM framebuffer and immediately copied it back out
 * into the transmit band. Measured on the watch at 19 fps, the blit was 705,665
 * us of CPU per second -- 37,140 us of a 52,980 us frame, 70 percent of it --
 * for an image that lives in 61,440 bytes before conversion. Three passes over
 * external memory to reach 5120-byte band of internal RAM.
 *
 * display_picture_rows() inverts the arrangement: it drives the band loop and
 * asks for one row at a time, so the palette lookup writes straight into the
 * transmit band. One PSRAM read per pixel, none written, and the scratch buffer
 * is gone entirely.
 *
 * The band size that call chooses is 16 rows, which is BAND_ROWS and a property
 * of this panel -- a taller window tears (LCD-009). So a frame is 15 window
 * setups either way; what changed is the memory traffic, not the protocol.
 */
static void nes_fill_row(void *ctx, int row, int col, uint16_t *dst, int n)
{
    const uint8_t *buf = (const uint8_t *)ctx;

    if (!buf || row < 0 || row >= NES_SCREEN_HEIGHT || col < 0)
        return;
    if (col + n > NES_SCREEN_WIDTH)
        n = NES_SCREEN_WIDTH - col;
    if (n <= 0)
        return;

    const uint8_t *src = NES_SCREEN_GETPTR(buf, col, row);

    /* s_pal_tx, not s_pal: the destination is the panel's transmit band, and the
     * values are already in the order the wire wants.
     *
     * FOUR SOURCE BYTES PER LOAD. The plain byte-at-a-time loop measured 5,105 us
     * of a 25,196 us frame -- 20 cycles per pixel, which is far more than a table
     * lookup and a store should cost, so the byte loads from the PSRAM frame
     * buffer were the cost rather than the lookups. Reading a uint32 gives four
     * indices per load and takes the source traffic down by four; the shifts and
     * masks are register work.
     *
     * Alignment: NES_SCREEN_PITCH is 272, a multiple of 4, and `col` is 0 for
     * every call this port makes, so a row start is 4-aligned whenever the buffer
     * is -- which it is, being a heap allocation. The check is here anyway
     * because a future caller with an odd column would otherwise fault on an
     * unaligned read, and falling back to the byte loop for that case is cheaper
     * than proving it can never happen. */
    int x = 0;

    if (((uintptr_t)src & 3) == 0)
    {
        const uint32_t *s32 = (const uint32_t *)src;

        for (; x + 4 <= n; x += 4)
        {
            uint32_t v = s32[x >> 2];

            dst[x + 0] = s_pal_tx[v & 0xFF];
            dst[x + 1] = s_pal_tx[(v >> 8) & 0xFF];
            dst[x + 2] = s_pal_tx[(v >> 16) & 0xFF];
            dst[x + 3] = s_pal_tx[v >> 24];
        }
    }

    for (; x < n; x++)
        dst[x] = s_pal_tx[src[x]];
}

/* Convert one rendered frame into the transmit band and push it.
 *
 * This runs on the blit task, on core 1, while the emulator is already working
 * on the next frame. Everything it touches is either its own (s_tx_app inside
 * the display driver) or a buffer this task currently owns. */
static void blit_one(const uint8_t *buf)
{
    int64_t b0 = esp_timer_get_time();

    /* `buf` is the buffer the pool handed over, so it is the context the row
     * callback reads. */
    if (!display_picture_rows((void *)buf, NES_OX, NES_OY,
                              NES_SCREEN_WIDTH, NES_SCREEN_HEIGHT, nes_fill_row))
    {
        /* Ownership not claimed, or no framebuffer. Nothing was drawn, and the
         * only reason to be here is a start-up ordering fault -- say so rather
         * than leaving a black screen with a healthy frame counter. */
        static bool warned;
        if (!warned)
        {
            warned = true;
            printf("blit refused: pixels not claimed\n");
        }
    }

    s_us_blit_last = (uint32_t)(esp_timer_get_time() - b0);
    s_blit_us_total += s_us_blit_last;
    s_blit_frames_total++;
}

/*
 * The blit task: core 1, one job -- take a finished frame and send it.
 *
 * It never touches the emulator's state. The only shared things are the two
 * queues and the buffers themselves, and ownership moves through them.
 */
static void nes_blit_task(void *arg)
{
    (void)arg;

    while (s_blit_run)
    {
        int idx;

        if (xQueueReceive(s_filled_q, &idx, pdMS_TO_TICKS(50)) != pdTRUE)
            continue;

        blit_one(s_vb[idx]);
        s_blit_done++;

        /* The renderer may be waiting for a buffer; give this one back even if
         * the loop is about to end, so a stop cannot strand it. */
        xQueueSend(s_free_q, &idx, 0);
    }

    s_blit_task = NULL;
    vTaskDelete(NULL);
}

/* Which pool index a pointer belongs to, or -1. */
static int vb_index(const void *p)
{
    for (int i = 0; i < NES_VBUF_N; i++)
        if (s_vb[i] == p)
            return i;

    return -1;
}

/*
 * The blit callback, called by the core at the end of every emulated frame on
 * the emulator task.
 *
 * It does no drawing. It hands the finished frame to the blit task and swaps the
 * renderer onto a free buffer, which is O(1) work and is what makes the overlap
 * possible: the emulator returns to the 6502 immediately instead of waiting 8.4
 * ms for the SPI.
 *
 * THE ORDER OF THESE TWO STEPS IS THE CORRECTNESS ARGUMENT. A free buffer is
 * taken BEFORE the finished one is published, so if the pool is dry the frame is
 * not queued and the renderer keeps writing the buffer it already has. Done the
 * other way round, a dry pool would publish a buffer and then keep rendering
 * into it, and the blitter would push a half-written frame -- tearing that
 * appears only under load and would look like a panel fault.
 */
static void nes_blit(uint8_t *buf)
{
    int cur = vb_index(buf);
    int nxt;

    if (cur < 0)
        return;

    /* No blitter to hand to. Returning keeps `cur`, so the renderer carries on
     * into the same buffer and the measurement is of the interpreter alone. */
    if (s_nobl_diag)
        return;

    if (xQueueReceive(s_free_q, &nxt, 0) != pdTRUE)
    {
        /* No free buffer: the blitter is still busy. Skip the handoff and carry
         * on rendering into `cur`, which is still ours. */
        s_blit_skipped++;
        return;
    }

    if (xQueueSend(s_filled_q, &cur, 0) != pdTRUE)
    {
        s_blit_skipped++;
        xQueueSend(s_free_q, &nxt, 0);
        return;
    }

    nes_setvidbuf(s_vb[nxt]);
}

/*
 * The core's vsync callback is deliberately not installed: it is called at the
 * end of every emulated frame, and pacing is done in the task loop instead so
 * that the measurement and the sleep are in one place. Setting `.vsync` to NULL
 * is the correct signal for "the caller paces"; a no-op function would be
 * indistinguishable from one that meant to do something and did not.
 */

/* --- input ---------------------------------------------------------------- */

/*
 * The margin buttons, as an NES pad.
 *
 * The layout follows the Tamagotchi's: the three bottom bars are the NES's three
 * most-reached-for buttons and the right column's arrows are the D-pad, which
 * leaves the left column for the emulator's own controls (pause, reset, save).
 *
 *   strip 0 -> B      strip 1 -> A     strip 2 -> START
 *   right 1 -> LEFT   right 2 -> RIGHT right 3 -> UP    right 4 -> DOWN
 *
 * SELECT has no button yet, which is a real gap: a few games use it to choose a
 * mode and cannot be played properly without it. Recorded rather than papered
 * over -- the left column has three free slots.
 */
static int pad_from_frame(const touch_frame_t *f)
{
    int pad = 0;

    if (!f)
        return 0;

    for (int i = 0; i < f->n && i < TOUCH_POINTS; i++)
    {
        if (f->p[i].region == EDGE_BOTTOM)
        {
            if (f->p[i].index == 0) pad |= NES_PAD_B;
            if (f->p[i].index == 1) pad |= NES_PAD_A;
            if (f->p[i].index == 2) pad |= NES_PAD_START;
        }
        else if (f->p[i].region == EDGE_RIGHT)
        {
            if (f->p[i].index == 1) pad |= NES_PAD_LEFT;
            if (f->p[i].index == 2) pad |= NES_PAD_RIGHT;
            if (f->p[i].index == 3) pad |= NES_PAD_UP;
            if (f->p[i].index == 4) pad |= NES_PAD_DOWN;
        }
    }

    return pad;
}

/*
 * THE PAD IS SHARED WITH THE EMULATION TASK AND THAT IS WHY IT IS A PLAIN INT.
 *
 * touch.c runs on core 0 at priority 3; the emulation task is unpinned. Reading
 * and writing an aligned int is atomic on this core, so a press that lands
 * between two frames is either seen whole or not at all -- never half. A struct
 * or a multi-field update would need a lock for the same guarantee, and the
 * Tamagotchi learned this the hard way: its button state went through a queue
 * because a press applied from the wrong task raced the emulated CPU (EMU-019).
 * Here the only shared thing is one integer that the emulation task reads, which
 * is why a queue is not needed and why this must stay one integer.
 */
static volatile int s_pad;

/*
 * A REAL CONTROLLER BEATS THE MARGINS, and both are read.
 *
 * The margin buttons work and are the fallback, but a gamepad is what a game
 * wants: a D-pad that is a D-pad, and enough buttons for START and SELECT.
 *
 * THE BIT NUMBERS ARE MEASURED, NOT ASSUMED. tests/xbox_btle.h holds the
 * descriptor and the raw reports captured from the operator's Xbox Wireless
 * Controller while each button was pressed in a stated order, and
 * tests/test_gamepad.c pins what those reports decode to. What is known: the
 * presses landed on button bits 0, 1, 3 and 4; the hat reports 1..8 clockwise
 * from up; and two more buttons sit at bits 10 and 11.
 *
 * WHICH FACE BUTTON IS WHICH IS BY PRESS ORDER, so this mapping is only as good
 * as that order: bits 0..3 are taken to be A, B, X and Y in the sequence the
 * operator pressed them. If a game disagrees, the fix is this table and nothing
 * else -- `nes pad` prints the live mask so the correction can be observed
 * rather than guessed.
 */
#define GP_BIT_A      0
#define GP_BIT_B      1
#define GP_BIT_X      3
#define GP_BIT_Y      4
#define GP_BIT_VIEW   10
#define GP_BIT_MENU   11

static int pad_from_gamepad(void)
{
    hid_gamepad_mask_t m = hid_gamepad_state();
    uint8_t hat = hid_gamepad_hat();
    int pad = 0;

    if (m & (1u << GP_BIT_A)) pad |= NES_PAD_A;
    if (m & (1u << GP_BIT_B)) pad |= NES_PAD_B;
    if (m & (1u << GP_BIT_VIEW)) pad |= NES_PAD_SELECT;
    if (m & (1u << GP_BIT_MENU)) pad |= NES_PAD_START;

    /* 0..7 clockwise from up, 0xFF for centred. Diagonals are deliberately not
     * offered: a 4-bit hat switch cannot report one, so every other value is
     * either a cardinal direction or centred. */
    if (hat == 0) pad |= NES_PAD_UP;
    else if (hat == 2) pad |= NES_PAD_RIGHT;
    else if (hat == 4) pad |= NES_PAD_DOWN;
    else if (hat == 6) pad |= NES_PAD_LEFT;

    return pad;
}

static void nes_app_touch(void *user, const touch_frame_t *f)
{
    (void)user;

    if (!s_run)
        return;

    s_pad = pad_from_frame(f);
}

/* --- the emulation task --------------------------------------------------- */

static void nes_task(void *arg)
{
    (void)arg;
    int64_t next = esp_timer_get_time();
    int64_t fps_deadline = next + 1000000;
    uint32_t fps_frames = 0;
    uint32_t acc_emu = 0, acc_blit = 0;

    while (s_run)
    {
        if (s_paused)
        {
            /* Nothing stepped, nothing drawn, but the loop stays alive so a
             * resume is noticed and the pad keeps being read. */
            vTaskDelay(pdMS_TO_TICKS(16));
            next = esp_timer_get_time();
            fps_deadline = next + 1000000;
            fps_frames = 0;
            continue;
        }

        /* The pad goes in BEFORE the frame is stepped, or a press is one frame
         * late -- 16 ms, which a player feels on a jump. */
        /* A connected gamepad wins; the margins are the fallback. Both are
         * plain integers, so reading them here is atomic. */
        int pad = pad_from_gamepad();

        input_update(0, pad ? pad : s_pad);

        int64_t t0 = esp_timer_get_time();


        nes_emulate(true);

        /* Synthesise this frame's samples and hand them to the DAC. The write
         * never blocks -- see sound_stream_write(): a full ring drops samples
         * rather than stalling the frame loop, which matters here because the
         * emulator has about 4.5 ms of slack and one frame of audio is 13.9 ms
         * of real time. */
        if (s_audio && s_aud_buf)
        {
            const int n = s_aud_samples > NES_AUD_MAX ? NES_AUD_MAX : s_aud_samples;

            /* TAKE ONE CHANNEL of what the APU already produced. apu_emulate()
             * runs at the end of nes_emulate() and fills the APU's own buffer
             * with s_aud_samples stereo frames; sound_stream_write() duplicates
             * a mono sample into both slots, so half of it has to go. */
            if (s_aud_stereo)
                for (int i = 0; i < n; i++)
                    s_pcm[i] = s_aud_buf[2 * i];
            else
                for (int i = 0; i < n; i++)
                    s_pcm[i] = s_aud_buf[i];

            const uint32_t before = sound_stream_dropped();
            sound_stream_write(s_pcm, n);

            /* WHEN, not how many. A total says the ring refused something; the
             * frame number says whether that was a startup transient or a
             * moment in the game, and the two want opposite fixes. */
            const uint32_t after = sound_stream_dropped();
            if (after != before)
            {
                /* TO A FILE, because printf scrolls off a 26-row terminal long
                 * before anyone can read it and the interesting question is the
                 * frame number, not the total. */
                FILE *f = fopen("/sd/nes_audio.log", "a");
                if (f)
                {
                    fprintf(f, "frame %lu dropped %lu of %d\n",
                            (unsigned long)s_frames,
                            (unsigned long)(after - before), n);
                    fclose(f);
                }
            }
        }

        s_frames++;
        fps_frames++;
        acc_emu += (uint32_t)(esp_timer_get_time() - t0);
        acc_blit += s_us_blit_last;

        /* How long the frame took, so `nes status` can say whether the board is
         * keeping up rather than merely still running. */
        int64_t spent = esp_timer_get_time() - t0;

        next += (int64_t)NES_FRAME_US;

        int64_t now = esp_timer_get_time();

        if (now < next)
        {
            /* Tick is 1000 Hz, so round up: a 0 ms delay would spin. */
            vTaskDelay(pdMS_TO_TICKS((next - now + 999) / 1000));
        }
        else
        {
            /* Behind. Skip the backlog rather than running frames back to back:
             * a burst moves the game further ahead of the wall clock, not less,
             * and the visible result is a stutter followed by a speed-up. */
            if (now - next > (int64_t)NES_FRAME_US)
                s_dropped++;

            next = now;
        }

        if (now >= fps_deadline)
        {
            s_fps = fps_frames;
            s_us_emu = acc_emu;
            s_us_blit = acc_blit;
            fps_frames = 0;
            acc_emu = acc_blit = 0;
            fps_deadline = now + 1000000;
        }

        (void)spent;
    }

    s_task = NULL;
    vTaskDelete(NULL);
}

/* --- start / stop --------------------------------------------------------- */

bool nes_start(const char *rom_path)
{
    if (s_task)
    {
        printf("already running\n");
        return false;
    }

    if (rom_path && *rom_path)
    {
        strncpy(s_rom_path, rom_path, sizeof(s_rom_path) - 1);
        s_rom_path[sizeof(s_rom_path) - 1] = '\0';
    }

    if (!s_nes)
    {
        s_nes = nes_init(SYS_NES_NTSC, 44100, true, NULL);

        if (!s_nes)
        {
            printf("nes_init failed\n");
            return false;
        }

        s_nes->blit_func = nes_blit;
    }

    /*
     * THE FRAME BUFFERS, FROM PSRAM -- all three of them.
     *
     * 272x240 is 65,280 bytes each, so the pool is 195,840 bytes. They are
     * touched twice per pixel per frame -- the PPU writes a buffer, the blitter
     * reads one -- so internal DRAM would be the faster home, and it was tried
     * on 2026-11-12: a single static internal buffer measured 45 fps against 42,
     * saving 577 us of an 8,409 us blit, because the blit is dominated by the SPI
     * push and not by reading the buffer. Three of them would be 195,840 bytes,
     * which this board does not have to spare and does not need to spend for
     * 577 us. So the experiment is not carried forward here: it is measured,
     * recorded in NES-003, and incompatible with the pool by size.
     *
     * Allocated at `nes` start and returned on stop, never at boot. That is the
     * refcount discipline the Tamagotchi's frame buffer follows, and it is what
     * keeps 195 KB from being a permanent claim on a board whose scarcest
     * resource is memory (EMU-018).
     */
    const size_t vidbytes = (size_t)NES_SCREEN_PITCH * NES_SCREEN_HEIGHT;

    for (int i = 0; i < NES_VBUF_N; i++)
    {
        s_vb[i] = (uint8_t *)heap_caps_malloc(vidbytes, MALLOC_CAP_SPIRAM);

        if (!s_vb[i])
        {
            printf("no memory for %d frame buffers\n", NES_VBUF_N);

            for (int j = 0; j < i; j++)
                heap_caps_free(s_vb[j]);

            for (int j = 0; j < NES_VBUF_N; j++)
                s_vb[j] = NULL;

            return false;
        }
    }

    s_vidbuf_internal = false;

    printf("vidbuf %u x %d = %u bytes, PSRAM (internal free=%u)\n",
           (unsigned)vidbytes, NES_VBUF_N, (unsigned)(vidbytes * NES_VBUF_N),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    s_nes->vidbuf = NULL;   /* in case a previous run left it set */

    if (nes_loadfile(s_rom_path) < 0)
    {
        printf("unable to load %s (unsupported mapper?)\n", s_rom_path);
        vidbuf_release();
        return false;
    }

    /* AFTER the load, per the note above: nos_loadfile() calls nes_reset(), which
     * nulls vidbuf, so a buffer installed beforehand is silently discarded and
     * the PPU renders nothing while the emulator reports a healthy frame rate. */
    nes_setvidbuf(s_vb[0]);

    build_palette();

    /* Claim the screen and clear it, exactly as the Tamagotchi does: claiming
     * stops the painter, clearing removes what the console left behind. */
    display_pixels_claim(true);

    if (!display_fill(0x0000))
    {
        display_pixels_claim(false);
        vidbuf_release();
        printf("no framebuffer\n");
        return false;
    }

    display_margins_set(DISPLAY_MARGINS_APP);

    /* The labels, in the app's own terms. The bottom strip is the NES pad rather
     * than the pet's three buttons, and the right column is its D-pad. */
    display_margin_label(0, 0, "Pause");
    display_margin_label(0, 1, "Reset");
    display_margin_label(0, 2, "Save");
    display_margin_label(0, 3, NULL);
    display_margin_label(0, 4, NULL);

    display_margin_label(1, 0, "Hold");
    display_margin_label(1, 1, "Left");
    display_margin_label(1, 2, "Right");
    display_margin_label(1, 3, "Up");
    display_margin_label(1, 4, "Down");

    s_frames = 0;
    s_dropped = 0;
    s_fps = 0;
    display_push_stats_reset();
    s_blit_us_total = 0;
    s_blit_frames_total = 0;

    {
        int64_t w0 = 0, x0 = 0;

        display_push_stats(&w0, &x0, NULL);
        s_push_baseline_us = w0 + x0;
    }
    s_pad = 0;
    s_paused = false;
    /* Audio last, so a failure here does not leave a half-started machine. Best
     * effort by design: a watch with no working amplifier still runs the game. */
    {
        nes_t *m = nes_getptr();
        apu_t *a = m ? m->apu : NULL;

        s_aud_buf = a ? (const int16_t *)a->buffer : NULL;
        s_aud_samples = a ? a->samples_per_frame : 0;
        s_aud_stereo = a ? a->stereo : false;

        if (!s_aud_buf || s_aud_samples <= 0)
        {
            printf("nes: APU buffer not available\n");
            s_aud_buf = NULL;
        }
    }

    {
        FILE *f = fopen("/sd/nes_audio.log", "w");
        if (f)
            fclose(f);
    }
    s_audio = sound_stream_open(NES_AUDIO_HZ);

    if (!s_audio)
        printf("nes: no audio (amplifier did not come up)\n");

    /* LET THE CHANNEL SETTLE BEFORE THE FIRST WRITE. Measured: without this the
     * very first frame loses all 735 of its samples and the third loses 159 --
     * about 20 ms of audio refused while the freshly enabled DMA ring is not
     * yet accepting, which is heard as a stutter at the start of every run and
     * was reported as "the same underruns in the same place" because the same
     * place was the beginning.
     *
     * The delay is paid once, at start, and the loop has not begun so nothing
     * is being produced during it. */
    if (s_audio)
        vTaskDelay(pdMS_TO_TICKS(100));

    s_run = true;
    s_blit_done = s_blit_skipped = 0;

    /* The pool. Two queues rather than flags: ownership of a buffer moves by
     * being popped and pushed, so there is no state two cores can disagree
     * about. Buffer 0 is the renderer's to start with, so 1 and 2 are free. */
    s_free_q = xQueueCreate(NES_VBUF_N, sizeof(int));
    s_filled_q = xQueueCreate(NES_VBUF_N, sizeof(int));

    if (!s_free_q || !s_filled_q)
    {
        printf("no queue\n");
        goto fail_queues;
    }

    for (int i = 1; i < NES_VBUF_N; i++)
        xQueueSend(s_free_q, &i, 0);

    s_blit_run = true;

    if (!s_nobl_diag && xTaskCreatePinnedToCore(nes_blit_task, "nesblit", NES_BLIT_STACK, NULL,
                                NES_PRIO, &s_blit_task, 1) != pdPASS)
    {
        s_blit_run = false;
        printf("no blit task\n");
        goto fail_queues;
    }

    /* Core 0 explicitly, so the two really are on different cores rather than
     * both landing wherever the scheduler felt like it. */
    if (xTaskCreatePinnedToCore(nes_task, "nes", NES_STACK, NULL,
                                NES_PRIO, &s_task, 0) != pdPASS)
    {
        s_run = false;
        s_task = NULL;
        s_blit_run = false;

        for (int i = 0; i < 100 && s_blit_task; i++)
            vTaskDelay(pdMS_TO_TICKS(10));

        display_margins_set(DISPLAY_MARGINS_DESKTOP);
        display_pixels_claim(false);
        printf("no task\n");
        goto fail_queues;
    }

    ESP_LOGI(TAG, "running %s", s_rom_path);
    return true;

fail_queues:
    /* Audio is opened before the queues, so this path can reach it. Closing it
     * here is not tidiness: an unclosed stream leaves the I2S channel
     * reconfigured to 44.1 kHz stereo, and the next `play` of a WAV would come
     * out at the wrong rate and the BEL would be pitched wrong. */
    if (s_audio)
    {
        sound_stream_close();
        s_audio = false;
    }
    s_run = false;

    if (s_free_q)
    {
        vQueueDelete(s_free_q);
        s_free_q = NULL;
    }
    if (s_filled_q)
    {
        vQueueDelete(s_filled_q);
        s_filled_q = NULL;
    }
    vidbuf_release();
    return false;
}

void nes_stop(void)
{
    if (!s_task)
        return;

    s_run = false;

    for (int i = 0; i < 100 && s_task; i++)
        vTaskDelay(pdMS_TO_TICKS(10));

    if (s_task)
    {
        ESP_LOGE(TAG, "task did not stop; not freeing");
        return;
    }

    /* The blit task before anything it touches. It exits within one queue
     * timeout (50 ms) once s_blit_run is clear, and it returns its buffer before
     * doing so. Waiting for it here, rather than assuming it noticed, is what
     * stops a stop from racing a push into a display that is being released
     * underneath it. */
    s_blit_run = false;

    for (int i = 0; i < 100 && s_blit_task; i++)
        vTaskDelay(pdMS_TO_TICKS(10));

    if (s_blit_task)
        ESP_LOGE(TAG, "blit task did not stop; not freeing");

    if (s_free_q)
    {
        vQueueDelete(s_free_q);
        s_free_q = NULL;
    }
    if (s_filled_q)
    {
        vQueueDelete(s_filled_q);
        s_filled_q = NULL;
    }

    /* After the task, so nothing can write into a stream that is being taken
     * away -- the same ordering argument as the blit task above. */
    if (s_audio)
    {
        sound_stream_close();
        s_audio = false;
    }

    nes_shutdown();
    s_nes = NULL;

    vidbuf_release();

    /* The converted-frame buffer is gone with the old blit: there is nothing to
     * keep between runs because there is nothing to convert into. That returns
     * 120 KB of PSRAM to the pool on every stop instead of holding it for the
     * life of the firmware. */

    display_margins_set(DISPLAY_MARGINS_DESKTOP);
    display_pixels_claim(false);
}

bool nes_running(void)
{
    return s_task != NULL;
}

void nes_pause(bool paused)
{
    s_paused = paused;
}

bool nes_paused(void)
{
    return s_paused;
}

/* --- the shell command ---------------------------------------------------- */

static int cmd_nes(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    const char *sub = argc >= 2 ? argv[1] : "start";

    if (argc >= 2 && (strcmp(sub, "stop") == 0 || strcmp(sub, "off") == 0))
    {
        if (!nes_running())
        {
            printf("not running\n");
            return 0;
        }
        nes_stop();
        printf("stopped, screen released\n");
        return 0;
    }

    if (argc >= 2 && strcmp(sub, "pause") == 0)
    {
        if (!s_nes)
        {
            printf("not running\n");
            return 1;
        }
        nes_pause(!s_paused);
        printf("%s\n", s_paused ? "paused" : "resumed");
        return 0;
    }

    if (argc >= 2 && strcmp(sub, "pad") == 0)
    {
        /*
         * THE CONTROLLER'S LIVE STATE, TO THE CARD.
         *
         * A command that printed to the terminal asked the operator to press
         * buttons DURING a three-second window that opened when the command was
         * typed -- which cannot be coordinated, because the person has to read
         * the instruction first. Two attempts produced zero captures that way.
         * Writing to a file instead means the command can be started and the
         * presses made whenever, and it is the same shape as `touchdump`, which
         * settled the multi-touch question for the same reason.
         *
         * 30 seconds is long enough to press everything and short enough that
         * the file cannot grow without bound.
         */
        FILE *pf = fopen("/sd/nespad.txt", "w");

        if (!pf)
        {
            printf("no card\n");
            return 1;
        }

        printf("sampling 30 s -> /sd/nespad.txt\n");

        int last = -1;

        for (int i = 0; i < 300; i++)
        {
            int gp = pad_from_gamepad();

            if (gp != last)
            {
                fprintf(pf, "gamepad=0x%04x NES=0x%02x hat=%u margins=0x%02x\n",
                        (unsigned)hid_gamepad_state(), (unsigned)gp,
                        (unsigned)hid_gamepad_hat(), (unsigned)s_pad);
                fflush(pf);
                last = gp;
            }

            vTaskDelay(pdMS_TO_TICKS(100));
        }

        fclose(pf);
        printf("done\n");
        return 0;
    }

    if (argc >= 2 && strcmp(sub, "status") == 0)
    {
        printf("running=%d frames=%lu fps=%lu dropped=%lu\n",
               nes_running() ? 1 : 0,
               (unsigned long)s_frames, (unsigned long)s_fps,
               (unsigned long)s_dropped);

        /* Split so the bottleneck is named rather than guessed at. */
        printf("emu=%luus blit=%luus of 16667\n",
               (unsigned long)s_us_emu, (unsigned long)s_us_blit);

        if (s_nes)
            printf("rom=%s paused=%d\n", s_rom_path, s_paused ? 1 : 0);

        /* Whether the amp came up and whether samples are being dropped. A
         * climbing drop count means the ring is the wrong shape, not that the
         * sound is broken. */
        if (s_audio)
            printf("audio %d Hz mono, %lu drops\n", NES_AUDIO_HZ,
                   (unsigned long)sound_stream_dropped());
        else
            printf("audio off\n");

        printf("vidbuf=%s%s blits=%lu skipped=%lu\n",
               s_vidbuf_internal ? "INTERNAL" : "PSRAM",
               s_nobl_diag ? " noblit" : "",
               (unsigned long)s_blit_done, (unsigned long)s_blit_skipped);

        /* Push breakdown, for deciding whether the blit's remaining 8 ms is the
         * transfer or the window programming around it. */
        {
            int64_t w_us = 0, x_us = 0;
            uint32_t pushes = 0;

            display_push_stats(&w_us, &x_us, &pushes);
            printf("push: bands=%lu window=%lldus xmit=%lldus\n",
                   (unsigned long)pushes, (long long)w_us, (long long)x_us);

            if (s_blit_frames_total)
            {
                int64_t push_us = w_us + x_us;
                int64_t blit_avg = (int64_t)(s_blit_us_total / s_blit_frames_total);
                int64_t conv_avg = (int64_t)(display_convert_us() /
                                             (int64_t)s_blit_frames_total);
                int64_t push_avg = (int64_t)((push_us - s_push_baseline_us) /
                                             (int64_t)s_blit_frames_total);

                /* `convert` here is MEASURED, from display_convert_us(); the
                 * subtraction that used to stand in for it is kept beside it so
                 * the two can be compared and the difference noticed rather than
                 * explained away. */
                printf("per-frame: blit=%lldus convert=%lldus push=%lldus "
                       "(subtract=%lldus) over %lu frames\n",
                       (long long)blit_avg, (long long)conv_avg,
                       (long long)push_avg, (long long)(blit_avg - push_avg),
                       (unsigned long)s_blit_frames_total);
            }
        }

        return 0;
    }

    const char *rom = NULL;

    /*
     * nes abench [frames] -- what audio would cost.
     *
     * THE QUESTION THIS ANSWERS BEFORE ANY AUDIO IS WRITTEN. The APU is already
     * initialised (nes_init passes 44100 and stereo) and its channel state
     * already advances, so the register side is inside the measured interpreter
     * cost. What is NOT paid is apu_process(), which synthesises five channels
     * per sample -- and nothing in this port calls it, so the whole of audio is
     * that function plus an I2S write.
     *
     * Guessing at it would be the same mistake as the -Og discovery: the number
     * is either small enough to add without touching anything else, or large
     * enough that the frame budget has to be found first, and the two lead to
     * completely different plans. So it is timed.
     *
     * Runs the real apu_process over a real APU state after a few hundred
     * emulated frames, so the channels are producing signal rather than sitting
     * at their reset values.
     */
    if (strcmp(sub, "abench") == 0)
    {
        const int rounds = argc >= 3 ? atoi(argv[2]) : 200;

        if (rounds < 1)
            return 2;
        if (!s_nes)
        {
            printf("abench: start a ROM first (the APU needs a machine)\n");
            return 1;
        }

        /* 44,100 / 60.0988 = 733.7, so 733 whole samples a frame. */
        const int spf = 734;
        static int16_t pcm[734 * 2];

        int64_t t0 = esp_timer_get_time();
        for (int i = 0; i < rounds; i++)
            apu_process((short *)pcm, (size_t)spf, true);
        const int64_t us = esp_timer_get_time() - t0;

        int64_t sum = 0;
        for (int i = 0; i < spf * 2; i++)
            sum += pcm[i];

        printf("nes abench %d frames x %d stereo samples\n", rounds, spf);
        printf("    apu_process %lld us total, %.3f ms/frame\n",
               (long long)us, (double)us / rounds / 1000.0);
        printf("    %.2f us/sample, checksum %lld\n",
               (double)us / ((double)rounds * spf), (long long)sum);
        return 0;
    }

    /*
     * nes convbench [frames] -- how much of the conversion is memory.
     *
     * THE QUESTION. Conversion is 60 percent of the blit (5,826 us of 9,728)
     * and the source buffer lives in PSRAM. Two explanations fit that number:
     * the loop is doing too many instructions, or it is waiting on external
     * memory. They call for opposite fixes, and a guess already went wrong once
     * this cycle -- a software-prefetch rewrite measured 6,897 us against the
     * 5,826 it replaced, so the prefetch was adding work rather than hiding
     * latency.
     *
     * This runs the SAME row callback over the same number of pixels twice,
     * once from the PSRAM frame buffer and once from an internal-RAM copy. If
     * internal is much faster the cost is memory and the fix is about traffic;
     * if the two are close the cost is instruction throughput and no amount of
     * buffering will help.
     *
     * It needs the palette built, so it calls build_palette() itself and works
     * whether or not the emulator is running.
     */
    if (strcmp(sub, "convbench") == 0)
    {
        const int rounds = argc >= 3 ? atoi(argv[2]) : 100;

        if (rounds < 1)
            return 2;

        build_palette();

        /*
         * THE INTERNAL BUFFER IS SIZED TO WHAT WILL ACTUALLY ALLOCATE. A full
         * frame is 65,280 bytes; 106,895 was free when this was written and the
         * allocation still failed, because free bytes are not contiguous bytes.
         * Both passes therefore run over however many rows fit, which keeps the
         * comparison honest -- it is the same work either way, which is the only
         * property the measurement needs.
         */
        int rows = NES_SCREEN_HEIGHT;
        uint8_t *isrc = NULL;

        while (rows >= 16 && !isrc)
        {
            isrc = (uint8_t *)heap_caps_malloc((size_t)rows * NES_SCREEN_PITCH,
                                               MALLOC_CAP_INTERNAL);
            if (!isrc)
                rows >>= 1;
        }

        if (!isrc)
        {
            printf("convbench: no internal buffer even at 16 rows\n");
            return 1;
        }

        const size_t bytes = (size_t)rows * NES_SCREEN_PITCH;
        uint8_t *psrc = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);

        if (!psrc)
        {
            printf("convbench: no psram buffer\n");
            heap_caps_free(isrc);
            return 1;
        }

        for (size_t i = 0; i < bytes; i++)
        {
            psrc[i] = (uint8_t)(i * 7);
            isrc[i] = (uint8_t)(i * 7);
        }

        /* A band-sized destination, so the stores are not all landing in the
         * same 512 bytes where the cache would flatter them.
         *
         * HEAP, INTERNAL, AND DELIBERATELY NOT PSRAM. This buffer stands in for
         * the real transmit band (s_tx_app in display.c), which is internal
         * because SPI DMA reads it every frame. Moving this one to PSRAM would
         * make the benchmark measure writes to a different memory than the
         * shipping path uses, and the number it reports would stop describing
         * the thing it exists to describe.
         *
         * What it may not do is stay a static: that held 8,192 bytes of internal
         * DRAM for the whole uptime on behalf of a command run a few times ever.
         * Heap-freed-internal gets both properties -- same memory as production,
         * none of the cost when the tool is not running. */
        uint16_t *band = (uint16_t *)heap_caps_malloc(16 * NES_SCREEN_WIDTH * sizeof(uint16_t),
                                                      MALLOC_CAP_INTERNAL);

        if (!band)
        {
            printf("convbench: no internal band buffer (%u bytes)\n",
                   (unsigned)(16 * NES_SCREEN_WIDTH * sizeof(uint16_t)));
            heap_caps_free(psrc);
            heap_caps_free(isrc);
            return 1;
        }

        /* Warm both so the first pass is not paying for cold cache lines that
         * the second never sees. */
        for (int y = 0; y < rows; y++)
        {
            nes_fill_row(psrc, y, 0, band, NES_SCREEN_WIDTH);
            nes_fill_row(isrc, y, 0, band, NES_SCREEN_WIDTH);
        }

        int64_t t0 = esp_timer_get_time();
        for (int r = 0; r < rounds; r++)
            for (int y = 0; y < rows; y++)
                nes_fill_row(psrc, y, 0, band, NES_SCREEN_WIDTH);
        const int64_t psram_us = esp_timer_get_time() - t0;

        t0 = esp_timer_get_time();
        for (int r = 0; r < rounds; r++)
            for (int y = 0; y < rows; y++)
                nes_fill_row(isrc, y, 0, band, NES_SCREEN_WIDTH);
        const int64_t int_us = esp_timer_get_time() - t0;

        const double px = (double)rounds * rows * NES_SCREEN_WIDTH;

        printf("convbench %d rounds x %d rows, %u bytes source\n", rounds, rows, (unsigned)bytes);
        printf("    psram  %lld us  %.3f ms/frame  %.2f cycles/px\n",
               (long long)psram_us, (double)psram_us / rounds / 1000.0,
               (double)psram_us * 240.0 / px);
        printf("    inter  %lld us  %.3f ms/frame  %.2f cycles/px\n",
               (long long)int_us, (double)int_us / rounds / 1000.0,
               (double)int_us * 240.0 / px);
        printf("    memory is %s (%.1f%% of the psram case)\n",
               int_us * 2 < psram_us ? "MOST of the cost" : "not the dominant cost",
               100.0 * (double)(psram_us - int_us) / (double)(psram_us ? psram_us : 1));

        heap_caps_free(psrc);
        heap_caps_free(isrc);
        heap_caps_free(band);
        return 0;
    }

    if (argc >= 2 && strcmp(sub, "start") == 0)
    {
        rom = argc >= 3 ? argv[2] : NULL;
        s_nobl_diag = (argc >= 4 && strcmp(argv[3], "noblit") == 0);
    }
    else if (sub[0] == '/')
        rom = sub;

    if (!nes_start(rom))
        return 1;

    printf("nes running %s\n", s_rom_path);
    printf("nes stop to leave\n");
    return 0;
}

static const tdsh_command_t s_nes_cmd = {
    "nes",
    "nes [start [rom] [noblit] | stop | pause | status | pad | convbench | abench]",
    "Run the NES emulator on the screen",
    cmd_nes,
    TDSH_CMD_INTERACTIVE,
};

bool nes_register_command(void)
{
    /* The margins belong to whichever app owns the screen, and only one app can
     * own it, so the sink is registered once and both apps decline when they are
     * not running. */
    if (!touch_add_app_touch(nes_app_touch, NULL))
        ESP_LOGW(TAG, "touch consumer table full");

    return tdsh_register_command(&s_nes_cmd) == 0;
}
