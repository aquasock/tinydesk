/*
 * gb.c - Game Boy / Game Boy Color on the watch, on the core-1 blit.
 *
 * The shape is pm.c's, deliberately: one task steps the guest, a second task on
 * core 1 pushes the finished frame, and the display driver is untouched. That
 * machinery was built for the NES and reused for the Pokemon Mini, so this port
 * is the third user of it rather than the inventor of anything.
 *
 * THE ROW SCALER IS pm.c's IDEA, NOT ITS CODE. The band loop asks for rows in
 * order within a band, so one row of cache turns a 2x vertical scale into one
 * expansion and one repeat: 288 destination rows from 144 source rows, expanded
 * 144 times instead of 46,080. The first Pokemon Mini version divided per pixel
 * and cost 9.9 ms; this one memorises the expanded row and memcpy's.
 *
 * THE PIXELS ARRIVE IN THE PANEL'S BYTE ORDER. gnuboy's GB_PIXEL_565_BE writes
 * 16-bit colour with the high byte first, which is exactly what the CO5300 wants,
 * so this port does no palette lookup and no byte swap -- where the NES spends
 * 4,355 us a frame turning palette indices into colours. That is why the blit is
 * cheap here even at 92,160 pixels against the NES's 61,440.
 */
#include "gb.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "display.h"
#include "edges.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "gamepad.h"
#include "pwrkey.h"
#include "statusbar.h"
#include "touch.h"
#include "sdpaths.h"
#include "tdsh.h"

/*
 * sound.h IS THIS PORT'S, NOT THE CORE'S, and the name collides: the gnuboy
 * component ships its own sound.h and puts its directory on the include path.
 * A quoted include searches the including file's own directory FIRST, so this
 * resolves to main/sound.h -- which is the sink, not the emulated APU. Getting
 * that backwards would fail at link, not at compile, which is the good kind of
 * mistake, but it is worth knowing which one is which: components/gnuboy/sound.c
 * is the Game Boy's four channels, and this is the amplifier's sample sink.
 */
#include "sound.h"

#include "gnuboy.h"


static const char *TAG = "gb";

/* Two frame buffers: the guest renders into one while core 1 pushes the other.
 * 160x144 at 2 bytes is 46,080 bytes each. Both in PSRAM, because the guest
 * writes them every frame and PSRAM is where the NES and PM keep theirs. */
#define GB_VBUF_N 2
#define GB_FRAME_BYTES (GB_WIDTH * GB_HEIGHT * sizeof(uint16_t))

static uint16_t *s_fb[GB_VBUF_N];
static uint8_t *s_rom;
static size_t s_rom_len;

static volatile bool s_run;
static TaskHandle_t s_task;
static TaskHandle_t s_blit_task;

/* The worst single blit since this session started. A blitter that stops
 * part-way through a session shows here and in the display driver's bus wait,
 * and nowhere else: `blits` merely stops climbing. */
static uint32_t s_blit_max_us;

/* When the last blit completed. A blitter that stops mid-session is dated by
 * this rather than by the shared push phase, which any other writer can update. */
static volatile int64_t s_blit_last_us;

/* Frames the guest could not render because no buffer was free. Non-zero means
 * the frame pool ran dry -- the state that stops blits while emulation runs. */
static uint32_t s_starved;

/* Which buffer the guest may render into next, and which holds a finished
 * frame. Depth 1 each is enough: the guest cannot produce faster than core 1
 * consumes, and a deeper pool would only add latency. */
static QueueHandle_t s_free_q;
static QueueHandle_t s_filled_q;

static uint32_t s_frames;
static uint32_t s_blits;
static uint32_t s_dropped;
static volatile uint32_t s_emu_us;
static volatile uint32_t s_blit_us;
static int s_pad;

/* --- the row scaler ------------------------------------------------------- */

/*
 * ONE EXPANDED SOURCE ROW, reused across the two destination rows it feeds.
 *
 * `s_xrow_sy` is which source row is currently in the cache, and it is compared
 * before expanding because the band loop asks for consecutive rows: a 2x scale
 * wants source row 7 for destination rows 14 and 15, and expanding once and
 * memcpy'ing the second is the difference between 144 expansions a frame and
 * 288.
 */
static uint16_t s_xrow[GB_WIDTH * GB_SCALE];
static int s_xrow_sy = -1;

static void gb_expand_row(const uint16_t *fb, int sy)
{
    const uint16_t *src = &fb[sy * GB_WIDTH];

    for (int x = 0; x < GB_WIDTH; x++)
    {
        const uint16_t v = src[x];
        uint16_t *d = &s_xrow[x * GB_SCALE];

        d[0] = v;
        d[1] = v;
    }
}

static void gb_fill_row(void *ctx, int row, int col, uint16_t *dst, int n)
{
    const uint16_t *fb = (const uint16_t *)ctx;
    const int sy = row / GB_SCALE;

    if (sy < 0 || sy >= GB_HEIGHT || col < 0)
    {
        for (int i = 0; i < n; i++)
            dst[i] = 0;
        return;
    }

    if (col + n > GB_WIDTH * GB_SCALE)
        n = GB_WIDTH * GB_SCALE - col;
    if (n <= 0)
        return;

    if (sy != s_xrow_sy)
    {
        gb_expand_row(fb, sy);
        s_xrow_sy = sy;
    }

    /* Destination is the panel's transmit band and the values are already in
     * wire order, so this is a straight copy of 16-bit elements. */
    memcpy(dst, &s_xrow[col], (size_t)n * sizeof(uint16_t));
}

/* --- the blit task, on core 1 -------------------------------------------- */

/* Defined with the controls further down, where `s_gp` exists. */

static void gb_blit_task(void *arg)
{
    (void)arg;

    while (s_run || uxQueueMessagesWaiting(s_filled_q))
    {
        int slot = -1;

        if (xQueueReceive(s_filled_q, &slot, pdMS_TO_TICKS(50)) != pdTRUE)
            continue;

        if (slot < 0 || slot >= GB_VBUF_N)
            continue;                    /* a garbage index must not reach s_fb[] */

        const int64_t t_blit = esp_timer_get_time();

        /* The cache belongs to this task alone, so a band boundary that lands
         * mid-row cannot leave a stale expansion behind. */
        s_xrow_sy = -1;

        display_picture_rows(s_fb[slot], GB_OX, GB_OY,
                             GB_WIDTH * GB_SCALE, GB_HEIGHT * GB_SCALE,
                             gb_fill_row);

        s_blit_us = (uint32_t)(esp_timer_get_time() - t_blit);

        if (s_blit_us > s_blit_max_us)
            s_blit_max_us = s_blit_us;

        s_blit_last_us = esp_timer_get_time();
        s_blits++;

        xQueueSend(s_free_q, &slot, 0);
    }

    /* CLEAR THE HANDLE BEFORE LEAVING, which is what makes gb_stop()'s wait on
     * it mean anything. The guest task has always done this (s_task = NULL
     * before vTaskDeleteWithCaps); this one never did, so the wait loop could
     * not observe the exit it was written to wait for and ran its full 100 x
     * 10 ms every single time the emulator closed. Measured on hardware
     * 2026-11-21: `gb stop` reported 1020 ms, of which about 1000 was this wait
     * doing nothing. The loop's own comment -- "let the blitter drain the last
     * frame and leave" -- describes behaviour that could not happen. */
    s_blit_task = NULL;
    vTaskDelete(NULL);
}

/* --- gnuboy's video callback --------------------------------------------- */

static uint16_t *s_render_target;

static void gb_video_cb(void *buffer)
{
    (void)buffer;

    /* gnuboy has finished a frame into the buffer it was given. Hand it to core 1
     * and take the next free one; if none is free the frame is dropped rather
     * than the guest being asked to wait, because a guest that blocks stops
     * producing input for as long as the push takes. */
    if (!s_filled_q || !s_free_q)
        return;                      /* callback fired before the queues exist */

    /*
     * PUBLISH IF THERE IS ANYTHING TO PUBLISH, THEN ALWAYS TRY TO REACQUIRE.
     *
     * The reacquire is outside the "did we publish" branch on purpose, and getting
     * that wrong hung the emulator: the first version returned early when there
     * was no render target, which is exactly the state a dry pool produces, so it
     * could never take a buffer back. The guest then ran with no framebuffer at
     * all -- gnuboy skips rendering, emulation dropped to 8.5 ms a frame, and the
     * glass kept displaying the last frame ever pushed. It looked like a working
     * emulator showing a still picture, and only the counters showed it:
     * `frames` advanced 2671 while `blits` advanced 1.
     */
    if (s_render_target)
    {
        int slot = -1;

        for (int i = 0; i < GB_VBUF_N; i++)
            if (s_fb[i] == s_render_target)
                slot = i;

        if (slot >= 0)
        {
            /* Publish before claiming the next, which is the ordering that
             * matters: take a free buffer before publishing the finished one and
             * a dry pool pushes half-written frames. */
            if (xQueueSend(s_filled_q, &slot, 0) != pdTRUE)
                s_dropped++;
        }
    }

    {
        int next = -1;

        if (xQueueReceive(s_free_q, &next, 0) == pdTRUE)
            s_render_target = s_fb[next];
        else
        {
            /* COUNTED, because this is the state that looks like a dead blitter
             * from outside: the guest keeps emulating, publishes nothing, and the
             * blitter waits on an empty queue for ever. Entry 87's `frames=17288
             * blits=1165` was this, and until now the only way to see it after
             * the fact was the frames-minus-blits gap. */
            s_starved++;
            s_render_target = NULL;  /* renders nothing until one comes back */
        }
    }
}

/* --- audio ---------------------------------------------------------------- */

/*
 * AUDIO WAS ALREADY HERE, AND THE PORT'S OWN NOTES WERE WRONG ABOUT IT.
 *
 * This file used to say the APU needed an external sample generator
 * (minigb_apu, the usual dependency of a bare gnuboy), and that was repeated in
 * the handoff notes and in the scoping document. It is not true of THIS fork:
 * components/gnuboy/sound.c is a complete four-channel emulation -- two squares,
 * the 4-bit wave channel, noise, envelopes, sweep, length counters, the NR50
 * master volume and the NR51 panning -- that synthesises straight into the host
 * buffer. All it ever needed was a buffer to synthesise into and somewhere to
 * send the result. That is this and the next twenty lines.
 *
 * AND IT WAS ALREADY BEING PAID FOR. gb_sound_emulate() runs from gb_hw_vblank()
 * on every frame whatever the host does with the samples, so the channel
 * arithmetic was inside the 13,768 us per frame already measured. What was
 * missing was the two lines of store into a NULL buffer that the core skips: the
 * `if (!output_buf) continue;` guard does not skip the synthesis above it. So
 * turning audio on costs the writes and the DMA copy, not a second synthesis
 * pass. Measured after wiring: frame time did not move.
 */

/*
 * THE RATE HAS TO BE THE RATE THE CORE PRODUCES, NOT THE RATE YOU ASKED FOR.
 *
 * gnuboy does not resample. gb_sound_reset() sets `snd.rate = round(2^21 /
 * samplerate)` and then emits exactly one sample every `snd.rate` double-speed
 * cycles, so the true output rate is 2^21 / snd.rate and the requested value only
 * chooses it. Ask for 44100 and the rounding lands on 48, which is 2^21/48 =
 * 43690.67 Hz -- 0.94 percent flat of what the sink would be clocked at.
 *
 * That gap is not cosmetic. Opened at 44100 the ring drains 9.3 samples a second
 * faster than it fills; the DMA holds 4096 frames, so the first click is about
 * seven minutes in and presents as "the sound goes bad after a while" -- the
 * same shape of bug pm.c documents for a 0.7-sample rounding loss. Picking
 * 43691 makes the rounding land on the same 48, and the requested rate, the
 * produced rate and the clock rate are then the same number to 0.008 percent.
 *
 * 48 is also why a frame is 731.5 samples and not a whole number: the frame is
 * 35112 double-speed cycles and no divisor helps. The core handles it by
 * carrying the remainder in snd.cycles across frames, so 731 and 732 alternate
 * and the average is exact. The buffer below only has to be larger than the
 * larger of the two.
 */
#define GB_AUDIO_HZ     43691
#define GB_AUDIO_BUF    1024      /* samples; a frame is 731.5          */

/* gnuboy's synthesised samples, in internal RAM rather than PSRAM because it is
 * written once per frame on the guest's own path and read immediately. */
static int16_t s_snd[GB_AUDIO_BUF];
static bool s_audio;

/*
 * gnuboy calls this at the end of gnuboy_run(), on the guest task, with the
 * samples it produced for that frame. sound_stream_write() never blocks and
 * drops what does not fit, which is the only contract a frame loop can accept:
 * a full ring is a clipped tick, while waiting for the DMA to drain is 16.8 ms of
 * stall on a 16.7 ms budget.
 */
static volatile bool s_audio_short;   /* the last write did not take it all */

static void gb_audio_cb(void *buffer, size_t length)
{
    if (!s_audio || !buffer || !length)
        return;

    if (length > GB_AUDIO_BUF)
        length = GB_AUDIO_BUF;

    /* THE RETURN VALUE IS THE WHOLE POINT OF THE HISTOGRAM. A short write is a
     * frame of audio that was partly refused; recording that it happened, in
     * the second it happened, is what tells a burst from a trickle. */
    const size_t got = sound_stream_write((const int16_t *)buffer, (int)length);

    s_audio_short = (got < (size_t)length);
}

/* --- save RAM ------------------------------------------------------------- */

/*
 * BATTERY-BACKED CARTRIDGE RAM, AS A FILE ON THE CARD.
 *
 * The core has the whole thing already: `gnuboy_load_sram()` and
 * `gnuboy_save_sram()` read and write the eight kilobyte banks the cartridge's
 * mapper exposes, plus the MBC3 time registers when the cartridge has them. What
 * it does not have is an opinion about where that file lives or when it is
 * written, which is what this section supplies.
 *
 * THE ONE ORDERING TRAP, AND IT IS SILENT: `gnuboy_reset(true)` calls
 * `gb_hw_reset(true)` with `hard` set, which does
 * `memset(cart.rambanks, 0xff, cart.ramsize * 8192)` and zeroes `cart.rtc`. Load
 * before the reset and every byte is overwritten a moment later -- the save is
 * read from the card, reported as loaded, and gone, with the game starting from
 * a blank cartridge. So the load is after `gnuboy_reset()`, and the comment is
 * here because the call order in gb_start() looks arbitrary otherwise.
 *
 * THE SAVE NAME COMES FROM THE ROM'S OWN NAME, which means one file per
 * cartridge with no bookkeeping and nothing to configure: `Zelda...gbc` writes
 * `Zelda....sav` in the same directory the layout reserves for saves. Renaming a
 * ROM abandons its save, which is the standard behaviour of every emulator that
 * does this and the only alternative is a catalogue nobody maintains.
 */
#define GB_SAVE_DIR     "/sd/saves/gbc"
#define GB_SAVE_NAME_MAX 120          /* basename, before the extension      */

/*
 * HOW OFTEN A DIRTY SAVE IS WRITTEN WITHOUT BEING ASKED.
 *
 * The cartridge's own save command decides when the game writes SRAM, and this
 * port has no way to know whether the write is finished. Twenty seconds is long
 * enough that a game's burst of writes has settled and short enough that a
 * yanked battery costs one save rather than a session. The first dirty frame
 * saves immediately -- `s_sram_due_us` starts at zero -- because the common case
 * is a player who saves and then puts the watch down, and that is exactly the
 * window a fixed timer would miss.
 */
#define GB_AUTOSAVE_US  20000000

static char s_save_path[256];
static bool s_saveable;               /* cartridge has battery-backed RAM    */

/*
 * THE LAST STOP'S SAVE OUTCOME, RECORDED RATHER THAN PRINTED.
 *
 * gb_stop() now has two callers in different tasks: the shell, and the Pusher
 * A hook on pwrkey_task. This port has a measured rule that a command prints
 * from the task that invoked it, because a foreign task's stdout does not
 * survive the desktop's terminal capture -- pmbench.c records the run that
 * established it, where a measurement completed and its output was simply not
 * there. So the stop records what it did here, and whoever can print prints it:
 * `gb stop` from the shell, and `gb status` afterwards from either path.
 *
 * A save that FAILED must stay findable, which is the whole reason this is not
 * discarded along with the print that used to carry it.
 */
static char s_last_stop[24];

/* How long the last stop took, in milliseconds, from the entry that began it
 * to the save being on the card. THIS IS THE NUMBER THE PUSHER A HOOK OWES THE
 * RECORD: the press is answered on pwrkey_task, which also owns the PMU
 * interrupt, so this is a delay in seeing a cable event as well as in getting
 * the desktop back. It is reported through `gb status` rather than through the
 * log, because the log is not readable over the USB mirror once a picture has
 * owned the panel -- and the emulator is exactly when it matters. */
static uint32_t s_last_stop_ms;

/*
 * WHERE THE STOP'S TIME GOES, phase by phase, for the last stop.
 *
 * A single total could not say which of three independent waits had run to its
 * limit, and all three were capped at a round number of milliseconds -- so a
 * 1000 ms stop could be any of them. These are the same numbers that separated
 * a real stall from a stale reading on 2026-11-21.
 */
static uint16_t s_stop_guest_ms, s_stop_blit_ms, s_stop_sram_ms, s_stop_save_ms;

/* Set when a stop had to abandon a blitter that would not exit, so the frames
 * it may still hold were leaked rather than freed. Cleared by the next start. */
static bool s_blit_leak;

/* Where the panel push was, and how long it had been there, when that happened.
 * Meaningless unless s_blit_leak is set. */
static uint8_t s_blit_stuck_phase;
static int64_t s_blit_stuck_ms;
static int64_t s_blit_stuck_strip_ms;

/* What FreeRTOS reported for the blitter's task when a stop gave up on it:
 * ready, blocked, suspended or deleted. Three different faults, one number. */
static int s_blit_stuck_state;

/* The frame pool when that happened: how many buffers were free, how many were
 * waiting to be blitted, and how many frames the guest had failed to render. */
static uint16_t s_blit_stuck_free, s_blit_stuck_filled;
static uint32_t s_blit_stuck_starved;
static uint32_t s_sram_saves;         /* writes that succeeded               */
static uint32_t s_sram_us;            /* cost of the last write              */
static uint32_t s_sram_snap_us;       /* cost of the last snapshot           */
static int64_t s_sram_due_us;

/*
 * THE WRITE DOES NOT HAPPEN ON THE FRAME PATH, AND THAT IS THE WHOLE DESIGN.
 *
 * A 32 KB save measured 121 ms on this board, eight times in a row, and the
 * cause is not this code: `tdsh_sdcard.c` mounts the card over SPI at
 * `SD_MAX_KHZ` 10 MHz rather than through SDMMC, which puts the achieved
 * throughput near 270 KB/s. Against a 16.7 ms frame and a 93 ms audio ring, a
 * write taken inline drops seven frames and underruns the speaker -- audible as
 * a click every time a game saves.
 *
 * So the guest task only ever does the cheap half: `gnuboy_sram_snapshot()`
 * copies the banks into one of two PSRAM buffers, which keeps the file format in
 * the core where it already lived, and hands the slot to a task whose entire job
 * is `fopen`/`fwrite`/`fclose`. Two slots so a second save can be snapshotted
 * while the first is still on the wire; when both are busy the save is skipped
 * and retried on the next due tick, because losing one autosave is free and
 * stalling the guest is not.
 *
 * The old code called `gnuboy_save_sram()` directly from the guest task and
 * worked, which is why the cost was invisible until it was measured.
 */
#define GB_SRAM_SNAP_N   2
#define GB_SRAM_SNAP_MAX (16 * 8192 + 48)   /* largest cartridge + RTC block */
#define GB_SRAM_LOG      8

static uint8_t *s_snap[GB_SRAM_SNAP_N];
static size_t s_snap_len[GB_SRAM_SNAP_N];
static QueueHandle_t s_snap_free;
static QueueHandle_t s_snap_ready;
static TaskHandle_t s_sram_task;
static volatile uint32_t s_sram_pending;

/* The shell asks, the guest task snapshots, the writer task writes, the shell
 * reads the answer. A save from the tdsh task while the guest runs would be a
 * second reader of the very banks the guest is writing, so the request is a
 * counter rather than an action. */
static volatile int s_sram_req;
static volatile int s_sram_rc;
static uint32_t s_sram_log[GB_SRAM_LOG];
static volatile int s_sram_log_n;

static void sram_path_for(const char *rom_path)
{
    s_save_path[0] = 0;

    if (!rom_path || !*rom_path)
        return;

    const char *base = strrchr(rom_path, '/');
    base = base ? base + 1 : rom_path;

    size_t n = strlen(base);
    const char *dot = strrchr(base, '.');

    if (dot && dot > base)
        n = (size_t)(dot - base);

    if (n > GB_SAVE_NAME_MAX)
        n = GB_SAVE_NAME_MAX;

    /* Created on demand, like sd_out() does for diagnostics: a save that fails
     * because a directory was never made is a support call, not a policy. */
    (void)mkdir(GB_SAVE_DIR, 0777);

    snprintf(s_save_path, sizeof(s_save_path), "%s/%.*s.sav",
             GB_SAVE_DIR, (int)n, base);
}

/*
 * THE FILE HALF. One `fopen`, one `fwrite`, one `fclose`, on a snapshot the
 * guest task already took. `wb` truncates, which is what makes the file exactly
 * the snapshot's length every time rather than leaving a longer previous save's
 * tail behind it -- the loader reads bank by bank from fixed offsets, and a
 * stale RTC block past the end of a shortened save would be read back.
 */
static int sram_write(const char *path, const uint8_t *data, size_t len)
{
    FILE *f = fopen(path, "wb");

    if (!f)
        return -1;

    const size_t put = fwrite(data, 1, len, f);
    const int shut = fclose(f);

    return (put == len && shut == 0) ? 0 : -1;
}

/*
 * Snapshot into a free slot and hand it to the writer.
 *
 * Returns true when a write was queued, false when the slots were busy or the
 * cartridge has no save RAM. Called ONLY from the guest task, between frames,
 * which is what makes reading `cart.rambanks` here safe.
 */
static bool sram_snapshot_and_queue(void)
{
    if (!s_saveable)
        return false;

    int slot = -1;

    if (xQueueReceive(s_snap_free, &slot, 0) != pdTRUE)
        return false;                    /* both slots in flight               */

    const int64_t t0 = esp_timer_get_time();

    s_snap_len[slot] = gnuboy_sram_snapshot(s_snap[slot], GB_SRAM_SNAP_MAX);
    s_sram_snap_us = (uint32_t)(esp_timer_get_time() - t0);

    if (!s_snap_len[slot])
    {
        xQueueSend(s_snap_free, &slot, 0);
        return false;
    }

    s_sram_pending++;
    xQueueSend(s_snap_ready, &slot, 0);
    return true;
}

/* --- the drop log, as a list of dirty seconds -----------------------------
 *
 * WHY A LIST AND NOT A BUCKET ARRAY, and the first version is the reason. Buckets
 * indexed by second held 180 of them, so a session longer than three minutes
 * silently stopped recording -- and on a 230-second run HALF THE DROPS ARRIVED
 * AFTER THE ARRAY WAS FULL, 6,615 of 13,052, with no way to see their shape.
 * That is an instrument that stops measuring exactly where the interesting part
 * might be, which is the worst kind.
 *
 * So only seconds THAT HAD A DROP are recorded, up to 64 of them. A run with
 * four dirty seconds in three minutes would need forty-five minutes to fill
 * this, so the reach is effectively the whole session, and every burst is kept
 * whenever it happens. The cost is that the total elapsed time is not implicit
 * in the index any more, so each entry carries its own second number.
 *
 * WHAT IT RECORDS IS THE DISCRIMINATOR. A stall and a starved audio path both
 * show up as drops, and they have different cures:
 *
 *   frames short + long worst frame  -> the guest loop stalled
 *   frames normal + long worst frame -> something else held the CPU
 *   frames normal + short worst frame -> the audio path itself
 *
 * `worst_blit_us` is recorded alongside because a 75 ms blit was observed in a
 * status line against a normal 7-8 ms, and if blits and stalls coincide that is
 * the cause rather than a coincidence.
 *
 * WRITTEN AT STOP, not during play: an SD write takes milliseconds and would
 * perturb what is being measured, the same reason `gb save` snapshots to PSRAM
 * and writes from another task.
 */
#define GB_HIST_EVENTS 64

typedef struct
{
    uint16_t sec;           /* which second of the session               */
    uint16_t frames;        /* video frames completed in that second     */
    uint16_t drops;         /* audio samples the ring refused            */
    uint16_t worst_us;      /* slowest single frame, WALL microseconds   */
    uint16_t worst_cpu_us;  /* CPU that same frame actually consumed     */
    uint16_t worst_blit_us; /* slowest single blit, microseconds         */
    uint16_t short_writes;  /* audio writes that did not take it all     */
} gb_drop_ev_t;

static gb_drop_ev_t *s_hist;       /* PSRAM: GB_HIST_EVENTS entries */
static int s_hist_n;               /* entries used, saturating       */
static uint32_t s_hist_total_sec;  /* seconds observed, for the report */
static int64_t s_hist_sec_us;      /* the second the current bucket ends */
static uint32_t s_hist_frames0, s_hist_drops0;
static uint16_t s_hist_worst_us, s_hist_worst_cpu, s_hist_worst_blit;

/* Per-iteration timing carried to the next iteration, because the kernel's
 * run-time counter only settles when the task switches out. */
static configRUN_TIME_COUNTER_TYPE s_cpu_prev;
static int64_t s_work_start_prev, s_work_end_prev;
static bool s_have_prev;
static uint16_t s_hist_short;

/* THIS TASK'S CUMULATIVE CPU TIME, in microseconds.
 *
 * `ulTaskGetRunTimeCounter()` does not exist in this kernel build -- only the
 * SMP variant has it -- so the value is read out of the task's own status block,
 * which `vTaskGetInfo()` fills from `ulRunTimeCounter`. That field is
 * accumulated by the kernel when a task switches out, using the esp_timer at
 * 1 MHz, so it is exact to the microsecond for the interval being measured.
 *
 * `pdFALSE` for the stack figure: the high-water mark is not wanted here and
 * computing it walks the stack. */
static configRUN_TIME_COUNTER_TYPE cpu_time_us(void)
{
    TaskStatus_t st;

    if (!s_task)
        return 0;

    vTaskGetInfo(s_task, &st, pdFALSE, eRunning);
    return st.ulRunTimeCounter;
}

/* Called once per frame from the guest task. Cheap: one comparison against a
 * clock the loop has already read. */
static void hist_frame(int64_t now_us, uint32_t frames, uint32_t drops,
                       uint32_t frame_us, uint32_t frame_cpu_us,
                       uint32_t blit_us, bool short_write)
{
    /* THE WORST FRAME IS TRACKED BY WALL TIME, AND ITS CPU TIME IS KEPT WITH IT.
     * Those two numbers together are the whole point: a 52 ms frame that used
     * 50 ms of CPU is slow work, and the same frame using 12 ms of CPU was off
     * the processor for 40 ms. Nothing else in this file can tell them apart. */
    if (frame_us > s_hist_worst_us && frame_us < 65535)
    {
        s_hist_worst_us = (uint16_t)frame_us;
        s_hist_worst_cpu = (frame_cpu_us < 65535) ? (uint16_t)frame_cpu_us : 65534;
    }
    if (blit_us > s_hist_worst_blit && blit_us < 65535)
        s_hist_worst_blit = (uint16_t)blit_us;
    if (short_write)
        s_hist_short++;

    if (now_us < s_hist_sec_us)
        return;

    /* ONLY A SECOND WITH DROPS IS KEPT. Everything else is discarded at the end
     * of its second, which is what makes the list reach the whole session. */
    const uint32_t d = drops - s_hist_drops0;

    if (d && s_hist && s_hist_n < GB_HIST_EVENTS)
    {
        gb_drop_ev_t *e = &s_hist[s_hist_n++];

        e->sec = (uint16_t)(s_hist_total_sec < 65535 ? s_hist_total_sec : 65535);
        e->frames = (uint16_t)(frames - s_hist_frames0);
        e->drops = (uint16_t)(d > 65535 ? 65535 : d);
        e->worst_us = s_hist_worst_us;
        e->worst_cpu_us = s_hist_worst_cpu;
        e->worst_blit_us = s_hist_worst_blit;
        e->short_writes = s_hist_short;
    }

    s_hist_total_sec++;
    s_hist_frames0 = frames;
    s_hist_drops0 = drops;
    s_hist_worst_us = 0;
    s_hist_worst_cpu = 0;
    s_have_prev = false;
    s_hist_worst_blit = 0;
    s_hist_short = 0;
    s_hist_sec_us += 1000000;
    if (now_us - s_hist_sec_us > 2000000)
        s_hist_sec_us = now_us + 1000000;   /* stalled past a whole second */
}

/* Write the dirty seconds out as one line each. A file, because a 40-column
 * screen cannot show a table and scrollback does not survive a repaint. */
static void hist_dump(void)
{
    const char *path = sd_out("/sd/diag", "gbhist.txt");
    FILE *f = fopen(path, "w");

    if (!f)
    {
        printf("gb: cannot write %s\n", path);
        return;
    }

    fprintf(f, "session %lu s, %d second(s) with drops\n",
            (unsigned long)s_hist_total_sec, s_hist_n);
    fprintf(f, "sec frames drops wallms cpums blitms short\n");

    for (int i = 0; i < s_hist_n; i++)
        fprintf(f, "%3u %6u %5u %6u %5u %6u %5u\n", (unsigned)s_hist[i].sec,
                (unsigned)s_hist[i].frames, (unsigned)s_hist[i].drops,
                (unsigned)s_hist[i].worst_us / 1000,
                (unsigned)s_hist[i].worst_cpu_us / 1000,
                (unsigned)s_hist[i].worst_blit_us / 1000,
                (unsigned)s_hist[i].short_writes);

    if (s_hist_n == GB_HIST_EVENTS)
        fprintf(f, "LIST FULL -- later dirty seconds were not recorded\n");

    fclose(f);

    /* The summary on screen says only what a 40-column line can hold: how many
     * seconds were dirty out of how many, and the worst of them. */
    unsigned worst = 0, worst_sec = 0;

    for (int i = 0; i < s_hist_n; i++)
        if (s_hist[i].drops > worst)
        {
            worst = s_hist[i].drops;
            worst_sec = s_hist[i].sec;
        }

    printf("gb: %d of %lu s dirty, worst %u at sec %u\n", s_hist_n,
           (unsigned long)s_hist_total_sec, worst, worst_sec);
    printf("gb: %s\n", path);
}

/*
 * Write what is known about a blitter that would not exit, to a file.
 *
 * THE MIRROR CANNOT CARRY THIS. A wedged blitter only happens while a picture
 * owns the panel, and that is exactly when the USB mirror is stale (DSP-003), so
 * a host polling the screen sees nothing and a console line is unreadable. The
 * card survives the session, so this is written once and read later.
 *
 * The fields answer the three questions in order: what the task is doing
 * (state), where the last panel push got to (phase and age), and whether the
 * frame pool had run dry (free/filled/starved). Those are different faults and
 * telling them apart from one line is the point of the file.
 */
static void wedge_dump(void)
{
    const char *path = sd_out("/sd/diag", "gbwedge.txt");
    FILE *f = fopen(path, "w");

    if (!f)
        return;

    fprintf(f, "blitter wedged at stop\n");
    fprintf(f, "frames %lu blits %lu dropped %lu starved %lu\n",
            (unsigned long)s_frames, (unsigned long)s_blits,
            (unsigned long)s_dropped, (unsigned long)s_starved);
    /* eRunning 0, eReady 1, eBlocked 2, eSuspended 3, eDeleted 4. */
    fprintf(f, "task state %d (0 run 1 ready 2 blocked 3 susp 4 deleted)\n",
            s_blit_stuck_state);
    /* 0 idle, 1 acquiring, 2 acquired, 3 window, 4 stream, 5 done. */
    fprintf(f, "push phase %u (0 idle 1 acq 2 got 3 win 4 stream 5 done)\n",
            (unsigned)s_blit_stuck_phase);
    fprintf(f, "ms since last blit %lld\n", (long long)s_blit_stuck_ms);
    fprintf(f, "pool free %u filled %u (capacity %d)\n",
            (unsigned)s_blit_stuck_free, (unsigned)s_blit_stuck_filled,
            GB_VBUF_N);
    fprintf(f, "ms since strip push %lld\n",
            (long long)s_blit_stuck_strip_ms);

    fclose(f);
}

/* --- the save writer, on core 1 alongside the blitter -------------------- */

static void gb_sram_task(void *arg)
{
    (void)arg;

    int slot;

    while (s_run || uxQueueMessagesWaiting(s_snap_ready))
    {
        if (xQueueReceive(s_snap_ready, &slot, pdMS_TO_TICKS(50)) != pdTRUE)
            continue;

        const int64_t t0 = esp_timer_get_time();
        const int rc = sram_write(s_save_path, s_snap[slot], s_snap_len[slot]);

        s_sram_us = (uint32_t)(esp_timer_get_time() - t0);
        s_sram_rc = rc ? -1 : 0;

        if (rc == 0)
            s_sram_saves++;
        else
            gnuboy_sram_mark_dirty();   /* the snapshot cleared it; retry later */

        if (s_sram_log_n < GB_SRAM_LOG)
            s_sram_log[s_sram_log_n++] = s_sram_us;

        xQueueSend(s_snap_free, &slot, 0);
        s_sram_pending--;
    }

    s_sram_task = NULL;
    /* The stack came from PSRAM, so the caps-aware delete is required: plain
     * vTaskDelete() leaves the idle task to free a TCB it did not allocate and
     * leaks the PSRAM stack. Same trap as the guest task's. */
    vTaskDeleteWithCaps(NULL);
}

/* --- the guest task, on core 0 ------------------------------------------- */

static void gb_task(void *arg)
{
    (void)arg;

    int64_t next = esp_timer_get_time();

    while (s_run)
    {
        /*
         * SETTLE THE PREVIOUS ITERATION, HERE AND NOT WHERE IT ENDED.
         *
         * The kernel adds to a task's run-time counter WHEN IT SWITCHES OUT, so
         * a reading taken mid-frame does not yet include the work in progress.
         * At the top of this iteration -- after last iteration's delay switched
         * us out and the timer switched us back in -- the counter is exact for
         * the interval that just finished.
         *
         * THE TWO NUMBERS MUST COVER THE SAME INTERVAL OR THE COMPARISON IS
         * MEANINGLESS. Wall is therefore WORK time only, from this point to the
         * moment the work ended, and it deliberately EXCLUDES the delay: the
         * delay is ~5.7 ms of intentional sleep on every frame, and including it
         * would make every frame look partly preempted. A 52 ms frame that used
         * 50 ms of CPU was doing slow work; the same frame using 12 ms was off
         * the processor for 40 ms. That is the whole question this measures.
         */
        {
            const configRUN_TIME_COUNTER_TYPE cpu = cpu_time_us();

            if (s_have_prev)
                hist_frame(esp_timer_get_time(), s_frames,
                           sound_stream_dropped(),
                           (uint32_t)(s_work_end_prev - s_work_start_prev),
                           (uint32_t)(cpu - s_cpu_prev), s_blit_us,
                           s_audio_short);

            s_cpu_prev = cpu;
            s_have_prev = true;
        }

        const int64_t t0 = esp_timer_get_time();

        s_work_start_prev = t0;

        /*
         * RECOVER A DRY POOL HERE AS WELL AS IN THE CALLBACK. The callback is the
         * only other place a buffer can be taken, and a callback that is never
         * called because there is no framebuffer is a deadlock rather than a slow
         * path. This line makes that state recoverable from the guest side.
         */
        if (!s_render_target)
        {
            int next = -1;

            if (xQueueReceive(s_free_q, &next, 0) == pdTRUE)
                s_render_target = s_fb[next];
        }

        gnuboy_set_framebuffer(s_render_target);
        gnuboy_run(true);

        s_emu_us = (uint32_t)(esp_timer_get_time() - t0);

        /* gnuboy swallows the pad; the value is read here so a press that lands
         * during a frame is in effect for the next one, which at 16.7 ms is
         * inside a single console frame. */
        gnuboy_set_pad(s_pad);

        s_frames++;

        /*
         * SAVE RAM: SNAPSHOT HERE, WRITE ELSEWHERE.
         *
         * This task is the only reader of `cart.rambanks`, so a snapshot taken
         * here between two frames cannot tear: the guest is not executing when
         * this line runs. That is why the shell's request is a counter. What
         * lands in this frame's budget is a memcpy of at most 128 KB, not the
         * 121 ms write.
         */
        {
            const int64_t sram_now = esp_timer_get_time();

            if (s_saveable &&
                (s_sram_req > 0 ||
                 (sram_now >= s_sram_due_us && gnuboy_sram_dirty())))
            {
                const bool queued = sram_snapshot_and_queue();

                s_sram_due_us = sram_now + GB_AUTOSAVE_US;

                if (s_sram_req > 0)
                {
                    s_sram_req--;

                    /* A forced save that could not be queued must not leave
                     * the shell waiting for an answer nobody will send. */
                    if (!queued)
                        s_sram_rc = -1;
                }
            }
        }

        /* Where the work ended. The delay that follows is not part of the
         * measurement -- see the top of the loop. */
        s_work_end_prev = esp_timer_get_time();

        next += GB_FRAME_US;
        const int64_t now = esp_timer_get_time();

        if (now < next)
            vTaskDelay(pdMS_TO_TICKS((next - now + 999) / 1000));
        else
            next = now;      /* behind: drop the backlog, never burst */
    }

    s_task = NULL;
    /* The stack came from PSRAM, so the caps-aware delete is required: plain
     * vTaskDelete() leaves the idle task to free a TCB it did not allocate and
     * leaks the PSRAM stack. Same trap as the music player's. */
    vTaskDeleteWithCaps(NULL);
}

/* --- input --------------------------------------------------------------- */

/*
 * THE POKEMON MINI'S FURNITURE, SIZED FOR A TIGHTER BAND.
 *
 * The operator asked for this port to carry the same drawn d-pad and two circles
 * the Pokemon Mini and NES have, rather than the margin columns it shipped with.
 * The shapes, the disc-shaped four-quadrant hit test and the 1.5x touch discs all
 * come from `gamepad.c`, which is where that furniture has lived since it was
 * factored out of `pm.c` -- so this is a layout, not a second rasteriser.
 *
 * THE ROTATION IS UPRIGHT, AT THE OPERATOR'S DIRECTION, and that is now
 * expressible: `gamepad_t` carries a cosine and a sine, so 65536 and 0 is no
 * rotation and the arms point at the four sides of the glass. Until that pair
 * existed the module could only draw 45-degree diamonds, whatever `rot_k` said.
 *
 * THE SIZES COULD NOT BE COPIED, AND THE REASON IS ARITHMETIC. A frame of S
 * paints a bounding span of about S * 0.943 when it is rotated 45 degrees, so the
 * NES fits a 168 px frame into 158 painted rows. This port's picture is 288 rows
 * tall, twice the NES's 2x scale of a 144-line screen, so the band below it is 169
 * rows -- smaller than the NES cluster itself.
 *
 * Upright, the cross reaches its full frame, so 148 paints 148: that is 10 rows
 * clear above and 11 below in a 169-row band. The circles are 74 instead of 84
 * with the gap cut from 32 to 12, which is 160 of the 169 available, and the
 * pair sits at rows 292..451 with six clear at the bottom. The alternatives were
 * measured rather than guessed: at the NES's 168 the frame touches the picture,
 * and at two 84 px circles the pair needs 200 rows in a 169-row band and does not
 * fit at all.
 *
 * THE HORIZONTAL POSITIONS ARE THE NES'S, EXCEPT WHERE THE OPERATOR MOVED THEM.
 * The d-pad's left edge lands on the left column's axis -- that is the same rule
 * the NES follows, and it is now the frame's own left edge because an upright
 * cross reaches its bounding box, where a 45-degree diamond did not: the diamond
 * needed a 7 px inset to put its leftmost painted pixel on that axis, and an
 * upright one needs none. A and B sit at the NES's columns of 236 and 208, with A
 * moved two cells (32 px) right to 268 at the operator's direction, which puts it
 * 22 px past the framebuffer wall and against the right column's inner edge. It is
 * fully painted because the whole cluster sits below the side columns, where the
 * push spans the entire glass.
 */
#define GB_GLASS_BOT    (DISPLAY_PANEL_H - DISPLAY_ORIGIN_Y)              /* 457 */
#define GB_CTL_Y        (GB_HEIGHT * GB_SCALE)                            /* 288 */
#define GB_CTL_BAND     (GB_GLASS_BOT - GB_CTL_Y)                         /* 169 */

#define GB_DPAD_S       148
#define GB_DPAD_TIP_INSET  0     /* upright: the painted edge is the frame edge */

/*
 * THE D-PAD'S TOUCH DISC IS 20 PERCENT WIDER THAN THE DRAWN CROSS, at the
 * operator's direction after a rectangular hit area was tried and rejected.
 *
 * gamepad.c's default is `size / 2 + 4` -- the cross's painted extent plus 4 px of
 * tolerance at the tips -- which is 78 at this frame size. 78 * 1.2 is 94, and the
 * "20 percent" is of the RADIUS, not the area: a thumb's reach is a length, so the
 * linear reading is the one that matters and the one the operator meant.
 *
 * IT COSTS TEN ROWS OF PICTURE. At 94 the disc's top is fb 278 and the guest's
 * last row is 287, so a press in a narrow band along the bottom of the screen can
 * register as Up. That is the same trade the circles have always made -- A's disc
 * reaches 14 rows in -- and it is accepted here for the same reason: the pad was
 * judged too small to use, and a slightly eager edge is better than a pad that
 * misses. The B circle's disc starts at fb 190, so the enlarged pad does not reach
 * it and the two cannot fire from one press.
 */
#define GB_DPAD_HIT_R   94

#define GB_COL_EDGE     (LEFT_X1 - PANEL_COL_OFFSET - DISPLAY_ORIGIN_X)   /* -1 */
#define GB_DPAD_X       (GB_COL_EDGE - GB_DPAD_TIP_INSET)                 /* -1 */
#define GB_DPAD_Y       (GB_CTL_Y + (GB_CTL_BAND - GB_DPAD_S) / 2)        /* 298 */

#define GB_BTN_D        74
#define GB_BTN_GAP      12
#define GB_BTN_HIT_D    (GB_BTN_D * 3 / 2)                               /* 111 */

#define GB_BTN_A_X      268     /* 236 + two 16 px cells, by direction */
#define GB_BTN_B_X      208
#define GB_BTN_Y        (GB_CTL_Y + (GB_CTL_BAND - (2 * GB_BTN_D + GB_BTN_GAP)) / 2)
#define GB_BTN_A_Y      (GB_BTN_Y + 4)  /* 296: four px down, by direction */
#define GB_BTN_B_Y      (GB_BTN_Y + GB_BTN_D + GB_BTN_GAP)               /* 378 */

/* Three column slots, as the NES has: Start and Select plus a blank. Fewer slots
 * than the five this port shipped with, because the d-pad and circles now carry
 * the directions and the face buttons and the column does not need to. */
#define GB_COL_SLOTS    3
#define GB_COL_BOT      (COL_Y0 + (GB_COL_SLOTS * (COL_Y1 - COL_Y0 + 1)) / COL_N - 1)

static const gamepad_t s_gp = {
    .cx = GB_DPAD_X + GB_DPAD_S / 2,
    .cy = GB_DPAD_Y + GB_DPAD_S / 2,
    .size = GB_DPAD_S,
    /* UPRIGHT. Cosine 65536 and sine 0 is no rotation at all, so the cross's
     * arms point at the four sides of the glass and its bounding box is exactly
     * the 148 px frame. */
    .rot_k = 65536,
    .rot_s = 0,
    .line = 2,
    .tip_r = 5,

    .pad_cx = { GB_BTN_A_X + GB_BTN_D / 2, GB_BTN_B_X + GB_BTN_D / 2 },
    .pad_cy = { GB_BTN_A_Y + GB_BTN_D / 2, GB_BTN_B_Y + GB_BTN_D / 2 },
    .pad_d = { GB_BTN_D, GB_BTN_D },
    .pad_hit_d = { GB_BTN_HIT_D, GB_BTN_HIT_D },
    .pad_label = { 'A', 'B' },

    .dpad_hit_r = GB_DPAD_HIT_R,

    .bg = 0x0000,
    .grid = 0,
};

static int gb_controls_hit(int panel_x, int panel_y)
{
    return gamepad_hit(&s_gp, panel_x - DISPLAY_ORIGIN_X, panel_y - DISPLAY_ORIGIN_Y);
}

/*
 * PAINTED IN TWO BANDS, for the reason `nes_draw_controls()` documents: the side
 * columns occupy the glass outside the framebuffer, so a glass-wide rectangle at
 * their height would cut the third button in half. Rows down to the last drawn
 * column slot are framebuffer-wide; below that the whole glass is free.
 *
 * Here the split barely matters -- the columns end at fb 293 and the lowest thing
 * this cluster paints starts at 301 -- but painting one band instead of two would
 * still be wrong the first time a control moved up, and the split costs nothing.
 */
static bool gb_draw_controls(void)
{
    if (!display_pixels_claimed())
        return false;

    uint16_t fill, edge, ink;
    display_margin_colours(&fill, &edge, &ink);

    const uint16_t face = display_swap16(fill);
    const uint16_t e = display_swap16(edge);
    const uint16_t i = display_swap16(ink);

    gamepad_t g = s_gp;
    g.bg = 0x0000;
    g.grid_line = 0;

    bool ok = true;

    ok = gamepad_paint(&g, 0, GB_CTL_Y, DISPLAY_W,
                       GB_COL_BOT - GB_CTL_Y + 1, face, e, i) && ok;

    ok = gamepad_paint(&g, -DISPLAY_ORIGIN_X, GB_COL_BOT + 1, DISPLAY_PANEL_W,
                       GB_GLASS_BOT - (GB_COL_BOT + 1), face, e, i) && ok;

    return ok;
}

/*
 * THE DRAWN CONTROLS AND THE MARGINS AS ONE GAME BOY PAD.
 *
 * gnuboy's `gb_padbtn_t` is active-HIGH bit flags, unlike the console's own
 * active-low register, so this builds the mask gnuboy expects rather than the one
 * the hardware reads.
 *
 * THE DRAWN CONTROLS WIN WHEN BOTH MATCH, per point rather than per frame. B's
 * 111 px touch disc reaches 9 px into the right column and the bottom strip's
 * touch area starts at fb 375, so one touch can be both a circle and a margin
 * slot; emitting both would fire B and Select from one press. The circle is what
 * the operator aimed at and what is visible under the thumb.
 *
 * WHAT IS LEFT ON THE MARGINS IS WHAT HAS NO DRAWN EQUIVALENT: Start and Select
 * on the right column, the first two slots. The rest of the column and the whole
 * bottom strip are blanked and their chrome off, because blank glass that fires a
 * button is the fault the operator reported on this project twice already.
 */
static int pad_from_frame(const touch_frame_t *f)
{
    int pad = 0;

    if (!f)
        return 0;

    for (int i = 0; i < f->n && i < TOUCH_POINTS; i++)
    {
        const int ctl = gb_controls_hit(f->p[i].x, f->p[i].y);

        switch (ctl)
        {
        case GP_A:     pad |= GB_PAD_A;      break;
        case GP_B:     pad |= GB_PAD_B;      break;
        case GP_UP:    pad |= GB_PAD_UP;     break;
        case GP_DOWN:  pad |= GB_PAD_DOWN;   break;
        case GP_LEFT:  pad |= GB_PAD_LEFT;   break;
        case GP_RIGHT: pad |= GB_PAD_RIGHT;  break;
        default:                             break;
        }

        if (ctl == GP_NONE && f->p[i].region == EDGE_RIGHT)
        {
            /* SECOND AND THIRD SLOTS, BY THE OPERATOR'S DIRECTION: Select above
             * Start, with the top slot left empty. Slot 0 therefore has no duty
             * at all, and the pair that a thumb actually reaches for sits lower
             * on the glass than it used to. */
            if (f->p[i].index == 1) pad |= GB_PAD_SELECT;
            if (f->p[i].index == 2) pad |= GB_PAD_START;
        }
    }

    return pad;
}

static bool gb_app_touch(void *user, const touch_frame_t *f)
{
    (void)user;

    if (!s_run)
        return false;

    bool used = false;

    if (f)
    {
        /* The predicate that answers for the pad state also answers for the
         * feedback, so a control cannot respond without also being felt. */
        for (int i = 0; i < f->n && i < TOUCH_POINTS; i++)
        {
            if (gb_controls_hit(f->p[i].x, f->p[i].y) != GP_NONE)
                used = true;

            /* SLOTS 1 AND 2, NOT 0 AND 1. This is the same predicate as the
             * mapping above, written separately, and the two have to agree: a
             * slot counted as live here but given no bit there would buzz
             * without acting, and one given a bit but not counted live would
             * act without being felt. */
            if (f->p[i].region == EDGE_RIGHT && f->p[i].index >= 1 &&
                f->p[i].index <= 2 &&
                display_margin_slot_live(1, f->p[i].index))
                used = true;
        }
    }

    s_pad = pad_from_frame(f);
    return used;
}

/*
 * PUSHER A WHILE THE EMULATOR IS UP: one tap closes it and returns to the
 * desktop. That is the operator's request, and the press is claimed only while
 * a guest is actually running -- with nothing running it is still Enter, which
 * is how the desktop is driven in the first place.
 *
 * WHO CALLS THIS, AND WHY THAT SHAPES IT. It runs on pwrkey_task, not on the
 * shell's task, so it must not print: this port has a measured rule that a
 * foreign task's stdout does not survive the desktop's terminal capture (the
 * run that established it is recorded in pmbench.c). Everything it touches is
 * safe from a foreign task -- the display driver marshals its work to the
 * painter and takes its own lock, which display_repaint_all() documents at
 * length, and the emulator owns pixels rather than a window-manager window, so
 * wm.c's unlocked state is not in reach here.
 *
 * gb_stop() saves SRAM before it tears anything down, so an accidental tap
 * costs the session's position at worst and never the save file.
 */
static bool gb_app_key(void *user)
{
    (void)user;

    if (!gb_running())
        return false;              /* not ours: Enter passes through */

    gb_stop();

    /* Nothing is printed here on purpose: this runs on pwrkey_task, where a
     * print is lost. What the stop did and what it cost are in s_last_stop and
     * s_last_stop_ms, which `gb status` reports from a task that can print. */
    return true;
}

/* --- start / stop -------------------------------------------------------- */

bool gb_start(const char *rom_path)
{
    /*
     * A WEDGED BLITTER CLOSES THE EMULATOR UNTIL THE WATCH IS REBOOTED.
     *
     * If a previous stop could not get its blitter to exit, that task is still
     * alive with pointers to the frame queues and the frame buffers. Starting
     * again would allocate a new set and hand the old task a world it was never
     * synchronised with, which is a race with no good outcome. Refusing is the
     * honest state: it is reported, it is not a crash, and a reboot clears it.
     */
    if (s_blit_leak)
    {
        printf("gb: a blitter from the last session never exited -- reboot\n");
        return false;
    }

    if (s_run)
    {
        printf("gb: already running -- gb stop first\n");
        return false;
    }

    const char *path = rom_path && *rom_path ? rom_path
                                             : sd_media(GB_ROM_DEFAULT, GB_ROM_FLAT);

    FILE *fp = fopen(path, "rb");
    if (!fp)
    {
        printf("gb: cannot open %s\n", path);
        return false;
    }

    fseek(fp, 0, SEEK_END);
    const long len = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    if (len <= 0 || len > (4L << 20))
    {
        printf("gb: %s is %ld bytes -- not a cartridge\n", path, len);
        fclose(fp);
        return false;
    }

    /* PSRAM: the largest cartridge is 4 MB and internal RAM holds none of it.
     * gnuboy does not copy the ROM, it reads through the pointer every fetch, so
     * this is on the hot path -- which is why the benchmark measured a real
     * cartridge out of this memory rather than out of internal RAM. */
    s_rom = (uint8_t *)heap_caps_malloc((size_t)len, MALLOC_CAP_SPIRAM);
    if (!s_rom)
    {
        printf("gb: no PSRAM for a %ld-byte cartridge\n", len);
        fclose(fp);
        return false;
    }

    s_rom_len = (size_t)len;

    if (fread(s_rom, 1, s_rom_len, fp) != s_rom_len)
    {
        printf("gb: short read on %s\n", path);
        fclose(fp);
        goto fail_rom;
    }

    fclose(fp);
    fp = NULL;

    for (int i = 0; i < GB_VBUF_N; i++)
    {
        s_fb[i] = (uint16_t *)heap_caps_malloc(GB_FRAME_BYTES, MALLOC_CAP_SPIRAM);

        if (!s_fb[i])
        {
            printf("gb: no PSRAM for frame buffer %d\n", i);
            goto fail_fb;
        }
    }

    if (gnuboy_init(GB_AUDIO_HZ, GB_AUDIO_MONO_S16, GB_PIXEL_565_BE,
                    gb_video_cb, gb_audio_cb) != 0)
    {
        printf("gb: gnuboy_init failed\n");
        goto fail_fb;
    }

    gnuboy_set_soundbuffer(s_snd, GB_AUDIO_BUF);

    /*
     * THE MACHINE TYPE COMES FROM THE CARTRIDGE, AND THIS PORT DOES NOT CHOOSE
     * IT. `gnuboy_load_rom()` below reads header byte 0x143 on the way in and
     * sets GB_HW_CGB for 0x80 or 0xC0, GB_HW_SGB for 0x146 == 3, and GB_HW_DMG
     * otherwise -- so it OVERWRITES whatever was set before it.
     *
     * There used to be a `gnuboy_set_hwtype(GB_HW_CGB)` here, with a comment
     * saying it gave every cartridge CGB hardware. It never did: it ran before
     * the load and was replaced a few lines later, so a DMG cartridge has always
     * run as a DMG. The behaviour was right and the comment was wrong, which is
     * the same failure this project has hit twice before -- a claim in a comment
     * that nobody had checked against the code beside it. Removed rather than
     * moved, because forcing CGB on a DMG cartridge is not what the hardware
     * this port models does with one.
     *
     * The palette is still set explicitly: it is not overwritten by the load. */
    gnuboy_set_palette(GB_PALETTE_CGB);

    if (gnuboy_load_rom(s_rom, s_rom_len) != 0)
    {
        printf("gb: %s: unsupported cartridge or mapper\n", path);
        gnuboy_free_rom();
        goto fail_fb;
    }

    /* NOT OPTIONAL. With no boot ROM loaded -- supported, and what real hardware
     * does once the boot ROM hands over -- `PC = GB.bios ? 0 : 0x100` is decided
     * inside the reset. Skip it and PC stays 0, the first fetch goes through
     * address 0, and gb_hw_read() dereferences a NULL BIOS pointer. */
    gnuboy_reset(true);

    /*
     * LOAD THE SAVE AFTER THE RESET, NEVER BEFORE. `gnuboy_reset(true)` fills
     * `cart.rambanks` with 0xFF and zeroes `cart.rtc`, so a load placed before it
     * is silently erased. `gnuboy_load_sram()`'s return value is also how this
     * port learns whether the cartridge has a battery at all, because the core
     * has no getter for it: -1 is "no battery or no RAM", -2 is "saveable but no
     * file yet", 0 is "loaded".
     */
    sram_path_for(path);
    s_sram_saves = 0;
    s_sram_us = 0;
    s_sram_due_us = 0;
    s_sram_req = 0;
    s_sram_rc = 0;
    s_sram_pending = 0;
    s_sram_saves = 0;
    s_sram_us = 0;
    s_sram_snap_us = 0;
    s_sram_log_n = 0;

    for (int i = 0; i < GB_SRAM_SNAP_N; i++)
    {
        if (!s_snap[i])
            s_snap[i] = (uint8_t *)heap_caps_malloc(GB_SRAM_SNAP_MAX, MALLOC_CAP_SPIRAM);

        if (!s_snap[i])
        {
            printf("gb: no PSRAM for the save snapshot\n");
            s_saveable = false;      /* run without saving rather than not run */
        }
    }

    {
        const int rc = gnuboy_load_sram(s_save_path);

        s_saveable = (rc != -1);

        if (rc == 0)
            printf("gb: save restored from %s\n", s_save_path);
        else if (rc == -2)
            printf("gb: no save yet at %s\n", s_save_path);
        else
            printf("gb: cartridge has no save RAM\n");
    }

    /*
     * BEST-EFFORT, like the other two ports: no amplifier is a quiet Game Boy,
     * not a failed start. The callback drops its samples when this is false, so
     * the only cost of a missing amp is that nothing is written.
     */
    s_audio = sound_stream_open(GB_AUDIO_HZ);

    /* Fresh buckets per session, so a histogram from one game is never mixed
     * with another's. Allocated once and kept, like the frame buffers, because
     * it is read for the whole session. */
    if (!s_hist)
    {
        s_hist = heap_caps_calloc(GB_HIST_EVENTS, sizeof(gb_drop_ev_t),
                                  MALLOC_CAP_SPIRAM);
    }

    s_hist_n = 0;
    s_hist_total_sec = 0;
    s_hist_sec_us = esp_timer_get_time() + 1000000;
    s_hist_frames0 = s_frames;
    s_hist_drops0 = sound_stream_dropped();
    s_hist_worst_us = 0;
    s_hist_worst_blit = 0;
    s_hist_short = 0;

    if (!s_audio)
        printf("gb: no audio (amplifier did not come up)\n");

    display_pixels_claim(true);
    display_fill(0x0000);

    /*
     * THREE COLUMN SLOTS, AND THE CONTROLS ARE DRAWN ON THE GLASS.
     *
     * The first version of this port used all five column slots for the d-pad and
     * the bottom strip for B, A and Start, on the reasoning that a 320-wide
     * picture leaves no room below it for a control cluster. That reasoning was
     * wrong: the picture is 288 rows tall in a 457-row glass, so there are 169
     * rows below it, which is enough for the same furniture the other two
     * emulators carry once it is sized for the band rather than copied from it.
     *
     * So the right column now carries only what has no drawn equivalent, Start and
     * Select, and every slot below them is blanked with its chrome off. A blank is
     * a single space rather than NULL, because NULL falls back to the shared table
     * and a cleared label has to render as nothing.
     */
    display_margin_column_slots(GB_COL_SLOTS);

    display_margin_label(0, 0, "Pause");
    display_margin_label(0, 1, "Reset");
    display_margin_label(0, 2, "Save");

    for (int s = 3; s < COL_N; s++)
    {
        display_margin_chrome(0, s, false);
        display_margin_label(0, s, " ");
    }

    /*
     * THE RIGHT COLUMN: EMPTY, SELECT, START -- top to bottom.
     *
     * Moved one slot down by the operator's direction from Start/Select/empty,
     * so the pair a thumb reaches for sits lower on the glass and the vacant
     * slot is now the TOP. All three keep their boxes, so the column still reads
     * as the same three-button stack as the other two emulators, and only slots
     * 1 and 2 have a duty: `pad_from_frame()` and `gb_app_touch()` answer for
     * those two and nothing else. The top box does nothing, which is the
     * pattern this file already accepted for the vacant slot before the move --
     * a box that does nothing is honest, where blank glass that fires a button
     * is the fault this project has reported twice.
     *
     * SET TOGETHER ON PURPOSE. A label and its duty lived in two places in this
     * function once, which is how the swapped index in paint_column() went
     * unnoticed; anything that moves a slot should move all three lines.
     */
    display_margin_label(1, 0, " ");
    display_margin_label(1, 1, "Select");
    display_margin_label(1, 2, "Start");

    /* The column slots past the drawn ones, and the whole bottom strip: blank and
     * not live, so blank glass cannot fire a button. The strip's touch area is
     * double height when an app owns the glass, so it reaches fb 375 and overlaps
     * B's own disc -- harmless only because the drawn control is tested first. */
    for (int s = 3; s < COL_N; s++)
    {
        display_margin_chrome(1, s, false);
        display_margin_label(1, s, " ");
    }

    for (int i = 0; i < BTN_N; i++)
    {
        display_margin_chrome(2, i, false);
        display_margin_btn_label(i, " ");
    }

    display_margins_set(DISPLAY_MARGINS_APP);

    /* After the margins, so the cluster lands on glass whose furniture is already
     * decided rather than under a repaint of it. */
    if (!gb_draw_controls())
        ESP_LOGW(TAG, "controls not drawn -- push refused?");

    s_free_q = xQueueCreate(GB_VBUF_N, sizeof(int));
    s_filled_q = xQueueCreate(GB_VBUF_N, sizeof(int));
    s_snap_free = xQueueCreate(GB_SRAM_SNAP_N, sizeof(int));
    s_snap_ready = xQueueCreate(GB_SRAM_SNAP_N, sizeof(int));

    if (!s_free_q || !s_filled_q || !s_snap_free || !s_snap_ready)
    {
        printf("gb: no queues\n");
        goto fail_gb;
    }

    for (int i = 0; i < GB_VBUF_N; i++)
    {
        int slot = i;
        xQueueSend(s_free_q, &slot, 0);
    }

    for (int i = 0; i < GB_SRAM_SNAP_N; i++)
    {
        int slot = i;
        xQueueSend(s_snap_free, &slot, 0);
    }

    /*
     * THE GUEST STARTS WITH ONE BUFFER ALREADY CLAIMED.
     *
     * Read into an INT and then index, because the queue carries slot numbers and
     * `s_render_target` is a POINTER. The first version passed `&s_render_target`
     * straight to xQueueReceive, which copies four bytes of slot index over a
     * four-byte pointer -- so the render target became address 0x1 and gnuboy
     * wrote its frame into whatever lives there. That is a hard fault inside the
     * guest task, on the first frame, with nothing printed because this firmware
     * builds with CONFIG_ESP_CONSOLE_NONE.
     */
    {
        int first = 0;

        if (xQueueReceive(s_free_q, &first, 0) != pdTRUE)
        {
            printf("gb: no free frame buffer at start\n");
            goto fail_gb;
        }

        s_render_target = s_fb[first];
    }

    s_frames = s_blits = s_dropped = 0;
    s_starved = 0;
    s_emu_us = s_blit_us = 0;
    s_blit_max_us = 0;
    s_blit_last_us = 0;
    s_blit_stuck_state = -1;
    s_pad = 0;
    s_run = true;

    /* THE LAST STOP'S RECORD IS CLEARED HERE, AT THE START OF A SESSION.
     *
     * It used to survive a start, so a `gb status` after a start that FAILED --
     * leaving s_run false and both task handles NULL -- reported the PREVIOUS
     * session's outcome and cost. That is how a stale `clean in 1020ms` read as
     * a fresh measurement on 2026-11-21 and briefly looked like a fourth stall.
     * An outcome belongs to one session, so it is born and dies with it. */
    s_last_stop[0] = 0;
    s_last_stop_ms = 0;
    s_stop_guest_ms = s_stop_blit_ms = s_stop_sram_ms = s_stop_save_ms = 0;

    /* Per-session, so a worst case belongs to the run that produced it. */
    display_push_stats_reset();

    /*
     * THE SAME PRIORITY AS THE CELL PAINTER, AND RAISING IT ABOVE THE PAINTER
     * HUNG THE PICTURE -- which is the reason this comment exists.
     *
     * Both tasks push to the panel through `spi_device_acquire_bus(s_spi,
     * portMAX_DELAY)`: the painter at display.c's band loop, this one through
     * display_picture_rows(). They are peers at priority 4 on core 1, and that
     * is LOAD-BEARING. At priority 5 this task preempts the painter while the
     * painter holds the bus, blocks on an acquire with an INFINITE timeout, and
     * the painter never runs again to release it. The blitter then holds a frame
     * buffer for ever, the pool runs dry, `s_render_target` goes NULL, the guest
     * renders nothing, and the glass keeps the last frame it was given.
     *
     * THE SIGNATURE IS IN THE COUNTERS AND NOWHERE ELSE: `frames=17288 blits=1165
     * dropped=0`. The guest ran the whole session, the blitter stopped after
     * about nineteen seconds, and `dropped` stayed zero because once the pool is
     * dry nothing is published to the queue that counter watches. A frozen
     * picture with a live emulator reads as a display fault; it is not one.
     *
     * WHAT THE RAISE WAS FOR IS STILL UNPROVEN. It was intended to stop blits
     * spiking to 18.9 ms against a 16.742 ms budget, attributed to painter
     * contention. Nothing established that attribution -- a long frame may simply
     * be a frame with many dirty rows -- and a hang is a far worse fault than a
     * stutter, so the bump is reverted rather than kept behind a condition.
     *
     * IF THIS IS EVER RAISED AGAIN, the bus acquisition has to change first: a
     * bounded timeout that counts misses, or one arbiter ordering both writers.
     * Priority alone cannot fix two blocking writers on one bus.
     */
    if (xTaskCreatePinnedToCore(gb_blit_task, "gbblit", 4096, NULL, 4,
                                &s_blit_task, 1) != pdPASS)
    {
        printf("gb: no blit task\n");
        s_run = false;
        goto fail_gb;
    }

    /*
     * THE GUEST TASK'S STACK COMES FROM PSRAM.
     *
     * It was 8 KB of internal DRAM, and that is the pool this board does not
     * have spare: with the desktop up and a Terminal open, internal free
     * measured 25,395 bytes with the largest block 9,728, and this allocation
     * failed with "gb: no guest task" -- the emulator would not start at all
     * from a shell. PSRAM had 7.7 MB free.
     *
     * Safe here for a specific reason rather than by luck: this task runs
     * gnuboy and nothing else. The cartridge and both frame buffers are PSRAM,
     * it opens no file, and it maps no partition -- so it never disables the
     * cache that a PSRAM stack lives behind.
     */
    if (xTaskCreatePinnedToCoreWithCaps(gb_task, "gb", 8192, NULL, 5,
                                        &s_task, 0,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS)
    {
        printf("gb: no guest task\n");
        s_run = false;
        goto fail_gb;
    }

    /* ABOVE THE BLITTER, ON CORE 1. Measured both ways: at a lower priority the
     * writer shares the core with the blitter and the same 32 KB takes 243 ms
     * instead of 121, so the picture freezes twice as long for no gain. Above it,
     * the write finishes in about 121 ms and the blitter is the one that waits --
     * which costs painted frames, not emulated ones, because the guest keeps
     * running and simply renders nothing while both buffers are held. */
    /* THE WRITER'S STACK COMES FROM PSRAM TOO, AND THE REASON THIS USED TO SAY
     * OTHERWISE WAS SIMPLY WRONG ABOUT ITS OWN CODE.
     *
     * It was pinned to internal DRAM because it "does write to /fs" -- and the
     * constraint it was reasoning about is real, because a littlefs write to the
     * `storage` partition goes through spi_flash and spi_flash disables the cache,
     * which a PSRAM stack cannot survive. But THIS TASK NEVER TOUCHES /fs: the
     * save goes to `/sd/saves/gbc/` on the card, through the SDSPI driver, which
     * is the SPI peripheral and not the flash controller. The cache stays on, so
     * a PSRAM stack is safe -- and the general rule is that the hazard is
     * INTERNAL FLASH, not files.
     *
     * It matters because internal DRAM ran out. On the build the feature was
     * written for, this allocation succeeded; the app has grown since, and on
     * 2026-11-20 `gb start` reported `gb: no save task -- running without
     * autosave`, which is a quiet way of saying NOTHING COULD BE SAVED AT ALL:
     * `gb save` waits on this task's queue, and the save on stop goes through
     * it too. A cartridge could be played and every byte of progress lost.
     *
     * If this ever does start touching internal flash, it must come back to
     * internal DRAM -- that is the one thing this comment is here to protect. */
    if (xTaskCreatePinnedToCoreWithCaps(gb_sram_task, "gbsram", 4096, NULL, 6,
                                        &s_sram_task, 1,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS)
    {
        printf("gb: no save task -- running without autosave\n");
        s_saveable = false;
    }

    printf("gb: running %s (%u bytes)\n", path, (unsigned)s_rom_len);
    printf("gb stop to leave\n");
    return true;

fail_gb:
    gnuboy_free_rom();
    gnuboy_free_bios();

fail_fb:
    for (int i = 0; i < GB_VBUF_N; i++)
    {
        if (s_fb[i])
        {
            heap_caps_free(s_fb[i]);
            s_fb[i] = NULL;
        }
    }

fail_rom:
    if (fp)
        fclose(fp);

    heap_caps_free(s_rom);
    s_rom = NULL;
    return false;
}

void gb_stop(void)
{
    if (!s_task && !s_run)
        return;

    const int64_t t0 = esp_timer_get_time();

    s_run = false;

    /* Before the tasks are waited on, so no late frame writes into a closed
     * stream -- and before the next app claims the amplifier. */
    if (s_audio)
    {
        sound_stream_close();
        s_audio = false;
    }

    for (int i = 0; i < 100 && s_task; i++)
        vTaskDelay(pdMS_TO_TICKS(10));

    const int64_t t_guest = esp_timer_get_time();

    /* Let the blitter drain the last frame and leave, so it is not writing into
     * a framebuffer that is about to be released. */
    for (int i = 0; i < 100 && s_blit_task; i++)
        vTaskDelay(pdMS_TO_TICKS(10));

    const int64_t t_blit = esp_timer_get_time();

    /*
     * IF THE BLITTER IS STILL ALIVE, NOTHING MAY BE FREED OR DELETED.
     *
     * This wait is bounded, so it can time out -- and what follows then frees
     * s_fb[] and deletes the frame queues that a live blitter still holds. That
     * is a use-after-free whose symptom arrives later and somewhere else. The
     * task cannot be reached any more, so the only safe thing to do with what it
     * holds is to leave it allocated and say so: a leak that is named beats
     * corruption that is not. gb_start() then refuses to run until a reboot, so
     * an orphan can never race a new session.
     */
    const bool blit_alive = s_blit_task != NULL;

    if (blit_alive)
    {
        s_blit_leak = true;

        /* WHERE IT WAS WHEN WE GAVE UP, captured now because the pusher cannot
         * report from inside a call that never returns. */
        s_blit_stuck_phase = display_bus_phase(&s_blit_stuck_ms);

        /* WHAT THE SCHEDULER THINKS: ready, blocked, suspended or deleted. A
         * blocked blitter on nothing, a starved one and a dead one are three
         * different faults and this is the line that tells them apart. */
        s_blit_stuck_state = (int)eTaskGetState(s_blit_task);

        s_blit_stuck_free = s_free_q ? (uint16_t)uxQueueMessagesWaiting(s_free_q) : 0;
        s_blit_stuck_filled = s_filled_q ? (uint16_t)uxQueueMessagesWaiting(s_filled_q) : 0;
        s_blit_stuck_starved = s_starved;

        /* AND HOW LONG SINCE IT ACTUALLY BLITTED, which is the real start of the
         * stall rather than the age of the shared push phase. */
        if (s_blit_last_us)
            s_blit_stuck_ms = (esp_timer_get_time() - s_blit_last_us) / 1000;

        /* AND HOW LONG SINCE THE STRIP TOUCHED THE BUS. The strip pushes once a
         * minute from its own task, so it is the one other writer that could
         * plausibly hold the blitter up. A small number here means the two
         * coincided; a large one clears it. */
        {
            uint32_t sp = 0, spms = 0;

            statusbar_push_stats(&sp, &spms, &s_blit_stuck_strip_ms);
        }

        /* TO A FILE, BECAUSE THE MIRROR IS GONE BY THIS POINT. Once a picture
         * owns the panel the USB mirror goes stale (DSP-003), so a host cannot
         * poll for this and a console print is not readable either. A run whose
         * emulator is played normally will produce the file on the card, and it
         * is still there afterwards. */
        wedge_dump();

        ESP_LOGE(TAG, "blitter did not exit; frames leaked, not freed");
    }

    s_blit_task = NULL;
    s_task = NULL;

    /*
     * THE WRITER IS WAITED FOR, AND THEN ASKED FOR ONE LAST SAVE.
     *
     * An autosave may be on the wire when the guest stops, so the task is given
     * time to drain rather than having its queues deleted underneath it. Once it
     * has exited, the guest is gone and `cart.rambanks` is stable, so the final
     * snapshot can be taken and written right here -- a 121 ms write at shutdown
     * costs nothing, which is exactly the moment to pay it.
     */
    for (int i = 0; i < 200 && (s_sram_task || s_sram_pending); i++)
        vTaskDelay(pdMS_TO_TICKS(10));

    s_sram_task = NULL;

    const int64_t t_sram = esp_timer_get_time();

    if (s_saveable && gnuboy_sram_dirty())
    {
        const size_t len = gnuboy_sram_snapshot(s_snap[0], GB_SRAM_SNAP_MAX);

        if (len && sram_write(s_save_path, s_snap[0], len) == 0)
        {
            snprintf(s_last_stop, sizeof s_last_stop, "saved");
            ESP_LOGI(TAG, "saved %s", s_save_path);
        }
        else
        {
            snprintf(s_last_stop, sizeof s_last_stop, "SAVE FAILED");
            ESP_LOGE(TAG, "could not write %s", s_save_path);
        }
    }
    else if (s_saveable)
    {
        snprintf(s_last_stop, sizeof s_last_stop, "clean");
    }
    else
    {
        snprintf(s_last_stop, sizeof s_last_stop, "no sram");
    }

    s_last_stop_ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);

    /* Each wait measured where it ends, so a stall names its own phase instead
     * of being bounded by whichever cap it happens to equal. */
    s_stop_guest_ms = (uint16_t)((t_guest - t0) / 1000);
    s_stop_blit_ms = (uint16_t)((t_blit - t_guest) / 1000);
    s_stop_sram_ms = (uint16_t)((t_sram - t_blit) / 1000);
    s_stop_save_ms = (uint16_t)((esp_timer_get_time() - t_sram) / 1000);

    if (s_snap_free)
    {
        vQueueDelete(s_snap_free);
        s_snap_free = NULL;
    }
    if (s_snap_ready)
    {
        vQueueDelete(s_snap_ready);
        s_snap_ready = NULL;
    }

    if (s_free_q)
    {
        if (!blit_alive)
        {
            vQueueDelete(s_free_q);
            s_free_q = NULL;
        }
    }
    if (s_filled_q)
    {
        if (!blit_alive)
        {
            vQueueDelete(s_filled_q);
            s_filled_q = NULL;
        }
    }

    gnuboy_free_rom();
    gnuboy_free_bios();

    /* THE FRAME BUFFERS ARE NOT FREED WHILE A BLITTER MAY STILL HOLD THEM. */
    for (int i = 0; i < GB_VBUF_N; i++)
    {
        if (s_fb[i] && !blit_alive)
        {
            heap_caps_free(s_fb[i]);
            s_fb[i] = NULL;
        }
    }

    heap_caps_free(s_rom);
    s_rom = NULL;
    s_render_target = NULL;

    /* Clear the overrides before handing the glass back: an app's labels survive
     * its exit otherwise, which is the fault entry 73 recorded. The chrome is
     * restored too, and that is not cosmetic: `s_nochrome` is state inside the
     * display driver that both the app and desktop paint paths read, so a slot
     * left bare by this app would come back as a desktop button with no box. */
    for (int i = 0; i < COL_N; i++)
    {
        display_margin_chrome(0, i, true);
        display_margin_chrome(1, i, true);
        display_margin_label(0, i, NULL);
        display_margin_label(1, i, NULL);
    }

    for (int i = 0; i < BTN_N; i++)
    {
        display_margin_chrome(2, i, true);
        display_margin_btn_label(i, NULL);
    }

    display_margin_column_slots(COL_N);
    display_margins_set(DISPLAY_MARGINS_DESKTOP);
    display_pixels_claim(false);
    display_repaint_all();

    /* THE HISTOGRAM IS WRITTEN HERE, ONCE THE SESSION IS OVER. A run too short
     * to fill a single bucket is skipped rather than dumped as a one-line
     * file, because it would say nothing and would overwrite a useful one. */
    /* ALWAYS DUMPED, even with no drops at all: "zero of 210 seconds dirty" is
     * a result, and it overwrites a previous session's file rather than leaving
     * a stale one to be misread. */
    if (s_hist_total_sec > 0)
        hist_dump();
}

bool gb_running(void)
{
    return s_task != NULL;
}

/* --- the shell command --------------------------------------------------- */

static int cmd_gb(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    const char *sub = argc >= 2 ? argv[1] : NULL;

    if (!sub || strcmp(sub, "start") == 0)
        return gb_start(argc >= 3 ? argv[2] : NULL) ? 0 : 1;

    if (strcmp(sub, "stop") == 0)
    {
        /* Read before the call, and against the same condition gb_stop() itself
         * returns on: with nothing running it changes nothing, and a stale
         * "saved" from a previous session would otherwise be reported as this
         * stop's outcome. */
        const bool was_up = gb_running() || s_run;

        gb_stop();

        printf("stopped, screen released");
        if (was_up && s_last_stop[0])
            printf(", %s in %lums", s_last_stop,
                   (unsigned long)s_last_stop_ms);
        printf("\n");
        return 0;
    }

    if (strcmp(sub, "status") == 0)
    {
        printf("running=%d frames=%lu blits=%lu dropped=%lu\n",
               gb_running() ? 1 : 0,
               (unsigned long)s_frames, (unsigned long)s_blits,
               (unsigned long)s_dropped);

        printf("audio=%s", s_audio ? "on" : "off");

        if (s_audio)
        {
            printf(" %dHz drops=%lu", GB_AUDIO_HZ,
                   (unsigned long)sound_stream_dropped());
            /* Only when it is non-zero: a healthy session's line stays the width
             * it was, and a broken channel announces itself. */
            if (sound_stream_errors())
                printf(" errs=%lu", (unsigned long)sound_stream_errors());
            /* WHICH RING THIS DROP RATE BELONGS TO. The fallback means the
             * geometry varies with what else is running, so a bare drop count
             * cannot be compared with another session's. */
            if (sound_ring_frames())
                printf(" ring=%lufr", (unsigned long)sound_ring_frames());
        }

        printf("\n");

        /* Numbers first, and in milliseconds, because the path is long enough to
         * wrap at 40 columns and microseconds made this line long enough to
         * straddle a break -- a value split across two lines is unreadable to an
         * operator and invisible to a grep. Milliseconds are also the unit these
         * two numbers are interesting in. */
        if (s_saveable)
            printf("sram saves=%lu write=%lums snap=%lums\n",
                   (unsigned long)s_sram_saves,
                   (unsigned long)(s_sram_us / 1000),
                   (unsigned long)(s_sram_snap_us / 1000));
        else
            printf("sram none\n");

        /* What the last stop saved, which is the only report a Pusher A close
         * can give: it happens on the key task, where a print is lost. */
        if (s_last_stop[0])
            printf("last stop: %s in %lums\n", s_last_stop,
                   (unsigned long)s_last_stop_ms);

        /* WHERE THAT LAST STOP SPENT ITS TIME, by phase. Three waits with round
         * caps made a total ambiguous; these three lines are what told a real
         * stall from a stale reading. */
        if (s_last_stop[0])
            printf("  guest=%ums blit=%ums sram=%ums save=%ums\n",
                   (unsigned)s_stop_guest_ms, (unsigned)s_stop_blit_ms,
                   (unsigned)s_stop_sram_ms, (unsigned)s_stop_save_ms);

        /* THE DISPLAY DRIVER'S OWN NUMBERS. buswait is the worst single wait for
         * the SPI panel bus, in milliseconds, and miss is how many bands were
         * skipped because another writer held it. The top strip pushes on its
         * own task once a minute, so a buswait that reaches hundreds names the
         * bus as the cause of a stall rather than leaving it to a theory. */
        {
            uint32_t miss = 0, waitmax = 0;

            display_bus_stats(&miss, &waitmax);
            printf("bus: waitmax=%lums miss=%lu\n",
                   (unsigned long)waitmax, (unsigned long)miss);
        }

        if (s_blit_leak)
            printf("blitter wedged -- frames leaked, reboot to clear\n");

        if (s_blit_leak)
            printf("  stuck: phase=%u for %lldms state=%d\n",
                   (unsigned)s_blit_stuck_phase, (long long)s_blit_stuck_ms,
                   s_blit_stuck_state);

        if (s_blit_leak)
            printf("  pool was free=%u filled=%u starved=%lu\n",
                   (unsigned)s_blit_stuck_free, (unsigned)s_blit_stuck_filled,
                   (unsigned long)s_blit_stuck_starved);

        if (gb_running())
            printf("emu=%luus blit=%luus max=%luus pad=%02x\n",
                   (unsigned long)s_emu_us, (unsigned long)s_blit_us,
                   (unsigned long)s_blit_max_us, (unsigned)s_pad);

        /* THE POOL, which is the only other way blits stop while frames keep
         * coming: a guest that cannot get a buffer renders nothing, and a
         * blitter waiting for a frame looks exactly like a dead one. */
        printf("pool: free=%u filled=%u starved=%lu\n",
               s_free_q ? (unsigned)uxQueueMessagesWaiting(s_free_q) : 0,
               s_filled_q ? (unsigned)uxQueueMessagesWaiting(s_filled_q) : 0,
               (unsigned long)s_starved);

        return 0;
    }

    if (strcmp(sub, "rsave") == 0)
    {
        /* Repeat writes, for measuring what one costs. Each pass rewrites the
         * whole file, so the reported microseconds are a distribution rather
         * than one first-write artefact. The writes are queued to the guest
         * task, one per frame, because doing them here would be a second writer
         * on the banks the guest is still writing. */
        if (!gb_running() || !s_saveable)
        {
            printf("gb: nothing to measure\n");
            return 1;
        }

        s_sram_log_n = 0;
        s_sram_req = GB_SRAM_LOG;

        for (int i = 0; i < 500 && (s_sram_req > 0 || s_sram_pending); i++)
            vTaskDelay(pdMS_TO_TICKS(10));

        for (int i = 0; i < s_sram_log_n; i++)
            printf("rsave %d: %luus\n", i, (unsigned long)s_sram_log[i]);

        return 0;
    }

    if (strcmp(sub, "save") == 0)
    {
        if (!gb_running())
        {
            printf("gb: not running\n");
            return 1;
        }

        if (!s_saveable)
        {
            printf("gb: this cartridge has no save RAM\n");
            return 1;
        }

        /* Queued, then waited for: the guest task snapshots and the writer task
         * writes, so this can neither do the work itself nor honestly claim it is
         * done before both have finished. */
        s_sram_rc = -2;                  /* not attempted yet                */
        s_sram_log_n = 0;
        s_sram_req = 1;

        for (int i = 0; i < 600 && (s_sram_req > 0 || s_sram_pending); i++)
            vTaskDelay(pdMS_TO_TICKS(10));

        if (s_sram_req > 0 || s_sram_pending)
        {
            printf("gb: save still queued, the guest is not reaching its loop\n");
            return 1;
        }

        if (s_sram_rc == 0)
        {
            printf("gb: saved %s (%luus write, %luus snapshot)\n", s_save_path,
                   (unsigned long)s_sram_us, (unsigned long)s_sram_snap_us);
            return 0;
        }

        printf("gb: save FAILED writing %s\n", s_save_path);
        return 1;
    }

    printf("gb [start [rom] | stop | save | rsave | status]\n");
    return 1;
}

static const tdsh_command_t s_gb_cmd = {
    "gb",
    "gb [start [rom] | stop | save | rsave | status]",
    "Game Boy / Game Boy Color emulator",
    cmd_gb,
    TDSH_CMD_INTERACTIVE,
};

bool gb_register_command(void)
{
    /* THE KEY HOOK IS SET ONCE, HERE, and not per app, for the reason pwrkey.h
     * gives: a registration torn down with the app leaves the driver holding a
     * pointer to something that has freed itself. It answers false while no
     * guest is running, so the desktop keeps its Enter and nothing else has to
     * know this exists. */
    pwrkey_set_app_key(gb_app_key, NULL);

    if (!touch_add_app_touch(gb_app_touch, NULL))
        ESP_LOGW(TAG, "gb touch consumer did not register");

    return tdsh_register_command(&s_gb_cmd) == 0;
}
