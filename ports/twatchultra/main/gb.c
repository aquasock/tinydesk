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
#include "gamepad.h"
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

        const int64_t t0 = esp_timer_get_time();

        /* The cache belongs to this task alone, so a band boundary that lands
         * mid-row cannot leave a stale expansion behind. */
        s_xrow_sy = -1;

        display_picture_rows(s_fb[slot], GB_OX, GB_OY,
                             GB_WIDTH * GB_SCALE, GB_HEIGHT * GB_SCALE,
                             gb_fill_row);

        s_blit_us = (uint32_t)(esp_timer_get_time() - t0);
        s_blits++;

        xQueueSend(s_free_q, &slot, 0);
    }

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
            s_render_target = NULL;  /* renders nothing until one comes back */
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
static void gb_audio_cb(void *buffer, size_t length)
{
    if (!s_audio || !buffer || !length)
        return;

    if (length > GB_AUDIO_BUF)
        length = GB_AUDIO_BUF;

    sound_stream_write((const int16_t *)buffer, (int)length);
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
        const int64_t t0 = esp_timer_get_time();

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
            if (f->p[i].index == 0) pad |= GB_PAD_START;
            if (f->p[i].index == 1) pad |= GB_PAD_SELECT;
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

            if (f->p[i].region == EDGE_RIGHT && f->p[i].index < 2 &&
                display_margin_slot_live(1, f->p[i].index))
                used = true;
        }
    }

    s_pad = pad_from_frame(f);
    return used;
}

/* --- start / stop -------------------------------------------------------- */

bool gb_start(const char *rom_path)
{
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

    display_margin_label(1, 0, "Start");
    display_margin_label(1, 1, "Select");

    /*
     * SLOT 2 KEEPS ITS BOX, WITH NO WORD ON IT. This port reserves three column
     * slots and draws all three, so the column reads as the same three-button stack
     * as the other two emulators; the Game Boy has only two margin controls, so the
     * third is furniture rather than a duty. It is NOT live -- `pad_from_frame()`
     * answers only for slots 0 and 1 -- and a box that does nothing is honest, where
     * blank glass that fires a button is the fault this project reported twice.
     *
     * THIS SLOT IS WHY THE SWAPPED INDEX IN paint_column() MATTERED. Before that was
     * fixed, the painter consulted the LEFT column's chrome state for the right one,
     * so this slot read the left column's slot 2 -- which carries "Save" and a box --
     * and drew a box nobody had asked for. Fixing the index made the column honour
     * its own state and the box vanished, which the operator noticed immediately.
     * The box is therefore requested here explicitly rather than left to a bug: the
     * line below is what says so, and slots 3 and 4 stay bare.
     */
    display_margin_chrome(1, 2, true);
    display_margin_label(1, 2, " ");

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
    s_emu_us = s_blit_us = 0;
    s_pad = 0;
    s_run = true;

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

    /* Let the blitter drain the last frame and leave, so it is not writing into
     * a framebuffer that is about to be released. */
    for (int i = 0; i < 100 && s_blit_task; i++)
        vTaskDelay(pdMS_TO_TICKS(10));

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

    if (s_saveable && gnuboy_sram_dirty())
    {
        const size_t len = gnuboy_sram_snapshot(s_snap[0], GB_SRAM_SNAP_MAX);

        if (len && sram_write(s_save_path, s_snap[0], len) == 0)
            printf("gb: saved %s\n", s_save_path);
        else
            printf("gb: could not write %s\n", s_save_path);
    }

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
        vQueueDelete(s_free_q);
        s_free_q = NULL;
    }
    if (s_filled_q)
    {
        vQueueDelete(s_filled_q);
        s_filled_q = NULL;
    }

    gnuboy_free_rom();
    gnuboy_free_bios();

    for (int i = 0; i < GB_VBUF_N; i++)
    {
        if (s_fb[i])
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
        gb_stop();
        printf("stopped, screen released\n");
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

        if (gb_running())
            printf("emu=%luus blit=%luus pad=%02x\n",
                   (unsigned long)s_emu_us, (unsigned long)s_blit_us,
                   (unsigned)s_pad);

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
    if (!touch_add_app_touch(gb_app_touch, NULL))
        ESP_LOGW(TAG, "gb touch consumer did not register");

    return tdsh_register_command(&s_gb_cmd) == 0;
}
