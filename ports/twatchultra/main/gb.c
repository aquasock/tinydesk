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

#include "display.h"
#include "edges.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
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

        next += GB_FRAME_US;
        const int64_t now = esp_timer_get_time();

        if (now < next)
            vTaskDelay(pdMS_TO_TICKS((next - now + 999) / 1000));
        else
            next = now;      /* behind: drop the backlog, never burst */
    }

    s_task = NULL;
    vTaskDelete(NULL);
}

/* --- input --------------------------------------------------------------- */

/*
 * The drawn and drawn-adjacent controls, as a Game Boy pad.
 *
 * gnuboy's gb_padbtn_t is active-HIGH bit flags, unlike the console's own
 * active-low register, so this builds the mask gnuboy expects rather than the
 * one the hardware reads.
 *
 * A and B have no drawn circles on this port -- the Game Boy's two face buttons
 * are where the NES's were, on the bottom strip -- because a 320-wide picture
 * leaves no room below it for a control cluster the way the NES's 256-wide
 * picture does. The margins are the whole controller, which is what the Pokemon
 * Mini did before its d-pad was drawn.
 */
static int pad_of_region(edge_region_t region, int index)
{
    if (region == EDGE_BOTTOM)
    {
        if (index == 0) return GB_PAD_B;
        if (index == 1) return GB_PAD_A;
        if (index == 2) return GB_PAD_START;
    }
    else if (region == EDGE_RIGHT)
    {
        if (index == 0) return GB_PAD_UP;
        if (index == 1) return GB_PAD_DOWN;
        if (index == 2) return GB_PAD_LEFT;
        if (index == 3) return GB_PAD_RIGHT;
        if (index == 4) return GB_PAD_SELECT;
    }

    return 0;
}

static bool gb_app_touch(void *user, const touch_frame_t *f)
{
    (void)user;

    if (!s_run)
        return false;

    int pad = 0;
    bool used = false;

    if (f)
    {
        for (int i = 0; i < f->n && i < TOUCH_POINTS; i++)
        {
            const int bit = pad_of_region(f->p[i].region, f->p[i].index);

            if (!bit)
                continue;

            const int col = f->p[i].region == EDGE_RIGHT ? 1 : 2;

            if (!display_margin_slot_live(col, f->p[i].index))
                continue;

            pad |= bit;
            used = true;
        }
    }

    s_pad = pad;
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
     * CGB HARDWARE FOR EVERY CARTRIDGE, including DMG ones, because that is what
     * a Game Boy Color does with them: a DMG cartridge in a CGB runs in
     * compatibility mode and gets the CGB's palette selection. The header at
     * 0x143 says 0x80 for "compatible" and 0xC0 for "only", so the core honours
     * the cartridge either way; what this sets is the machine, not the game.
     */
    gnuboy_set_hwtype(GB_HW_CGB);
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
     * ALL FIVE COLUMN SLOTS, unlike the NES and the Pokemon Mini.
     *
     * Those two draw a d-pad and circles on the framebuffer and give the columns
     * only three slots so the region below belongs to the drawn controls. This
     * port draws nothing -- the picture is 320 wide and fills the glass edge to
     * edge, leaving no framebuffer rows for a control cluster of the kind the
     * NES's 256-wide picture leaves 176 of. So the margins are the WHOLE
     * controller and every slot is used: the right column is the d-pad plus
     * Select, and the bottom strip is B, A and Start.
     *
     * The first version asked for three slots out of habit, which would have made
     * Right and Select unreachable -- the slots existed in the mapping and had no
     * glass to be touched on.
     */
    display_margin_column_slots(COL_N);

    display_margin_label(0, 0, "Pause");
    display_margin_label(0, 1, "Reset");
    display_margin_label(0, 2, "Save");
    display_margin_label(0, 3, " ");
    display_margin_label(0, 4, " ");

    display_margin_label(1, 0, "Up");
    display_margin_label(1, 1, "Down");
    display_margin_label(1, 2, "Left");
    display_margin_label(1, 3, "Right");
    display_margin_label(1, 4, "Select");

    display_margin_btn_label(0, "B");
    display_margin_btn_label(1, "A");
    display_margin_btn_label(2, "Start");

    display_margins_set(DISPLAY_MARGINS_APP);

    s_free_q = xQueueCreate(GB_VBUF_N, sizeof(int));
    s_filled_q = xQueueCreate(GB_VBUF_N, sizeof(int));

    if (!s_free_q || !s_filled_q)
    {
        printf("gb: no queues\n");
        goto fail_gb;
    }

    for (int i = 0; i < GB_VBUF_N; i++)
    {
        int slot = i;
        xQueueSend(s_free_q, &slot, 0);
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

    if (xTaskCreatePinnedToCore(gb_task, "gb", 8192, NULL, 5,
                                &s_task, 0) != pdPASS)
    {
        printf("gb: no guest task\n");
        s_run = false;
        goto fail_gb;
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
     * its exit otherwise, which is the fault entry 73 recorded. */
    for (int i = 0; i < COL_N; i++)
    {
        display_margin_label(0, i, NULL);
        display_margin_label(1, i, NULL);
    }

    for (int i = 0; i < BTN_N; i++)
        display_margin_btn_label(i, NULL);

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
            printf(" %dHz drops=%lu", GB_AUDIO_HZ,
                   (unsigned long)sound_stream_dropped());

        printf("\n");

        if (gb_running())
            printf("emu=%luus blit=%luus pad=%02x\n",
                   (unsigned long)s_emu_us, (unsigned long)s_blit_us,
                   (unsigned)s_pad);

        return 0;
    }

    printf("gb [start [rom] | stop | status]\n");
    return 1;
}

static const tdsh_command_t s_gb_cmd = {
    "gb",
    "gb [start [rom] | stop | status]",
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
