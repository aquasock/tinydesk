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
#include "touch.h"

/* nofrendo's own headers. Included through the component's exported include
 * path, which is why the component lists "." and "nes" in INCLUDE_DIRS. */
#include "nes/nes.h"
#include "palettes.h"

static const char *TAG = "nes";

/* --- state ---------------------------------------------------------------- */

#define NES_STACK       8192
#define NES_PRIO        3
/* The NES runs at 60.0988 Hz on NTSC. Pacing to the core's own refresh rate
 * rather than to a round 60 keeps audio pitch and game speed right -- the
 * difference is 0.16 percent, inaudible on its own, but it accumulates: a
 * minute of play would drift about a tenth of a second against anything else
 * keeping time. */
#define NES_FRAME_US    (1000000.0 / 60.0988)

static nes_t *s_nes;
static uint8_t *s_vidbuf;          /* PSRAM, 272x240 palette indices */
static uint16_t *s_row;            /* one converted row, for the blit      */
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

/* The blit's own duration, picked up by the task loop after nes_emulate returns.
 * A file-scope variable rather than a return value because the core calls the
 * blit through a function pointer. */
static volatile uint32_t s_us_blit_last;

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
}

/* --- the blit ------------------------------------------------------------- */

/*
 * Called once per frame by the core with the finished 272x240 index buffer.
 *
 * THE OVERDRAW IS SKIPPED, not copied. `NES_SCREEN_PITCH` is 8 + 256 + 8 because
 * the PPU writes eight pixels of slop on each side for a sprite that starts
 * there; presenting that would show a 272-pixel-wide picture and put the game
 * eight pixels off centre. So the source pointer is advanced by
 * `NES_SCREEN_OVERDRAW` on every row and only 256 pixels are converted.
 *
 * The conversion is a LUT copy into a row buffer, then one blit of the whole
 * image. Building the image in a scratch buffer and calling display_picture_rgb()
 * once per FRAME rather than once per ROW matters: each call takes the
 * framebuffer lock, reprograms the panel window registers and pushes over QSPI,
 * so sixteen calls per frame would be sixteen window setups for one picture.
 */
static void nes_blit(uint8_t *buf)
{
    if (!buf || !s_row)
        return;

    int64_t b0 = esp_timer_get_time();

    for (int y = 0; y < NES_SCREEN_HEIGHT; y++)
    {
        const uint8_t *src = NES_SCREEN_GETPTR(buf, 0, y);
        uint16_t *dst = s_row + (size_t)y * NES_SCREEN_WIDTH;

        for (int x = 0; x < NES_SCREEN_WIDTH; x++)
            dst[x] = s_pal[src[x]];
    }

    display_picture_rgb(NULL, NES_OX, NES_OY, s_row, NES_SCREEN_WIDTH, NES_SCREEN_HEIGHT);

    s_us_blit_last = (uint32_t)(esp_timer_get_time() - b0);
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
        input_update(0, s_pad);

        int64_t t0 = esp_timer_get_time();


        nes_emulate(true);
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
     * THE FRAME BUFFER, FROM PSRAM. 272x240 is 65,280 bytes and it is the single
     * largest thing this component needs; internal DRAM on this board idles near
     * 66 KB free and tdsh wants 65,536 of it for the SSH server, so a static
     * array here would break a peripheral that has nothing to do with the NES.
     * That is not hypothetical -- it is EMU-018, learned with the Tamagotchi's
     * 4 KB of dirty-row tables.
     *
     * Set AFTER loading, always: nes_loadfile() calls nes_reset(), which nulls
     * vidbuf, so a buffer installed beforehand is silently discarded and the PPU
     * renders nothing while the emulator reports a healthy frame rate. That cost
     * two benchmark runs on the host (NES-001).
     */
    s_vidbuf = (uint8_t *)heap_caps_malloc((size_t)NES_SCREEN_PITCH * NES_SCREEN_HEIGHT,
                                           MALLOC_CAP_SPIRAM);

    if (!s_vidbuf)
    {
        printf("no psram for the frame buffer\n");
        return false;
    }

    /* One converted frame, so the blit is one call per frame rather than one per
     * scanline. Also PSRAM: 120 KB. */
    if (!s_row)
    {
        s_row = (uint16_t *)heap_caps_malloc((size_t)NES_SCREEN_WIDTH * NES_SCREEN_HEIGHT *
                                             sizeof(uint16_t), MALLOC_CAP_SPIRAM);

        if (!s_row)
        {
            heap_caps_free(s_vidbuf);
            s_vidbuf = NULL;
            printf("no psram for the converted frame\n");
            return false;
        }
    }

    s_nes->vidbuf = NULL;   /* in case a previous run left it set */

    if (nes_loadfile(s_rom_path) < 0)
    {
        printf("unable to load %s (unsupported mapper?)\n", s_rom_path);
        heap_caps_free(s_vidbuf);
        s_vidbuf = NULL;
        return false;
    }

    /* AFTER the load, per the note above. */
    nes_setvidbuf(s_vidbuf);

    build_palette();

    /* Claim the screen and clear it, exactly as the Tamagotchi does: claiming
     * stops the painter, clearing removes what the console left behind. */
    display_pixels_claim(true);

    if (!display_fill(0x0000))
    {
        display_pixels_claim(false);
        heap_caps_free(s_vidbuf);
        s_vidbuf = NULL;
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
    s_pad = 0;
    s_paused = false;
    s_run = true;

    if (xTaskCreate(nes_task, "nes", NES_STACK, NULL, NES_PRIO, &s_task) != pdPASS)
    {
        s_run = false;
        s_task = NULL;
        display_margins_set(DISPLAY_MARGINS_DESKTOP);
        display_pixels_claim(false);
        heap_caps_free(s_vidbuf);
        s_vidbuf = NULL;
        printf("no task\n");
        return false;
    }

    ESP_LOGI(TAG, "running %s", s_rom_path);
    return true;
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

    nes_shutdown();
    s_nes = NULL;

    heap_caps_free(s_vidbuf);
    s_vidbuf = NULL;

    /* s_row is kept between runs: it is a fixed 120 KB that never changes size,
     * and reallocating it on every start would fragment PSRAM for no benefit. */

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

        return 0;
    }

    const char *rom = NULL;

    if (argc >= 2 && strcmp(sub, "start") == 0)
        rom = argc >= 3 ? argv[2] : NULL;
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
    "nes [start [rom] | stop | pause | status]",
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
