/*
 * gbcbench.c - the Game Boy Color core benchmark, on the watch, as a shell command.
 *
 * THE ONE QUESTION THIS ANSWERS. docs/gbc-scoping.md in Escapement chose gnuboy
 * over binjgb on a host measurement (1.54-1.79x faster, and it emits pixels in
 * the panel's byte order so no conversion is needed), then projected 28-45 fps
 * onto the watch by normalising through the NES interpreter. That projection is
 * arithmetic, not measurement, and it was made BEFORE this firmware gained -O2,
 * 120 MHz PSRAM and flash, and the 32 KB instruction cache -- so the band is
 * stale and nobody knows which end of it is true.
 *
 * This replaces the band with a number. If the watch clears about 50 fps, a
 * Game Boy Color port is worth building for a picture that fills the glass
 * instead of floating in it; if it lands near 37, the Z80 interpreter needs work
 * first and it is worth knowing that before a port is written.
 *
 * WHY IT REPORTS THREE FIGURES RATHER THAN ONE. `run_us` is the whole of what a
 * port would pay per frame on the emulation side. `frame_us` adds one blit at
 * the size a 2x port would push, using the cost per pixel the NES port measured,
 * which is what the player actually feels. And `ratio` against the Game Boy's own
 * 59.73 Hz refresh says whether it is real time. A single "fps" hides which of
 * those moved.
 *
 * THE BLIT IS MODELLED, NOT PERFORMED. A real port pushes 320x288 through
 * display_picture_rows() on core 1. This command does not claim the screen -- it
 * must not, because it runs from a shell that owns the terminal -- so it
 * multiplies the measured pixels-per-microsecond instead. That is an estimate and
 * is labelled as one in the output. The emulation figure beside it is not.
 *
 * WHAT IS NOT MEASURED, deliberately: audio. gnuboy's `sound.c` is compiled but
 * the APU needs an external implementation to produce samples (minigb_apu is the
 * usual one), so there is nothing to measure yet, and the NES port shows that
 * audio is a few hundred microseconds a frame once it exists.
 */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdpaths.h"
#include "tdsh.h"

#include "gnuboy.h"

#define GB_FRAME_US     16742        /* 1e6 / 59.7275, the Game Boy's own refresh */
#define GB_BUDGET_US    16639.0      /* the NES port's frame budget, for comparison */

/* The NES port pushes 61,440 pixels in about 4,004 us with the blit on core 1,
 * which is 0.0652 us per pixel. A 2x GBC frame is 92,160 pixels, and unlike the
 * NES it needs no palette conversion. */
#define NES_PIXELS      61440
#define NES_PUSH_US     4004.0

/* Scratch for the frame gnuboy renders into. Allocated once, reused across runs.
 * PSRAM because nothing reads it in this command -- a real port would want it in
 * internal RAM for the same reason the NES blit does, and that difference is
 * already priced into the NES push figure the blit model uses. */
static uint16_t *s_fb;

/* gnuboy calls this once per rendered frame. Counting here rather than around
 * gnuboy_run() distinguishes "the core ran" from "the core rendered", which are
 * the same thing only if the framebuffer was installed correctly. */
static volatile uint32_t s_render_calls;

static void video_cb(void *buffer)
{
    (void)buffer;
    s_render_calls++;
}

static int run_one(const char *path, int frames, bool cpu_only)
{
    if (gnuboy_init(44100, GB_AUDIO_MONO_S16,
                    GB_PIXEL_565_BE, cpu_only ? NULL : video_cb, NULL) != 0)
    {
        printf("gbcbench: gnuboy_init failed\n");
        return -1;
    }

    gnuboy_set_hwtype(GB_HW_CGB);
    gnuboy_set_palette(GB_PALETTE_CGB);

    /*
     * THE FRAMEBUFFER MUST BE INSTALLED BEFORE THE ROM RUNS, and a NULL one is
     * supported but changes what is measured: lcd.c skips its line rendering
     * entirely, so the run becomes a CPU-only figure wearing a frame rate. That is
     * why `cpu` is a separate mode with its own label rather than the default --
     * the two numbers are both useful and they are not the same number.
     */
    gnuboy_set_framebuffer(cpu_only ? NULL : s_fb);

    if (gnuboy_load_rom_file(path) != 0)
    {
        printf("gbcbench: cannot load %s\n", path);
        return -1;
    }

    /*
     * THE RESET IS NOT OPTIONAL and omitting it is a segfault, not a wrong answer.
     * With no boot ROM loaded -- which is supported, and is what real hardware
     * does once the boot ROM hands over -- `gb_cpu_reset()` decides
     * `PC = GB.bios ? 0 : 0x100`, and that decision only happens inside the reset.
     * Skip it and PC stays 0 in the zeroed state, the first fetch goes through
     * address 0, and gb_hw_read() dereferences a NULL BIOS pointer. The host
     * harness died exactly this way before the line was added.
     */
    gnuboy_reset(true);

    /* Let the boot sequence settle, so the first measured frame is a real one and
     * not the cartridge's entry code still warming up. */
    for (int i = 0; i < 30; i++)
        gnuboy_run(!cpu_only);

    s_render_calls = 0;

    const int64_t t0 = esp_timer_get_time();

    for (int i = 0; i < frames; i++)
        gnuboy_run(!cpu_only);

    const int64_t dt = esp_timer_get_time() - t0;

    gnuboy_free_rom();
    gnuboy_free_bios();

    const double per_frame = (double)dt / (double)frames;
    const double fps = 1e6 / per_frame;
    const double realtime = (double)GB_FRAME_US / per_frame;

    /* The blit a 2x port would push, priced from the NES port's measured cost per
     * pixel. Labelled as a model in the output because it is one. */
    const double blit_us = NES_PUSH_US * ((double)(320 * 288) / (double)NES_PIXELS);
    const double frame_us = per_frame > blit_us ? per_frame : blit_us;

    printf("rom=%s\n", path);
    printf("frames=%d run_us=%.0f fps=%.1f realtime=%.3f\n",
           frames, per_frame, fps, realtime);
    printf("renders=%lu\n", (unsigned long)s_render_calls);
    printf("blit_model=%.0fus frame_us=%.0f frame_fps=%.1f\n",
           blit_us, frame_us, 1e6 / frame_us);

    /*
     * THE RESULT IS ALSO WRITTEN TO THE CARD, AND THAT IS NOT A CONVENIENCE.
     *
     * Loading a Game Boy cartridge prints one line per bank, several hundred of
     * them, and this panel shows 26 rows with no scrollback -- so the four result
     * lines are pushed off the glass by the time they are printed and cannot be
     * read back. The USB mirror does not help: it carries the same floods and a
     * capture only ever sees its tail.
     *
     * Same shape as gbhist.txt and gbwedge.txt: an instrument whose answer must
     * survive the screen it was printed on.
     */
    {
        const char *out = sd_out("/sd/diag", "gbcbench.txt");
        FILE *f = fopen(out, "w+");

        if (f)
        {
            fprintf(f, "rom=%s\n", path);
            fprintf(f, "mode=%s frames=%d\n", cpu_only ? "cpu" : "render", frames);
            fprintf(f, "run_us=%.0f fps=%.1f realtime=%.3f\n",
                    per_frame, fps, realtime);
            fprintf(f, "renders=%lu\n", (unsigned long)s_render_calls);
            fprintf(f, "blit_model=%.0fus frame_us=%.0f frame_fps=%.1f\n",
                    blit_us, frame_us, 1e6 / frame_us);
            fprintf(f, "heap_internal_free=%u\n",
                    (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
            fprintf(f, "heap_internal_min=%u\n",
                    (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
            fclose(f);
            printf("gbcbench: %s\n", out);
        }
    }

    return 0;
}

static int cmd_gbcbench(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    const char *sub = argc >= 2 ? argv[1] : NULL;

    if (sub && strcmp(sub, "info") == 0)
    {
        printf("core=gnuboy (retro-go fork, GPL-2.0)\n");
        printf("frame_us_realtime=%.0f\n", (double)GB_FRAME_US);
        printf("budget_us=%.0f  (the NES port's frame, for comparison)\n", GB_BUDGET_US);
        printf("pixels_2x=%d  nes_pixels=%d\n", 320 * 288, NES_PIXELS);
        printf("fb=%p\n", (void *)s_fb);
        return 0;
    }

    if (!sub || strcmp(sub, "help") == 0)
    {
        printf("gbcbench run ROM [frames] [cpu]\n");
        printf("  runs gnuboy on the watch and reports us/frame, fps and realtime\n");
        printf("  'cpu' skips rendering, which prices the interpreter alone\n");
        printf("gbcbench info\n");
        return sub ? 0 : 1;
    }

    if (strcmp(sub, "run") != 0)
    {
        printf("gbcbench run ROM [frames] [cpu]\n");
        return 1;
    }

    if (argc < 3)
    {
        printf("gbcbench: need a ROM path\n");
        return 1;
    }

    const int frames = argc >= 4 ? atoi(argv[3]) : 600;
    const bool cpu_only = argc >= 5 && strcmp(argv[4], "cpu") == 0;

    if (frames < 1 || frames > 20000)
    {
        printf("gbcbench: frames must be 1..20000\n");
        return 1;
    }

    if (!s_fb)
    {
        s_fb = (uint16_t *)heap_caps_malloc((size_t)GB_WIDTH * GB_HEIGHT * sizeof(uint16_t),
                                            MALLOC_CAP_SPIRAM);

        if (!s_fb)
        {
            printf("gbcbench: no PSRAM for the %u-byte frame buffer\n",
                   (unsigned)(GB_WIDTH * GB_HEIGHT * sizeof(uint16_t)));
            return 1;
        }
    }

    printf("gbcbench: %d frames, %s\n", frames,
           cpu_only ? "CPU only (no rendering)" : "rendering on");

    return run_one(argv[2], frames, cpu_only) == 0 ? 0 : 1;
}

static const tdsh_command_t s_gbcbench_cmd = {
    "gbcbench",
    "gbcbench run ROM [frames] [cpu] | info",
    "Measure Game Boy Color frames/sec on this board (gnuboy)",
    cmd_gbcbench,
    TDSH_CMD_INTERACTIVE,
};

bool gbcbench_register_command(void)
{
    return tdsh_register_command(&s_gbcbench_cmd) == 0;
}
