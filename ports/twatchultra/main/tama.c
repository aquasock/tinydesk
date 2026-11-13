/*
 * tama.c - the Tamagotchi emulator, on the watch, as a shell command.
 *
 * The emulator itself is components/tamacore (tmc_core), which is the same code
 * the host harness runs. This file is the whole of what a board adds:
 *
 *   * a ROM, read from storage into PSRAM;
 *   * a 300x308 RGB565 frame in PSRAM (184,800 bytes -- it cannot be static:
 *     256 KB of .bss overflowed internal DRAM by 88,896 bytes when the test
 *     patterns tried it, and everything static in this component lands in
 *     internal DRAM by construction, see extram.lf);
 *   * ownership of the screen, so the desktop's painter does not repaint cells
 *     over the guest;
 *   * a task pacing the guest at its own 16 Hz and pushing the rows that
 *     changed.
 *
 * THE PACING IS IN GUEST CYCLES, NOT WALL-CLOCK FRAMES. tmc_run() takes a
 * delta and returns an absolute count, and the guest's display only moves when
 * the guest's own timers fire. Running a fixed 250,000 cycles per iteration is
 * what every other consumer of this emulator does and it is what makes the
 * animations run at the speed the artwork was authored for; if the board cannot
 * keep up, the delay below resyncs rather than accumulating a burst.
 *
 * WHAT THIS FILE DOES NOT DO YET: touch. The panel's reports still go to the
 * desktop as a mouse. Buttons are reachable from the shell (`tama press 0`) so
 * the guest's own input path can be proven before the panel is wired to it.
 */
#include "tama.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "display.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tdsh.h"
#include "tmc_core.h"

static const char *TAG = "tama";

#define TAMA_STACK      6144
#define TAMA_PRIO       3
#define TAMA_FRAME_US   (1000000 / TMC_FRAME_HZ)   /* 62500 */
#define TAMA_SAVE_PATH  "/sd/tama.sav"
#define TAMA_REPORT     "/sd/tama.txt"

static tmc *s_m;
static uint8_t *s_rom;
static uint16_t *s_frame;
static TaskHandle_t s_task;
static volatile bool s_run;
static uint32_t s_frames;
static uint32_t s_blits;
static char s_rom_path[64] = TAMA_ROM_DEFAULT;

/* --- report ---------------------------------------------------------------
 *
 * The USB mirror goes stale as soon as a bitmap owns the panel, and the desktop
 * log ring is 40 columns and clips. So the numbers that answer "is it running"
 * go to the card as well, where they can be read from the host over SFTP while
 * the guest is on the glass.
 */
static void report(const char *what, const char *extra)
{
    uint64_t cyc = tmc_cycles(s_m);
    FILE *f = fopen(TAMA_REPORT, "w");

    if (!f)
        return;

    fprintf(f, "%s%s%s frames=%lu blits=%lu reset=%d\n", what, extra ? " " : "",
            extra ? extra : "", (unsigned long)s_frames, (unsigned long)s_blits,
            (int)esp_reset_reason());
    /* Cycles, and guest seconds derived from them -- not wall-clock seconds.
     * The two are the same only while the board keeps up with 16 frames a
     * second, so printing both would invite the wrong inference; the caller
     * can compare this against uptime itself. */
    if (s_m)
        fprintf(f, "cycles=%llu guest_s=%llu.%03llu\n",
                (unsigned long long)cyc,
                (unsigned long long)(cyc / TMC_CLOCK_HZ),
                (unsigned long long)((cyc % TMC_CLOCK_HZ) / 1000));
    fclose(f);
}

/* --- the guest task ------------------------------------------------------ */

static void tama_task(void *arg)
{
    (void)arg;
    int64_t next = esp_timer_get_time();

    while (s_run)
    {
        int y_lo = 0, y_hi = 0;

        tmc_run(s_m, TMC_FRAME_CYCLES);

        /* Dirty rows only. An icon flip is one band; a blank frame is nothing
         * at all and costs no SPI traffic. */
        if (tmc_frame(s_m, &y_lo, &y_hi) > 0)
        {
            display_picture_rgb(NULL, TAMA_OX, TAMA_OY + y_lo,
                                s_frame + (size_t)y_lo * TMC_FRAME_W,
                                TMC_FRAME_W, y_hi - y_lo);
            s_blits++;
        }
        s_frames++;

        /* A progress file, rewritten every 64 frames (4 s). This exists to
         * answer one question that nothing else on this board can: when the
         * guest is on the glass, the mirror is stale and a panic prints into
         * nothing (CONFIG_ESP_CONSOLE_NONE), so a crash looks exactly like a
         * working emulator until somebody touches it. A file that stops
         * advancing says how far it got, and survives the reboot. */
        if ((s_frames & 63) == 0)
            report("run", NULL);

        next += TAMA_FRAME_US;
        int64_t now = esp_timer_get_time();

        if (now < next)
        {
            /* Tick is 1000 Hz, so round up: a 0 ms delay would spin. */
            vTaskDelay(pdMS_TO_TICKS((next - now + 999) / 1000));
        }
        else
        {
            /* Behind. Drop the backlog rather than running frames back to back
             * to catch up -- the guest is a real-time simulation and a burst
             * would move it further ahead of the wall clock, not less. */
            next = now;
        }
    }

    s_task = NULL;
    vTaskDelete(NULL);
}

/* --- ROM ----------------------------------------------------------------- */

/*
 * Read exactly TMC_ROM_BYTES into PSRAM.
 *
 * The 512 KiB image is the part of the file the emulator executes; a larger
 * file (the reconstructed TamaGo image is 640 KiB) still yields the same image
 * from its first 512 KiB, and a shorter one is refused by tmc_new's signature
 * check rather than silently padded.
 */
static uint8_t *read_rom(const char *path)
{
    FILE *f = fopen(path, "rb");

    if (!f)
        return NULL;

    uint8_t *buf = (uint8_t *)heap_caps_malloc(TMC_ROM_BYTES, MALLOC_CAP_SPIRAM);

    if (!buf)
    {
        fclose(f);
        return NULL;
    }

    size_t n = fread(buf, 1, TMC_ROM_BYTES, f);

    fclose(f);

    if (n < TMC_ROM_BYTES)
    {
        ESP_LOGE(TAG, "%s is %u bytes, need %u", path, (unsigned)n, (unsigned)TMC_ROM_BYTES);
        heap_caps_free(buf);
        return NULL;
    }

    return buf;
}

/* --- start / stop -------------------------------------------------------- */

bool tama_start(const char *rom_path)
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

    s_rom = read_rom(s_rom_path);

    if (!s_rom)
    {
        printf("no rom at %s\n", s_rom_path);
        return false;
    }

    s_frame = (uint16_t *)heap_caps_malloc((size_t)TMC_FRAME_W * TMC_FRAME_H * sizeof(uint16_t),
                                           MALLOC_CAP_SPIRAM);

    if (!s_frame)
    {
        printf("no psram for the frame\n");
        heap_caps_free(s_rom);
        s_rom = NULL;
        return false;
    }

    tmc_err err = tmc_new(s_rom, TMC_ROM_BYTES, s_frame, &s_m);

    if (err != TMC_OK)
    {
        printf("rom rejected (%d)\n", (int)err);
        heap_caps_free(s_frame);
        heap_caps_free(s_rom);
        s_frame = NULL;
        s_rom = NULL;
        return false;
    }

    /* The pet's own persistent state, if a previous run left one. A missing
     * file is the first boot, not an error. */
    {
        uint8_t save[TMC_EEPROM_BYTES];
        FILE *f = fopen(TAMA_SAVE_PATH, "rb");

        if (f)
        {
            if (fread(save, 1, sizeof(save), f) == sizeof(save))
                tmc_eeprom_load(s_m, save, sizeof(save));
            fclose(f);
        }
    }

    /* Claim BEFORE the first blit, or the painter repaints the cells it still
     * believes are on the glass and the console bleeds through the pet. */
    display_pixels_claim(true);

    s_frames = 0;
    s_blits = 0;
    s_run = true;

    if (xTaskCreate(tama_task, "tama", TAMA_STACK, NULL, TAMA_PRIO, &s_task) != pdPASS)
    {
        s_run = false;
        s_task = NULL;
        display_pixels_claim(false);
        tmc_free(s_m);
        heap_caps_free(s_frame);
        heap_caps_free(s_rom);
        s_m = NULL;
        s_frame = NULL;
        s_rom = NULL;
        printf("no task\n");
        return false;
    }

    ESP_LOGI(TAG, "running %s", s_rom_path);
    report("start", s_rom_path);
    return true;
}

void tama_stop(void)
{
    if (!s_task)
        return;

    s_run = false;

    /* Wait for the guest task to leave tmc_run() and delete itself before
     * touching the machine it was using. A one-second bound because a task that
     * has not stopped by then is not going to and freeing underneath it would
     * be worse than leaking. */
    for (int i = 0; i < 100 && s_task; i++)
        vTaskDelay(pdMS_TO_TICKS(10));

    if (s_task)
    {
        ESP_LOGE(TAG, "guest task did not stop; not freeing");
        return;
    }

    {
        uint8_t save[TMC_EEPROM_BYTES];
        FILE *f;

        tmc_eeprom_save(s_m, save);
        f = fopen(TAMA_SAVE_PATH, "wb");
        if (f)
        {
            fwrite(save, 1, sizeof(save), f);
            fclose(f);
        }
    }

    report("stop", NULL);

    tmc_free(s_m);
    heap_caps_free(s_frame);
    heap_caps_free(s_rom);
    s_m = NULL;
    s_frame = NULL;
    s_rom = NULL;

    /* Hand the screen back and let the painter rebuild the desktop: by now the
     * glass holds the guest, not the cells the painter's record describes. */
    display_pixels_claim(false);
    display_repaint_all();
}

bool tama_running(void)
{
    return s_task != NULL;
}

/* --- the shell command --------------------------------------------------- */

static int cmd_tama(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    const char *sub = argc >= 2 ? argv[1] : "start";

    if (argc >= 2 && (strcmp(sub, "stop") == 0 || strcmp(sub, "off") == 0))
    {
        if (!tama_running())
        {
            printf("not running\n");
            return 0;
        }
        tama_stop();
        printf("stopped, screen released\n");
        return 0;
    }

    if (argc >= 2 && strcmp(sub, "status") == 0)
    {
        printf("running=%d frames=%lu blits=%lu\n", tama_running() ? 1 : 0,
               (unsigned long)s_frames, (unsigned long)s_blits);
        if (s_m)
        {
            uint64_t cyc = tmc_cycles(s_m);

            printf("cycles=%llu guest=%llu.%03llus rom=%s\n",
                   (unsigned long long)cyc,
                   (unsigned long long)(cyc / TMC_CLOCK_HZ),
                   (unsigned long long)((cyc % TMC_CLOCK_HZ) / 1000),
                   s_rom_path);
        }
        report("status", NULL);
        return 0;
    }

    if (argc >= 2 && (strcmp(sub, "press") == 0 || strcmp(sub, "release") == 0))
    {
        int idx = argc >= 3 ? atoi(argv[2]) : 0;

        if (!s_m)
        {
            printf("not running\n");
            return 1;
        }
        if (idx < 0 || idx > 2)
        {
            printf("button is 0, 1 or 2\n");
            return 1;
        }
        tmc_button(s_m, idx, strcmp(sub, "press") == 0);
        printf("%s %d\n", sub, idx);
        return 0;
    }

    if (argc >= 2 && strcmp(sub, "save") == 0)
    {
        uint8_t save[TMC_EEPROM_BYTES];
        FILE *f;

        if (!s_m)
        {
            printf("not running\n");
            return 1;
        }
        tmc_eeprom_save(s_m, save);
        f = fopen(TAMA_SAVE_PATH, "wb");
        if (!f)
        {
            printf("no card\n");
            return 1;
        }
        fwrite(save, 1, sizeof(save), f);
        fclose(f);
        printf("saved %s\n", TAMA_SAVE_PATH);
        return 0;
    }

    /* `tama start [rom]`, and a bare `tama` means start with the current ROM. */
    const char *rom = NULL;

    if (argc >= 2 && strcmp(sub, "start") == 0)
        rom = argc >= 3 ? argv[2] : NULL;
    else if (sub[0] == '/')
        rom = sub;

    if (!tama_start(rom))
        return 1;

    printf("tama running from %s\n", s_rom_path);
    printf("tama stop to leave\n");
    return 0;
}

static const tdsh_command_t s_tama_cmd = {
    "tama",
    "tama [start [rom] | stop | status | press|release N | save]",
    "Run the Tamagotchi V3 emulator on the screen",
    cmd_tama,
    TDSH_CMD_INTERACTIVE,
};

bool tama_register_command(void)
{
    return tdsh_register_command(&s_tama_cmd) == 0;
}
