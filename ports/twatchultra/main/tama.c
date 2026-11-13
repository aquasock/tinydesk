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
#include "freertos/queue.h"
#include "freertos/task.h"
#include "touch.h"
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

/*
 * THE STRIP'S BUTTONS CROSS A TASK BOUNDARY, so they are queued rather than
 * written.
 *
 * touch.c runs its poll task pinned to core 0 at priority 3; the guest task here
 * is unpinned and can be on core 1 at the same priority. `tmc_button()` writes
 * the emulated CPU's pin state, and tmc_core.h states plainly that one instance
 * is not reentrant -- so calling it from the touch task would be a data race
 * against `tmc_run()` on the guest task, on two cores, with the guest's own
 * programmer's model as the victim. The symptom would not be a crash; it would
 * be a pet that occasionally acts on a button it never received.
 *
 * A queue is also what lets the emulator keep its own single-threaded contract
 * instead of gaining a lock: the guest task drains the queue between frames and
 * is the only thing that ever calls tmc_button().
 *
 * The queue is created once and never deleted, so the touch driver can never
 * hold a handle to freed memory -- which is why the sink is registered for good
 * rather than with each run (see touch.h).
 */
typedef struct
{
    uint8_t index;
    bool down;
} tama_btn_evt_t;

#define TAMA_BTN_QDEPTH 8

/* The strip's three bars are the V3's three buttons, positionally. If either
 * count changes, the mapping above stops being a mapping -- so it is checked at
 * compile time rather than trusted, and BTN_N comes from edges.h, which is the
 * geometry the painter and the hit test share. */
_Static_assert(BTN_N == TAMA_BTN_N, "the strip's bar count and the guest's button count disagree");

static QueueHandle_t s_btn_q;
static volatile bool s_accept_input;   /* false between stop and start */
static uint32_t s_presses;             /* accepted presses, for the report */
static uint32_t s_btn_drops;           /* events the queue would not take */
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

    fprintf(f, "%s%s%s frames=%lu blits=%lu presses=%lu drops=%lu reset=%d\n",
            what, extra ? " " : "", extra ? extra : "",
            (unsigned long)s_frames, (unsigned long)s_blits,
            (unsigned long)s_presses, (unsigned long)s_btn_drops,
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

/* --- the strip's three bars ---------------------------------------------- */

/*
 * touch.c offers every margin press here while the pixels are claimed.
 *
 * THE MAPPING IS POSITIONAL: left bar is the guest's button 0, middle 1, right
 * 2, which is the order tmc_button() documents for the V3's pin mask. The
 * columns are declined, not ignored -- returning false is what makes the driver
 * log `off`, which keeps "not wired yet" visible in the log instead of leaving
 * a press that silently does nothing.
 */
static bool tama_enqueue_button(int index, bool down)
{
    tama_btn_evt_t e = { (uint8_t)index, down };

    /* Never block. The queue is eight deep against a 66 ms re-report and a
     * 16 Hz drain, so a full queue means the guest task has stopped draining --
     * a fault, counted rather than hidden. */
    if (!s_accept_input || !s_btn_q || xQueueSend(s_btn_q, &e, 0) != pdTRUE)
    {
        s_btn_drops++;
        return false;
    }

    if (down)
        s_presses++;

    return true;
}

static bool tama_app_button(void *user, edge_region_t region, int index, bool down)
{
    (void)user;

    if (region != EDGE_BOTTOM || index < 0 || index >= TAMA_BTN_N)
        return false;

    if (!s_accept_input)
        return false;

    /* Queued, and reported as accepted even when the queue is full: the press
     * did reach the application, and telling the driver otherwise would relabel
     * a real fault as "not wired". */
    (void)tama_enqueue_button(index, down);
    return true;
}

/* --- the guest task ------------------------------------------------------ */

static void tama_task(void *arg)
{
    (void)arg;
    int64_t next = esp_timer_get_time();

    while (s_run)
    {
        int y_lo = 0, y_hi = 0;
        tama_btn_evt_t e;

        /* Buttons first, so a press that arrived during the last frame is in
         * effect for this one -- the guest polls its pins at its own rate and
         * a press applied after the step would be one frame late. */
        while (xQueueReceive(s_btn_q, &e, 0) == pdTRUE)
            tmc_button(s_m, e.index, e.down);

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

    /*
     * THEN CLEAR THE FRAMEBUFFER, because claiming the screen is not the same
     * as owning what is on it.
     *
     * The blit below only writes the 300x308 rectangle the emulator draws into.
     * Everything the painter had already put in the rest of the 320x416
     * framebuffer -- the shell's text, the window frame -- stays exactly where
     * it was, and once ownership is claimed nothing will ever overwrite it. So
     * without this the pet appears on top of the console it was launched from,
     * which is what the operator saw.
     *
     * Black rather than the emulator's bezel colour by coincidence: both are
     * 0x0000, so the frame blends into the screen and only the LCD's lit dots
     * show. The glass margins are not touched -- the buttons live outside the
     * framebuffer and are painted by display_init() -- so the watch's controls
     * keep working while the guest has the screen.
     */
    if (!display_fill(0x0000))
    {
        display_pixels_claim(false);
        tmc_free(s_m);
        heap_caps_free(s_frame);
        heap_caps_free(s_rom);
        s_m = NULL;
        s_frame = NULL;
        s_rom = NULL;
        printf("no framebuffer\n");
        return false;
    }

    /*
     * THE MARGINS STOP CLAIMING TO BE DESKTOP KEYS.
     *
     * touch.c already refuses them while the pixels are claimed, so pressing
     * Enter no longer types into the hidden shell -- but a button still reading
     * "Enter" would then be a label describing a duty it does not have. In APP
     * mode the columns go blank and the strip shows the degree circle they will
     * carry once they are wired to the guest.
     *
     * Called HERE, after the claim and before the guest task exists, because
     * display_margins_set() paints synchronously and requires the caller to be
     * the only writer of the glass. The painter is suppressed by the claim, and
     * the guest task that will blit does not exist yet, so that holds by
     * construction rather than by luck -- and this is why the call is not
     * deferred to the task.
     */
    display_margins_set(DISPLAY_MARGINS_APP);

    s_frames = 0;
    s_blits = 0;
    s_presses = 0;
    s_btn_drops = 0;

    /* One queue for the life of the process: the touch driver holds a handle to
     * this sink for ever, so deleting the queue between runs would leave it
     * pointing at nothing. Created before the guest task and before input is
     * accepted, so a press cannot arrive at a queue that does not exist. */
    if (!s_btn_q)
        s_btn_q = xQueueCreate(TAMA_BTN_QDEPTH, sizeof(tama_btn_evt_t));

    if (!s_btn_q)
    {
        display_pixels_claim(false);
        tmc_free(s_m);
        heap_caps_free(s_frame);
        heap_caps_free(s_rom);
        s_m = NULL;
        s_frame = NULL;
        s_rom = NULL;
        printf("no queue\n");
        return false;
    }

    s_run = true;
    s_accept_input = true;

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

    /* Stop accepting button presses FIRST: the sink writes to a queue the guest
     * task drains, and the guest task is about to stop existing. */
    s_accept_input = false;
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

    /* Restore the margins FIRST, while the pixels are still claimed: the call
     * paints synchronously and needs to be the only writer, which ownership
     * guarantees and the released state does not. */
    display_margins_set(DISPLAY_MARGINS_DESKTOP);

    /* Then hand the screen back and let the painter rebuild the desktop: by now
     * the glass holds the guest, not the cells the painter's record describes. */
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
        /* THROUGH THE QUEUE, not straight at the machine. The shell runs on
         * another task from the guest, and tmc_button() writes emulated pin
         * state that tmc_run() is reading; tmc_core.h says one instance is not
         * reentrant. This path had the race from Phase 4 until the strip's
         * buttons made it worth removing -- and taking the same road as a real
         * press means `tama press` exercises the queue rather than bypassing
         * it. */
        if (!tama_enqueue_button(idx, strcmp(sub, "press") == 0))
        {
            printf("not queued\n");
            return 1;
        }
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
    /* Registered for the life of the process, not per run: see touch.h. The
     * handler declines everything while the emulator is not running, which is
     * the same behaviour as being unregistered, without a window in which the
     * driver holds a pointer to something that has gone away. */
    touch_set_app_buttons(tama_app_button, NULL);

    return tdsh_register_command(&s_tama_cmd) == 0;
}
