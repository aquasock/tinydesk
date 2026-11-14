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
#include "sound.h"
#include "tama_ui.h"
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

/* --- the app's own controls ----------------------------------------------
 *
 * The button rules live in tama_ui.c, which has no ESP dependencies so the whole
 * chord rule can be swept on a host. This end owns the EFFECTS, and the state
 * the labels have to report.
 */
static tama_ui_t s_ui;
static volatile bool s_paused;
static volatile int s_speed = 1;         /* 1, 2 or 4 -- a guest-cycle multiplier */
static volatile bool s_sound_on = true;
static uint32_t s_shots;

/* The guest's cycle count as of the last frame, so the shell can report
 * progress without touching the machine from another task. Reads of a pointer
 * that the guest task may free on Load or Reset are exactly the kind of
 * use-after-free that is invisible until it is not. */
static volatile uint64_t s_cycles;
static uint32_t s_saves;
static uint32_t s_loads;
static uint32_t s_resets;

/* A Reset makes a fresh pet without touching the checkpoint, so quitting after
 * one must not overwrite that checkpoint with the fresh pet -- otherwise the
 * button would silently destroy the very save it was meant to be safe from.
 * Cleared when the user Saves or Loads, both of which are explicit statements
 * about what the file should hold. */
static bool s_protect_save;

/* Pending beeps, so the guest task never blocks on the amplifier: sound_beep()
 * plays 200 ms and 200 ms is three frames of guest time. */
static TaskHandle_t s_beep_task;
static bool s_sound_active;       /* last observed audio-active state */
static int64_t s_last_beep_us;
static uint32_t s_beeps;

/* --- report ---------------------------------------------------------------
 *
 * The USB mirror goes stale as soon as a bitmap owns the panel, and the desktop
 * log ring is 40 columns and clips. So the numbers that answer "is it running"
 * go to the card as well, where they can be read from the host over SFTP while
 * the guest is on the glass.
 */
static void report(const char *what, const char *extra)
{
    uint64_t cyc = s_cycles;
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
        fprintf(f, "cycles=%llu guest_s=%llu.%03llu paused=%d speed=%dx sound=%d "
                   "shots=%lu saves=%lu loads=%lu resets=%lu refused=%u beeps=%lu\n",
                (unsigned long long)cyc,
                (unsigned long long)(cyc / TMC_CLOCK_HZ),
                (unsigned long long)((cyc % TMC_CLOCK_HZ) / 1000),
                s_paused ? 1 : 0, s_speed, s_sound_on ? 1 : 0,
                (unsigned long)s_shots, (unsigned long)s_saves,
                (unsigned long)s_loads, (unsigned long)s_resets,
                s_ui.refused, (unsigned long)s_beeps);
    fclose(f);
}

/* --- the margins ---------------------------------------------------------- */

/*
 * touch.c hands over the STATE of the glass while the pixels are claimed, on
 * every report. That is what makes the interlock possible: the question "is the
 * arm held RIGHT NOW" is answerable on the same report the action arrives on,
 * which an event-shaped callback could never answer.
 *
 * The strip's three bars are not in the UI's map -- they are the pet's own
 * buttons, not the app's controls -- so they are handled here, and they queue
 * pin events exactly as before.
 *
 * Everything is queued rather than applied: this runs on the touch task, pinned
 * to core 0, while the guest task may be on core 1 and tmc_core.h says one
 * instance is not reentrant.
 */
static bool enqueue_pin(int index, bool down)
{
    if (!s_accept_input || !s_btn_q)
    {
        s_btn_drops++;
        return false;
    }

    tama_btn_evt_t e = { (uint8_t)index, down };

    if (xQueueSend(s_btn_q, &e, 0) != pdTRUE)
    {
        s_btn_drops++;
        return false;
    }

    if (down)
        s_presses++;

    return true;
}

static bool enqueue_action(tama_act_t act)
{
    if (!s_accept_input || !s_btn_q)
        return false;

    tama_btn_evt_t e = { (uint8_t)(TAMA_BTN_N + (int)act), false };

    if (xQueueSend(s_btn_q, &e, 0) != pdTRUE)
    {
        s_btn_drops++;
        return false;
    }

    return true;
}

static bool tama_app_touch(void *user, const touch_frame_t *tf)
{
    (void)user;

    if (!s_accept_input || !tf)
        return false;

    /*
     * The strip's bars go straight to the guest, positionally, as they did
     * before. Done first so a bar press under the arm still reaches the pet --
     * the arm gates the app's destructive controls, not the game.
     */
    for (int i = 0; i < tf->n && i < TOUCH_POINTS; i++)
    {
        if (tf->p[i].region == EDGE_BOTTOM &&
            tf->p[i].index >= 0 && tf->p[i].index < TAMA_BTN_N)
        {
            /* Press edge only: the controller re-reports a held finger, and a
             * bar that fired per report would hold the guest's button down
             * (which is right) but also count a hundred presses a second. */
            bool was = false;

            for (int j = 0; j < s_ui.prev.n && j < TAMA_UI_POINTS; j++)
                if (s_ui.prev.p[j].region == EDGE_BOTTOM &&
                    s_ui.prev.p[j].index == tf->p[i].index)
                    was = true;

            if (!was)
                enqueue_pin(tf->p[i].index, true);
        }
    }

    /* Bars that were down and are not any more get their release here, because
     * the guest's buttons are level-sensitive. */
    for (int j = 0; j < s_ui.prev.n && j < TAMA_UI_POINTS; j++)
    {
        if (s_ui.prev.p[j].region != EDGE_BOTTOM)
            continue;

        bool still = false;

        for (int i = 0; i < tf->n && i < TOUCH_POINTS; i++)
            if (tf->p[i].region == EDGE_BOTTOM && tf->p[i].index == s_ui.prev.p[j].index)
                still = true;

        if (!still)
            enqueue_pin(s_ui.prev.p[j].index, false);
    }

    tama_ui_frame_t f;

    f.n = tf->n;

    for (int i = 0; i < TOUCH_POINTS; i++)
    {
        f.p[i].region = i < tf->n ? tf->p[i].region : EDGE_NONE;
        f.p[i].index = i < tf->n ? tf->p[i].index : -1;
    }

    tama_act_t acts[TAMA_UI_MAX_ACTIONS];
    int n = tama_ui_update(&s_ui, &f, acts);

    for (int i = 0; i < n; i++)
        enqueue_action(acts[i]);

    /* Whether anything fired, for the driver's haptic tap: a live bottom-strip bar
     * or a margin slot the UI layer acted on. Bare glass reports false. */
    if (n > 0)
        return true;

    for (int i = 0; i < tf->n && i < TOUCH_POINTS; i++)
        if (tf->p[i].region == EDGE_BOTTOM &&
            display_margin_slot_live(2, tf->p[i].index))
            return true;

    return false;
}

/* --- effects -------------------------------------------------------------- */

/* Write the emulator's own frame as a 24-bit BMP.
 *
 * The frame rather than the whole panel: the bezel around it is the app's black
 * background and carries nothing, and `s_frame` is the only buffer this file can
 * read. A BMP because it needs no encoder -- 54 bytes of header, bottom-up rows,
 * three bytes per pixel -- and a screenshot that needs a library on the board is
 * a screenshot that does not get taken.
 */
static void write_bmp(const char *path)
{
    const int w = TMC_FRAME_W, h = TMC_FRAME_H;
    const int row = (w * 3 + 3) & ~3;    /* rows padded to a 4-byte boundary */
    const size_t datasz = (size_t)row * h;
    uint8_t hdr[54] = { 0 };

    FILE *f = fopen(path, "wb");

    if (!f)
        return;

    hdr[0] = 'B'; hdr[1] = 'M';
    uint32_t total = (uint32_t)(54 + datasz);
    memcpy(&hdr[2], &total, 4);
    uint32_t off = 54;
    memcpy(&hdr[10], &off, 4);
    uint32_t hs = 40;
    memcpy(&hdr[14], &hs, 4);
    int32_t ww = w, hh = h;
    memcpy(&hdr[18], &ww, 4);
    memcpy(&hdr[22], &hh, 4);
    uint16_t planes = 1, bpp = 24;
    memcpy(&hdr[26], &planes, 2);
    memcpy(&hdr[28], &bpp, 2);
    uint32_t raw = (uint32_t)datasz;
    memcpy(&hdr[34], &raw, 4);

    fwrite(hdr, 1, sizeof(hdr), f);

    /* Bottom-up, which is BMP's order, and the source is the frame's own
     * top-down layout -- so the row index is inverted rather than the image. */
    uint8_t *line = (uint8_t *)malloc((size_t)row);

    if (!line)
    {
        fclose(f);
        return;
    }

    for (int y = h - 1; y >= 0; y--)
    {
        memset(line, 0, (size_t)row);

        for (int x = 0; x < w; x++)
        {
            uint16_t c = s_frame[(size_t)y * w + x];
            uint8_t r = (uint8_t)(((c >> 11) & 0x1F) << 3);
            uint8_t g = (uint8_t)(((c >> 5) & 0x3F) << 2);
            uint8_t b = (uint8_t)((c & 0x1F) << 3);

            line[x * 3 + 0] = b;      /* BMP stores BGR */
            line[x * 3 + 1] = g;
            line[x * 3 + 2] = r;
        }

        fwrite(line, 1, (size_t)row, f);
    }

    free(line);
    fclose(f);
}

/* Start the machine over: a fresh guest, optionally seeded from the checkpoint.
 *
 * Rebuilt rather than reset because tmc_core exposes no reset -- and rebuilding
 * is the same thing to everything above it: the ROM and the frame buffer are the
 * caller's and survive, tmc_new() refills the bezel and invalidates the dirty
 * shadow so the next frame is a full repaint, and the wiring is re-applied by
 * the one function that knows the order it must be applied in.
 */
static bool restart_guest(bool from_save)
{
    tmc *fresh = NULL;

    if (tmc_new(s_rom, TMC_ROM_BYTES, s_frame, &fresh) != TMC_OK)
        return false;

    if (from_save)
    {
        uint8_t save[TMC_EEPROM_BYTES];
        FILE *f = fopen(TAMA_SAVE_PATH, "rb");

        if (f)
        {
            if (fread(save, 1, sizeof(save), f) == sizeof(save))
                tmc_eeprom_load(fresh, save, sizeof(save));
            fclose(f);
        }
        else
        {
            /* No checkpoint to load. Say so and keep the fresh pet rather than
             * pretending the Load worked -- a Load that silently does nothing
             * is worse than one that reports nothing to load. */
            s_loads--;
        }
    }

    tmc_free(s_m);
    s_m = fresh;

    return true;
}

/* Write the checkpoint. Returns false when the card will not take it. */
static bool save_checkpoint(void)
{
    uint8_t save[TMC_EEPROM_BYTES];
    FILE *f;

    if (!s_m)
        return false;

    tmc_eeprom_save(s_m, save);
    f = fopen(TAMA_SAVE_PATH, "wb");

    if (!f)
        return false;

    bool ok = fwrite(save, 1, sizeof(save), f) == sizeof(save);

    fclose(f);
    return ok;
}

/* --- the beep task -------------------------------------------------------- */

/*
 * sound_beep() plays 200 ms and blocks while it does, which is three frames of
 * guest time. So it runs here and the guest task only ever raises a
 * notification -- which coalesces, so a burst of guest audio becomes one beep
 * rather than a queue of them.
 */
static void beep_task(void *arg)
{
    (void)arg;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (s_sound_on)
        {
            s_beeps++;
            sound_beep();
        }
    }
}

/* --- actions -------------------------------------------------------------- */

/*
 * Repaint the three toggles whose word changes.
 *
 * Pause/Resume, the rate Speed will select, and whether sound is on. A fixed
 * label on a toggle is the label/duty lie this port has removed twice already:
 * a button reading "Pause" that resumes is worse than an unlabelled one, because
 * it is confidently wrong.
 */
static void refresh_labels(void)
{
    char rate[8];

    snprintf(rate, sizeof(rate), "%dx", s_speed);

    display_margin_label(0, 0, s_paused ? "Resume" : "Pause");
    display_margin_label(0, 3, rate);
    display_margin_label(0, 4, s_sound_on ? "On" : "Off");
}

static void apply_action(tama_act_t act)
{
    switch (act)
    {
    case TAMA_ACT_PAUSE:
        s_paused = !s_paused;
        break;

    case TAMA_ACT_SPEED:
        s_speed = (s_speed == 1) ? 2 : (s_speed == 2) ? 4 : 1;
        break;

    case TAMA_ACT_SOUND:
        s_sound_on = !s_sound_on;
        break;

    case TAMA_ACT_SHOT:
    {
        char path[32];

        snprintf(path, sizeof(path), "/sd/tshot-%lu.bmp", (unsigned long)(s_shots + 1));
        write_bmp(path);
        s_shots++;
        break;
    }

    case TAMA_ACT_SAVE:
        if (save_checkpoint())
        {
            s_saves++;
            s_protect_save = false;   /* an explicit statement about the file */
        }
        break;

    case TAMA_ACT_LOAD:
        s_loads++;

        if (restart_guest(true))
        {
            s_protect_save = false;
            s_sound_active = false;
        }
        break;

    case TAMA_ACT_RESET:
        s_resets++;

        if (restart_guest(false))
        {
            /* The checkpoint is deliberately left alone, and quitting must not
             * overwrite it -- see s_protect_save. */
            s_protect_save = true;
            s_sound_active = false;
        }
        break;

    default:
        return;
    }

    refresh_labels();
    ESP_LOGI(TAG, "%s", tama_ui_action_name(act));
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

        /* Queued work first, so a press that arrived during the last frame is
         * in effect for this one -- the guest polls its pins at its own rate and
         * a press applied after the step would be one frame late. */
        while (xQueueReceive(s_btn_q, &e, 0) == pdTRUE)
        {
            if (e.index < TAMA_BTN_N)
                tmc_button(s_m, e.index, e.down);
            else
                apply_action((tama_act_t)(e.index - TAMA_BTN_N));
        }

        if (s_paused)
        {
            /* Nothing to step, so nothing to draw. The loop stays alive so
             * queued actions -- Resume above all -- are still drained. */
            vTaskDelay(pdMS_TO_TICKS(TAMA_FRAME_US / 1000));
            next = esp_timer_get_time();
            continue;
        }

        /*
         * THE RISING EDGE OF "IS MAKING A SOUND", not a counter.
         *
         * This used to watch tmc_audio_emits() and treat any change as an event.
         * That counter increments on every square-wave toggle, so one held tone
         * raised it thousands of times a second and the beep task played a beep
         * back to back for as long as the tone lasted -- a BEL that never
         * stopped, and that went quiet while paused because pausing stops
         * stepping the guest and therefore stops the waveform.
         *
         * The waveform is a clock; the envelope is the signal. Silence to
         * not-silence is one event, which is what a speaker wants.
         *
         * The gap floor is a separate guard and is not the same fix: a guest
         * that chatters its tone on and off would otherwise produce a stutter of
         * beeps, and a beep is 200 ms long, so two inside 250 ms overlap into
         * noise rather than into two sounds.
         */
        {
            bool active = tmc_audio_active(s_m) != 0;

            if (active && !s_sound_active && s_sound_on && s_beep_task)
            {
                int64_t now_us = esp_timer_get_time();

                if (s_last_beep_us == 0 || now_us - s_last_beep_us > 250000)
                {
                    s_last_beep_us = now_us;
                    xTaskNotifyGive(s_beep_task);
                }
            }

            s_sound_active = active;
        }

        tmc_run(s_m, (uint64_t)TMC_FRAME_CYCLES * (uint64_t)s_speed);
        s_cycles = tmc_cycles(s_m);

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

    /* The app's controls start from their defaults on every run. The label
     * refresh must come after display_margins_set() above, because that call
     * repaints the columns from the tables and would overwrite an override
     * applied before it. */
    s_paused = false;
    s_speed = 1;
    s_sound_on = true;
    s_shots = s_saves = s_loads = s_resets = 0;
    s_beeps = 0;
    s_cycles = 0;
    s_sound_active = false;
    s_last_beep_us = 0;
    s_protect_save = false;
    tama_ui_init(&s_ui);

    refresh_labels();

    /* One task for the beeps, for the life of the process: sound_beep() blocks
     * for 200 ms and that is three frames of guest time, so the guest raises a
     * notification and this plays it. */
    if (!s_beep_task)
        xTaskCreate(beep_task, "tama-beep", 3072, NULL, 2, &s_beep_task);

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

    /*
     * The automatic checkpoint, skipped after a Reset.
     *
     * This is the one place the two meanings of "save" could quietly fight. The
     * exit write exists so the pet you were playing is the pet you come back
     * to; a Reset exists so you can start a new one. Put together, quitting
     * after a Reset would overwrite the deliberate checkpoint with the fresh
     * pet -- destroying exactly what the arming interlock was protecting. So a
     * Reset sets s_protect_save and this write is skipped until the user says
     * otherwise by pressing Save or Load.
     */
    if (!s_protect_save)
        (void)save_checkpoint();
    else
        ESP_LOGI(TAG, "checkpoint kept (reset since last save)");

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
        if (tama_running())
        {
            /* The snapshot, not tmc_cycles(s_m): Load and Reset free the
             * machine from the guest task, and dereferencing that pointer here
             * would be a use-after-free on exactly the runs that matter. */
            uint64_t cyc = s_cycles;

            printf("cyc=%llu g=%llu.%03llus p=%d %dx %s\n",
                   (unsigned long long)cyc,
                   (unsigned long long)(cyc / TMC_CLOCK_HZ),
                   (unsigned long long)((cyc % TMC_CLOCK_HZ) / 1000),
                   s_paused ? 1 : 0, s_speed, s_sound_on ? "on" : "off");
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
        if (!enqueue_pin(idx, strcmp(sub, "press") == 0))
        {
            printf("not queued\n");
            return 1;
        }
        printf("%s %d\n", sub, idx);
        return 0;
    }

    if (argc >= 2 && strcmp(sub, "save") == 0)
    {
        /* Through the queue, like every other action: the write reads the
         * emulated EEPROM, which belongs to the guest task. */
        if (!tama_running() || !enqueue_action(TAMA_ACT_SAVE))
        {
            printf("not running\n");
            return 1;
        }
        printf("save queued\n");
        return 0;
    }

    /* The app's controls from the shell, so they can be exercised without a
     * finger -- the same road a press takes. */
    if (argc >= 2 && strcmp(sub, "do") == 0)
    {
        const char *what = argc >= 3 ? argv[2] : "";
        tama_act_t act = TAMA_ACT_NONE;

        for (int i = TAMA_ACT_PAUSE; i <= TAMA_ACT_RESET; i++)
            if (strcmp(what, tama_ui_action_name((tama_act_t)i)) == 0)
                act = (tama_act_t)i;

        if (act == TAMA_ACT_NONE)
        {
            printf("pause save load speed sound shot reset\n");
            return 1;
        }
        if (!tama_running() || !enqueue_action(act))
        {
            printf("not running\n");
            return 1;
        }
        printf("%s queued\n", what);
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
    if (!touch_add_app_touch(tama_app_touch, NULL))
        ESP_LOGW(TAG, "touch consumer table full");

    return tdsh_register_command(&s_tama_cmd) == 0;
}
