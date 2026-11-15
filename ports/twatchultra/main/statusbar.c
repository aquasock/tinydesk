/*
 * statusbar.c - a permanent strip of always-visible information, above the
 *               desktop's own taskbar.
 *
 * WHY IT IS NOT THE TASKBAR. TinyDesk already has one, in the framebuffer, with a
 * clock and a network tray, and it is better than anything written here would be.
 * But it is framebuffer content, so an emulator that claims the pixels covers it
 * -- and that is exactly when a clock is wanted: mid-game, wrist raised, no
 * intention of leaving Zelda to find out what time it is.
 *
 * So this lives in the 44 glass rows ABOVE the framebuffer, which no app can
 * claim because no app can address them. Rows 0..43 are outside the 320x416
 * window that every app draws into, and nothing else in this firmware touches
 * them: `panel_fill()` blacks the whole panel at start-up and after that the
 * margin painters begin at row 44.
 *
 * THE BUDGET, in the order it constrains the design:
 *
 *   rows 1..43   43 rows. Row 0 is SWALLOWED by the controller (LCD-004, measured
 *                with a 2 px ring: the first row and column simply are not drawn)
 *                and row 44 belongs to `paint_column()`, which programs its
 *                window one row up from the first slot as a sacrificial row for
 *                the same reason. Both facts are already in this codebase; taking
 *                either row would fight something that exists.
 *
 *   32 px tall   an 8x16 glyph at scale 2. That leaves 11 rows of slack in a
 *                43-row band, so the text is centred with 5 above and 6 below.
 *                Scale 1 would be 16 px in 43 -- a strip of dots at arm's length
 *                on a 2-inch panel -- and scale 3 is 48 and does not fit.
 *
 *   no framebuffer   it draws into its own buffer and pushes it. Nothing here
 *                reads or writes the framebuffer, so an emulator cannot corrupt
 *                it and it cannot corrupt an emulator.
 *
 * WHY THE PUSHES ARE RARE. The strip shows HH:MM, so the pixels change once a
 * MINUTE. The task wakes at 1 Hz -- that is the resolution at which a minute
 * boundary can be noticed -- formats the string, compares it with what is
 * already on the glass, and returns without touching the bus when it matches.
 * A push costs 80x8 pixels per band, four bands, once a minute.
 */
#include "statusbar.h"

#include <stdio.h>
#include <string.h>

#include "display.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "font8x16.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "td_apps.h"
#include "tdsh.h"
#include "tinydesk/td_sysinfo.h"

static const char *TAG = "statusbar";

/*
 * GEOMETRY. All of it derived from constants that already exist, so a change to
 * the panel or the origin moves this rather than leaving it stale.
 */
#define SB_ROW_TOP     1                            /* row 0 is swallowed     */
#define SB_ROW_BOT     43                           /* row 44 is paint_column's */
#define SB_BAND_ROWS   (SB_ROW_BOT - SB_ROW_TOP + 1) /* 43                    */

#define SB_SCALE       2
#define SB_GLYPH_W     (8 * SB_SCALE)               /* 16                     */
#define SB_GLYPH_H     (16 * SB_SCALE)              /* 32                     */

#define SB_CHARS       5                            /* "HH:MM"                */
#define SB_TEXT_W      (SB_CHARS * SB_GLYPH_W)      /* 80                     */

/* Centred in the band: (43 - 32) / 2 = 5, so rows 6..37 with 6 clear below. */
#define SB_TOP_Y       (SB_ROW_TOP + (SB_BAND_ROWS - SB_GLYPH_H) / 2)   /* 6 */

/* Centred on the glass, in controller coordinates. PANEL_COL_OFFSET is the
 * controller RAM's hidden columns to the left of the glass. */
#define SB_LEFT_X      (PANEL_COL_OFFSET + (DISPLAY_PANEL_W - SB_TEXT_W) / 2)  /* 187 */

/*
 * One band of the strip, not the whole thing. 80 x 8 pixels is 1,280 bytes, which
 * is the entire persistent cost of this feature; four bands make a whole strip.
 * `display_panel_push()` pushes a caller's buffer atomically, so a band is either
 * on the glass or not on it, never half-written.
 */
#define SB_PUSH_ROWS   8
static uint16_t s_band[SB_TEXT_W * SB_PUSH_ROWS];

/*
 * Black and white, in the panel's byte order. Both are palindromic in 16 bits,
 * so no swap is needed -- noted because every other constant in this firmware has
 * to be written byte-swapped and this one silently does not.
 */
#define SB_BG   0x0000
#define SB_FG   0xFFFF

/* What is currently on the glass, so a minute that has not changed costs no bus.
 * "HH:MM" plus a terminator; also the "no clock" form. */
#define SB_TEXT_MAX 8
static char s_shown[SB_TEXT_MAX];

/* Counters for the `statusbar` command. `s_pushed` is how many times the strip
 * has actually written the panel, which is the number that matters when asking
 * whether the strip is behind something that happens once a minute. */
static volatile uint32_t s_pushed;
static volatile uint32_t s_ticks;

/* The user's UTC offset, cached in this task. See sb_text() for why the strip does
 * not ask the desktop's side on every tick. */
static long s_tz;
static bool s_tz_valid;
static int64_t s_tz_read_us;

/*
 * TIME OF DAY, WITHOUT A CALENDAR.
 *
 * Only hours and minutes are wanted, so the epoch is reduced to a seconds-into-day
 * and the date is never computed. That is four lines instead of a civil-date
 * conversion, and it cannot be wrong about leap years because it never looks at
 * them.
 *
 * The source is `td_sysinfo()`'s `time_now` and `get_tz`, which is the SAME pair
 * the taskbar clock reads through -- so the strip and the taskbar cannot disagree
 * about what time it is, including across a timezone change, and neither needs to
 * know how the other is implemented.
 *
 * A clock that is not set is reported as "--:--" rather than as a plausible wrong
 * time. This project's rule about the RTC is the same one: a watch that says it
 * does not know is better than one that is confidently an hour out.
 */
static void sb_text(char *out, size_t cap, int64_t *minute_out)
{
    const td_sysinfo_t *si = td_sysinfo();
    int64_t utc = 0;

    if (!si || !si->time_now || !si->time_now(&utc))
    {
        snprintf(out, cap, "--:--");
        *minute_out = 0;
        return;
    }

    /*
     * AN OFFSET WE COULD NOT READ IS NOT AN OFFSET OF ZERO.
     *
     * `get_tz()` used to report success while handing back 0 when its file could
     * not be read, so this caller could not tell "your time zone is UTC" from "I
     * could not find the file" -- and displayed UTC for the second case, which the
     * operator saw as the clock reverting after a while. It now returns false on a
     * failed read, and a failure here KEEPS THE PREVIOUS OFFSET rather than
     * replacing it.
     *
     * If no offset has ever been read the strip shows dashes instead of UTC. That is
     * the same rule this project applies to the RTC, and the same reason: a clock
     * that admits it does not know beats one that is confidently an hour out. The
     * case is not hypothetical -- the first read happens at start-up, before the
     * desktop has logged in and built the home path.
     */
    const int64_t now_us = esp_timer_get_time();

    /*
     * RETRY FAST UNTIL THE OFFSET IS KNOWN, THEN SLOWLY.
     *
     * The first attempt happens at start-up, and on this board that is BEFORE the
     * desktop logs in -- `td_session_home()` is still empty, so the path does not
     * exist yet and the read fails. A failed read must therefore be retried soon:
     * the offset becomes available within a second or two of login, and until then
     * this strip would otherwise show dashes for a whole cache period. Every boot
     * paid that, which is why the clock appeared to revert after a while.
     *
     * 5 s while unknown is 12 attempts a minute at worst, of which all but the
     * last fail with one open; once a value is in hand the window goes back to a
     * minute, because a zone change needs the operator to open a window and pick
     * one and cannot happen more often than that.
     */
    const int64_t window = s_tz_valid ? 60000000 : 5000000;

    if (!s_tz_valid || now_us - s_tz_read_us >= window)
    {
        long tz = 0;

        if (si->get_tz && si->get_tz(&tz))
        {
            s_tz = tz;
            s_tz_valid = true;
        }

        s_tz_read_us = now_us;
    }

    if (!s_tz_valid)
    {
        snprintf(out, cap, "--:--");
        *minute_out = 0;
        return;
    }

    const int64_t local = utc + s_tz;

    /* Floor division, because a negative epoch must not round toward zero and
     * put the clock a day out at midnight. */
    const int64_t days = local >= 0 ? local / 86400 : -((-local + 86399) / 86400);
    const int64_t rem = local - days * 86400;

    snprintf(out, cap, "%02d:%02d", (int)(rem / 3600), (int)(rem / 60 % 60));

    *minute_out = local / 60;
}

/* One pixel of the strip: `x` is a controller column, `y` a controller row. */
static bool sb_ink(int x, int y, const char *text)
{
    const int gx = x - SB_LEFT_X;
    const int gy = y - SB_TOP_Y;

    if (gx < 0 || gx >= SB_TEXT_W || gy < 0 || gy >= SB_GLYPH_H)
        return false;

    const int cell = gx / SB_GLYPH_W;
    const int px = (gx % SB_GLYPH_W) / SB_SCALE;
    const int py = gy / SB_SCALE;

    if (!text[cell])
        return false;

    const uint8_t *glyph = font8x16_glyph((uint16_t)(unsigned char)text[cell]);

    return glyph && (glyph[py] & (0x80 >> px)) != 0;
}

/*
 * Push the strip, one band at a time. Returns false if any push was refused,
 * which means the geometry is wrong rather than that the glass is busy.
 */
static bool sb_paint(const char *text)
{
    for (int row = 0; row < SB_GLYPH_H; row += SB_PUSH_ROWS)
    {
        const int rows = SB_GLYPH_H - row > SB_PUSH_ROWS ? SB_PUSH_ROWS
                                                        : SB_GLYPH_H - row;

        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < SB_TEXT_W; c++)
            {
                const int x = SB_LEFT_X + c;
                const int y = SB_TOP_Y + row + r;

                s_band[r * SB_TEXT_W + c] = sb_ink(x, y, text) ? SB_FG : SB_BG;
            }
        }

        if (!display_panel_push(SB_LEFT_X, SB_TOP_Y + row, SB_TEXT_W, rows, s_band))
            return false;
    }

    return true;
}

static void sb_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "strip up: rows %d..%d, %dx%d px at scale %d",
             SB_TOP_Y, SB_TOP_Y + SB_GLYPH_H - 1, SB_TEXT_W, SB_GLYPH_H, SB_SCALE);

    /*
     * THE GATE IS THE MINUTE INDEX, NOT THE STRING, AND THAT IS THE FIX FOR A
     * FLASH THAT SHOULD NEVER HAVE BEEN POSSIBLE.
     *
     * The previous version pushed whenever the formatted text differed from what
     * was last shown. That is correct only if the text is a pure function of the
     * minute, and the operator saw the whole panel flash once a SECOND -- which can
     * only happen if the text differed on every tick, or if the push was not the
     * thing changing at all. Either way the guard was the wrong shape: it depended
     * on the string being stable rather than on the thing that actually causes a
     * redraw.
     *
     * So the decision is made from an integer that cannot jitter. `sb_minute()`
     * returns the number of minutes since the epoch, and a push happens only when
     * that value changes. A wrong or missing offset cannot make it drift faster
     * than the clock, because it is derived the same way the text is and compared
     * as a number.
     *
     * The text comparison is kept as a second condition, because it is free and it
     * catches the one case the index cannot: an offset change inside the same
     * minute, where the wall clock jumps but the minute number does too. Both have
     * to agree that something changed.
     */
    int64_t last_minute = -1;

    for (;;)
    {
        int64_t minute = 0;
        char now[SB_TEXT_MAX];

        sb_text(now, sizeof(now), &minute);

        if (minute != last_minute || strcmp(now, s_shown) != 0)
        {
            const int64_t t0 = esp_timer_get_time();

            if (sb_paint(now))
            {
                memcpy(s_shown, now, sizeof(s_shown));
                last_minute = minute;
                s_pushed++;

                ESP_LOGI(TAG, "%s pushed in %lld us (#%lu)", now,
                         (long long)(esp_timer_get_time() - t0),
                         (unsigned long)s_pushed);
            }
            else
            {
                ESP_LOGW(TAG, "push refused at %d,%d", SB_LEFT_X, SB_TOP_Y);
            }
        }

        s_ticks++;

        /*
         * ONE SECOND, because the only thing that can change is a minute. Keeping
         * the tick at 1 Hz rather than 60 s is what makes the minute boundary land
         * within a second of the real one instead of up to a minute late, and the
         * cost of the extra wake-ups is a snprintf and a strcmp.
         */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*
 * `statusbar` -- what the strip believes, and what the panel is being asked to do.
 *
 * THIS EXISTS BECAUSE THE STRIP SHOWED THE RIGHT TIME AND THEN WENT BACK, and the
 * only way to tell which input had moved was to print them. `utc` is the system
 * clock, `tz` is the offset the strip adds, and `text` is the result -- so a
 * disagreement names its own cause instead of being guessed at.
 *
 * `pushed` is the strip's own count and `display pushes` is the driver's TOTAL
 * pixel pushes, every band from every writer. Sampling both across a minute
 * separates "the strip wrote four bands" from "something repainted the whole
 * screen": four bands add four, where a full repaint adds hundreds.
 */
static int cmd_statusbar(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;
    (void)argc;
    (void)argv;

    const td_sysinfo_t *si = td_sysinfo();
    int64_t utc = 0;
    long tz = 0;
    const bool have = si && si->time_now && si->time_now(&utc);

    if (si && si->get_tz)
        si->get_tz(&tz);

    char text[SB_TEXT_MAX];
    int64_t minute = 0;
    sb_text(text, sizeof(text), &minute);

    int64_t us_w = 0, us_x = 0;
    uint32_t dpush = 0;
    display_push_stats(&us_w, &us_x, &dpush);

    printf("text=%s utc=%lld tz=%ld have=%d\n",
           text, (long long)utc, tz, have ? 1 : 0);    printf("pushed=%lu ticks=%lu shown='%s'\n",
           (unsigned long)s_pushed, (unsigned long)s_ticks, s_shown);
    printf("display pushes=%lu\n", (unsigned long)dpush);

    /*
     * THE PATH `get_tz()` WOULD BUILD, AND WHETHER IT CAN BE OPENED.
     *
     * Added after the first sample showed `tz=0` with `-25200` sitting in the
     * file: the offset was set and stored correctly, and the reader was not
     * finding it. Printing the path turns "the reader is wrong" into one of three
     * facts -- the home is empty, the path is right but unopenable, or the value is
     * there and something else is overwriting it.
     */
    printf("home='%s'\n", td_session_home());

    {
        char path[160];
        snprintf(path, sizeof(path), "%s/.tdsh_tz", td_session_home());

        long v = 0;
        FILE *f = fopen(path, "r");
        int rc = f ? (fscanf(f, "%ld", &v) == 1 ? 1 : -1) : 0;
        if (f)
            fclose(f);

        printf("tzfile '%s' open=%d val=%ld\n", path, rc, v);
    }

    return 0;
}

static const tdsh_command_t s_sb_cmd = {
    "statusbar",
    "statusbar",
    "Status strip: what it reads and what it has pushed",
    cmd_statusbar,
    TDSH_CMD_INTERACTIVE,
};

bool statusbar_start(void)
{
    s_shown[0] = 0;
    s_tz_valid = false;
    s_tz_read_us = 0;

    if (tdsh_register_command(&s_sb_cmd) != 0)
        ESP_LOGW(TAG, "statusbar command did not register");

    /* Priority 1 and a small stack: this runs once a second and must never
     * compete with an emulator. It does not need to win a race -- a strip that
     * appears a few milliseconds late is invisible. */
    /*
     * 8192, NOT 3072, AND THE NUMBER IS MEASURED RATHER THAN CHOSEN.
     *
     * The first version asked for 3072 on the reasoning that this task does almost
     * nothing -- one snprintf and a pixel push a minute. It panicked instead:
     *
     *   ***ERROR*** A stack overflow in task statusbar has been detected.
     *
     * The push is not the cheap part. `display_panel_push()` calls
     * `push_pixels_raw_buf()`, which puts an `spi_transaction_ext_t` and a bus
     * acquisition on the stack and calls into the SPI driver's polling transmit;
     * that path alone is deeper than three kilobytes. Worse, the failure was
     * INVISIBLE: this firmware builds with `CONFIG_ESP_CONSOLE_NONE`, and
     * `CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT` reboots with no message, so the watch
     * simply rebooted every thirty-three seconds and looked like a display flash.
     * The timezone appeared to revert because every reboot re-ran the first-timezone
     * read before login. It took enabling the console to see the four words that
     * named the fault.
     *
     * 8192 is double the failing size, which is the cheap side of this trade: a task
     * that runs once a minute cannot make 5 KB matter, and a stack overflow is a
     * reboot rather than an error.
     */
    if (xTaskCreatePinnedToCore(sb_task, "statusbar", 8192, NULL, 1, NULL, 0) != pdPASS)
    {
        ESP_LOGE(TAG, "no task");
        return false;
    }

    return true;
}
