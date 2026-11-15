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
 * THE WIDTH BUDGET IS THE CHAMFER, NOT THE GLASS. An early version of this comment
 * claimed 165 px of margin either side, which assumed the strip had the glass's
 * full 410 columns. Above row 45 it does not: docs/panel-geometry.md measures the
 * corners as straight 45-degree chamfers with
 *
 *     top-left   x + y         = 80
 *     top-right  (410 - x) + y = 90
 *
 * so at the row where the glyphs first put ink down -- row 10 for a scale-2 glyph
 * block starting at row 6 -- the glass is only x 70..330, which is 261 px and not
 * 410. The strip's whole design follows from that measurement, and a host harness
 * that compiled this file's own font array and placement arithmetic and audited
 * every ink pixel against those two lines is what put numbers on it.
 *
 * WHAT THE STRIP SHOWS, AND THE TWO THINGS REMOVED TO MAKE IT FIT. The budget is
 * 261 px at the binding row, and the operator wants the clock at the size already
 * accepted -- 8x16 at scale 2, which is 32 rows of glyph and 16 px per character.
 * Measured on the host against those chamfer lines, with the date order and the
 * 12/24-hour clock both taken from the Settings app:
 *
 *     date with year + clock + battery, all scale 2 ..... 446 ink px clipped
 *     date with year + clock, all scale 2 ............... 121 ink px clipped
 *     date without year + clock, all scale 2 ............ 0, clears by 24/21 px
 *
 * The middle row is what the first attempt at "add the date and the battery"
 * produced: to fit it, the date and the battery had to drop to scale 1 while the
 * clock stayed at scale 2. It fitted, and the operator looked at the glass and
 * said it did not really fit -- three sizes on one line read as three separate
 * strips that happen to share a row, not as one strip.
 *
 * So the YEAR and the BATTERY are gone, by operator direction, and every field is
 * back at the one scale the strip has ever been accepted at. The whole line is now
 * at most 224 px of the 261 available in the worst case (a 12-hour clock with a
 * two-digit hour), leaving 24 px of clearance on the left and 21 on the right.
 *
 * THIS IS A DELIBERATE NARROWING, NOT A LIMIT, and restoring either is small: the
 * year is one `%04d` in sb_date() and the battery is a gauge read plus four
 * characters in sb_layout(). What must not be lost is the measurement above. A
 * future attempt to put both back has to answer the 446 px, and the honest answer
 * is a narrower clock, a smaller date, or a second row -- none of which is free.
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
 * WHAT IT SHOWS IS THE USER'S CHOICE, NOT THIS FILE'S. The date order and the
 * 12/24-hour clock come from the Settings app through `td_clock_prefs()`, the same
 * structure `apps/datetime.c` formats the taskbar clock from. The three date
 * formats the settings offer are day-month, year-month-day and month-day-year, and
 * with the year removed they shorten to `15-11`, `11-15` and `11/15` -- still
 * distinguishable in order, because the separator and the first field still say
 * which convention is in force.
 *
 * WHY THE PUSHES ARE RARE. The strip's content changes once a MINUTE. The task
 * wakes at 1 Hz, which is the resolution at which a minute boundary can be
 * noticed, and returns without touching the bus when nothing has changed.
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
/*
 * THE FONT CELL IS UNSCALED HERE, AND THE SCALE IS APPLIED AT USE.
 *
 * This looks pedantic and is not: an earlier revision defined the cell as
 * `8 * SB_SCALE` and then ALSO multiplied a field's width by `f->scale`, so every
 * width doubled, `sb_push_field()` refused the field against SB_MAX_W, and the
 * whole strip silently never drew -- `statusbar` reported `left=-16` and
 * `pushes=0`. Folding the scale into the cell and applying it separately is how
 * that happened, so the two are now strictly separate: SB_FONT_W/H are the font's
 * own 8x16, and nothing here is scaled until it is multiplied by a field's scale.
 */
#define SB_FONT_W      8                            /* font column, unscaled  */
#define SB_FONT_H      16                           /* font row, unscaled     */

#define SB_GLYPH_W     (SB_FONT_W * SB_SCALE)       /* 16 on the glass        */
#define SB_GLYPH_H     (SB_FONT_H * SB_SCALE)       /* 32 on the glass        */

/*
 * THE CHAMFERED CENTRE, not the glass centre.
 *
 * The glass is 410 columns but the cuts take the corners, and the strip lives at
 * the very top where they bind hardest. The visible span at row 10 is x 70..330,
 * which is 261 px inclusive; the glass's own centre is 205 but that span's centre
 * is 200. Centreing on 200 is what puts equal margin against both cuts rather than
 * 5 px more against the left.
 */
#define SB_GLASS_CX    200

/* Centred in the band: (43 - 32) / 2 = 5, so rows 6..37 with 6 clear below. */
#define SB_TOP_Y       (SB_ROW_TOP + (SB_BAND_ROWS - SB_GLYPH_H) / 2)   /* 6 */

/* Gap between the two fields: one character cell, so the spacing reads as a word
 * space rather than as a kerning accident. The harness measured this against the
 * chamfer: the worst case needs 224 of the 261 px available. */
#define SB_GAP         SB_GLYPH_W

/*
 * The band buffer is sized to the WIDEST LINE the strip can build, not to one
 * field, because the whole line is pushed as one rectangle -- see sb_paint(). The
 * widest line is the 12-hour clock's eight cells (128 px) plus the gap plus the
 * date's five (80 px), which is 224 px. 224 x 4 x 2 is 1,792 bytes.
 */
#define SB_MAX_W       224
#define SB_PUSH_ROWS   4
static uint16_t s_band[SB_MAX_W * SB_PUSH_ROWS];

/*
 * Black and white, in the panel's byte order. Both are palindromic in 16 bits,
 * so no swap is needed -- noted because every other constant in this firmware has
 * to be written byte-swapped and this one silently does not.
 */
#define SB_BG   0x0000
#define SB_FG   0xFFFF

/* One field: its text and the scale it is drawn at. The scale is a field rather
 * than a constant because the strip has already lived through one attempt at
 * mixing sizes, and a future field may need it; today every field is SB_SCALE. */
#define SB_FIELD_MAX   12
typedef struct
{
    char text[SB_FIELD_MAX];
    int scale;
} sb_field_t;

#define SB_NFIELDS     2                            /* date, clock            */

/* The extent of the last paint, in glass columns, so a field that MOVED can be
 * erased where it used to be. See sb_paint() for why this exists -- its absence
 * put "--" on the glass next to the date. */
static int s_prev_x0;
static int s_prev_x1;
static bool s_prev_valid;

/* What is currently on the glass, so a minute that has not changed costs no bus.
 * The composite key, "date|clock", which is at most 25 bytes. */
#define SB_KEY_MAX     32
static char s_shown[SB_KEY_MAX];

/* Counters for the `statusbar` command. `s_pushed` is how many times the strip
 * has actually written the panel, which is the number that matters when asking
 * whether the strip is behind something that happens once a minute. */
static volatile uint32_t s_pushed;
static volatile uint32_t s_ticks;

/* The user's UTC offset, cached in this task. See sb_build() for why the strip
 * does not ask the desktop's side on every tick. */
static long s_tz;
static bool s_tz_valid;
static int64_t s_tz_read_us;

#define SB_TZ_KNOWN_US   (60LL * 1000000)
#define SB_TZ_UNKNOWN_US (5LL * 1000000)

/*
 * DAYS TO A CALENDAR DATE, WITH NO LIBRARY.
 *
 * The taskbar's formatter lives in `apps/datetime.c` behind a static function, so
 * the strip cannot borrow it and should not: this is Howard Hinnant's civil-date
 * algorithm, which is exact for every proleptic Gregorian date and needs no
 * lookup tables. It is here rather than a table of month lengths because the
 * arithmetic that gets leap years wrong is the kind of bug that shows up once
 * every four years, on someone else's wrist -- and 29 February is still on screen
 * even with the year removed from the TEXT.
 */
static void sb_civil_from_days(long long z, int *year, int *month, int *day)
{
    z += 719468;
    const long long era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned long long doe = (unsigned long long)(z - era * 146097);
    const unsigned long long yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const long long y = (long long)yoe + era * 400;
    const unsigned long long doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned long long mp = (5 * doy + 2) / 153;
    const unsigned long long d = doy - (153 * mp + 2) / 5 + 1;
    const unsigned long long m = mp + (mp < 10 ? 3 : -9);

    *year = (int)(y + (m <= 2));
    *month = (int)m;
    *day = (int)d;
}

/*
 * The date, in the order the Settings app was set to, WITHOUT THE YEAR.
 *
 * The year was removed to buy the width that let every field return to scale 2 --
 * see the header. `ok` false means the clock is not set, and all three formats
 * fall back to `--/--`. That is deliberate rather than an oversight: a run of five
 * dashes with the separator of whichever format happened to be active would read
 * as damage, where grouped dashes read as a placeholder. It is the one state where
 * legibility beats matching the separator, and it lasts at most thirty seconds at
 * boot.
 */
static void sb_date(char *out, size_t cap, const td_clock_prefs_t *p,
                    bool ok, int mo, int d)
{
    const uint8_t fmt = p ? p->date_format % TD_DATE_FORMATS : TD_DATE_DMY;

    if (!ok)
    {
        snprintf(out, cap, "--/--");
        return;
    }

    switch (fmt)
    {
    case TD_DATE_YMD:
        snprintf(out, cap, "%02d-%02d", mo, d);
        break;
    case TD_DATE_MDY:
        snprintf(out, cap, "%02d/%02d", mo, d);
        break;
    case TD_DATE_DMY:
    default:
        snprintf(out, cap, "%02d-%02d", d, mo);
        break;
    }
}

/* The clock, in the user's 12- or 24-hour form.
 *
 * THE 12-HOUR FORM IS PADDED TO A FIXED EIGHT CELLS. `apps/datetime.c` prints
 * "9:42 AM" with a bare hour, which is right for a taskbar that re-centres its
 * whole string anyway. Here the field's width decides where the date sits, so an
 * unpadded hour would shift the date 16 px sideways at 10 o'clock every morning.
 * `%2d` pads with a space, which is invisible against the strip's background and
 * keeps the geometry still.
 */
static void sb_clock(char *out, size_t cap, const td_clock_prefs_t *p,
                     bool ok, int h, int m)
{
    if (!ok)
    {
        snprintf(out, cap, "--:--");
        return;
    }

    if (p && p->clock_12h)
    {
        const int h12 = h % 12 ? h % 12 : 12;

        snprintf(out, cap, "%2d:%02d %s", h12, m, h < 12 ? "AM" : "PM");
    }
    else
    {
        snprintf(out, cap, "%02d:%02d", h, m);
    }
}

/* Re-read the user's offset on its own schedule, keeping the last good value. */
static void sb_refresh_tz(const td_sysinfo_t *si)
{
    const int64_t now_us = esp_timer_get_time();
    const int64_t window = s_tz_valid ? SB_TZ_KNOWN_US : SB_TZ_UNKNOWN_US;

    if (s_tz_valid && now_us - s_tz_read_us < window)
        return;

    if (si && si->get_tz)
    {
        long tz = 0;

        if (si->get_tz(&tz))
        {
            s_tz = tz;
            s_tz_valid = true;
        }
    }

    s_tz_read_us = now_us;
}

/*
 * Build the two fields. `minute_out` is the minute index, which is the gate the
 * task uses to decide whether anything could have changed.
 *
 * A CLOCK THAT IS NOT SET IS REPORTED AS DASHES, not as a plausible wrong time.
 * This project's rule about the RTC is the same one: a watch that says it does
 * not know is better than one that is confidently an hour out. The same is true
 * of an offset that has never been read -- `get_tz()` used to report success
 * while handing back 0, so "I could not find your time zone" and "your time zone
 * is UTC" arrived as the same answer and the strip displayed UTC for both. It now
 * returns false on a failed read and this keeps the previous offset rather than
 * replacing it.
 *
 * The first read happens at start-up, before the desktop has logged in and built
 * the home path, so it fails -- which is why the unknown-retry window is five
 * seconds rather than a minute. `get_tz()` itself caches a failure for thirty
 * seconds, so the strip can show dashes for up to half a minute after a reboot.
 * That is a real wart and it is not fixable here: the retry belongs to the
 * function, and entry 10 of core-log.md records why an immediate retry was tried
 * and reverted -- the function sits on the taskbar's paint path, and retrying
 * there turned one failed open into an open per tick forever.
 */
static void sb_build(sb_field_t f[SB_NFIELDS], int64_t *minute_out)
{
    const td_sysinfo_t *si = td_sysinfo();
    const td_clock_prefs_t *p = td_clock_prefs();
    int64_t utc = 0;
    bool ok = false;
    int mo = 0, d = 0, h = 0, mi = 0;

    *minute_out = 0;

    f[0].scale = SB_SCALE;
    f[1].scale = SB_SCALE;

    sb_refresh_tz(si);

    if (s_tz_valid && si && si->time_now && si->time_now(&utc))
    {
        const long long local = (long long)utc + s_tz;

        /* Floor division, because a negative epoch must not round toward zero
         * and put the clock a day out at midnight. */
        const long long days = local >= 0 ? local / 86400 : -((-local + 86399) / 86400);
        const long long rem = local - days * 86400;

        int year = 0;

        sb_civil_from_days(days, &year, &mo, &d);
        (void)year;                  /* not shown; see the header */

        h = (int)(rem / 3600);
        mi = (int)(rem / 60 % 60);
        ok = true;

        *minute_out = local / 60;
    }

    sb_date(f[0].text, sizeof(f[0].text), p, ok, mo, d);
    sb_clock(f[1].text, sizeof(f[1].text), p, ok, h, mi);
}

/* Lay the two fields out across the band: widths from the text, every field
 * centred on the band's centre row, the pair centred on the chamfer. */
static void sb_layout(const sb_field_t f[SB_NFIELDS], int left[SB_NFIELDS],
                      int top[SB_NFIELDS])
{
    int total = SB_GAP * (SB_NFIELDS - 1);

    for (int i = 0; i < SB_NFIELDS; i++)
    {
        total += (int)strlen(f[i].text) * SB_FONT_W * f[i].scale;
        top[i] = SB_TOP_Y;
    }

    int x = SB_GLASS_CX - total / 2;

    for (int i = 0; i < SB_NFIELDS; i++)
    {
        left[i] = x;
        x += (int)strlen(f[i].text) * SB_FONT_W * f[i].scale + SB_GAP;
    }
}

/* One pixel of one field: `x` is a glass column, `y` a panel row. */
static bool sb_ink(int x, int y, const sb_field_t *f, int left, int top)
{
    const int gw = SB_FONT_W * f->scale;
    const int gh = SB_FONT_H * f->scale;
    const int gx = x - left;
    const int gy = y - top;

    if (gx < 0 || gx >= (int)strlen(f->text) * gw || gy < 0 || gy >= gh)
        return false;

    const int cell = gx / gw;
    const int px = (gx % gw) / f->scale;
    const int py = gy / f->scale;

    if (!f->text[cell])
        return false;

    const uint8_t *glyph = font8x16_glyph((uint16_t)(unsigned char)f->text[cell]);

    return glyph && (glyph[py] & (0x80 >> px)) != 0;
}

/*
 * Push the WHOLE LINE as one rectangle, a band at a time.
 *
 * THE LINE IS ONE RECTANGLE BECAUSE THE FIELDS MOVE, and getting this wrong put
 * two stray dashes on the glass. The strip is centred as a group, so a field's x
 * depends on the OTHER field's width: the placeholder clock `--:--` is five cells
 * and the 12-hour clock ` 7:49 AM` is eight, so when the timezone resolved at boot
 * the date moved 24 px left. Pushing each field over only its own new rectangle
 * left the old date's right-hand dashes sitting where nothing would ever overwrite
 * them, and the operator read the result as `15-11--`.
 *
 * A host harness reproduced that exactly -- 28 stale pixels in one character cell
 * at x 168..183 -- before this function was written. The cure is to push the union
 * of where the line IS and where it WAS: a rectangle that covers both, filled with
 * background first and then the glyphs, so vacated pixels are cleared by the same
 * transfer that draws the new ones. It also makes the whole strip atomic per band:
 * one window, one pixel stream, no interleaving.
 *
 * The cost is that a repaint always moves the union's width rather than each
 * field's, which is at most 224 px against 128 for the widest single field -- 8
 * bands of 4 rows instead of 8 of 8. Once a minute, on a 5.6 MB/s bus.
 */
static bool sb_paint(const sb_field_t f[SB_NFIELDS], const int left[SB_NFIELDS],
                     const int top[SB_NFIELDS])
{
    int x0 = left[0], x1 = left[0] + (int)strlen(f[0].text) * SB_FONT_W * f[0].scale - 1;
    int y0 = top[0], y1 = top[0] + SB_FONT_H * f[0].scale - 1;

    for (int i = 1; i < SB_NFIELDS; i++)
    {
        const int l = left[i];
        const int r = l + (int)strlen(f[i].text) * SB_FONT_W * f[i].scale - 1;
        const int b = top[i] + SB_FONT_H * f[i].scale - 1;

        if (l < x0)
            x0 = l;
        if (r > x1)
            x1 = r;
        if (top[i] < y0)
            y0 = top[i];
        if (b > y1)
            y1 = b;
    }

    /* Cover where it used to be as well, so a field that moved is erased. */
    if (s_prev_valid)
    {
        if (s_prev_x0 < x0)
            x0 = s_prev_x0;
        if (s_prev_x1 > x1)
            x1 = s_prev_x1;
    }

    const int w = x1 - x0 + 1;
    const int h = y1 - y0 + 1;

    if (w <= 0 || w > SB_MAX_W || h <= 0 || h > SB_GLYPH_H * 2)
        return false;

    for (int row = 0; row < h; row += SB_PUSH_ROWS)
    {
        const int rows = h - row > SB_PUSH_ROWS ? SB_PUSH_ROWS : h - row;

        for (int r = 0; r < rows; r++)
        {
            const int y = y0 + row + r;

            for (int c = 0; c < w; c++)
            {
                const int x = x0 + c;
                bool ink = false;

                for (int i = 0; i < SB_NFIELDS; i++)
                {
                    if (sb_ink(x, y, &f[i], left[i], top[i]))
                    {
                        ink = true;
                        break;
                    }
                }

                s_band[r * w + c] = ink ? SB_FG : SB_BG;
            }
        }

        if (!display_panel_push(PANEL_COL_OFFSET + x0, y0 + row, w, rows, s_band))
            return false;
    }

    s_prev_x0 = x0;
    s_prev_x1 = x1;
    s_prev_valid = true;

    return true;
}

static void sb_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "strip up: rows %d..%d, glyph %dx%d at scale %d",
             SB_TOP_Y, SB_TOP_Y + SB_GLYPH_H - 1, SB_GLYPH_W, SB_GLYPH_H, SB_SCALE);

    /*
     * THE GATE IS THE MINUTE INDEX, NOT THE STRING.
     *
     * The first version pushed whenever the formatted text differed from what was
     * last shown, and the operator saw the whole panel flash once a SECOND. The
     * guard was the wrong shape: it depended on the string being stable rather
     * than on the thing that actually causes a redraw. A push now happens only
     * when the minute index changes.
     *
     * The text comparison is kept as a second condition because it is free and it
     * catches the cases the index cannot: a date-format or 12/24-hour change in
     * the Settings app, and the date rolling over at midnight.
     */
    int64_t last_minute = -1;

    for (;;)
    {
        sb_field_t f[SB_NFIELDS];
        int left[SB_NFIELDS], top[SB_NFIELDS];
        int64_t minute = 0;
        char key[SB_KEY_MAX];

        sb_build(f, &minute);
        sb_layout(f, left, top);

        snprintf(key, sizeof(key), "%s|%s", f[0].text, f[1].text);

        if (minute != last_minute || strcmp(key, s_shown) != 0)
        {
            const int64_t t0 = esp_timer_get_time();

            if (sb_paint(f, left, top))
            {
                memcpy(s_shown, key, sizeof(s_shown));
                last_minute = minute;
                s_pushed++;

                ESP_LOGI(TAG, "%s %s pushed in %lld us (#%lu)", f[0].text, f[1].text,
                         (long long)(esp_timer_get_time() - t0),
                         (unsigned long)s_pushed);
            }
            else
            {
                ESP_LOGW(TAG, "push refused for '%s'", key);
            }
        }

        s_ticks++;

        /*
         * ONE SECOND, because the only thing that can change is a minute. Keeping
         * the tick at 1 Hz rather than 60 s is what makes the minute boundary land
         * within a second of the real one instead of up to a minute late, and the
         * cost of the extra wake-ups is two snprintf calls and a strcmp.
         */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*
 * `statusbar` -- what the strip believes, and what the panel is being asked to do.
 *
 * THIS EXISTS BECAUSE THE STRIP SHOWED THE RIGHT TIME AND THEN WENT BACK, and the
 * only way to tell which input had moved was to print them. `utc` is the system
 * clock, `tz` is the offset the strip adds, and the two fields are the result --
 * so a disagreement names its own cause instead of being guessed at. `fmt` and
 * `12h` are the Settings values the fields were built from, because a strip that
 * draws the wrong date order is otherwise indistinguishable from a strip that
 * reads the wrong clock.
 *
 * `pushed` is the strip's own count and `display pushes` is the driver's TOTAL
 * pixel pushes, every band from every writer. Sampling both across a minute
 * separates "the strip wrote its bands" from "something repainted the whole
 * screen": a strip repaint adds eight, where a full repaint adds hundreds.
 */
static int cmd_statusbar(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;
    (void)argc;
    (void)argv;

    const td_sysinfo_t *si = td_sysinfo();
    const td_clock_prefs_t *p = td_clock_prefs();
    int64_t utc = 0;
    long tz = 0;
    const bool have = si && si->time_now && si->time_now(&utc);

    if (si && si->get_tz)
        si->get_tz(&tz);

    sb_field_t f[SB_NFIELDS];
    int left[SB_NFIELDS], top[SB_NFIELDS];
    int64_t minute = 0;

    sb_build(f, &minute);
    sb_layout(f, left, top);

    int64_t us_w = 0, us_x = 0;
    uint32_t dpush = 0;
    display_push_stats(&us_w, &us_x, &dpush);

    printf("date=%s clock=%s\n", f[0].text, f[1].text);
    printf("utc=%lld tz=%ld have=%d fmt=%d 12h=%d\n",
           (long long)utc, tz, have ? 1 : 0,
           p ? (int)p->date_format : -1, p ? (int)p->clock_12h : -1);
    printf("pushes=%lu ticks=%lu shown='%s'\n",
           (unsigned long)s_pushed, (unsigned long)s_ticks, s_shown);
    printf("left=%d,%d glass-cx=%d\n", left[0], left[1], SB_GLASS_CX);

    /* What the last paint actually covered, which is the union of the current line
     * and the previous one. This is the number that catches a stale pixel: if it is
     * narrower than the fields' own extent, a field moved and something was left
     * behind. */
    printf("painted x %d..%d (w=%d prev=%d)\n", s_prev_x0, s_prev_x1,
           s_prev_valid ? s_prev_x1 - s_prev_x0 + 1 : 0, s_prev_valid ? 1 : 0);
    printf("display pushes=%lu\n", (unsigned long)dpush);

    /*
     * THE PATH `get_tz()` WOULD BUILD, AND WHETHER IT CAN BE OPENED.
     *
     * Added after the first sample showed `tz=0` with `-25200` sitting in
     * the file: the offset was set and stored correctly, and the reader was not
     * finding it. Printing the path turns "the reader is wrong" into one of three
     * facts -- the home is empty, the path is right but unopenable, or the value is
     * there and something else is overwriting it.
     */
    printf("home='%s'\n", td_session_home());

    {
        char path[160];
        snprintf(path, sizeof(path), "%s/.tdsh_tz", td_session_home());

        long v = 0;
        FILE *fp = fopen(path, "r");
        int rc = fp ? (fscanf(fp, "%ld", &v) == 1 ? 1 : -1) : 0;
        if (fp)
            fclose(fp);

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
    s_prev_valid = false;
    s_prev_x0 = 0;
    s_prev_x1 = 0;

    if (tdsh_register_command(&s_sb_cmd) != 0)
        ESP_LOGW(TAG, "statusbar command did not register");

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
     * reboot rather than an error. Priority 1 so it never competes with an
     * emulator: a strip that appears a few milliseconds late is invisible.
     */
    if (xTaskCreatePinnedToCore(sb_task, "statusbar", 8192, NULL, 1, NULL, 0) != pdPASS)
    {
        ESP_LOGE(TAG, "no task");
        return false;
    }

    return true;
}
