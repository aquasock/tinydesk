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
 * two-digit hour), and it is centred 8 px right of the chamfered centre at the
 * operator's direction -- see SB_GLASS_CX for the clearance that leaves, which was
 * measured against the font rather than assumed.
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
 * WHERE THE LINE IS CENTRED, AND WHY IT IS NOT THE GLASS CENTRE.
 *
 * The glass is 410 columns but the cuts take the corners, and the strip lives at
 * the very top where they bind hardest. The visible span at the row where ink
 * first appears -- row 10, for a scale-2 glyph block starting at row 6 -- is
 * x 70..330, which is 261 px inclusive. The glass's own centre is 205 and that
 * span's centre is 200.
 *
 * IT IS NO LONGER CENTRED ON EITHER, AT THE OPERATOR'S DIRECTION: half a
 * character to the right. A character is SB_GLYPH_W = 16 px at scale 2, so half
 * is 8, and 200 + 8 = 208. The operator asked for exactly that offset and gave no
 * reason beyond wanting the line moved right, which is recorded here rather than
 * given a motive it did not have; the effect is to trade margin from the left cut
 * to the right.
 *
 * WHAT THAT COSTS, MEASURED RATHER THAN ESTIMATED. A host script compiled this
 * file's font table and every string sb_build() can produce, then checked each
 * inked pixel against the two chamfer lines (`x + y >= 80` and
 * `410 - x + y >= 90`). At the old centre the worst case -- a 12-hour clock with
 * a two-digit hour beside a five-character date, 224 px -- inked x 104..309 with
 * nothing outside. At 208 it inks x 112..317, still with nothing outside: 42 px
 * of clearance on the left against 13 on the right at that row. The margin is now
 * asymmetric by design, and the right side is the side with less of it, so this
 * constant is not free to raise again without re-running that check.
 *
 * Rows 6..9 are inside the band and above the worst case's ink, which is why the
 * binding row is 10 and not the band's top. Every character these strings draw
 * starts at glyph row 2 or below -- digits, letters, `-`, `/` and `:` -- so the
 * topmost ink is row 10. A glyph with ink in its first two rows would instead
 * meet the cut at row 6, where the right limit is x <= 326 rather than 330; the
 * worst case's ink reaches 317, so even that would clear by 9 px.
 */
#define SB_GLASS_CX    208

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

/*
 * AN APP'S CLAIM ON THE WHOLE STRIP.
 *
 * ONE SLOT, because there is one strip. The claim is a copy of a short string
 * rather than a pointer, so the caller's buffer can go away the moment the call
 * returns -- which matters because the caller is an emulator's guest task and its
 * buffers are PSRAM it owns.
 *
 * THE LOCK IS A SPINLOCK AND NOT A MUTEX because the critical sections are a
 * memcpy and a flag each; a mutex here would put a scheduler call between the
 * touch path and the strip for nothing. It is taken from two tasks that never
 * block while holding it.
 */
static char s_app_text[SB_FIELD_MAX];
static bool s_app_on;
static portMUX_TYPE s_app_mux = portMUX_INITIALIZER_UNLOCKED;

/*
 * BANDS THE PANEL REFUSED, AND THIS COUNTER IS THE POINT OF THE CHANGE THAT ADDED IT.
 *
 * A band is refused when another task is mid-transaction on the panel bus
 * (LCD-014: `spi_device_acquire_bus()` answers ESP_ERR_INVALID_STATE rather than
 * waiting). Before `display_panel_push()` reported that, the strip could paint
 * eight bands, lose six of them, call the result a success, record the text as
 * shown, and never try again -- a label that is not on the glass while every
 * counter says it is. This number makes that visible instead of inferable.
 */
static volatile uint32_t s_bands_lost;

/* Whether this personality has a clock at all. See statusbar_clock_present(). */
static bool s_clock_present = true;

void statusbar_clock_present(bool present)
{
    s_clock_present = present;
}

/*
 * THE TASK'S STACK, IN ONE PLACE, AND ITS HANDLE.
 *
 * The size is used by the create call and reported by `statusbar`, so the
 * figure on the glass is always the figure that was asked for. The handle is
 * kept for the same reason: an 8 KB stack chosen to stop a panic is not the
 * same thing as an 8 KB stack that is needed, and the only way to tell them
 * apart is uxTaskGetStackHighWaterMark() -- never-touched bytes at the deepest
 * point the task has ever reached.
 */
/* 6,144, DOWN FROM 8,192. Measured high-water mark on the board: 4,624 bytes
 * unused of 8,192, so 3,568 used. The earlier recorded worst case for this task
 * was 3,644, which agrees. 6,144 leaves 2,576 bytes above the deepest observed
 * use, and 2,500 above the older worst-case figure. This is the task that paints
 * the top strip once a minute, so it is shallow by construction. */
#define SB_STACK       6144
static TaskHandle_t s_task;

/*
 * WHICH HALF OF THE TICK IS DEEP, measured separately.
 *
 * sb_build() ends in the platform's get_tz(), which opens a file on the card,
 * so it carries the FATFS and VFS call chain; sb_paint() ends in a pixel push,
 * so it carries the SPI driver's. Both are deep and neither is obvious, and
 * the size this task needs is set by the deeper of the two -- so each records
 * its own minimum free stack and `statusbar` prints both.
 */
static uint32_t s_wm_build, s_wm_paint;

/* Counters for the `statusbar` command. `s_pushed` is how many times the strip
 * has actually written the panel, which is the number that matters when asking
 * whether the strip is behind something that happens once a minute. */
static volatile uint32_t s_pushed;
static volatile uint32_t s_ticks;

/*
 * WHEN THE LAST PUSH FINISHED, AND HOW LONG IT TOOK.
 *
 * The strip writes the panel on its own task, once a minute, through the same
 * SPI bus the emulator's blitter uses. A blitter that stops part-way through a
 * session therefore has one obvious suspect, and this is the instrument that
 * can confirm or clear it: the time since that push, read from outside.
 */
static volatile int64_t s_last_push_us;
static volatile uint32_t s_last_push_ms;

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
static int sb_build(sb_field_t f[SB_NFIELDS], int64_t *minute_out)
{
    const td_sysinfo_t *si = td_sysinfo();
    const td_clock_prefs_t *p = td_clock_prefs();
    int64_t utc = 0;
    bool ok = false;
    int mo = 0, d = 0, h = 0, mi = 0;

    *minute_out = 0;

    f[0].scale = SB_SCALE;
    f[1].scale = SB_SCALE;

    /*
     * AN APP THAT HAS TAKEN THE STRIP REPLACES THE WHOLE LINE, and this is the
     * earliest exit on purpose: while an app holds it there is no date to format
     * and no clock to read, so the work below is not done at all rather than done
     * and discarded.
     *
     * NO MINUTE INDEX IS REPORTED, because a minute is a property of the clock and
     * the clock is not on this line any more. The string comparison in the task
     * loop is what repaints, which is what makes an age that changes on its own
     * schedule -- 4x faster at 4x speed -- visible without a second rule.
     */
    if (statusbar_app_text_get(f[0].text, sizeof(f[0].text)))
        return 1;

    /* NOTHING TO SHOW AND NO CLOCK TO SHOW IT WITH: see statusbar_clock_present().
     * Slot B is the personality this exists for -- its strip is there for an app's
     * text and would otherwise sit on placeholders for ever. */
    if (!s_clock_present)
        return 0;

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

    return SB_NFIELDS;
}

/* Lay the fields out across the band: widths from the text, every field
 * centred on the band's centre row, the group centred on the chamfer. `n` is 2
 * for the date and clock and 1 for an app's claim, which is why it is a
 * parameter and not SB_NFIELDS. */
static void sb_layout(const sb_field_t f[], int n, int left[], int top[])
{
    int total = SB_GAP * (n - 1);

    for (int i = 0; i < n; i++)
    {
        total += (int)strlen(f[i].text) * SB_FONT_W * f[i].scale;
        top[i] = SB_TOP_Y;
    }

    int x = SB_GLASS_CX - total / 2;

    for (int i = 0; i < n; i++)
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
/*
 * A REFUSED BAND IS RETRIED IN PLACE, briefly, before it is counted as lost.
 *
 * The refusal is momentary by definition: it means another task was mid
 * transaction on the panel bus at that instant, and those last milliseconds. Four
 * attempts five milliseconds apart is well inside one panel refresh and costs
 * nothing on the common path, where the first one succeeds. Waiting for the task's
 * next one-second tick instead would leave a half-painted label on the glass for
 * up to a second every time the emulator happened to be blitting.
 */
#define SB_PUSH_TRIES  4
#define SB_PUSH_GAP_MS 5

static bool sb_paint(const sb_field_t f[], int n, const int left[],
                     const int top[])
{
    int x0 = left[0], x1 = left[0] + (int)strlen(f[0].text) * SB_FONT_W * f[0].scale - 1;
    int y0 = top[0], y1 = top[0] + SB_FONT_H * f[0].scale - 1;

    for (int i = 1; i < n; i++)
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

    bool ok = true;

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

                for (int i = 0; i < n; i++)
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

        bool band = false;

        for (int attempt = 0; attempt < SB_PUSH_TRIES && !band; attempt++)
        {
            if (attempt)
                vTaskDelay(pdMS_TO_TICKS(SB_PUSH_GAP_MS));

            band = display_panel_push(PANEL_COL_OFFSET + x0, y0 + row, w, rows, s_band);
        }

        if (!band)
        {
            /* ATTEMPT EVERY BAND ANYWAY, and remember that one was lost.
             *
             * Returning at the first refused band used to look equivalent, and it
             * is not: the caller keeps `s_prev` at the last SUCCESSFUL rectangle so
             * the retry covers where the line used to be, and a partially painted
             * line leaves glyph fragments behind. Pushing the remaining bands costs
             * a few hundred microseconds and leaves fewer fragments for the retry
             * to clean up. */
            ok = false;
            s_bands_lost++;
        }
    }

    /* THE RECTANGLE IS ONLY REMEMBERED WHEN EVERY BAND LANDED. If it is remembered
     * after a dropped band, the retry's union shrinks to the new line alone and the
     * old line's pixels are stranded on the glass for ever -- which is exactly how
     * a strip that "painted successfully" ends up showing the previous text. */
    if (!ok)
        return false;

    s_prev_x0 = x0;
    s_prev_x1 = x1;
    s_prev_valid = true;

    return true;
}

/* Blank whatever the strip last drew, for the no-clock personality when nothing
 * holds the claim. Idempotent: it returns immediately when the glass is already
 * empty, so it can run on every tick. If a band is refused it leaves
 * `s_prev_valid` set, which is what makes the next tick try again -- the same rule
 * sb_paint() follows. */
static void sb_blank(void)
{
    if (!s_prev_valid)
        return;

    const int w = s_prev_x1 - s_prev_x0 + 1;

    if (w <= 0 || w > SB_MAX_W)
    {
        s_prev_valid = false;
        return;
    }

    for (int row = 0; row < SB_GLYPH_H; row += SB_PUSH_ROWS)
    {
        const int rows = SB_GLYPH_H - row > SB_PUSH_ROWS ? SB_PUSH_ROWS : SB_GLYPH_H - row;

        for (int i = 0; i < rows * w; i++)
            s_band[i] = SB_BG;

        if (!display_panel_push(PANEL_COL_OFFSET + s_prev_x0, SB_TOP_Y + row,
                                w, rows, s_band))
            return;                  /* try again next tick */
    }

    s_prev_valid = false;
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

        const int nf = sb_build(f, &minute);

        {
            const uint32_t wm = (uint32_t)uxTaskGetStackHighWaterMark(NULL);

            if (!s_wm_build || wm < s_wm_build)
                s_wm_build = wm;
        }

        /* NOTHING TO DRAW: blank the strip if anything is on it and go round again.
         * This is the no-clock personality with no app claim, which is slot B's
         * idle state. */
        if (nf == 0)
        {
            sb_blank();
            s_ticks++;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        sb_layout(f, nf, left, top);

        /* THE KEY CARRIES WHICH MODE BUILT IT, so taking the strip and giving it
         * back repaint even when the text happens to be the same length. */
        if (nf == 1)
            snprintf(key, sizeof(key), "app|%s", f[0].text);
        else
            snprintf(key, sizeof(key), "%s|%s", f[0].text, f[1].text);

        if (minute != last_minute || strcmp(key, s_shown) != 0)
        {
            const int64_t t0 = esp_timer_get_time();

            const bool painted = sb_paint(f, nf, left, top);

            {
                const uint32_t wm = (uint32_t)uxTaskGetStackHighWaterMark(NULL);

                if (!s_wm_paint || wm < s_wm_paint)
                    s_wm_paint = wm;
            }

            if (painted)
            {
                memcpy(s_shown, key, sizeof(s_shown));
                last_minute = minute;
                s_pushed++;

                s_last_push_us = esp_timer_get_time();
                s_last_push_ms = (uint32_t)((s_last_push_us - t0) / 1000);

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

    const int nf = sb_build(f, &minute);
    sb_layout(f, nf, left, top);

    int64_t us_w = 0, us_x = 0;
    uint32_t dpush = 0;
    display_push_stats(&us_w, &us_x, &dpush);

    if (nf == 1)
        printf("app='%s' date and clock suppressed\n", f[0].text);
    else
        printf("date=%s clock=%s\n", f[0].text, f[1].text);
    printf("utc=%lld tz=%ld have=%d fmt=%d 12h=%d\n",
           (long long)utc, tz, have ? 1 : 0,
           p ? (int)p->date_format : -1, p ? (int)p->clock_12h : -1);
    printf("pushes=%lu ticks=%lu shown='%s'\n",
           (unsigned long)s_pushed, (unsigned long)s_ticks, s_shown);
    printf("bands lost=%lu\n", (unsigned long)s_bands_lost);
    printf("left=%d,%d glass-cx=%d\n", left[0], nf > 1 ? left[1] : 0,
           SB_GLASS_CX);

    /* What the last paint actually covered, which is the union of the current line
     * and the previous one. This is the number that catches a stale pixel: if it is
     * narrower than the fields' own extent, a field moved and something was left
     * behind. */
    printf("painted x %d..%d (w=%d prev=%d)\n", s_prev_x0, s_prev_x1,
           s_prev_valid ? s_prev_x1 - s_prev_x0 + 1 : 0, s_prev_valid ? 1 : 0);
    printf("display pushes=%lu\n", (unsigned long)dpush);

    /*
     * WHAT THE STACK ACTUALLY USED, at its deepest point ever.
     *
     * `free` is never-touched bytes from the current stack pointer to the end
     * of the stack, so it is a high-water mark rather than an instantaneous
     * reading -- a value that only shrinks. Read from THIS task, which is the
     * shell's, not the strip's: asking a task about its own stack would measure
     * whatever depth the query itself reached.
     *
     * An 8192-byte stack was chosen because 3072 panicked, which is a ceiling
     * and not a requirement. This line is what makes the difference visible.
     */
    if (s_task)
    {
        printf("stack free=%u of %d\n",
               (unsigned)uxTaskGetStackHighWaterMark(s_task), SB_STACK);
        printf(" deep: build<=%u paint<=%u\n",
               (unsigned)s_wm_build, (unsigned)s_wm_paint);
    }
    else
        printf("stack none -- no task\n");

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

void statusbar_push_stats(uint32_t *pushes, uint32_t *last_ms, int64_t *since_ms)
{
    if (pushes)
        *pushes = s_pushed;
    if (last_ms)
        *last_ms = s_last_push_ms;
    if (since_ms)
        *since_ms = s_last_push_us ?
                   (esp_timer_get_time() - s_last_push_us) / 1000 : -1;
}

/*
 * The claim's two halves, and neither of them paints.
 *
 * THE SETTER REFUSES A TEXT IT CANNOT HOLD rather than truncating it: a
 * truncated age reads as a different age, which is the same reason
 * tama_age_format() refuses a short buffer. It also returns false so a caller can
 * tell "the strip is yours" from "the strip did not take it" -- the difference
 * between an app that knows its number is on the glass and one that does not.
 *
 * THE GETTER IS THE STRIP TASK'S ONLY VIEW OF THE CLAIM, and it copies under the
 * lock so a setter on another core cannot change the bytes mid-copy. A
 * non-reentrant memcpy out of a buffer another core writes is the kind of fault
 * that appears once a fortnight as one wrong character.
 */
bool statusbar_app_text(const char *text)
{
    const size_t n = text ? strlen(text) : 0;

    if (n >= SB_FIELD_MAX)
        return false;

    portENTER_CRITICAL(&s_app_mux);

    if (n == 0)
    {
        s_app_text[0] = '\0';
        s_app_on = false;
    }
    else
    {
        memcpy(s_app_text, text, n + 1);
        s_app_on = true;
    }

    portEXIT_CRITICAL(&s_app_mux);

    return true;
}

bool statusbar_app_text_get(char *out, size_t cap)
{
    bool on;

    if (!out || cap == 0)
        return false;

    portENTER_CRITICAL(&s_app_mux);

    on = s_app_on;

    if (on)
    {
        const size_t n = strlen(s_app_text);

        if (n >= cap)
        {
            /* The strip's field is the same size as the claim's, so this cannot
             * happen from the setter -- but a caller that asked for less gets no
             * text rather than half of one. */
            out[0] = '\0';
            on = false;
        }
        else
            memcpy(out, s_app_text, n + 1);
    }

    portEXIT_CRITICAL(&s_app_mux);

    return on;
}

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
    if (xTaskCreatePinnedToCore(sb_task, "statusbar", SB_STACK, NULL, 1,
                                &s_task, 0) != pdPASS)
    {
        ESP_LOGE(TAG, "no task");
        return false;
    }

    return true;
}
