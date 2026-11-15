/*
 * gps.c - the GNSS receiver's state: whether it has a fix, where it is, and how
 *         much sky it can hear. Opened from the start menu.
 *
 * WHY IT SHOWS SATELLITES IN VIEW AND NOT ONLY SATELLITES USED. A receiver
 * indoors is working perfectly and cannot fix, because a fix needs four
 * satellites and a building admits fewer. Reporting only the used count makes
 * that healthy receiver look broken: it reads zero, and zero is also what a
 * disconnected antenna reads. Satellites IN VIEW separates the two, and it is
 * the number this project's own `gps raw` command reached for when it needed to
 * tell "no antenna" from "no fix" (GPS-002). So both are shown, and a "no fix"
 * line beside a healthy view count is a truthful and useful reading.
 *
 * THE RECEIVER IS POWERED ONLY WHILE THIS WINDOW IS OPEN. That is the port's
 * choice, expressed through td_gps_ops_t: start() on open, stop() on close. A
 * GNSS module draws tens of milliamps and a watch does not have them to spare
 * for a window nobody is looking at. The cost is that the first reading after
 * opening is empty and a cold fix takes minutes -- the window therefore shows
 * what it knows at once (the view count appears in a second or two) rather than
 * appearing blank until a fix arrives.
 *
 * Everything goes through td_sysinfo()->gps, supplied by the port, so this file
 * compiles and runs on a board with no receiver at all and says so.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "td_apps.h"

/*
 * The desktop is 40x26 on this board but other ports are wider, so the window is
 * laid out in a block narrow enough for the smallest and left at that width. A
 * label's width is a minimum, so a wider desktop simply has more space to the
 * right of the figures rather than a different layout to maintain.
 */
#define GPS_W 38
#define GPS_H 13

static td_window_t *s_win;
static td_widget_t *s_fix, *s_lat, *s_lon, *s_sats, *s_hdop, *s_alt, *s_utc, *s_rx;
static td_widget_t *s_msg;
static char s_reason[64];   /* why the receiver did not start, shown in place of
                             * the readings rather than in a message box: it is
                             * advice to keep on screen while the window is open */

static const td_gps_ops_t *gps(void)
{
    return td_sysinfo()->gps;
}

/* Fix quality, in words. The numeric codes are the receiver's; a user wants to
 * know whether the position can be trusted, and "RTK fixed" is a meaningfully
 * different claim from "GPS". */
static const char *fix_name(int fix)
{
    switch (fix)
    {
    case TD_GPS_FIX_GPS:
        return "GPS";
    case TD_GPS_FIX_DGPS:
        return "DGPS";
    case TD_GPS_FIX_PPS:
        return "PPS";
    case TD_GPS_FIX_RTK:
        return "RTK float";
    case TD_GPS_FIX_FRTK:
        return "RTK fixed";
    case TD_GPS_FIX_DR:
        return "dead reckoning";
    case TD_GPS_FIX_NONE:
    default:
        return "no fix";
    }
}

/*
 * 1e-7 degrees to a signed decimal string with five decimal places.
 *
 * FIVE, not seven, because that is what the receiver measures: NMEA gives four
 * decimal places of arc-minute, about 1.7e-6 degrees, so a fifth decimal place
 * is the last one that means anything and a seventh would be invented digits.
 *
 * The magnitude is taken in int64 before negating, because INT32_MIN has no
 * positive int32 counterpart and a coordinate at exactly -180 degrees would be
 * the value to overflow on.
 */
static void fmt_coord(char *out, size_t cap, int32_t v, char pos, char neg)
{
    const char hemi = v < 0 ? neg : pos;
    const int64_t a = v < 0 ? -(int64_t)v : (int64_t)v;

    snprintf(out, cap, "%ld.%05ld %c", (long)(a / 10000000),
             (long)((a % 10000000) / 100), hemi);
}

static void update(void)
{
    const td_gps_ops_t *g = gps();
    td_gps_t s;

    memset(&s, 0, sizeof(s));

    if (!g || !g->status || !g->status(&s))
    {
        /* Not running: either the receiver refused to start, or this window is
         * being redrawn after a close. Say which rather than showing stale
         * figures. */
        td_widget_printf(s_fix, "Receiver:  %s", s_reason[0] ? s_reason : "not started");
        td_widget_printf(s_lat, "Latitude:  -");
        td_widget_printf(s_lon, "Longitude: -");
        td_widget_printf(s_sats, "Satellites: -");
        td_widget_printf(s_hdop, "HDOP:      -");
        td_widget_printf(s_alt, "Altitude:  -");
        td_widget_printf(s_utc, "UTC:       -");
        return;
    }

    /*
     * THE FIX LINE CARRIES THE DISTINCTION THE WHOLE WINDOW EXISTS FOR. "no fix"
     * with satellites in view is a receiver waiting for a better sky; "no fix"
     * with none is an antenna that hears nothing. The second is a fault and the
     * first is not, so they must not read the same.
     */
    if (s.fix > TD_GPS_FIX_NONE)
        td_widget_printf(s_fix, "Fix:       %s", fix_name(s.fix));
    else if (s.sats_in_view > 0)
        td_widget_printf(s_fix, "Fix:       none (%d heard)", s.sats_in_view);
    else
        td_widget_printf(s_fix, "Fix:       none, no sky seen");

    if (s.have_pos)
    {
        char buf[24];

        fmt_coord(buf, sizeof(buf), s.lat_e7, 'N', 'S');
        td_widget_printf(s_lat, "Latitude:  %s", buf);
        fmt_coord(buf, sizeof(buf), s.lon_e7, 'E', 'W');
        td_widget_printf(s_lon, "Longitude: %s", buf);
    }
    else
    {
        /* A receiver without a fix sends an empty position, and an empty field
         * read as a number would put a plausible point in the Gulf of Guinea. */
        td_widget_printf(s_lat, "Latitude:  -");
        td_widget_printf(s_lon, "Longitude: -");
    }

    td_widget_printf(s_sats, "Satellites: %d in view, %d used", s.sats_in_view, s.sats_used);

    if (s.hdop_valid)
        td_widget_printf(s_hdop, "HDOP:      %d.%02d", s.hdop_x100 / 100, s.hdop_x100 % 100);
    else
        td_widget_printf(s_hdop, "HDOP:      -");

    if (s.have_alt)
        td_widget_printf(s_alt, "Altitude:  %ld m", (long)(s.alt_cm / 100));
    else
        td_widget_printf(s_alt, "Altitude:  -");

    if (s.time_valid)
        td_widget_printf(s_utc, "UTC:       %02d:%02d:%02d", s.hour, s.minute, s.second);
    else
        td_widget_printf(s_utc, "UTC:       waiting");
}

/*
 * The clock button.
 *
 * THE RESULT IS REPORTED AS A WORD, NOT AS A SILENT SUCCESS. A button that does
 * nothing visible is indistinguishable from one whose press was missed -- and
 * this is the one control here that changes something outside the window, so it
 * has to say so. The failure case is the common one for the first half minute
 * after opening: the receiver has no UTC yet and the press must say that rather
 * than appear to have worked.
 */
static void on_set_clock(td_widget_t *w, void *user)
{
    (void)w;
    (void)user;

    const td_gps_ops_t *g = gps();

    if (!g || !g->set_time)
    {
        td_widget_printf(s_msg, "This board cannot set the clock.");
        return;
    }

    if (!g->set_time())
    {
        td_widget_printf(s_msg, "No time from the receiver yet.");
        return;
    }

    td_gps_t s;

    memset(&s, 0, sizeof(s));
    if (g->status && g->status(&s) && s.time_valid)
        td_widget_printf(s_msg, "Clock set to %02d:%02d:%02d UTC.", s.hour, s.minute, s.second);
    else
        td_widget_printf(s_msg, "Clock set.");

    update();
}

static void on_tick(td_window_t *win)
{
    (void)win;
    update();
}

static void on_close(td_window_t *win)
{
    (void)win;

    /* The window is the reason the receiver was powered, so losing it is the
     * moment to drop the rail. Done here rather than in a timer because it must
     * happen on every path out -- the close button, an app taking the screen,
     * shutdown. */
    if (gps() && gps()->stop)
        gps()->stop();

    s_win = NULL;
    s_msg = NULL;
    s_reason[0] = '\0';
}

static void launch(void)
{
    if (td_win_is_open(s_win))
    {
        td_win_focus(s_win);
        return;
    }

    const td_gps_ops_t *g = gps();

    if (!g || !g->start || !g->status)
    {
        td_msgbox("GPS", "This board has no GNSS receiver,\nor none is configured.", "OK", NULL, NULL);
        return;
    }

    s_reason[0] = '\0';

    const char *why = NULL;

    if (!g->start(&why))
    {
        /* A receiver that will not power is still worth a window: the reason is
         * the useful part, and closing it is obvious. */
        snprintf(s_reason, sizeof(s_reason), "%s", (why && *why) ? why : "would not start");
    }

    td_window_desc_t d = {
        .title = "GPS",
        .rect = td_rect(1, 1, GPS_W, GPS_H),
        .flags = TD_WIN_MOVABLE | TD_WIN_CLOSABLE,
        .on_close = on_close,
        .on_tick = on_tick,
        .tick_ms = 1000,
    };
    s_win = td_win_create(&d);
    if (!s_win)
    {
        if (g->stop)
            g->stop();          /* do not leave the rail up behind a failed window */
        return;
    }

    /* Eight readings and the receiver's own name, which is the model when the
     * unit answered the identification poll and empty when it did not -- most do
     * not, which GPS-003 records, so the line is dropped rather than shown
     * blank. */
    s_fix = td_label(s_win, 1, 0, 0, "");
    s_lat = td_label(s_win, 1, 1, 0, "");
    s_lon = td_label(s_win, 1, 2, 0, "");
    s_sats = td_label(s_win, 1, 3, 0, "");
    s_hdop = td_label(s_win, 1, 4, 0, "");
    s_alt = td_label(s_win, 1, 5, 0, "");
    s_utc = td_label(s_win, 1, 6, 0, "");
    s_rx = td_label(s_win, 1, 8, 0, "");

    /* The message and the button sit on the client's LAST TWO ROWS, addressed
     * negatively so they follow the window if it is resized -- the bug the
     * Network app's list hit when it computed those rows from the height it was
     * given at launch. */
    s_msg = td_label(s_win, 1, -2, 0, "");
    if (g->set_time)
        td_button(s_win, 0, -1, "Set clock", on_set_clock, NULL);

    update();

    /* The receiver's own name, on its own row and empty when unknown. Most units
     * on this board do not answer the identification poll (GPS-003), so an empty
     * row is the common case and is better than a line claiming a model. */
    if (s_rx)
    {
        td_gps_t st;

        memset(&st, 0, sizeof(st));
        if (g->status(&st) && st.model[0])
            td_widget_printf(s_rx, "Receiver:  %s", st.model);
    }
}

static const td_app_t s_app = {"GPS", launch, "()"};

void td_gps_register(void)
{
    td_app_register(&s_app);
}
