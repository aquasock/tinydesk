/*
 * imu.c - the wrist's inertial sensor, live: which way it is pointing and how it
 *         is moving. Opened from the start menu.
 *
 * WHAT IT SHOWS AND WHY THOSE THINGS. Acceleration in g, and orientation in
 * degrees when the board's part computes it. Acceleration alone answers "is this
 * moving" but not "which way am I facing"; orientation alone loses the sharp
 * detail of a tap or a shake. On a wrist the second question is the one people
 * ask, so heading leads and the three axes sit under it.
 *
 * THE SENSOR IS ON ONLY WHILE THIS WINDOW IS OPEN, and that is the port's
 * decision expressed through td_imu_ops_t: start() on open, stop() on close. It
 * matters more here than for the GPS app, because this part's first start()
 * uploads about 121 KB of firmware into it -- seconds of work, once per boot.
 * The window therefore opens immediately and calls start() from its tick rather
 * than during launch, so the desktop never blocks waiting for a sensor.
 *
 * A SAMPLE COUNT IS SHOWN. A reading that has stopped updating looks exactly
 * like a reading that has not: the numbers stay put either way. The count is the
 * difference between "the watch is perfectly still" and "the link died", and
 * that distinction is the whole reason this project keeps a sample counter on
 * every sensor it talks to.
 *
 * Everything goes through td_sysinfo()->imu, supplied by the port, so this file
 * compiles and runs on a board with no inertial sensor at all -- it is simply
 * not registered there.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "td_apps.h"

#define IMU_W 34
#define IMU_H 14
#define TICK_MS 250

static td_window_t *s_win;
static td_widget_t *s_part, *s_fw, *s_head, *s_pitch, *s_roll;
static td_widget_t *s_ax, *s_ay, *s_az, *s_count, *s_msg;

static char s_reason[64];    /* why the sensor did not start, kept on screen
                              * while the window is open: it is advice, not a
                              * one-off message */

static const td_imu_ops_t *imu(void)
{
    return td_sysinfo()->imu;
}

/* One line: heading, or a dash when the part does not compute it. A part with
 * acceleration but no fusion still deserves a window; it just shows fewer
 * numbers. */
static void set_or_dash(td_widget_t *w, bool valid, const char *fmt, float v)
{
    char buf[24];
    if (valid)
        snprintf(buf, sizeof(buf), fmt, (double)v);
    else
        snprintf(buf, sizeof(buf), "--");
    td_widget_set_text(w, buf);
}

static void on_draw(td_window_t *win, int w, int h)
{
    (void)win;
    const td_theme_t *t = td_theme();
    td_fill(td_rect(0, 0, w, h), ' ', t->win_fg, t->win_bg);
}

static void on_tick(td_window_t *win)
{
    const td_imu_ops_t *m = imu();
    if (!m)
        return;

    /* START LAZILY, FROM THE TICK. The first start uploads firmware and takes
     * seconds; doing that during launch would freeze the desktop. */
    static bool started;
    if (!started)
    {
        const char *why = "";
        if (m->start(&why))
        {
            started = true;
            s_reason[0] = '\0';
        }
        else
        {
            snprintf(s_reason, sizeof(s_reason), "%s", why ? why : "no sensor");
            td_widget_set_text(s_msg, s_reason);
            return;
        }
    }

    td_imu_t r;
    if (!m->latest(&r))
    {
        td_widget_set_text(s_msg, "waiting for the first sample...");
        return;
    }
    s_reason[0] = '\0';
    td_widget_set_text(s_msg, "");

    td_widget_set_text(s_part, r.part[0] ? r.part : "?");
    td_widget_set_text(s_fw, r.version[0] ? r.version : "--");

    set_or_dash(s_head, r.ori_valid, "%.0f", r.heading);
    set_or_dash(s_pitch, r.ori_valid, "%.0f", r.pitch);
    set_or_dash(s_roll, r.ori_valid, "%.0f", r.roll);

    char buf[24];
    if (r.accel_valid)
    {
        snprintf(buf, sizeof(buf), "%+.2f", (double)r.ax);
        td_widget_set_text(s_ax, buf);
        snprintf(buf, sizeof(buf), "%+.2f", (double)r.ay);
        td_widget_set_text(s_ay, buf);
        snprintf(buf, sizeof(buf), "%+.2f", (double)r.az);
        td_widget_set_text(s_az, buf);
    }
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)r.accel_samples);
    td_widget_set_text(s_count, buf);

    td_win_invalidate(win);
}

static void on_close(td_window_t *win)
{
    (void)win;
    s_win = NULL;
    if (imu())
        imu()->stop();      /* leave the part quiet; see the header comment */
}

static void launch(void)
{
    if (imu() == NULL)
        return;
    if (td_win_is_open(s_win))
    {
        td_win_focus(s_win);
        return;
    }

    td_window_desc_t d = {0};
    d.title = "IMU";
    d.rect = td_rect(-1, -1, IMU_W, IMU_H);
    d.flags = TD_WIN_MOVABLE | TD_WIN_CLOSABLE;
    d.on_draw = on_draw;
    d.on_tick = on_tick;
    d.on_close = on_close;
    d.tick_ms = TICK_MS;
    s_win = td_win_create(&d);
    if (s_win == NULL)
        return;

    const td_theme_t *t = td_theme();
    int y = 0;
    (void)t;

    s_part = td_label(s_win, 1, y, 28, "");
    y += 1;
    s_fw = td_label(s_win, 1, y, 30, "");
    y += 2;

    td_label(s_win, 1, y, 30, "Heading / pitch / roll");
    y += 1;
    s_head = td_label(s_win, 1, y, 25, "--");
    s_pitch = td_label(s_win, 12, y, 25, "--");
    s_roll = td_label(s_win, 23, y, 25, "--");
    y += 2;

    td_label(s_win, 1, y, 30, "Acceleration, in g");
    y += 1;
    s_ax = td_label(s_win, 1, y, 25, "--");
    s_ay = td_label(s_win, 12, y, 25, "--");
    s_az = td_label(s_win, 23, y, 25, "--");
    y += 2;

    td_label(s_win, 1, y, 12, "Samples");
    s_count = td_label(s_win, 12, y, 20, "0");
    y += 1;

    s_msg = td_label(s_win, 1, y, IMU_W - 2, "");

    on_tick(s_win);
}

static const td_app_t s_app = {"IMU", launch, "IM"};

void td_imu_register(void)
{
    if (td_sysinfo()->imu)
        td_app_register(&s_app);
}
