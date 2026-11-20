/*
 * imu.c - the wrist's inertial sensor, live: which way it is pointing, how it is
 *         moving, and a button that teaches the watch a gesture. Opened from the
 *         start menu.
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
 * THE GESTURE SECTION IS OPTIONAL AND THAT IS NOT DEFENSIVENESS. It exists only
 * where the port supplies learn_begin, so a board with an accelerometer but no
 * learning engine gets the same window it always had rather than a button that
 * cannot work. Nothing here talks to a Bosch part: the whole pass is the port's
 * state machine, driven by the latest() call this tick already makes.
 *
 * THE PERCENTAGE CAN GO DOWN, AND THE WINDOW SAYS SO. The engine counts
 * repetitions of one movement and returns to 0 when it decides the next
 * repetition is a different movement -- that is it working, not failing. A bar
 * that only ever climbed would be misreporting the part, and a wearer who
 * watched it drop would reasonably conclude the watch was broken. So the reason
 * the engine gives is shown in words beneath the bar.
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
#define IMU_H 14        /* without a learning engine */
#define IMU_H_LEARN 19  /* with one: the gesture section needs five rows */
#define IMU_H_MAP 25    /* plus the "opens" section: a title, a button, a box */
#define TICK_MS 150     /* fast enough that a percent readout moves as you move */

static td_window_t *s_win;
static td_widget_t *s_part, *s_fw, *s_up, *s_head, *s_pitch, *s_roll;
static td_widget_t *s_ax, *s_ay, *s_az, *s_count, *s_msg;
static td_widget_t *s_learn_btn, *s_bar, *s_gmsg;
static td_widget_t *s_app_btn, *s_arm_box, *s_ammap;

/* True while a pass this window started is running. The port knows too, but a
 * window that has to guess from a tick whether it owns a pass will get it wrong
 * exactly once -- when the pass ends between two ticks -- and either strand the
 * button on "Stop" or cancel somebody else's. */
static bool s_was_learning;

static char s_reason[64];    /* why the sensor did not start, kept on screen
                              * while the window is open: it is advice, not a
                              * one-off message */

static const td_imu_ops_t *imu(void)
{
    return td_sysinfo()->imu;
}

/* Whether this port can learn at all. Asked of the ops table rather than of a
 * field in the reading, because the layout is decided at launch and no reading
 * exists yet. */
static bool can_learn(void)
{
    const td_imu_ops_t *m = imu();
    return m && m->learn_begin && m->learn_cancel;
}

/* Whether this port can map a gesture to an app and watch for it. Separate from
 * can_learn because they are different capabilities: a port could learn without
 * being able to launch anything. */
static bool can_map(void)
{
    const td_imu_ops_t *m = imu();
    return m && m->gapp && m->garmed && m->gset && m->arm && m->disarm &&
           m->detect;
}

static void on_draw(td_window_t *win, int w, int h)
{
    (void)win;
    const td_theme_t *t = td_theme();
    td_fill(td_rect(0, 0, w, h), ' ', t->win_fg, t->win_bg);
}

/* Learn starts a pass; Stop abandons one. One button, because a wearer pressing
 * it twice in a row means "stop", and two buttons for a two-state control would
 * waste a row of a 34-column window. */
static void on_learn(td_widget_t *w, void *user)
{
    (void)w;
    (void)user;
    const td_imu_ops_t *m = imu();
    if (!m || !can_learn())
        return;

    if (s_was_learning)
    {
        m->learn_cancel();
        s_was_learning = false;
        td_widget_set_text(s_learn_btn, "Learn");
        td_widget_set_text(s_gmsg, "stopped");
        return;
    }

    /* THE SENSOR HAS TO BE STREAMING FIRST. begin() needs the part up, and a
     * press can arrive while the firmware is still uploading. start() is the
     * same call the tick makes and is cheap once it has taken. */
    const char *why = "";
    if (!m->start(&why))
    {
        td_widget_set_text(s_gmsg, why && *why ? why : "sensor not ready");
        return;
    }

    if (!m->learn_begin())
    {
        td_widget_set_text(s_gmsg, "the engine would not start learning");
        return;
    }

    s_was_learning = true;
    td_widget_set_text(s_learn_btn, "Stop");
    td_progress_set(s_bar, 0);
    td_widget_set_text(s_gmsg, "repeat one gesture, the same way");
}

/* ------------------------------------------------------- the app picker */

/* EVERY REGISTERED APP, plus "(none)" at the top to clear a mapping. Built from
 * the registry rather than a list written here, because the registry is what the
 * Start menu shows -- a hardcoded list would offer apps that do not exist on some
 * build and miss the ones that do. */
static int picker_index_to_app(int i)
{
    return i - 1;             /* 0 is "(none)" */
}

static const char *picker_item(td_widget_t *w, int i, int *fg, void *user)
{
    (void)w;
    (void)user;
    if (i < 0 || i >= td_app_count() + 1)
        return "";
    if (i == 0)
    {
        *fg = td_theme()->dim;
        return "(none)";
    }
    *fg = td_theme()->win_fg;
    return td_app_get(picker_index_to_app(i))->name;
}

static td_window_t *s_picker;

static void on_pick_cancel(td_widget_t *w, void *user)
{
    (void)w;
    (void)user;
    if (s_picker)
        td_win_close(s_picker);
    s_picker = NULL;
}

static void picker_choose(td_widget_t *list, void *user)
{
    (void)user;
    const td_imu_ops_t *m = imu();
    if (!m || !can_map())
        return;

    const int i = td_list_selected(list);
    const char *app = (i <= 0) ? "" : td_app_get(picker_index_to_app(i))->name;

    /* KEEP THE EXISTING SWITCH when only the app changes. Clearing the mapping
     * should not silently switch detection off, and choosing an app should not
     * switch it on behind the wearer's back -- the checkbox is the control for
     * that, and it is right there. */
    if (!m->gset(app, m->garmed()))
        td_widget_set_text(s_ammap, "could not save -- try again");
    else
        td_widget_set_text(s_ammap, app[0] ? "" : "opens nothing");

    if (s_picker)
        td_win_close(s_picker);
    s_picker = NULL;
    if (s_app_btn)
        td_widget_set_text(s_app_btn, app[0] ? app : "(none)");
}

static void on_pick(td_widget_t *w, void *user)
{
    (void)w;
    (void)user;
    if (s_picker)
    {
        td_win_focus(s_picker);
        return;
    }

    td_window_desc_t d = {0};
    d.title = "Opens...";
    d.rect = td_rect(-1, -1, 30, 14);
    d.flags = TD_WIN_MOVABLE | TD_WIN_CLOSABLE;
    s_picker = td_win_create(&d);
    if (!s_picker)
        return;

    td_widget_t *list = td_list(s_picker, td_rect(1, 1, -2, -3), picker_item,
                                picker_choose, NULL);
    td_list_set_count(list, td_app_count() + 1);

    /* START ON THE CURRENT MAPPING, so opening the picker shows where you are
     * rather than always at the top. */
    const char *cur = imu()->gapp();
    int sel = 0;
    for (int i = 0; cur && cur[0] && i < td_app_count(); i++)
    {
        if (!strcmp(td_app_get(i)->name, cur))
        {
            sel = i + 1;
            break;
        }
    }
    td_list_select(list, sel);
    td_widget_focus(list);

    td_button(s_picker, 1, -1, "Cancel", on_pick_cancel, NULL);
}

static void on_arm(td_widget_t *w, void *user)
{
    (void)user;
    const td_imu_ops_t *m = imu();
    if (!m || !can_map())
        return;

    const bool want = (w->value != 0);
    if (!m->gset(m->gapp(), want))
    {
        /* PUT THE BOX BACK where it was. A checkbox that shows a state the store
         * refused is a lie about what will happen at the next boot. */
        td_checkbox_set(s_arm_box, !want);
        td_widget_set_text(s_ammap, "could not save -- try again");
        return;
    }
    td_widget_set_text(s_ammap, "");
}

/* Keep the mapping controls showing what the STORE says, not what was last
 * clicked. The store is the truth and it can change underneath -- a failed save,
 * a re-learn that kept the old mapping, or a reboot. */
static void show_mapping(td_window_t *win, const td_imu_t *r)
{
    (void)win;
    if (!s_app_btn)
        return;

    const td_imu_ops_t *m = imu();
    const char *app = m->gapp ? m->gapp() : "";
    td_widget_set_text(s_app_btn, app && app[0] ? app : "(none)");

    /* NOT WHILE A PASS RUNS: the store cannot be written then, and flipping the
     * box would show a state the port refused. */
    if (!r->learn_active && s_arm_box)
        td_checkbox_set(s_arm_box, m->garmed && m->garmed());

    /* THE WATCH RUNS ONLY WHEN THERE IS SOMETHING TO WATCH FOR. Re-evaluated
     * every tick because it depends on three things that can each change: a
     * mapping existing, the switch being on, and a pass not owning the sensors.
     *
     * Stopping here rather than in on_close matters: the watch is why the sensors
     * stay on while this window is open, and leaving it running after the wearer
     * switches detection off would keep the part busy for nothing. */
    if (!m->arm || !m->disarm)
        return;

    const bool want = app && app[0] && m->garmed() && !r->learn_active;
    static bool running;
    if (want && !running)
    {
        m->arm();
        running = true;
    }
    else if (!want && running)
    {
        m->disarm();
        running = false;
    }
}

/* A GESTURE THAT HAS FIRED OPENS ITS APP. One line, because everything hard
 * about it -- edges, debouncing, which slot -- is in the port. */
static void show_fired(td_window_t *win)
{
    (void)win;
    const td_imu_ops_t *m = imu();
    if (!m->detect)
        return;

    char app[24];    /* longer than "On-screen keyboard" */
    if (!m->detect(app, (int)sizeof(app)) || !app[0])
        return;

    /* LAUNCH FROM THE TICK, which is the window manager's own context -- the
     * desktop composes and dispatches on this thread, so this is the same place
     * a click would arrive from. That is the whole reason detection lives here
     * for now: src/wm.c has no lock, so a background task calling td_app_launch
     * would be a data race on the window array, not a slow path. */
    td_app_launch(app);
    td_widget_set_text(s_ammap, "opened");
}

/* The gesture section, from a reading. Split out of on_tick because it is the
 * only part of that function with state of its own. */
static void show_gesture(td_window_t *win, const td_imu_t *r)
{
    if (!s_learn_btn)
        return;

    if (r->learn_active)
    {
        td_progress_set(s_bar, r->learn_progress);

        /* The engine's own vocabulary, in words a wearer can act on. 1 and 2 are
         * bhy2_klio_defs.h's change_reason values and they mean different
         * things: one says the movement was not the same, the other says there
         * was not enough movement to judge. */
        switch (r->learn_reason)
        {
        case 1:
            td_widget_set_text(s_gmsg, "that one differed -- keep it the same");
            break;
        case 2:
            td_widget_set_text(s_gmsg, "no movement -- keep going");
            break;
        default:
            td_widget_set_text(s_gmsg, "repeat the same motion");
            break;
        }
        td_widget_set_text(s_learn_btn, "Stop");
        s_was_learning = true;
        return;
    }

    if (!s_was_learning)
        return;                 /* idle, and already saying so */

    /* The pass ended between the last tick and this one. */
    s_was_learning = false;
    td_widget_set_text(s_learn_btn, "Learn");

    if (r->learn_ok)
    {
        td_progress_set(s_bar, 100);
        /* SAID EXPLICITLY, because "learnt" alone would leave a wearer unsure
         * whether it survives taking the watch off. It does: the port writes it
         * to flash and restores it at the next boot. */
        td_widget_set_text(s_gmsg, "learned and kept");
    }
    else
    {
        td_progress_set(s_bar, r->learn_progress);
        td_widget_set_text(s_gmsg, "no pattern -- try again");
    }
    td_win_invalidate(win);
}

static void on_tick(td_window_t *win)
{
    const td_imu_ops_t *m = imu();
    if (!m)
        return;

    /* START EVERY TICK UNTIL IT TAKES -- not once, and that distinction is a
     * bug that was shipped for one build. start() is idempotent and cheap when
     * the port is already streaming, but the window CLOSES with stop() (by
     * design: the sensor is turned off when nothing reads it). A once-only flag
     * therefore left the second open waiting for a sample for ever, because the
     * firmware was up and the sensors were off and nothing ever asked again.
     *
     * The retry is bounded so that a genuinely absent part does not re-attempt a
     * three-second firmware upload on every tick. */
    static int attempts;
    const char *why = "";
    if (!m->start(&why))
    {
        if (++attempts >= 3)
        {
            snprintf(s_reason, sizeof(s_reason), "%s", why ? why : "no sensor");
            td_widget_set_text(s_msg, s_reason);
            return;
        }
        td_widget_set_text(s_msg, "starting...");
        return;
    }
    attempts = 0;
    s_reason[0] = '\0';

    /* ZEROED FIRST: latest() fills every field whether or not it returns true
     * (see the contract in td_sysinfo.h), so a pass that ended on a tick with no
     * sample still reports its result -- but only if this struct started clear. */
    td_imu_t r;
    memset(&r, 0, sizeof(r));
    if (!m->latest(&r))
    {
        /* A LEARNING PASS STILL HAS NEWS EVEN WHEN THERE IS NO READING. The tick
         * that ends a pass takes the sensors down to write the pattern to flash,
         * so latest() can return false on exactly the tick the result arrives --
         * and returning here would leave the button on "Stop" and the wearer
         * waiting for a bar that had already finished. */
        show_gesture(win, &r);
        td_widget_set_text(s_msg, "waiting for the first sample...");
        return;
    }
    s_reason[0] = '\0';
    td_widget_set_text(s_msg, "");

    td_widget_set_text(s_part, r.part[0] ? r.part : "?");
    td_widget_set_text(s_fw, r.version[0] ? r.version : "--");

    /* The headline: which way the watch is pointing, in words. */
    /* The word comes from the PORT, not from here. Turning acceleration into
     * "TILT UP" is a data question and belongs with the driver, where the shell
     * can use the same function -- otherwise the window and `imu accel` can
     * drift apart on what the watch is doing. */
    td_widget_set_text(s_up, r.accel_valid ? r.posture : "--");

    char buf[24];
    /* Tilt in degrees, from gravity. This is the reading a person expects when
     * they rotate a watch. */
    if (r.tilt_valid)
    {
        snprintf(buf, sizeof(buf), "%+.0f\xc2\xb0", (double)r.tilt_pitch);
        td_widget_set_text(s_pitch, buf);
        snprintf(buf, sizeof(buf), "%+.0f\xc2\xb0", (double)r.tilt_roll);
        td_widget_set_text(s_roll, buf);
    }
    else
    {
        td_widget_set_text(s_pitch, "--");
        td_widget_set_text(s_roll, "--");
    }

    /* Heading needs the part's fusion; a dash until it reports. */
    if (r.ori_valid)
    {
        snprintf(buf, sizeof(buf), "%.0f\xc2\xb0", (double)r.heading);
        td_widget_set_text(s_head, buf);
    }
    else
    {
        td_widget_set_text(s_head, "--");
    }

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

    show_gesture(win, &r);
    show_mapping(win, &r);
    show_fired(win);

    td_win_invalidate(win);
}

static void on_close(td_window_t *win)
{
    (void)win;
    s_win = NULL;
    if (!imu())
        return;

    /* A PASS MUST NOT OUTLIVE THE WINDOW. A wearer who walks away mid-pass
     * would otherwise leave the engine training in the background, at 100 Hz,
     * with nothing on screen to say so or any way to stop it short of a reboot.
     * Cancelling here keeps the pattern already learnt -- cancel() does not
     * reset, which is the distinction that matters. */
    if (s_was_learning && imu()->learn_cancel)
        imu()->learn_cancel();
    s_was_learning = false;

    /* AND THE WATCH STOPS WITH THE WINDOW, for the same reason: detection here is
     * driven by this window's tick, so leaving the part armed would keep the
     * sensors streaming with nothing to read them. */
    if (imu()->disarm)
        imu()->disarm();

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

    const bool learn = can_learn();
    const bool map = can_map();

    td_window_desc_t d = {0};
    d.title = "IMU";
    d.rect = td_rect(-1, -1, IMU_W,
                     (learn && map) ? IMU_H_MAP : (learn ? IMU_H_LEARN : IMU_H));
    d.flags = TD_WIN_MOVABLE | TD_WIN_CLOSABLE;
    d.on_draw = on_draw;
    d.on_tick = on_tick;
    d.on_close = on_close;
    d.tick_ms = TICK_MS;
    s_win = td_win_create(&d);
    if (s_win == NULL)
        return;

    int y = 0;

    s_part = td_label(s_win, 1, y, 30, "");
    y += 1;
    s_fw = td_label(s_win, 1, y, 30, "");
    y += 2;

    /* THE HEADLINE. One word beats three numbers for "which way is it pointing",
     * and it is the thing a person actually wants when they turn their wrist. */
    s_up = td_label(s_win, 1, y, 30, "--");
    y += 2;

    td_label(s_win, 1, y, 30, "Tilt (gravity) / heading");
    y += 1;
    s_pitch = td_label(s_win, 1, y, 25, "--");
    s_roll = td_label(s_win, 10, y, 25, "--");
    s_head = td_label(s_win, 20, y, 25, "--");
    y += 2;

    td_label(s_win, 1, y, 30, "x / y / z, in g");
    y += 1;
    s_ax = td_label(s_win, 1, y, 25, "--");
    s_ay = td_label(s_win, 10, y, 25, "--");
    s_az = td_label(s_win, 20, y, 25, "--");
    y += 2;

    td_label(s_win, 1, y, 12, "Samples");
    s_count = td_label(s_win, 10, y, 20, "0");
    y += 1;

    s_msg = td_label(s_win, 1, y, IMU_W - 2, "");

    if (learn)
    {
        y += 2;                                  /* a blank row, then the section */
        td_label(s_win, 1, y, 30, "Gesture");
        y += 1;
        s_learn_btn = td_button(s_win, 1, y, "Learn", on_learn, NULL);
        y += 1;
        s_bar = td_progress(s_win, 1, y, IMU_W - 2);
        y += 1;
        s_gmsg = td_label(s_win, 1, y, IMU_W - 2, "press Learn to teach one");
    }

    if (map)
    {
        y += 2;
        td_label(s_win, 1, y, 30, "Gesture opens");
        y += 1;
        s_app_btn = td_button(s_win, 1, y, "(none)", on_pick, NULL);
        y += 1;
        s_arm_box = td_checkbox(s_win, 1, y, "detect", imu()->garmed(), on_arm, NULL);
        y += 1;
        s_ammap = td_label(s_win, 1, y, IMU_W - 2, "");
    }

    on_tick(s_win);
}

static const td_app_t s_app = {"IMU", launch, "IM"};

void td_imu_register(void)
{
    if (td_sysinfo()->imu)
        td_app_register(&s_app);
}
