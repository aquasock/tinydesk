/*
 * test_imu_app.c - the IMU window's gesture section, on a host.
 *
 * WHY THIS EXISTS. The gesture section is the only part of this window with
 * layout risk and state of its own: it adds five rows to a window that already
 * filled fourteen, and its button, bar and status line change as a learning pass
 * runs. Both can be walked here in milliseconds, and both are expensive to get
 * wrong on a watch -- a clipped control is invisible until somebody looks at the
 * glass, and a button stuck on "Stop" after a finished pass is exactly the kind
 * of state bug that only shows up while wearing it.
 *
 * The port is faked rather than simulated: this test owns the readings, so it
 * can drive the window through every state the real part can report -- including
 * progress that goes BACKWARDS, which is what the engine does when it decides the
 * next repetition is a different movement, and which a UI must survive without
 * pretending it did not happen.
 *
 * It also pins the one thing a wearer was promised: a finished pass says the
 * pattern was KEPT, because the port writes it to flash and restores it at the
 * next boot. If that claim ever stops being true, this test still passes -- it
 * checks the window, not the store -- which is why the store has its own suite
 * (tests/test_klio_store.c) and the round trip is proven on hardware.
 */
#include <stdlib.h>
#include <string.h>

#include "td_apps.h"
#include "td_test.h"
#include "tinydesk/td.h"

static uint32_t now = 1000;
static int read_byte(void *p)
{
    (void)p;
    return -1;
}
static int write_bytes(void *p, const uint8_t *b, int n)
{
    (void)p;
    (void)b;
    return n;
}
static uint32_t millis(void *p)
{
    (void)p;
    return now;
}
static void sleep_ms(void *p, uint32_t ms)
{
    (void)p;
    now += ms;
}
static const td_hal_t hal = {read_byte, write_bytes, millis, sleep_ms, NULL};

/* ------------------------------------------------------------- the fake part */

static bool s_started;
static bool s_learn_on;
static bool s_learn_result;          /* what the next finished pass reports */
static td_imu_t s_reading;
static int s_start_calls;
static bool s_no_sample;      /* latest() reports no reading this tick */

static bool f_start(const char **why)
{
    (void)why;
    s_start_calls++;
    s_started = true;
    return true;
}
static void f_stop(void)
{
    s_started = false;
}
static bool f_latest(td_imu_t *out)
{
    /* THE CONTRACT: every field filled whether or not this returns true. */
    memset(out, 0, sizeof(*out));
    out->accel_valid = true;
    out->ax = 0.0f;
    out->ay = 0.0f;
    out->az = 1.0f;
    out->tilt_valid = true;
    snprintf(out->posture, sizeof(out->posture), "FACE UP");
    snprintf(out->part, sizeof(out->part), "BHI260AP");
    snprintf(out->version, sizeof(out->version), "test");
    out->accel_samples = 1234;

    /* THE TICK THAT ENDS A PASS TAKES THE SENSORS DOWN to write the pattern to
     * flash, so the port can have a result to report and no sample to show. The
     * learning fields are filled either way -- that is the contract -- and the
     * return value says only whether there is a READING. */
    out->accel_valid = !s_no_sample;
    out->tilt_valid = !s_no_sample;

    out->learn_available = true;
    out->learn_active = s_reading.learn_active;
    out->learn_ok = s_reading.learn_ok;
    out->learn_progress = s_reading.learn_progress;
    out->learn_reason = s_reading.learn_reason;
    return !s_no_sample;
}
static bool f_learn_begin(void)
{
    if (s_learn_on)
        return true;
    s_learn_on = true;
    s_learn_result = false;
    memset(&s_reading, 0, sizeof(s_reading));
    s_reading.learn_active = true;
    return true;
}
static void f_learn_cancel(void)
{
    if (!s_learn_on)
        return;
    s_learn_on = false;
    s_reading.learn_active = false;
    s_reading.learn_ok = false;
}

static const td_imu_ops_t s_ops = {
    .start = f_start,
    .stop = f_stop,
    .latest = f_latest,
    .learn_begin = f_learn_begin,
    .learn_cancel = f_learn_cancel,
};

static td_buffer_t back;
static void tick(unsigned ms)
{
    now += ms;
    td_timers_run(now);
    td_wm_compose(&back);
}

static td_widget_t *widget_with(td_window_t *w, const char *text)
{
    for (td_widget_t *p = w->widgets; p; p = p->next)
        if (!strcmp(td_widget_text(p), text))
            return p;
    return NULL;
}

static td_widget_t *first_button(td_window_t *w)
{
    for (td_widget_t *p = w->widgets; p; p = p->next)
        if (p->type == TD_WT_BUTTON)
            return p;
    return NULL;
}

static td_widget_t *first_progress(td_window_t *w)
{
    for (td_widget_t *p = w->widgets; p; p = p->next)
        if (p->type == TD_WT_PROGRESS)
            return p;
    return NULL;
}

static void press(td_widget_t *w)
{
    CHECK(w != NULL);
    if (w && w->on_activate)
        w->on_activate(w, w->user);
}

/* Every focusable widget must lie inside the window's client area, or it is
 * clipped on the glass and nobody can press it. */
static void check_fits(td_window_t *w)
{
    const td_rect_t c = td_win_client(w);
    for (td_widget_t *p = w->widgets; p; p = p->next)
    {
        if (!p->focusable || !p->visible)
            continue;
        const td_rect_t r = td_widget_rect(p);
        const bool fits = r.w > 0 && r.h > 0 && r.x >= c.x && r.y >= c.y &&
                          r.x + r.w <= c.x + c.w && r.y + r.h <= c.y + c.h;
        if (!fits)
            fprintf(stderr, "clipped: %s (%d,%d %dx%d) client %dx%d\n",
                    td_widget_text(p), r.x - c.x, r.y - c.y, r.w, r.h, c.w, c.h);
        CHECK(fits);
    }
}

/* The bar's percent, read back through the widget the window actually draws. */
static int bar_percent(td_widget_t *bar)
{
    return bar ? bar->value : -1;      /* the percent, per td_progress_set */
}

int main(void)
{
    CHECK_EQ(td_init(&hal), 0);

    static const td_sysinfo_t info = {.platform = "Test", .chip = "ESP32-S3",
                                      .imu = &s_ops};
    td_set_sysinfo(&info);

    td_imu_register();
    CHECK(td_app_launch("IMU"));
    tick(1000);

    td_window_t *w = td_win_focused();
    CHECK(w != NULL);
    if (!w)
        return TD_TEST_RESULT();

    /* --- the layout, at this board's grid -------------------------------- */

    check_fits(w);

    td_widget_t *btn = first_button(w);
    td_widget_t *bar = first_progress(w);
    CHECK(btn != NULL);
    CHECK(bar != NULL);
    CHECK(strcmp(td_widget_text(btn), "Learn") == 0);
    CHECK(widget_with(w, "Gesture") != NULL);
    CHECK(widget_with(w, "press Learn to teach one") != NULL);
    CHECK(bar_percent(bar) == 0);

    /* The rest of the window is still there: this app is not only the button. */
    CHECK(widget_with(w, "FACE UP") != NULL);
    CHECK(widget_with(w, "Tilt (gravity) / heading") != NULL);

    /* The whole window has to fit the screen, taskbar and all. */
    int cols = 0, rows = 0;
    td_wm_screen_size(&cols, &rows);
    CHECK(w->rect.x >= 0 && w->rect.y >= 0);
    CHECK(w->rect.x + w->rect.w <= cols);
    CHECK(w->rect.y + w->rect.h <= rows);

    /* --- pressing Learn starts a pass ----------------------------------- */

    press(btn);
    CHECK(s_learn_on);
    CHECK(strcmp(td_widget_text(btn), "Stop") == 0);
    CHECK(bar_percent(bar) == 0);

    /* --- progress climbs, and the bar follows ---------------------------- */

    s_reading.learn_active = true;
    s_reading.learn_progress = 40;
    s_reading.learn_reason = 0;
    tick(500);
    CHECK(bar_percent(bar) == 40);
    CHECK(widget_with(w, "repeat the same motion") != NULL);

    /* A repetition the engine judged different. PROGRESS FALLS, and the window
     * has to show that rather than hold the old value. */
    s_reading.learn_progress = 0;
    s_reading.learn_reason = 1;
    tick(500);
    CHECK(bar_percent(bar) == 0);
    CHECK(widget_with(w, "that one differed -- keep it the same") != NULL);

    /* The other interruption, which means something different and must say so. */
    s_reading.learn_progress = 20;
    s_reading.learn_reason = 2;
    tick(500);
    CHECK(bar_percent(bar) == 20);
    CHECK(widget_with(w, "no movement -- keep going") != NULL);

    /* --- a pass that succeeds ------------------------------------------- */

    s_reading.learn_active = false;
    s_reading.learn_ok = true;
    s_reading.learn_progress = 100;
    s_learn_on = false;
    tick(500);
    CHECK(bar_percent(bar) == 100);
    CHECK(strcmp(td_widget_text(btn), "Learn") == 0);
    CHECK(widget_with(w, "learned and kept") != NULL);

    /* It stays said: the result does not flicker away on the next tick. */
    tick(500);
    CHECK(widget_with(w, "learned and kept") != NULL);

    /* --- and a pass that fails ------------------------------------------ */

    press(btn);
    CHECK(s_learn_on);
    s_reading.learn_progress = 60;
    tick(500);
    CHECK(bar_percent(bar) == 60);

    s_reading.learn_active = false;
    s_reading.learn_ok = false;
    s_reading.learn_progress = 60;
    s_learn_on = false;
    tick(500);
    CHECK(strcmp(td_widget_text(btn), "Learn") == 0);
    CHECK(widget_with(w, "no pattern -- try again") != NULL);
    CHECK(bar_percent(bar) == 60);

    /* --- a pass that ends on a tick with no sample ----------------------- */
    /* This is the real sequence on the watch, not a contrived one: the tick that
     * completes a pass stops the sensors to write the pattern to flash, so the
     * very tick carrying the result has no reading. If the window returns early
     * on `latest() == false` without looking at the learning fields, the button
     * stays on "Stop" for ever and the wearer waits for a bar that already
     * finished. */
    press(btn);
    CHECK(s_learn_on);
    s_reading.learn_progress = 100;
    s_reading.learn_active = true;
    tick(500);
    CHECK(bar_percent(bar) == 100);

    s_no_sample = true;
    s_reading.learn_active = false;
    s_reading.learn_ok = true;
    s_learn_on = false;
    tick(500);
    CHECK(strcmp(td_widget_text(btn), "Learn") == 0);
    CHECK(widget_with(w, "learned and kept") != NULL);
    s_no_sample = false;

    /* --- Stop abandons a running pass ------------------------------------ */

    press(btn);
    CHECK(s_learn_on);
    press(btn);                 /* the same button, now reading "Stop" */
    CHECK(!s_learn_on);
    CHECK(strcmp(td_widget_text(btn), "Learn") == 0);
    CHECK(widget_with(w, "stopped") != NULL);

    /* --- closing the window must not leave the engine training ------------ */

    s_start_calls = 0;
    press(btn);
    CHECK(s_learn_on);
    td_win_close(w);
    tick(500);
    CHECK(!s_learn_on);         /* cancelled by on_close */
    CHECK(!s_started);          /* and the sensor was stopped */

    /* --- a board with no learning engine draws no gesture section --------- */

    static const td_imu_ops_t no_learn = {
        .start = f_start,
        .stop = f_stop,
        .latest = f_latest,
    };
    static const td_sysinfo_t bare = {.platform = "Test", .chip = "ESP32-S3",
                                      .imu = &no_learn};
    td_set_sysinfo(&bare);

    td_imu_register();          /* already registered: the second call is a no-op */
    td_wm_close_all();
    CHECK(td_app_launch("IMU"));
    tick(1000);
    w = td_win_focused();
    CHECK(w != NULL);
    if (w)
    {
        CHECK(first_button(w) == NULL);
        CHECK(first_progress(w) == NULL);
        CHECK(widget_with(w, "Gesture") == NULL);
        CHECK(widget_with(w, "FACE UP") != NULL);  /* the sensor display remains */
        check_fits(w);
        /* And the window is the short one it always was. */
        CHECK_EQ(w->rect.h, 14);
    }

    return TD_TEST_RESULT();
}
