/*
 * mouse.c - Mouse: the pointer settings of a mouse the board reads itself
 * (a USB mouse on the ESP32-S3; td_sysinfo()->mouse). Only there does the
 * app exist. Each setting has a bar to click or drag and - / + buttons;
 * changes apply at once and the port saves them (root only). A row of
 * small targets is there to try the pointer out.
 */
#include <stdint.h>

#include "td_apps.h"

enum
{
    SPEED,
    ACCEL,
    PRECISION,
    SETTINGS
};

static const struct
{
    const char *name;
    int min, max, step, def;
} s_rows[SETTINGS] = {
    {"Speed", 25, 300, 5, 100},
    {"Acceleration", 0, 300, 10, 100},
    {"Precision", 0, 100, 10, 50},
};

#define ROW_Y(i) (3 + 2 * (i))
/* The bar's geometry is computed from the client width now, so the click
 * handler reads these rather than compiling in a position the panel may not
 * have room for. */
static int s_bar_x = 23, s_bar_w = 20;
#define TARGET_Y 10

static td_window_t *s_win;
static td_widget_t *s_status, *s_bar[SETTINGS], *s_value[SETTINGS], *s_note, *s_hits;
static td_mouse_settings_t s_set;
static int s_hit_count;

static int *value(int i)
{
    return i == SPEED ? &s_set.speed : i == ACCEL ? &s_set.accel
                                                  : &s_set.precision;
}

static void show(void)
{
    char buf[64];
    const td_mouse_ops_t *m = td_sysinfo()->mouse;
    td_widget_set_text(s_status, m->status && m->status(buf, sizeof(buf)) ? buf : "No mouse attached");
    for (int i = 0; i < SETTINGS; i++)
    {
        int v = *value(i);
        td_progress_set(s_bar[i], (v - s_rows[i].min) * 100 / (s_rows[i].max - s_rows[i].min));
        td_widget_printf(s_value[i], "%3d %%", v);
    }
}

static void change(int i, int v)
{
    if (!td_session_has_right(TD_RIGHT_SYSTEM))
    {
        td_widget_set_text(s_note, "The system permission is required to change these.");
        return;
    }
    if (v < s_rows[i].min)
        v = s_rows[i].min;
    if (v > s_rows[i].max)
        v = s_rows[i].max;
    if (v == *value(i))
        return;
    *value(i) = v;
    bool saved = td_sysinfo()->mouse->set(&s_set);
    td_widget_set_text(s_note, saved ? "Applied and saved." : "Applied, but could not be saved.");
    show();
}

static void on_minus(td_widget_t *w, void *user)
{
    (void)w;
    int i = (int)(intptr_t)user;
    change(i, *value(i) - s_rows[i].step);
}

static void on_plus(td_widget_t *w, void *user)
{
    (void)w;
    int i = (int)(intptr_t)user;
    change(i, *value(i) + s_rows[i].step);
}

static void on_defaults(td_widget_t *w, void *user)
{
    (void)w;
    (void)user;
    for (int i = 0; i < SETTINGS; i++)
        change(i, s_rows[i].def);
}

static void on_target(td_widget_t *w, void *user)
{
    (void)w;
    (void)user;
    s_hit_count++;
    td_widget_printf(s_hits, "Hits: %d", s_hit_count);
}

/* A click or drag on a bar sets its value. */
static bool on_event(td_window_t *win, const td_event_t *ev)
{
    (void)win;
    if (ev->type != TD_EV_MOUSE || ev->button != TD_BUTTON_LEFT ||
        (ev->action != TD_MOUSE_PRESS && ev->action != TD_MOUSE_DRAG))
        return false;
    for (int i = 0; i < SETTINGS; i++)
    {
        if (ev->y != ROW_Y(i) || ev->x < s_bar_x || ev->x >= s_bar_x + s_bar_w)
            continue;
        int span = s_rows[i].max - s_rows[i].min, step = s_rows[i].step;
        int v = s_rows[i].min + (ev->x - s_bar_x) * span / (s_bar_w - 1);
        change(i, (v + step / 2) / step * step);
        return true;
    }
    return false;
}

/* The status line follows the mouse being plugged in or out. */
static void on_tick(td_window_t *win)
{
    (void)win;
    show();
}

static void on_close(td_window_t *win)
{
    (void)win;
    s_win = NULL;
}

static void launch(void)
{
    if (td_win_is_open(s_win))
    {
        td_win_focus(s_win);
        return;
    }
    td_window_desc_t d = {
        .title = "Mouse",
        .rect = td_rect(-1, -1, 60, 16),
        .flags = TD_WIN_MOVABLE | TD_WIN_CLOSABLE,
        .on_event = on_event,
        .on_close = on_close,
        .on_tick = on_tick,
        .tick_ms = 1000,
    };
    s_win = td_win_create(&d);
    if (!s_win)
        return;
    td_sysinfo()->mouse->get(&s_set);
    s_hit_count = 0;

    /*
     * THE BAR WAS FIXED AT 20 COLUMNS STARTING AT COLUMN 23, with the "+" at 44
     * and the value at 50. On a 40-column panel the client is 38 wide, so the
     * bar ran four columns past the frame and both the "+" button and the value
     * were off the glass entirely -- the settings could be turned down but not
     * up. The row below it lost "Hits: 0" to a single "H".
     *
     * Everything is measured from the client now, and the bar takes whatever is
     * left between the "-" and the "+" so it fits any width.
     */
    {
        int cw = td_win_client(s_win).w;
        /* name at 1 (13 wide, "Acceleration" fits), "-" at 15, then the bar
         * takes everything up to the value and the "+" at the right edge. */
        int val_x = cw - 5;
        int plus_x = val_x - 5;
        int bar_x = 20;
        int bar_w = plus_x - 1 - bar_x;

        if (bar_w < 8)
            bar_w = 8;

        s_bar_x = bar_x;
        s_bar_w = bar_w;

        s_status = td_label(s_win, 2, 1, 0, "");

        for (int i = 0; i < SETTINGS; i++)
        {
            td_label(s_win, 1, ROW_Y(i), 13, s_rows[i].name);
            td_button(s_win, 15, ROW_Y(i), "-", on_minus, (void *)(intptr_t)i);
            s_bar[i] = td_progress(s_win, bar_x, ROW_Y(i), bar_w);
            td_button(s_win, plus_x, ROW_Y(i), "+", on_plus, (void *)(intptr_t)i);
            s_value[i] = td_label(s_win, val_x, ROW_Y(i), 5, "");
        }

        /* The four targets, then "Hits: n" on the row below rather than at
         * column 37 where it was cut to "H". */
        td_label(s_win, 2, TARGET_Y, 0, "Try it:");
        for (int i = 0; i < 4; i++)
            td_button(s_win, 11 + 6 * i, TARGET_Y, "o", on_target, NULL);
        s_hits = td_label(s_win, 2, TARGET_Y + 1, 0, "Hits: 0");

        td_button(s_win, 2, 12, "Defaults", on_defaults, NULL);
        s_note = td_label(s_win, 2, 13, 0,
                          td_session_has_right(TD_RIGHT_SYSTEM)
                              ? ""
                              : "The system permission is required.");
    }
    td_widget_set_color(s_note, td_theme()->dim, TD_COLOR_DEFAULT);
    show();
}

static const td_app_t s_app = {"Mouse", launch, "^"};

void td_mouse_register(void)
{
    if (td_sysinfo()->mouse)
        td_app_register(&s_app);
}
