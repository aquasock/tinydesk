/*
 * bluetooth.c - the Bluetooth window: the radio, and the keyboard, mouse and
 * combo slots. Written by aquasock for the TinyTang port (MIT,
 * https://github.com/aquasock/TinyTang, tinydesk issue #7) and taken into
 * TinyDesk with its interface, td_bt_ops_t (td_sysinfo.h).
 *
 * Each slot can be paired from a scan, turned off and on, and forgotten.
 * Everything goes through td_sysinfo()->bt, whose calls return at once, so
 * nothing here blocks the desktop: a request starts work or says why it was
 * refused, and the window follows it and the slots through status().
 * Opening the window does not start the radio; a scan does.
 *
 * Letting go of a device that is connected confirms first, since it may be
 * the keyboard or mouse driving this desktop.
 *
 * Copyright (c) 2026 the TinyTang contributors. MIT License, as TinyDesk.
 */
#include <stdio.h>
#include <string.h>

#include "td_apps.h"

/* The port's Bluetooth table; NULL on a port or build without it, and then
 * the window does not open. */
static const td_bt_ops_t *bt(void)
{
    return td_sysinfo()->bt;
}

#define TICK_MS   250
#define COL       11     /* where values start, after the headings */

/*
 * THE WINDOW FILLS THE SCREEN, AND THE LAYOUT IS COMPUTED FROM IT.
 *
 * It used to be a floating 72x34 window, which is larger than most boards'
 * screens and larger than this one (40x25). The window manager clamps the
 * RECTANGLE to the screen but cannot re-flow its contents, so the device list
 * and the buttons below it were laid out at rows 21..30 -- off the bottom of
 * the glass. On a watch that reads as "the scan found my keyboard but there is
 * nowhere to click it".
 *
 * So: open full screen, and derive every row from the client height instead of
 * hard-coding them. SLOT_ROWS is fixed (three text rows and a row of buttons)
 * because a slot needs that much to say what it is, what it is paired with and
 * what it last did; everything below the slots takes what is left, so the
 * device list is the part that grows on a taller screen.
 */
#define SLOT_ROWS 4

/* One row per slot block, then the scan section. Comfortably inside 25 rows:
 * 2 + 3*4 = 14, leaving 11 for the list, its buttons and the message line. */
#define SLOTS_TOP 2

typedef struct
{
    int slot_y[TD_BT_SLOTS];   /* three text rows, then the button row */
    int scan_hdr;              /* "Pair a keyboard: ..." */
    int scan_msg;              /* "Listening..." or how many were found */
    int list_y, list_h;        /* the scan results */
    int btn_y;                 /* Pair / Scan again / Cancel */
} layout_t;

static layout_t s_lay;

/* Where everything goes for a client area h rows tall. Called before the
 * widgets are made, so the drawing and the buttons agree even if the window
 * manager hands back a different height than the one asked for. */
static void compute_layout(int h)
{
    int used;

    for (int i = 0; i < TD_BT_SLOTS; i++)
        s_lay.slot_y[i] = SLOTS_TOP + i * SLOT_ROWS;

    used = SLOTS_TOP + TD_BT_SLOTS * SLOT_ROWS;
    s_lay.scan_hdr = used;
    s_lay.scan_msg = used + 1;
    s_lay.list_y = used + 2;

    /* The bottom two rows are the buttons and the status line. Whatever is
     * left above them is the list, and it keeps at least one row so the list
     * widget always has somewhere to draw. */
    s_lay.btn_y = h - 2;
    s_lay.list_h = s_lay.btn_y - s_lay.list_y;
    if (s_lay.list_h < 1)
        s_lay.list_h = 1;
}

static td_window_t *s_win;
static td_bt_status_t s_info;

static td_widget_t *s_pair[TD_BT_SLOTS], *s_toggle[TD_BT_SLOTS],
    *s_forget[TD_BT_SLOTS];
static td_widget_t *s_list, *s_scroll, *s_pick, *s_rescan, *s_cancel;

/* The slot a scan is choosing a device for, or -1 when no scan is shown. */
static int s_scan_slot = -1;
static bool s_scan_pending;          /* our scan job has not reported yet */
static td_bt_device_t s_found[TD_BT_MAX_DEVICES];
static int s_found_count;

/* A line of the window's own, such as why a request was refused; it gives
 * way to the job's message once a job is running. */
static char s_note[112];

/* What a confirmation is for. */
static enum
{
    ACT_NONE,
    ACT_OFF,
    ACT_FORGET,
    ACT_PAIR
} s_act;
static int s_act_slot;
static int s_act_device;

static const char *const s_slot_names[TD_BT_SLOTS] = {"Keyboard", "Mouse", "Combo"};
static const char *const s_slot_lower[TD_BT_SLOTS] = {"keyboard", "mouse", "combo"};
static const td_bt_kind_t s_slot_kind[TD_BT_SLOTS] = {
    TD_BT_KIND_KEYBOARD,
    TD_BT_KIND_MOUSE,
    TD_BT_KIND_HID,
};

static void note(const char *text)
{
    snprintf(s_note, sizeof(s_note), "%s", text != NULL ? text : "");
}

/* An address as the shell prints it: most significant byte first. */
static void format_addr(char *out, size_t n, const uint8_t a[6])
{
    snprintf(out, n, "%02X:%02X:%02X:%02X:%02X:%02X", a[5], a[4], a[3], a[2], a[1], a[0]);
}

static const char *kind_name(td_bt_kind_t k)
{
    switch (k)
    {
    case TD_BT_KIND_KEYBOARD:
        return "keyboard";
    case TD_BT_KIND_MOUSE:
        return "mouse";
    case TD_BT_KIND_HID:
        return "HID";
    default:
        return "other";
    }
}

/* ---- the scan list ------------------------------------------------------- */

/* The devices worth offering for `slot`: its own kind first, then other HID
 * devices, then anything else that gave a name -- as `pair <name>` allows a
 * device that does not advertise its kind -- each strongest first. */
static void collect(int slot)
{
    td_bt_device_t all[TD_BT_MAX_DEVICES];
    const int n = bt()->scan_results(all, TD_BT_MAX_DEVICES);
    s_found_count = 0;
    for (int pass = 0; pass < 3; pass++)
    {
        for (int i = 0; i < n; i++)
        {
            const td_bt_device_t *d = &all[i];
            const bool mine = d->kind == s_slot_kind[slot];
            const bool hid = !mine && d->kind != TD_BT_KIND_OTHER;
            const bool named = d->kind == TD_BT_KIND_OTHER && d->name[0] != '\0';
            if ((pass == 0 && mine) || (pass == 1 && hid) || (pass == 2 && named))
            {
                s_found[s_found_count++] = *d;
            }
        }
    }
    td_list_set_count(s_list, s_found_count);
    if (s_found_count > 0)
    {
        td_list_select(s_list, 0);
    }
}

static const char *get_item(td_widget_t *w, int index, int *fg, void *user)
{
    (void)w;
    (void)user;
    static char line[96];
    if (index < 0 || index >= s_found_count)
    {
        return "";
    }
    const td_bt_device_t *d = &s_found[index];
    char addr[18];
    format_addr(addr, sizeof(addr), d->addr);
    snprintf(line, sizeof(line), " %-8s  %-24s %4d dBm  %s", kind_name(d->kind),
             d->name[0] ? d->name : "(no name)", d->rssi, addr);
    if (s_scan_slot >= 0 && d->kind == s_slot_kind[s_scan_slot])
    {
        *fg = td_theme()->accent;
    }
    return line;
}

static void show_scan(bool on)
{
    td_widget_set_visible(s_list, on);
    td_widget_set_visible(s_scroll, on);
    td_widget_set_visible(s_pick, on);
    td_widget_set_visible(s_rescan, on);
    td_widget_set_visible(s_cancel, on);
    if (!on)
    {
        s_scan_slot = -1;
        s_scan_pending = false;
    }
}

static void start_scan(int slot)
{
    s_found_count = 0;
    td_list_set_count(s_list, 0);
    if (!bt()->scan())
    {
        /* The reason is the interface's: status().message carries it. */
        td_bt_status_t st;
        bt()->status(&st);
        note(st.message);
        show_scan(false);
        return;
    }
    note(NULL);
    s_scan_slot = slot;
    s_scan_pending = true;
    show_scan(true);
    td_widget_focus(s_cancel);
}

/* ---- requests, with a confirmation where one is due ---------------------- */

static void run(void)
{
    const int slot = s_act_slot;
    const char *why = NULL;
    td_bt_status_t st;
    switch (s_act)
    {
    case ACT_OFF:
        if (!bt()->disconnect(slot))
        {
            bt()->status(&st);
            why = st.message;
        }
        break;
    case ACT_FORGET:
        if (!bt()->forget(slot))
        {
            bt()->status(&st);
            why = st.message;
        }
        break;
    case ACT_PAIR:
        if (s_act_device < 0 || s_act_device >= s_found_count)
        {
            why = "choose a device in the list first";
            break;
        }
        if (bt()->pair(slot, &s_found[s_act_device]))
        {
            show_scan(false);
        }
        else
        {
            bt()->status(&st);
            why = st.message;
        }
        break;
    case ACT_NONE:
        break;
    }
    s_act = ACT_NONE;
    note(why);
    if (s_win != NULL)
    {
        td_win_invalidate(s_win);
    }
}

static void confirm_answer(int button, void *user)
{
    (void)user;
    if (s_win == NULL || button != 0)
    {
        s_act = ACT_NONE;
        return;
    }
    run();
}

/* The slot's device by name, or its kind when it has none. */
static const char *slot_device(int slot)
{
    const td_bt_slot_t *s = &s_info.slot[slot];
    if (s->paired && s->paired_name[0])
    {
        return s->paired_name;
    }
    return s->name[0] ? s->name : s_slot_lower[slot];
}

static void ask(const char *buttons, const char *fmt_question, int slot)
{
    char text[320];
    const td_bt_slot_t *s = &s_info.slot[slot];
    int n = snprintf(text, sizeof(text), fmt_question, s_slot_lower[slot], slot_device(slot));
    if (s->ready && n > 0 && n < (int)sizeof(text))
    {
        snprintf(text + n, sizeof(text) - (size_t)n,
                 "\n\nIt is connected now and stops working at once. If it\n"
                 "drives this desktop, carry on with another keyboard or\n"
                 "mouse, or from a terminal.");
    }
    td_msgbox("Bluetooth", text, buttons, confirm_answer, NULL);
}

static void on_pair(td_widget_t *w, void *user)
{
    (void)w;
    start_scan((int)(intptr_t)user);
}

static void on_toggle(td_widget_t *w, void *user)
{
    (void)w;
    const int slot = (int)(intptr_t)user;
    if (!s_info.slot[slot].reconnects)
    {
        td_bt_status_t st;
        if (!bt()->set_reconnect(slot, true))
        {
            bt()->status(&st);
            note(st.message);
        }
        else
        {
            note(NULL);
        }
        return;
    }
    s_act = ACT_OFF;
    s_act_slot = slot;
    if (!s_info.slot[slot].ready)
    {
        run();
        return;
    }
    ask("Turn off|Cancel", "Turn off the %s %s? Its pairing is kept, and On\n"
                           "reconnects it.",
        slot);
}

static void on_forget(td_widget_t *w, void *user)
{
    (void)w;
    const int slot = (int)(intptr_t)user;
    s_act = ACT_FORGET;
    s_act_slot = slot;
    ask("Forget|Cancel", "Forget the %s %s? It will have to be paired\n"
                         "again, in pairing mode, to be used.",
        slot);
}

static void pick(void)
{
    if (s_scan_slot < 0)
    {
        return;
    }
    const int slot = s_scan_slot;
    const int index = td_list_selected(s_list);
    if (index < 0 || index >= s_found_count)
    {
        note(s_scan_pending ? "wait for the scan to finish" : "choose a device in the list first");
        return;
    }
    s_act = ACT_PAIR;
    s_act_slot = slot;
    s_act_device = index;
    const td_bt_slot_t *s = &s_info.slot[slot];
    if (s->paired && memcmp(s->paired_addr, s_found[index].addr, 6) != 0)
    {
        ask("Replace|Cancel", "The %s %s is paired now. Pairing this one\n"
                              "forgets it.",
            slot);
        return;
    }
    run();
}

static void on_pick(td_widget_t *w, void *user)
{
    (void)w;
    (void)user;
    pick();
}

static void on_rescan(td_widget_t *w, void *user)
{
    (void)w;
    (void)user;
    if (s_scan_slot >= 0)
    {
        start_scan(s_scan_slot);
    }
}

static void on_cancel(td_widget_t *w, void *user)
{
    (void)w;
    (void)user;
    show_scan(false);
    note(NULL);
    td_win_invalidate(s_win);
}

/* ---- drawing ------------------------------------------------------------- */

static void row(int y, int w, const char *heading, const char *value, uint8_t fg)
{
    const td_theme_t *t = td_theme();
    if (heading != NULL)
    {
        td_text(1, y, heading, t->win_fg, t->win_bg, TD_BOLD);
    }
    td_textn(COL, y, value, w - COL - 1, fg, t->win_bg, 0);
}

static void draw_slot(int y, int w, int i)
{
    const td_theme_t *t = td_theme();
    const td_bt_slot_t *s = &s_info.slot[i];
    /* A narrow window has about 28 columns for a value; two of these lines are
     * longer than that, and the part that gets cut is the part worth reading.
     * So the trailing detail is dropped when there is no room for it rather
     * than being truncated mid-address. */
    bool narrow = w < 50;
    char line[128];
    char addr[18];

    if (s->active)
    {
        format_addr(addr, sizeof(addr), s->addr);
        int n = snprintf(line, sizeof(line), "%s, %s%s%s", s->state,
                         s->name[0] ? s->name : "", s->name[0] ? " " : "", addr);
        if (s->ready && !narrow && n > 0 && n < (int)sizeof(line))
        {
            snprintf(line + n, sizeof(line) - (size_t)n, " (%s, %lu reports)",
                     s->boot_protocol ? "boot" : "report", (unsigned long)s->reports);
        }
    }
    else
    {
        snprintf(line, sizeof(line), "%s", s->state);
    }
    row(y, w, s_slot_names[i], line, s->ready ? t->win_fg : t->dim);

    if (s->paired)
    {
        format_addr(addr, sizeof(addr), s->paired_addr);
        if (narrow)
        {
            /* The On/Off button already says whether it reconnects. */
            snprintf(line, sizeof(line), "paired: %s%s%s",
                     s->paired_name[0] ? s->paired_name : "",
                     s->paired_name[0] ? " " : "", addr);
        }
        else
        {
            snprintf(line, sizeof(line), "paired with %s%s%s, %s",
                     s->paired_name[0] ? s->paired_name : "",
                     s->paired_name[0] ? " " : "", addr,
                     s->reconnects ? "reconnects by itself" : "not reconnecting");
        }
        row(y + 1, w, NULL, line, s->reconnects ? t->win_fg : t->dim);
    }
    else
    {
        row(y + 1, w, NULL, "not paired", t->dim);
    }
    if (s->last[0])
    {
        snprintf(line, sizeof(line), "last: %s", s->last);
        row(y + 2, w, NULL, line, t->dim);
    }
}

static void on_draw(td_window_t *win, int w, int h)
{
    (void)win;
    const td_theme_t *t = td_theme();
    char line[128];
    td_fill(td_rect(0, 0, w, h), ' ', t->win_fg, t->win_bg);

    switch (s_info.radio)
    {
    case TD_BT_RADIO_OFF:
        row(0, w, "Radio", "off; a scan starts it", t->dim);
        break;
    case TD_BT_RADIO_STARTING:
        row(0, w, "Radio", "starting", t->win_fg);
        break;
    case TD_BT_RADIO_FAILED:
        snprintf(line, sizeof(line), "failed to start (%d)", s_info.enable_error);
        row(0, w, "Radio", line, t->win_fg);
        break;
    case TD_BT_RADIO_UP:
        if (s_info.stack_heap)
        {
            snprintf(line, sizeof(line), "up, %lu B heap",
                     (unsigned long)s_info.stack_heap);
        }
        else
        {
            snprintf(line, sizeof(line), "up");
        }
        row(0, w, "Radio", line, t->win_fg);
        break;
    }
    if (s_info.pairings != NULL)
    {
        row(1, w, "Pairings", s_info.pairings, t->win_fg);
    }
    if (s_info.wifi_exclusive)
    {
        /* One line, not a row of its own: this board cannot run both radios,
         * and on 25 rows the space is better spent on the device list. */
        row(1, w, s_info.pairings != NULL ? NULL : "Wi-Fi",
            "off while Bluetooth runs", t->dim);
    }

    for (int i = 0; i < TD_BT_SLOTS; i++)
    {
        draw_slot(s_lay.slot_y[i], w, i);
    }

    if (s_scan_slot >= 0)
    {
        snprintf(line, sizeof(line), "Pair a %s: pairing mode",
                 s_slot_lower[s_scan_slot]);
        td_textn(1, s_lay.scan_hdr, line, w - 2, t->win_fg, t->win_bg, TD_BOLD);
        if (s_scan_pending)
        {
            snprintf(line, sizeof(line), "Listening... %lu s",
                     (unsigned long)((s_info.scan_ms_left + 999) / 1000));
        }
        else if (s_found_count == 0)
        {
            snprintf(line, sizeof(line), "Nothing found; scan again");
        }
        else
        {
            snprintf(line, sizeof(line), "%d found; pick one, press Pair",
                     s_found_count);
        }
        td_textn(1, s_lay.scan_msg, line, w - 2, t->win_fg, t->win_bg, 0);
    }

    /* The bottom line: a job's progress or outcome, else the window's note.
     * It starts after the Close button, which is "[ Close ]" -- nine columns
     * -- so column 10 rather than 9, or the two run together. */
    const char *msg = s_note;
    bool failed = s_note[0] != '\0';
    if (s_info.busy || (s_note[0] == '\0' && s_info.message[0] != '\0'))
    {
        msg = s_info.message;
        failed = s_info.job_failed;
        if (s_info.busy && msg[0] == '\0')
        {
            msg = "working...";
        }
    }
    td_textn(10, h - 1, msg, w - 11, failed ? t->accent : t->win_fg, t->win_bg, 0);
}

/* ---- the window ---------------------------------------------------------- */

static void refresh(void)
{
    bt()->status(&s_info);

    for (int i = 0; i < TD_BT_SLOTS; i++)
    {
        const td_bt_slot_t *s = &s_info.slot[i];
        td_widget_set_visible(s_toggle[i], s->paired);
        td_widget_set_visible(s_forget[i], s->paired);
        td_widget_set_text(s_toggle[i], s->reconnects ? "Off" : "On");
    }

    /* scan_results() is -1 while a scan runs, which is how this tells that the
     * job has finished without knowing anything about the engine's jobs. */
    if (s_scan_slot >= 0 && s_scan_pending && bt()->scan_results(NULL, 0) != -1)
    {
        s_scan_pending = false;
        collect(s_scan_slot);
        if (s_found_count > 0)
        {
            td_widget_focus(s_list);
        }
    }
}

static void on_tick(td_window_t *win)
{
    refresh();
    td_win_invalidate(win);
}

static void on_close_btn(td_widget_t *w, void *user)
{
    (void)user;
    td_win_close(w->win);
}

static void on_close(td_window_t *win)
{
    (void)win;
    s_win = NULL;
    s_scan_slot = -1;
    s_scan_pending = false;
    s_act = ACT_NONE;
}

static void launch(void)
{
    if (bt() == NULL)
    {
        /* No Bluetooth on this build: there is nothing to show. */
        return;
    }
    if (td_win_is_open(s_win))
    {
        td_win_focus(s_win);
        return;
    }
    td_window_desc_t d = {0};
    d.title = "Bluetooth";
    d.rect = td_rect(-1, -1, 40, 12);  /* replaced by the screen: see below */
    /* Full screen, not floating. The window is the only thing on the glass
     * while it is open and its Close button is on the bottom row, so there is
     * no frame to move, resize or close it by -- and no title bar stealing a
     * row from a 25-row screen. */
    d.flags = TD_WIN_FULLSCREEN;
    d.on_draw = on_draw;
    d.on_close = on_close;
    d.on_tick = on_tick;
    d.tick_ms = TICK_MS;
    s_win = td_win_create(&d);
    if (s_win == NULL)
    {
        return;
    }
    note(NULL);

    /* The layout comes from the client area the window manager actually gave
     * us, not from a constant, so the widgets and the drawing cannot disagree
     * about where the bottom is. */
    {
        td_rect_t c = td_win_client(s_win);
        compute_layout(c.h);
    }

    for (int i = 0; i < TD_BT_SLOTS; i++)
    {
        const int y = s_lay.slot_y[i] + SLOT_ROWS - 1;
        s_pair[i] = td_button(s_win, COL, y, "Pair...", on_pair, (void *)(intptr_t)i);
        s_toggle[i] = td_button(s_win, COL + 12, y, "Off", on_toggle, (void *)(intptr_t)i);
        s_forget[i] = td_button(s_win, COL + 20, y, "Forget", on_forget, (void *)(intptr_t)i);
    }
    s_list = td_list(s_win, td_rect(0, s_lay.list_y, -1, s_lay.list_h), get_item, on_pick, NULL);
    s_scroll = td_scrollbar(s_win, -1, s_lay.list_y, s_lay.list_h, s_list);
    s_pick = td_button(s_win, 1, s_lay.btn_y, "Pair", on_pick, NULL);
    s_rescan = td_button(s_win, 10, s_lay.btn_y, "Scan again", on_rescan, NULL);
    s_cancel = td_button(s_win, 25, s_lay.btn_y, "Cancel", on_cancel, NULL);
    td_widget_t *close = td_button(s_win, 0, -1, "Close", on_close_btn, NULL);

    show_scan(false);
    refresh();
    td_widget_focus(close);
}

static const td_app_t s_app = {"Bluetooth", launch, "B)"};

void td_bluetooth_register(void)
{
    if (td_sysinfo()->bt)
        td_app_register(&s_app);
}
