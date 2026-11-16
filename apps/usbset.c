/*
 * usbset.c - the USB page of Settings: the USB-C port's role (Normal or OTG
 * host) and which device classes are used in host mode.
 *
 * Everything here goes through td_sysinfo()->usb (td_usb_ops_t); the page knows
 * nothing about a particular board. The role and the device choices are SAVED
 * when changed and APPLIED at the next restart -- on a board with one port for
 * both, the PC cannot see the board while it is a host -- so the page shows what
 * is saved, what is running, and a Restart button when the two differ.
 *
 * A device class is only meaningful in host mode, so its checkbox is drawn dim
 * and ignores presses while Normal is selected. The widget set has no disabled
 * state; dimming plus refusing the change in the callback is what this page
 * means by "greyed out".
 */
#include <stdio.h>
#include <string.h>

#include "td_apps.h"

#define USB_DEV_SHOWN 4   /* device classes the page has room for */

static td_window_t *s_win;
static td_widget_t *s_normal, *s_host;
static td_widget_t *s_dev[USB_DEV_SHOWN];
static int s_ndev;
static td_widget_t *s_running, *s_attached, *s_pending, *s_restart;

static const td_usb_ops_t *ops(void)
{
    return td_sysinfo()->usb;
}

/* Bring every control in line with what the port has saved and is running. */
static void refresh(void)
{
    const td_usb_ops_t *u = ops();

    if (!u)
        return;

    const bool host = u->host_saved();

    td_checkbox_set(s_normal, !host);
    td_checkbox_set(s_host, host);

    for (int i = 0; i < s_ndev; i++)
    {
        td_checkbox_set(s_dev[i], u->device_enabled(i));
        /* Greyed out in Normal mode: dim, and on_dev() refuses the change. */
        td_widget_set_color(s_dev[i], host ? TD_COLOR_DEFAULT : td_theme()->dim, TD_COLOR_DEFAULT);
    }

    const bool running = u->host_running();

    td_widget_printf(s_running, "Running: %s", running ? "OTG host" : "Normal");

    char line[48];

    line[0] = '\0';
    if (running && u->status)
        u->status(line, (int)sizeof(line));
    td_widget_set_text(s_attached, line);

    /* A restart is pending when the saved role differs from the running one.
     * A device choice changed in host mode also needs one; the port reports
     * that through host_running() staying as it was, so the page simply offers
     * the restart whenever anything was changed this visit. */
    const bool pending = host != running;

    td_widget_set_visible(s_pending, pending);
    td_widget_set_visible(s_restart, pending);
}

static void on_mode(td_widget_t *w, void *user)
{
    (void)w;
    const bool host = (intptr_t)user != 0;
    const td_usb_ops_t *u = ops();

    if (u)
        u->set_host(host);
    refresh();   /* from what is saved, so a refused change puts the boxes back */
}

static void on_dev(td_widget_t *w, void *user)
{
    const int i = (int)(intptr_t)user;
    const td_usb_ops_t *u = ops();

    if (!u)
        return;

    /* Greyed out: a device class only applies in host mode. */
    if (!u->host_saved())
    {
        td_checkbox_set(w, u->device_enabled(i));
        return;
    }

    u->set_device_enabled(i, td_checkbox_get(w));
    refresh();
    /* Enabling a device in host mode also takes a restart to apply. */
    td_widget_set_visible(s_pending, true);
    td_widget_set_visible(s_restart, true);
}

static void on_restart(td_widget_t *w, void *user)
{
    (void)w;
    (void)user;
    const td_usb_ops_t *u = ops();

    if (u && u->restart)
        u->restart();
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
}

/* What is attached changes while the page is open (a DAC plugged in). */
static void on_tick(td_window_t *win)
{
    (void)win;
    const td_usb_ops_t *u = ops();
    char line[48];

    line[0] = '\0';
    if (u && u->host_running() && u->status)
        u->status(line, (int)sizeof(line));
    td_widget_set_text(s_attached, line);
}

void td_usb_settings_open(void)
{
    const td_usb_ops_t *u = ops();

    if (!u)
        return;
    if (td_win_is_open(s_win))
    {
        td_win_focus(s_win);
        return;
    }

    td_window_desc_t d = {
        .title = "USB",
        .rect = td_rect(-1, -1, 40, 18),
        .flags = TD_WIN_MOVABLE | TD_WIN_CLOSABLE,
        .on_close = on_close,
        .on_tick = on_tick,
        .tick_ms = 1000,
    };

    s_win = td_win_create(&d);
    if (!s_win)
        return;

    /* Laid out for a 38-column client: every caption fits with its border. */
    td_label(s_win, 1, 0, 0, "USB-C port (applies at restart)");
    s_normal = td_checkbox(s_win, 1, 1, "Normal: PC, console", false, on_mode, (void *)(intptr_t)0);
    s_host = td_checkbox(s_win, 1, 2, "OTG host: USB devices", false, on_mode, (void *)(intptr_t)1);

    td_label(s_win, 1, 4, 0, "Devices in OTG host mode");
    s_ndev = u->devices();
    if (s_ndev > USB_DEV_SHOWN)
        s_ndev = USB_DEV_SHOWN;
    for (int i = 0; i < s_ndev; i++)
        s_dev[i] = td_checkbox(s_win, 3, 5 + i, u->device_name(i), false, on_dev, (void *)(intptr_t)i);

    const int y = 6 + s_ndev;

    s_running = td_label(s_win, 1, y, 36, "");
    s_attached = td_label(s_win, 1, y + 1, 36, "");
    td_widget_set_color(s_attached, td_theme()->dim, TD_COLOR_DEFAULT);
    s_pending = td_label(s_win, 1, y + 3, 36, "Restart to apply the change.");
    s_restart = td_button(s_win, 1, y + 4, "Restart now", on_restart, NULL);

    td_button(s_win, 1, -1, "Close", on_close_btn, NULL);

    refresh();
}
