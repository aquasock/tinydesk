/*
 * link_nano.c - the Arduino Nano ESP32's link, with two sinks.
 *
 * USB Serial/JTAG is the primary: when a host terminal is attached the
 * desktop is served there, at whatever size that terminal reports, and the
 * panel is blanked. With nothing attached the desktop renders on the ILI9340
 * panel at its 40x15 grid instead.
 *
 * The switch is decided by usb_serial_jtag_is_connected() and re-checked on
 * every write. Writes happen at least once a second because the desktop polls
 * the terminal size, so the switch is noticed promptly without a timer task.
 *
 * Nothing here has to know about the desktop's layout. The desktop asks the
 * terminal how big it is with ESC[6n and re-lays-out when the answer changes:
 *
 *   USB   the host terminal answers for itself, so the desktop fills the
 *         window it is actually running in, up to TD_MAX_COLS x TD_MAX_ROWS.
 *   panel there is no terminal, so the query is intercepted here and answered
 *         with the panel's grid.
 *
 * A switch therefore looks to the desktop like a resize, which it already
 * knows how to handle.
 *
 * Input is USB only. The panel HAT on this bench has no touchscreen, keypad or
 * buttons, so when no host is attached there is nothing to type into: the
 * desktop keeps rendering and the node keeps running, but the screen is
 * read-only until USB comes back.
 */
#include "link.h"

#include <stdio.h>
#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "rs485.h"
#include "font8x16.h"
#include "node.h"
#include "ili9340.h"
#include "vt.h"

/* A full ring drops USB packets, so hold a paste while a frame is drawn. */
#define RX_BUFFER        8192
/* One 160x50 frame is about 40 KB with its escape sequences. */
#define TX_BUFFER        65536
#define WRITE_CHUNK      256
/* Long enough for a frame to leave the FIFO, short enough that a terminal
 * which has stopped reading cannot stall the UI. */
#define WRITE_TIMEOUT_MS 200
#define REPLY_MAX        32

static const char *TAG = "link_nano";

static char s_reply[REPLY_MAX];
static int s_reply_len, s_reply_pos;
static bool s_panel_ready;

/* Which sink the last write went to. Set at init from the connection state so
 * the first write does not look like a switch. */
static bool s_on_usb;

/* True once the desktop has actually written to USB. Before that, USB is free
 * for boot logging; after it, USB is the desktop and logs must not paint over
 * it. See node.cpp's node_log(). */
static bool s_usb_has_desktop;

bool link_usb_is_desktop(void);

static void panel_blank(void)
{
    if (s_panel_ready)
        ili9340_fill(0x0000);
}

/* Called when the sink changes. The panel keeps its own copy of the screen and
 * would otherwise render stale content after a spell on USB, so it is reset
 * and the desktop's next size query answers 40x15, which makes it repaint. */
static void on_sink_changed(bool on_usb)
{
    s_on_usb = on_usb;
    s_reply_len = s_reply_pos = 0;
    if (on_usb)
    {
        panel_blank();
        ESP_LOGI(TAG, "host terminal attached: desktop on USB, panel off");
    }
    else
    {
        if (s_panel_ready)
        {
            vt_init(LCD_W / FONT_W, LCD_H / FONT_H);
            ili9340_fill(0x0000);
        }
        ESP_LOGI(TAG, "host terminal gone: desktop on the panel");
    }
}

bool link_init(void)
{
    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = RX_BUFFER,
        .tx_buffer_size = TX_BUFFER,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "usb install: %s", esp_err_to_name(err));
        return false;
    }

    /* The panel is always initialised: it is the fallback sink, and it is what
     * the board shows when a cable is pulled. A missing panel is not fatal,
     * because USB may still be the sink. */
    s_panel_ready = ili9340_init();
    if (s_panel_ready)
    {
        ili9340_fill(0x0000);
        vt_init(LCD_W / FONT_W, LCD_H / FONT_H);
        ESP_LOGI(TAG, "panel %dx%d px, %dx%d cells",
                 LCD_W, LCD_H, vt_cols(), vt_rows());
    }
    else
    {
        ESP_LOGE(TAG, "panel init failed; USB only");
    }

    s_on_usb = usb_serial_jtag_is_connected();
    ESP_LOGI(TAG, "start on %s", s_on_usb ? "USB" : "the panel");
    if (s_on_usb)
        panel_blank();

    /* Reticulum runs beside the desktop. Started here because this is the
     * port's own entry point; its task waits for the filesystem before reading
     * credentials, and a failure leaves the desktop working. */
    node_start();
    return true;
}

bool link_usb_is_desktop(void)
{
    return s_on_usb && s_usb_has_desktop;
}

int link_read(uint8_t *buf, int cap)
{
    /* A queued panel size reply goes out before any real input, because the
     * desktop is waiting on it before it draws. */
    if (s_reply_pos < s_reply_len)
    {
        int n = s_reply_len - s_reply_pos;
        if (n > cap)
            n = cap;
        memcpy(buf, s_reply + s_reply_pos, (size_t)n);
        s_reply_pos += n;
        return n;
    }
    /* Input is USB only: nothing on the panel can type. */
    int n = usb_serial_jtag_read_bytes(buf, (uint32_t)cap, 0);
    return n > 0 ? n : 0;
}

static int usb_write(const uint8_t *buf, int len)
{
    /* No host at all (power-only cable, PC asleep): don't wait. */
    if (!usb_serial_jtag_is_connected())
        return 0;
    int done = 0;
    while (done < len)
    {
        int n = len - done > WRITE_CHUNK ? WRITE_CHUNK : len - done;
        if (usb_serial_jtag_write_bytes(buf + done, (size_t)n,
                                        pdMS_TO_TICKS(WRITE_TIMEOUT_MS)) != n)
            break;
        done += n;
    }
    return done;
}

int link_write(const uint8_t *buf, int len)
{
    const bool usb = usb_serial_jtag_is_connected();
    if (usb != s_on_usb)
        on_sink_changed(usb);

    if (usb)
    {
        int n = usb_write(buf, len);
        if (n > 0)
            s_usb_has_desktop = true;
        return n;
    }

    /* Panel: there is no terminal to answer the size query, so answer it here
     * with the panel's grid. */
    if (s_panel_ready)
    {
        vt_feed(buf, len);
        if (vt_take_size_query(NULL, NULL))
        {
            s_reply_len = snprintf(s_reply, sizeof(s_reply), "\x1b[%d;%dR",
                                   vt_rows(), vt_cols());
            s_reply_pos = 0;
        }
    }
    return len;
}

const char *board_platform(void)
{
    return s_on_usb ? "Arduino Nano ESP32 (USB Serial/JTAG)"
                    : "Arduino Nano ESP32 (ILI9340 320x240)";
}
const char *board_hostname(void)
{
    return "nanoesp32";
}
const td_mb_serial_t *board_rtu_lines(void)
{
    return rs485_serial();
}
