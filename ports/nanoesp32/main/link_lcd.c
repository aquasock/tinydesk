/*
 * link_lcd.c - the desktop on the ILI9340 panel.
 *
 * Input still comes from the USB Serial/JTAG port and output goes to the
 * panel, so the desktop can be driven from a PC terminal while it renders on
 * the 2.2" display. That split is deliberate: the HAT on this bench has no
 * touchscreen, and this proves the whole render path before an input device
 * is chosen.
 *
 * The one thing the link must do beyond moving bytes is answer the terminal
 * size query: TinyDesk asks how big the terminal is by jumping the cursor
 * far away and reading it back with ESC[6n, expecting ESC[rows;colsR. The VT
 * layer notices the query and this link queues the reply ahead of any real
 * input.
 */
#include "link.h"

#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "rs485.h"
#include "font8x16.h"
#include "ili9340.h"
#include "vt.h"

#define RX_BUFFER 8192
#define REPLY_MAX 32

static const char *TAG = "link_lcd";

static char s_reply[REPLY_MAX];
static int s_reply_len, s_reply_pos;
static bool s_ready;

bool link_init(void)
{
    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = RX_BUFFER,
        .tx_buffer_size = 1024,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "usb install: %s", esp_err_to_name(err));
        return false;
    }

    if (!ili9340_init())
    {
        ESP_LOGE(TAG, "panel init failed");
        return false;
    }

    /* Start black; the first frame paints over it. */
    ili9340_fill(0x0000);
    vt_init(LCD_W / FONT_W, LCD_H / FONT_H);
    s_ready = true;
    ESP_LOGI(TAG, "panel %dx%d px, %dx%d cells",
             LCD_W, LCD_H, vt_cols(), vt_rows());
    return true;
}

int link_read(uint8_t *buf, int cap)
{
    /* A queued terminal-size reply goes out before any real input. */
    if (s_reply_pos < s_reply_len)
    {
        int n = s_reply_len - s_reply_pos;
        if (n > cap)
            n = cap;
        memcpy(buf, s_reply + s_reply_pos, (size_t)n);
        s_reply_pos += n;
        return n;
    }
    int n = usb_serial_jtag_read_bytes(buf, (uint32_t)cap, 0);
    return n > 0 ? n : 0;
}

int link_write(const uint8_t *buf, int len)
{
    if (!s_ready)
        return len;
    vt_feed(buf, len);
    if (vt_take_size_query(NULL, NULL))
    {
        int rows = vt_rows(), cols = vt_cols();
        s_reply_len = snprintf(s_reply, sizeof(s_reply), "\x1b[%d;%dR", rows, cols);
        s_reply_pos = 0;
    }
    return len;
}

const char *board_platform(void)
{
    return "Arduino Nano ESP32 (ILI9340 320x240)";
}
const char *board_hostname(void)
{
    return "nanoesp32";
}
const td_mb_serial_t *board_rtu_lines(void)
{
    return rs485_serial();
}
