/*
 * link_twatch.c - the LilyGo T-Watch Ultra: the desktop on the watch's own
 * AMOLED and touch panel, with the keyboard on the ESP32-S3's built-in USB
 * Serial/JTAG port.
 *
 * This board has no USB-UART bridge chip, so the link shape is the C6's
 * (ports/esp_idf/app/link_usj.c) rather than the CrowPanel's UART0. Output
 * goes to the panel and, unless lcd.mirror is 0, also to the USB port, so a
 * host terminal shows the same picture. Input is the USB port (the keyboard,
 * and the host terminal's mouse) and the touch panel (touch.c).
 *
 * The desktop's size is the panel's: its answers to the size query are the
 * only ones the desktop gets, and the host terminal's are dropped. That
 * matters here more than on a bigger board -- a 40x25 grid is small, and
 * letting a PuTTY window decide the geometry would make the desktop depend on
 * which terminal happened to be attached.
 *
 * Board configuration keys (see board.example.conf):
 *   lcd.mirror    1 (default): also send the picture to the USB port
 */
#include "link.h"

#include <stdio.h>
#include <string.h>

#include "display.h"
#include "touch.h"
#include "hal_mux.h"
#include "tinydesk/td_picture.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tdsh_board.h"

#define RX_BUFFER        8192
#define TX_BUFFER        1024
#define WRITE_CHUNK      256
#define WRITE_TIMEOUT_MS 150
#define INJECT_SIZE      512          /* the panel's answers and touch reports */

static const char *TAG = "link_twatch";
static bool s_mirror = true;
static bool s_touch;
static char s_platform[64] = "ESP32-S3 T-Watch Ultra";

/* ---- input injection from the panel and the touch controller ------------ */

static portMUX_TYPE s_inject_mux = portMUX_INITIALIZER_UNLOCKED;
static char s_inject[INJECT_SIZE];
static int s_inject_head, s_inject_tail;

static void inject(const char *data, int len)
{
    taskENTER_CRITICAL(&s_inject_mux);
    for (int i = 0; i < len; i++)
    {
        int next = (s_inject_head + 1) % INJECT_SIZE;

        if (next == s_inject_tail)
            break;                 /* full: drop the rest */
        s_inject[s_inject_head] = data[i];
        s_inject_head = next;
    }
    taskEXIT_CRITICAL(&s_inject_mux);
}

static int take_injected(uint8_t *buf, int cap)
{
    int n = 0;

    taskENTER_CRITICAL(&s_inject_mux);
    while (n < cap && s_inject_tail != s_inject_head)
    {
        buf[n++] = (uint8_t)s_inject[s_inject_tail];
        s_inject_tail = (s_inject_tail + 1) % INJECT_SIZE;
    }
    taskEXIT_CRITICAL(&s_inject_mux);
    return n;
}

/* ---- link --------------------------------------------------------------- */

bool link_init(void)
{
    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = RX_BUFFER,
        .tx_buffer_size = TX_BUFFER,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "usb_serial_jtag_driver_install failed: %s",
                 esp_err_to_name(err));
        return false;
    }

    s_mirror = tdsh_board_int("lcd.mirror", 1) != 0;

    if (!display_init(inject))
    {
        ESP_LOGE(TAG, "the panel did not start");
        return false;
    }

    /* No picture support: the panel is a character terminal here, and the
     * desktop is told so rather than being left to draw Sixel nobody shows. */
    td_picture_set_sink(NULL, NULL);
    td_picture_set_rgb_sink(NULL, NULL);
    td_picture_set_runs_sink(NULL, NULL);

    s_touch = touch_init(inject);
    snprintf(s_platform, sizeof(s_platform),
             "ESP32-S3 T-Watch Ultra (%dx%d cells%s)",
             display_cols(), display_rows(), s_touch ? ", touch" : "");
    ESP_LOGI(TAG, "%s", s_platform);
    return true;
}

int link_read(uint8_t *buf, int cap)
{
    int n = take_injected(buf, cap);

    if (n > 0)
        return n;
    n = usb_serial_jtag_read_bytes(buf, (uint32_t)cap, 0);
    return n > 0 ? n : 0;
}

int link_write(const uint8_t *buf, int len)
{
    display_write(buf, len);

    if (s_mirror && usb_serial_jtag_is_connected())
    {
        int done = 0;

        while (done < len)
        {
            int n = len - done > WRITE_CHUNK ? WRITE_CHUNK : len - done;

            if (usb_serial_jtag_write_bytes(buf + done, (size_t)n,
                                            pdMS_TO_TICKS(WRITE_TIMEOUT_MS)) != n)
                break;
            done += n;
        }
    }
    return len;
}

const char *board_platform(void)
{
    return s_platform;
}

const char *board_hostname(void)
{
    return "twatch";
}

const td_mb_serial_t *board_rtu_lines(void)
{
    return NULL;                   /* no RS-485 on this board */
}
