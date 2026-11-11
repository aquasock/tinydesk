/*
 * touch.c - the T-Watch Ultra's CST9217 capacitive panel as the desktop's
 * mouse.
 *
 * This is the second file in the port that is not modelled on the CrowPanel:
 * that board has a GT911, this one a CST9217, and the two share nothing but
 * the fact that both answer on I2C. Everything below was established on this
 * hardware by the Escapement project and is in its reference library as
 * TCH-001 to TCH-010; the short version:
 *
 *   * The part is at I2C 0x1A. Register 0x0000 reads 0xCA, the family
 *     checkcode, which is how it is told apart from the DRV2605 haptic driver
 *     at 0x5A that the vendor header wrongly also calls a CST92xx.
 *
 *   * A reset part is in COMMAND mode and reports nothing. It must be put in
 *     normal mode: write 0xD1 = 0x01, then 0xD1 = 0x1E twice, read back
 *     0x0002 and require byte 1 to be 0x1E, then write 0xD1 = 0x09.
 *
 *   * The report block at 0xD000 is 15 bytes and the whole block must be read
 *     even though only the first seven are decoded. A short read leaves the
 *     controller's acknowledge mid-block, and the symptom is a touch that
 *     works once and then freezes until the next full read -- TCH-003.
 *
 *   * Byte 0 is 0x06 for a down event; 0xAB means "no new report". The
 *     controller emits NO release event, so a release has to be inferred from
 *     the finger count going to zero (TCH-008).
 *
 *   * Coordinates: x = (b1 << 4) | (b3 >> 4), y = (b2 << 4) | (b3 & 0x0F),
 *     already in panel coordinates -- no swap, no scale (TCH-006).
 *
 * Output is SGR mouse reports, which is what the desktop reads: a terminal
 * mouse is the only input abstraction TinyDesk has, so the panel's absolute
 * coordinates are mapped onto the terminal's cell grid here rather than
 * anywhere else.
 *
 * THE INPUT MODEL IS A POINTER, NOT A TAP. The desktop expects a mouse, so a
 * finger down is button 1 held and a finger up is button 1 released, with the
 * cursor following the finger. A tap is therefore press-and-release at the
 * same cell, which is what the window manager reads as a click.
 */

#include "touch.h"

#include <stdio.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display.h"
#include "tinydesk/td_vterm.h"

#define TAG "twatch-touch"

#define TOUCH_I2C_PORT  I2C_NUM_0
#define TOUCH_SDA       3
#define TOUCH_SCL       2
#define TOUCH_HZ        400000
#define TOUCH_ADDR      0x1A

#define REPORT_ADDR     0xD000
#define REPORT_LEN      15
#define EVENT_DOWN      0x06
#define NO_REPORT       0xAB

#define POLL_MS         15
/* A pressed finger that stops reporting is treated as released after this,
 * because the controller never sends a release event of its own. */
#define RELEASE_MS      250

static touch_input_fn s_input;
static bool s_down;
static int s_last_x = -1, s_last_y = -1;
static int s_idle_ms;

static esp_err_t wr(uint16_t reg, uint8_t v)
{
    uint8_t b[3] = { (uint8_t)(reg >> 8), (uint8_t)reg, v };
    return i2c_master_write_to_device(TOUCH_I2C_PORT, TOUCH_ADDR, b, 3,
                                      100 / portTICK_PERIOD_MS);
}

static esp_err_t rd(uint16_t reg, uint8_t *buf, size_t n)
{
    uint8_t a[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
    return i2c_master_write_read_device(TOUCH_I2C_PORT, TOUCH_ADDR, a, 2,
                                        buf, n, 100 / portTICK_PERIOD_MS);
}

/* The unlock handshake. Returns false if the part is not there or will not
 * come out of command mode. */
static bool bring_up(void)
{
    uint8_t id = 0, v = 0;

    if (rd(0x0000, &id, 1) != ESP_OK || id != 0xCA)
    {
        ESP_LOGW(TAG, "no 0xCA checkcode at 0x%02X (read 0x%02X)",
                 TOUCH_ADDR, id);
        return false;
    }

    wr(0xD1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
    wr(0xD1, 0x1E);
    wr(0xD1, 0x1E);
    vTaskDelay(pdMS_TO_TICKS(10));
    rd(0x0002, &v, 1);
    if (v != 0x1E)
    {
        ESP_LOGW(TAG, "unlock did not take (0x0002 = 0x%02X)", v);
        return false;
    }
    wr(0xD1, 0x09);
    vTaskDelay(pdMS_TO_TICKS(10));
    return true;
}

/* Emit one SGR mouse report. `press` selects button 1 down or up, and the
 * final 'M'/'m' character; the terminal uses the case to carry the state. */
static void emit(bool press, int col, int row)
{
    char buf[32];
    int n = snprintf(buf, sizeof(buf), "\x1b[<%d;%d;%d%c",
                     press ? 0 : 3, col + 1, row + 1, press ? 'M' : 'm');

    if (n > 0 && s_input)
        s_input(buf, n);
}

/* Panel pixels to a terminal cell. The desktop's grid is the link's, not the
 * panel's, so the scale comes from the cell counts display_init reported. */
static void to_cell(int x, int y, int *col, int *row)
{
    int cw = DISPLAY_W / display_cols();
    int ch = DISPLAY_H / display_rows();

    *col = x / cw;
    *row = y / ch;
    if (*col < 0)
        *col = 0;
    if (*row < 0)
        *row = 0;
    if (*col >= display_cols())
        *col = display_cols() - 1;
    if (*row >= display_rows())
        *row = display_rows() - 1;
}

static void poll_task(void *arg)
{
    uint8_t b[REPORT_LEN];

    (void)arg;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));

        /*
         * Read the WHOLE 15-byte block. A short read leaves the controller's
         * acknowledge mid-block and the next report never arrives, which
         * presents as a frozen touch panel rather than as an I2C error.
         */
        if (rd(REPORT_ADDR, b, REPORT_LEN) != ESP_OK)
            continue;
        if (b[0] == NO_REPORT)
        {
            /*
             * No new report. If a finger was down it may simply have lifted:
             * this controller sends no release event, so a held press that
             * stops reporting is a release.
             */
            if (s_down && ++s_idle_ms >= (RELEASE_MS / POLL_MS))
            {
                int col, row;

                to_cell(s_last_x, s_last_y, &col, &row);
                emit(false, col, row);
                s_down = false;
                s_idle_ms = 0;
            }
            continue;
        }

        s_idle_ms = 0;

        if (b[0] == EVENT_DOWN)
        {
            int fingers = b[5] & 0x7F;
            int x = (b[1] << 4) | (b[3] >> 4);
            int y = (b[2] << 4) | (b[3] & 0x0F);
            int col, row;

            if (fingers == 0)
                continue;

            to_cell(x, y, &col, &row);
            s_last_x = x;
            s_last_y = y;

            /* A move while down is a drag: the desktop gets motion with the
             * button held, which is what a window manager expects for a
             * title bar or a scroll bar. */
            if (!s_down)
            {
                s_down = true;
                emit(true, col, row);
            }
            else
            {
                char buf[32];
                int n = snprintf(buf, sizeof(buf), "\x1b[<32;%d;%dM",
                                 col + 1, row + 1);

                if (n > 0 && s_input)
                    s_input(buf, n);
            }
        }
    }
}

bool touch_init(touch_input_fn input)
{
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TOUCH_SDA,
        .scl_io_num = TOUCH_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = TOUCH_HZ,
    };

    if (i2c_param_config(TOUCH_I2C_PORT, &cfg) != ESP_OK)
        return false;
    if (i2c_driver_install(TOUCH_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK)
        return false;

    s_input = input;
    if (!bring_up())
        return false;

    if (xTaskCreatePinnedToCore(poll_task, "tdtouch", 3072, NULL, 3,
                                NULL, 0) != pdPASS)
        return false;

    ESP_LOGI(TAG, "CST9217 up at 0x%02X", TOUCH_ADDR);
    return true;
}
