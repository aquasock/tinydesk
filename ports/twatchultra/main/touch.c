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
 *   * The part is at I2C 0x1A and its bring-up is four attribute reads taken
 *     in command mode, exactly as LilyGo's SensorLib does it: reset, write
 *     0xD1 = 0x01, then read 0xD1FC (checkcode, 0xCACA____), 0xD1F8
 *     (resolution), 0xD204 (chip type) and 0xD208 (firmware). Then 0xD1 =
 *     0x09 into normal mode, where a read of 0x0002 returns 0x00 0x09.
 *
 *     There is NO 0xD1 = 0x1E unlock step. This file previously had one --
 *     write 0xD1 = 0x01, then 0xD1 = 0x1E twice, and require 0x1E in byte 1
 *     of 0x0002 -- and that sequence is retracted: the 0x1E writes live in
 *     SensorLib's setMode(), whose only caller is sleep(), and a working part
 *     reads back 0x00 0x09, not 0x1E. Requiring 0x1E made this driver refuse
 *     a healthy controller, which is why touch never worked. See TCH-007 and
 *     the 2026-11-10 revision note under TCH-002.
 *
 *   * The report block at 0xD000 is 15 bytes and the whole block must be read
 *     even though only the first seven are decoded. **Every read must be
 *     acknowledged with a separate write of 0xD000 = 0xAB**: without it the
 *     controller's buffer never advances, the same block comes back for ever
 *     and a release is never seen (TCH-003, TCH-008).
 *
 *   * A block is a press only if byte 6 is 0xAB, byte 5's low seven bits give
 *     a finger count of 1 or 2, and bits 3:0 of byte 0 are 0x06. Event 0x00
 *     with a non-zero count is not a touch -- accepting it invents a phantom
 *     press at a stale position (TCH-008). The controller emits no release
 *     event, so a release is inferred from a gap in reports.
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

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display.h"
#include "power.h"
#include "tinydesk/td_vterm.h"

#define TAG "twatch-touch"

#define TOUCH_SDA       3
#define TOUCH_SCL       2
#define TOUCH_HZ        400000
#define TOUCH_ADDR      0x1A

#define REPORT_ADDR     0xD000
#define REPORT_LEN      15
#define EVENT_DOWN      0x06
#define ACK_BYTE        0xAB
#define MAX_FINGERS     2

/* Command mode entry, the attribute registers it unlocks, and normal mode.
 * The attribute registers are only readable in command mode (TCH-007). */
#define REG_CMD         0xD1
#define CMD_MODE        0x01
#define NORMAL_MODE     0x09
#define REG_CHECKCODE   0xD1FC
#define REG_RESOLUTION  0xD1F8
#define REG_CHIPTYPE    0xD204
#define REG_FWVERSION   0xD208
#define REG_PROBE_MODE  0x0002

#define CHECKCODE_MASK  0xFFFF0000u
#define CHECKCODE_ID    0xCACA0000u
#define CHIP_9217       0x9217
#define CHIP_9220       0x9220

#define POLL_MS         15
/* A pressed finger that stops reporting is treated as released after this,
 * because the controller never sends a release event of its own. */
#define RELEASE_MS      250

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static touch_input_fn s_input;
static bool s_down;
static int s_last_x = -1, s_last_y = -1;
static int s_idle_ms;

/* Diagnostics. Touch is the one subsystem that so far failed with no evidence
 * at all: bring_up() succeeding and then no report ever arriving looks exactly
 * the same from the desktop's side as a panel that never came out of reset.
 * So the first blocks the controller returns are logged verbatim, then only
 * the transitions, then a periodic count -- the Log Viewer is a 64-line ring
 * and a 15 ms poll would bury everything else in it. */
static uint32_t s_reads;
static uint32_t s_reports;
static int s_logged_blocks;
static int s_logged_events;

static void log_block(const char *what, const uint8_t *b)
{
    ESP_LOGI(TAG, "%s: %02x %02x %02x %02x %02x %02x %02x %02x "
                  "%02x %02x %02x %02x %02x %02x %02x",
             what,
             b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
             b[8], b[9], b[10], b[11], b[12], b[13], b[14]);
}

/* IDF 5.3 dropped the legacy driver/i2c.h: only the new master API exists. */
static esp_err_t wr(uint16_t reg, uint8_t v)
{
    uint8_t b[3] = { (uint8_t)(reg >> 8), (uint8_t)reg, v };
    return i2c_master_transmit(s_dev, b, sizeof(b), 100);
}

static esp_err_t rd(uint16_t reg, uint8_t *buf, size_t n)
{
    uint8_t a[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
    return i2c_master_transmit_receive(s_dev, a, sizeof(a), buf, n, 100);
}

/* The bring-up LilyGo's SensorLib performs, in the same order and with the
 * same checks (TCH-007). Returns false if the part is not there, is the wrong
 * part, or has no firmware -- all of which accept writes and then never report,
 * so continuing hopefully would only turn into a phantom-input bug later. */
static bool bring_up(void)
{
    uint8_t b[8] = { 0 };
    uint32_t checkcode, fw;
    uint16_t chiptype, max_x, max_y;

    /* Command mode: the 0xD1xx attribute registers are unreadable outside it. */
    if (wr(REG_CMD, CMD_MODE) != ESP_OK)
    {
        ESP_LOGW(TAG, "no answer at 0x%02X (command mode write failed)",
                 TOUCH_ADDR);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    if (rd(REG_CHECKCODE, b, 4) != ESP_OK)
    {
        ESP_LOGW(TAG, "0x%04X did not read", REG_CHECKCODE);
        return false;
    }
    checkcode = (uint32_t)b[3] << 24 | (uint32_t)b[2] << 16 |
                (uint32_t)b[1] << 8 | b[0];
    if ((checkcode & CHECKCODE_MASK) != CHECKCODE_ID)
    {
        ESP_LOGW(TAG, "bad checkcode 0x%08lx at 0x%04X (want 0xcaca____)",
                 (unsigned long)checkcode, REG_CHECKCODE);
        return false;
    }

    if (rd(REG_RESOLUTION, b, 4) == ESP_OK)
    {
        max_x = (uint16_t)(b[1] << 8 | b[0]);
        max_y = (uint16_t)(b[3] << 8 | b[2]);
    }
    else
    {
        max_x = max_y = 0;
    }

    if (rd(REG_CHIPTYPE, b, 4) != ESP_OK)
    {
        ESP_LOGW(TAG, "0x%04X did not read", REG_CHIPTYPE);
        return false;
    }
    chiptype = (uint16_t)(b[3] << 8 | b[2]);
    if (chiptype != CHIP_9217 && chiptype != CHIP_9220)
    {
        ESP_LOGW(TAG, "unexpected chip type 0x%04X", chiptype);
        return false;
    }

    if (rd(REG_FWVERSION, b, 8) != ESP_OK)
    {
        ESP_LOGW(TAG, "0x%04X did not read", REG_FWVERSION);
        return false;
    }
    fw = (uint32_t)b[3] << 24 | (uint32_t)b[2] << 16 |
         (uint32_t)b[1] << 8 | b[0];
    if (fw == 0xa5a5a5a5u)
    {
        ESP_LOGW(TAG, "no firmware in the controller");
        return false;
    }

    /* Normal mode. A part left in command mode answers a report read with 15
     * bytes of 0xFF, so this is a state change worth confirming rather than
     * assuming. The readback is a check, not an unlock: 0x00 0x09 on a working
     * part, never 0x1E. */
    wr(REG_CMD, NORMAL_MODE);
    vTaskDelay(pdMS_TO_TICKS(10));
    if (rd(REG_PROBE_MODE, b, 2) == ESP_OK && b[1] != NORMAL_MODE)
        ESP_LOGW(TAG, "mode readback 0x%02x, expected 0x%02x",
                 b[1], NORMAL_MODE);

    ESP_LOGI(TAG, "CST%04x at 0x%02X, %ux%u, fw 0x%08lx",
             chiptype, TOUCH_ADDR, max_x, max_y, (unsigned long)fw);

    if (max_x != DISPLAY_W || max_y != DISPLAY_H)
        ESP_LOGW(TAG, "controller reports %ux%u, panel window is %dx%d",
                 max_x, max_y, DISPLAY_W, DISPLAY_H);

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
        bool press;

        vTaskDelay(pdMS_TO_TICKS(POLL_MS));

        /*
         * Read the WHOLE 15-byte block. A short read leaves the controller's
         * acknowledge mid-block and the next report never arrives, which
         * presents as a frozen touch panel rather than as an I2C error.
         */
        if (rd(REPORT_ADDR, b, REPORT_LEN) != ESP_OK)
            continue;

        /*
         * Acknowledge the block with a separate write. The controller's buffer
         * does not advance until it has this, so without it every read returns
         * the same block for ever and a release is never seen (TCH-008). It is
         * sent whether or not the block turns out to be a press, because the
         * buffer has to be advanced either way -- the same reason the read is
         * the full 15 bytes.
         */
        wr(REPORT_ADDR, ACK_BYTE);

        s_reads++;
        if (s_logged_blocks < 3)
        {
            log_block("first blocks", b);
            s_logged_blocks++;
        }
        else if (s_reads % 300 == 0)
        {
            ESP_LOGI(TAG, "%lu reads, %lu reports, down=%d",
                     (unsigned long)s_reads, (unsigned long)s_reports,
                     s_down ? 1 : 0);
        }

        /* A block carries a press only if byte 6 has the validity marker, the
         * finger count in byte 5's low seven bits is 1 or 2, and the event
         * nibble of byte 0 is 0x06. A non-zero count with event 0x00 is not a
         * touch: taking it for one invents a phantom press at a stale
         * position (TCH-008). Byte 0 is 0xAB on an acknowledged read and 0x00
         * when the part handed nothing over. */
        press = false;
        if (b[0] != ACK_BYTE && b[0] != 0x00)
        {
            int fingers = b[5] & 0x7F;

            press = b[6] == ACK_BYTE && fingers > 0 &&
                    fingers <= MAX_FINGERS && (b[0] & 0x0F) == EVENT_DOWN;
            if (press && s_logged_events < 5)
            {
                log_block("press", b);
                s_logged_events++;
            }
        }

        if (!press)
        {
            /* This part has no release event: a lifted finger simply stops
             * being reported, so a held press that goes quiet is a release. */
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
        s_reports++;

        {
            int x = (b[1] << 4) | (b[3] >> 4);
            int y = (b[2] << 4) | (b[3] & 0x0F);
            int col, row;

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
    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TOUCH_ADDR,
        .scl_speed_hz = TOUCH_HZ,
    };

    /* The bus belongs to power.c: every peripheral on this board shares it,
     * and installing two masters on one port is not possible. */
    s_bus = board_i2c();
    if (!s_bus)
    {
        ESP_LOGE(TAG, "no I2C bus");
        return false;
    }
    if (i2c_master_bus_add_device(s_bus, &dev, &s_dev) != ESP_OK)
    {
        ESP_LOGE(TAG, "touch device did not register");
        return false;
    }

    s_input = input;
    if (!bring_up())
    {
        ESP_LOGW(TAG, "touch not started: the controller did not come up");
        return false;
    }

    if (xTaskCreatePinnedToCore(poll_task, "tdtouch", 3072, NULL, 3,
                                NULL, 0) != pdPASS)
        return false;

    ESP_LOGI(TAG, "CST9217 up at 0x%02X", TOUCH_ADDR);
    return true;
}
