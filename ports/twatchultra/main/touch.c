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
 * THE PANEL IS NOT A POINTER ON THE DESKTOP. A cell of the 40x26 grid is 8x16
 * px, so aiming a finger at the desktop's own controls is not practical, and
 * making them big enough would cost the screen. So while the desktop is
 * showing, the controller's reports are read as presses on the margins around
 * the framebuffer -- the left column, the bottom strip, the right scroll
 * column -- and a touch on the framebuffer itself does nothing. Those margins
 * are painted by display.c and their geometry lives in edges.h, shared with
 * the painter so that a button and its hit test cannot disagree.
 *
 * When an application owns the whole screen instead -- kiosk mode, or a
 * full-screen window -- the panel goes back to being a mouse for that
 * application: absolute position mapped onto the cell grid as SGR reports, with
 * drag as button-held motion. A game wants raw touch and gets it.
 */

#include "touch.h"

#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "display.h"
#include "edges.h"
#include "tinydesk/td_wm.h"
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

#define POLL_MS         20
/* A pressed finger that stops reporting is treated as released after this,
 * because the controller never sends a release event of its own. */
#define RELEASE_MS      250

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static touch_input_fn s_input;
static bool s_down;              /* pointer button down: only in pass-through */
static bool s_pressed;           /* the controller is reporting a press */
static int s_last_x = -1, s_last_y = -1;
static int s_idle_ms;

/* Diagnostics. Touch is the one subsystem that so far failed with no evidence
 * at all: bring_up() succeeding and then no report ever arriving looks exactly
 * the same from the desktop's side as a panel that never came out of reset.
 * So the first blocks the controller returns are logged, then only a count --
 * the Log Viewer is a 64-line ring and a 15 ms poll would bury everything else
 * in it.
 *
 * COMPACT ON PURPOSE. The Log Viewer window is 40 columns and silently
 * truncates anything longer, and that is how the checkcode fault stayed
 * invisible across several builds. Six bytes, no spaces: the event/count bytes
 * (0, 5, 6) and the three coordinate bytes are everything needed to tell a
 * press from a phantom. */
static uint32_t s_reads;
static uint32_t s_reports;
static int s_logged_blocks;

static void log_block(const char *what, const uint8_t *b)
{
    ESP_LOGI(TAG, "%s %02x%02x%02x%02x%02x%02x",
             what, b[0], b[1], b[2], b[3], b[5], b[6]);
}

/* IDF 5.3 dropped the legacy driver/i2c.h: only the new master API exists.
 *
 * TWO WRITE SHAPES, and they are not interchangeable. A command-mode entry is
 * a two-byte write -- the register pair alone, 0xD1 0x01 -- exactly as the
 * measured driver's cst92xx_write_cmd() does it. A register write that carries
 * a value is three bytes: high, low, value. Sending the two-byte command as
 * three (0x00 0xD1 0x01) addresses register 0x00D1 instead, which this part
 * accepts and ignores, so command mode is never entered and the 0xD1FC read
 * afterwards returns normal-mode content rather than the checkcode. */
static esp_err_t wr(uint16_t reg, uint8_t v)
{
    uint8_t b[3] = { (uint8_t)(reg >> 8), (uint8_t)reg, v };
    return i2c_master_transmit(s_dev, b, sizeof(b), 100);
}

static esp_err_t cmd(uint8_t hi, uint8_t lo)
{
    uint8_t b[2] = { hi, lo };
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

    /* Command mode: the 0xD1xx attribute registers are unreadable outside it.
     * Two bytes, not three -- see the note above cmd(). */
    if (cmd(REG_CMD, CMD_MODE) != ESP_OK)
    {
        ESP_LOGW(TAG, "no answer at 0x%02X", TOUCH_ADDR);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    if (rd(REG_CHECKCODE, b, 4) != ESP_OK)
    {
        ESP_LOGW(TAG, "0xD1FC did not read");
        return false;
    }
    checkcode = (uint32_t)b[3] << 24 | (uint32_t)b[2] << 16 |
                (uint32_t)b[1] << 8 | b[0];
    if ((checkcode & CHECKCODE_MASK) != CHECKCODE_ID)
    {
        /* Compact: the Log Viewer is 40 columns and truncates anything wider,
         * which is exactly how this fault stayed invisible. */
        ESP_LOGW(TAG, "bad cc %02x%02x%02x%02x",
                 b[3], b[2], b[1], b[0]);
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
        ESP_LOGW(TAG, "0xD204 did not read");
        return false;
    }
    chiptype = (uint16_t)(b[3] << 8 | b[2]);
    if (chiptype != CHIP_9217 && chiptype != CHIP_9220)
    {
        ESP_LOGW(TAG, "chip type %04x", chiptype);
        return false;
    }

    if (rd(REG_FWVERSION, b, 8) != ESP_OK)
    {
        ESP_LOGW(TAG, "0xD208 did not read");
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
    cmd(REG_CMD, NORMAL_MODE);
    vTaskDelay(pdMS_TO_TICKS(10));
    if (rd(REG_PROBE_MODE, b, 2) == ESP_OK && b[1] != NORMAL_MODE)
        ESP_LOGW(TAG, "mode %02x want %02x", b[1], NORMAL_MODE);

    ESP_LOGI(TAG, "CST%04x %ux%u fw%08lx",
             chiptype, max_x, max_y, (unsigned long)fw);

    /* The controller reports the glass, 410x502, not the 320x400 rectangle the
     * desktop draws into -- that is the expected answer (TCH-006), so this only
     * speaks up if the part reports something else entirely, which would mean
     * the attribute read or the part is not what this driver assumes. */
    if (max_x != DISPLAY_PANEL_W || max_y != DISPLAY_PANEL_H)
        ESP_LOGW(TAG, "res %ux%u want 410x502", max_x, max_y);

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

/* Panel pixels to a terminal cell.
 *
 * The controller reports in glass coordinates over the whole 410x502 panel,
 * but the desktop only occupies the 320x416 rectangle at (45,45). So the
 * origin comes off first, and only then is the position divided by the cell
 * size. Without that subtraction every touch lands 45 px down and right of
 * where the finger is and the right-hand fifth of the screen is unreachable --
 * the mapping was scaling the whole glass onto the desktop's grid.
 *
 * This is the pass-through path and it runs only when an application owns the
 * screen. On the desktop itself a touch on the framebuffer does nothing at
 * all, which is why the clamp below is a safety net rather than a routing
 * rule: it used to route the margins onto the taskbar, and that put the left
 * margin on the Start button and the right margin on Date and time. */
static void to_cell(int x, int y, int *col, int *row)
{
    int cw = DISPLAY_W / display_cols();
    int ch = DISPLAY_H / display_rows();
    int fx = x - DISPLAY_ORIGIN_X;
    int fy = y - DISPLAY_ORIGIN_Y;

    *col = fx / cw;
    *row = fy / ch;
    if (*col < 0)
        *col = 0;
    if (*row < 0)
        *row = 0;
    if (*col >= display_cols())
        *col = display_cols() - 1;
    if (*row >= display_rows())
        *row = display_rows() - 1;
}

/* ---- the glass margins, which are the only touch targets on the desktop ----
 *
 * The framebuffer is 320x416 on a 410x502 glass, and a finger on that grid is
 * not usable: a cell is 8x16 px, so aiming at a button means landing inside a
 * 16-pixel-tall target, and the desktop's own controls are one or two cells.
 * So the touch panel stops being a pointer while the desktop is showing. What
 * it does instead is the three margins, which are large, painted, and in the
 * same place every time.
 *
 * An application that owns the whole screen gets the panel back, as a pointer,
 * with no margins and no dead zones: kiosk mode and full screen are exactly the
 * cases where an application wants raw touch, and on those the glass is its,
 * not ours. td_wm_app_owns_screen() is the question, and it is the same one the
 * window manager asks before drawing a taskbar. */

/* A key, as the terminal would send it: the desktop's input parser turns these
 * back into key events, so an edge press is indistinguishable from the same key
 * on a Bluetooth keyboard.
 *
 * WHICH KEYS, AND WHY NOT F1 TO F5. The first version sent F1..F5, which was
 * wrong for the same reason the panel stopped being a pointer: those are app
 * keys. On the desktop they do nothing at all, and inside a window they are
 * already spoken for -- Files renames on F2 and reloads on F5, Editor finds on
 * F3 and runs on F5, Terminal forwards them to the shell. Five physical buttons
 * spent on functions that only ever fire inside a window, while the things that
 * lost their mouse go unmapped. These are window manager actions, and they are
 * the ones that keep the desktop usable with no pointer on the glass:
 *
 *   1  F10      the Start menu -- otherwise reachable only from the taskbar or
 *               an icon, both of which need a mouse
 *   2  Esc      cancel: closes a dialog, abandons a rename, backs out of a menu
 *   3  F6       the window switcher, the only way to change focus
 *   4  Super+D  show the desktop, and again to bring it all back -- the only way
 *               to restore a minimised window, since F6 skips hidden ones
 *   5  F11      full screen for the focused window
 *
 * Super+D goes in xterm's modifyOtherKeys form, ESC [ 27 ; 9 ; d ~, because
 * there is no single byte for it and the parser already understands that shape.
 * A lone Esc is the one key here that waits: 0x1b is the start of every escape
 * sequence, so the parser resolves it on TD_ESC_TIMEOUT_MS, which is 50 ms. */
static const char *const s_fkey[LEFT_N] = {
    "\x1b[21~",             /* F10     Start menu */
    "\x1b",                 /* Esc     cancel */
    "\x1b[17~",             /* F6      window switcher */
    "\x1b[27;9;100~",       /* Super+D show desktop */
    "\x1b[23~",             /* F11     full screen */
};

/* Left to right: Tab, Enter, Back.
 *
 * Back is Ctrl+Q because that is what the window manager already treats as
 * "close the focused window" -- the same path as its [x], routed through
 * td_win_request_close() so a window that has something to lose can refuse; the
 * Editor asks about unsaved changes and everything else closes outright. It is
 * Ctrl+Q rather than Alt+F4 because a terminal only passes Alt+F4 through when
 * the terminal layer chooses to, and this one is injecting the bytes itself. */
static const char *const s_bkey[BTN_N] = {
    "\t", "\r", "\x11",
};

static void send_str(const char *s)
{
    if (s && s_input)
        s_input(s, (int)strlen(s));
}

/* The right column scrolls: the wheel, at the middle of the desktop.
 *
 * The position matters because the window manager routes a wheel event to the
 * window under it, and there is no pointer on the glass anymore to take the
 * position from. The middle of the desktop is under whichever window is
 * focused, which is the one the operator is looking at. */
static void send_wheel(int y)
{
    char buf[32];
    int button = y < (LEFT_Y0 + LEFT_Y1) / 2 ? 64 : 65;   /* up / down */
    int n = snprintf(buf, sizeof(buf), "\x1b[<%d;%d;%dM",
                     button, display_cols() / 2 + 1, display_rows() / 2 + 1);

    if (n > 0 && s_input)
        s_input(buf, n);
}

/* What a press at the start of a touch does. Anything not in a margin does
 * nothing: on the desktop the framebuffer is not a touch surface. */
static void edge_press(const edge_hit_t *hit, int y)
{
    /* One short line per edge press, and short on purpose: the Log Viewer is 40
     * columns and truncates silently. This is what distinguishes the three ways
     * a margin button can fail -- no line means the press was not on an edge at
     * all, `e0` means the region was classified but not the button, and a
     * plausible line with the wrong action means the index. */
    ESP_LOGI(TAG, "e%d i%d", (int)hit->region, hit->index);

    switch (hit->region)
    {
    case EDGE_LEFT:
        if (hit->index >= 0 && hit->index < LEFT_N)
            send_str(s_fkey[hit->index]);
        break;
    case EDGE_BOTTOM:
        if (hit->index >= 0 && hit->index < BTN_N)
            send_str(s_bkey[hit->index]);
        break;
    case EDGE_RIGHT:
        send_wheel(y);
        break;
    default:
        break;
    }
}

/* Emit a release if a press has stopped reporting.
 *
 * This part has no release event: a lifted finger simply stops being reported,
 * so a held press that goes quiet is a release. Called both when a block is
 * not a press and when a press arrives where nothing is listening, so that the
 * count keeps running in every case -- and so that a finger which slides off
 * the glass does not leave the pointer latched down. */
static void release_if_idle(void)
{
    if (!s_pressed)
    {
        s_idle_ms = 0;
        return;
    }
    if (++s_idle_ms < (RELEASE_MS / POLL_MS))
        return;
    if (s_down)
    {
        int col, row;

        to_cell(s_last_x, s_last_y, &col, &row);
        emit(false, col, row);
        s_down = false;
    }
    s_pressed = false;
    s_idle_ms = 0;
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
            log_block("blk", b);
            s_logged_blocks++;
        }
        else if (s_reads % 300 == 0)
        {
            ESP_LOGI(TAG, "%lu rd %lu rep d%d",
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
            if (press)
                log_block("prs", b);
        }

        if (!press)
        {
            release_if_idle();
            continue;
        }

        {
            int x = (b[1] << 4) | (b[3] >> 4);
            int y = (b[2] << 4) | (b[3] & 0x0F);
            edge_hit_t hit;
            bool app = td_wm_app_owns_screen();

            s_reports++;
            s_idle_ms = 0;

            edges_hit(x, y, &hit);

            /* The action fires once per touch, on the first report, not once
             * per report: the controller re-reports the same finger about
             * every 66 ms while it is held, which would be a hundred presses a
             * second on a held Tab. */
            if (!s_pressed)
            {
                s_pressed = true;
                if (!app)
                    edge_press(&hit, y);
            }

            /* An application that owns the screen gets the panel as a pointer,
             * margins included -- on those the glass is its, and a game that
             * draws its own controls near the edge needs the whole surface.
             * On the desktop nothing here runs: the framebuffer is not a touch
             * surface, and the margins were already handled above. */
            if (!app)
                continue;

            {
                int col, row;

                to_cell(x, y, &col, &row);
                s_last_x = x;
                s_last_y = y;

                /* A move while down is a drag: the desktop gets motion with
                 * the button held, which is what a window manager expects for
                 * a title bar or a scroll bar. */
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

    /* The controller NACKs every read taken while it has nothing to hand over,
     * which at rest is most of them (TCH-003), and ESP-IDF's i2c.master logs
     * each one at ERROR. That is normal idle behaviour for this part, not a
     * fault, and left alone it fills the Log Viewer with three ERROR lines per
     * poll -- several hundred a second at a 20 ms period -- burying the driver's
     * own diagnostics and anything else the board wants to say. Failures of the
     * reads that matter are reported by this driver instead. */
    esp_log_level_set("i2c.master", ESP_LOG_NONE);

    if (!bring_up())
    {
        ESP_LOGW(TAG, "not started");
        return false;
    }

    if (xTaskCreatePinnedToCore(poll_task, "tdtouch", 3072, NULL, 3,
                                NULL, 0) != pdPASS)
        return false;

    ESP_LOGI(TAG, "CST9217 up at 0x%02X", TOUCH_ADDR);
    return true;
}
