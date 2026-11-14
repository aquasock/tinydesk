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
 *   * The INT line is GPIO 12, active low, and the part drives it low then
 *     releases it, so it is open-drain and the pull-up is part of the
 *     contract (TCH-009). LilyGo's own initTouch() configures it
 *     INPUT_PULLUP with a FALLING handler, and the handler does nothing but
 *     set an event bit -- the I2C read happens in the consumer, never in the
 *     interrupt. This driver does the same: the ISR only wakes this task.
 *     Polling is NOT removed, because a lost pull-up or a wrong polarity
 *     would otherwise lose touch entirely, whereas a slow poll degrades to
 *     latency (TCH-009).
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
 * the framebuffer -- a column of five down each side and a strip of three along
 * the bottom -- and a touch on the framebuffer itself does nothing. Those
 * margins are painted by display.c and their geometry lives in edges.h, shared
 * with the painter so that a button and its hit test cannot disagree.
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
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "display.h"
#include "edges.h"
#include "haptic.h"
#include "tinydesk/td_wm.h"
#include "power.h"
#include "tdsh.h"
#include "tdsh_board.h"
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

/* A pressed finger that stops reporting is treated as released after this,
 * because the controller never sends a release event of its own.
 *
 * 100 MS, DOWN FROM 250, AND THE THREE SYMPTOMS THAT FORCED IT ARE ONE FAULT.
 *
 * Reported from the glass: a single tap on the d-pad moved Zelda's name-entry
 * cursor two places, fast A presses were swallowed, and the d-pad felt laggy in
 * the overworld. All three are the release being late, because `s_pad` is a
 * HELD state: it stays set until this timeout expires, so a 120 ms tap became a
 * 370 ms press. A menu that scrolls while a direction is held therefore moved
 * twice, and two taps 150 ms apart were merged into one continuous press so the
 * second was never seen as a press at all.
 *
 * The old 250 was chosen against a claim that the controller re-reports a held
 * finger every ~66 ms, so 250 was about four report periods of tolerance. But
 * this driver does not wait for the controller's own cadence: while `s_pressed`
 * is true it reads the whole block every ACTIVE_MS, and each read is
 * acknowledged, so the timing that actually matters is the 20 ms poll, not the
 * part's report interval. A timeout of 100 ms therefore tolerates FOUR
 * consecutive failed reads, which is a much larger guard than one dropped
 * report and is the real margin to protect.
 *
 * The floor is set by the opposite risk: if a held finger ever goes longer than
 * this between press blocks, the driver will release it and a held direction
 * will stutter. That is why the value was not taken lower still, and if a hold
 * ever feels like it is dropping out, the number to raise is this one. */
#define RELEASE_MS      100
/* Wake-up period while a finger is down, and while nothing is touching.
 *
 * IDLE_MS IS NOW 50, DOWN FROM 250, AND THAT WAS THE SECOND HALF OF THE SAME
 * FAULT. This period is the fallback for when the interrupt does not wake the
 * task, and it is also the state the task sleeps in immediately after every
 * release. At 250 ms that made a fast second tap easy to miss outright: the
 * first tap's release put the loop back to a 250 ms sleep, and if the interrupt
 * did not fire the next press was not read until that expired -- by which time
 * the finger had often gone, so the press was never seen. That is the reported
 * "hit A fast enough and it stops capturing my presses".
 *
 * The cost is bus traffic, and it is small: 20 reads a second against 4, on a
 * bit-banged 100 kHz bus where a 15-byte block is on the order of 1.5 ms, so
 * roughly 3 percent duty. The interrupt is still the mechanism meant to do the
 * work, and the cost of these timers only matters when it is not.
 *
 * ACTIVE_MS while pressed keeps a drag up with the controller's own cadence.
 * Without the interrupt the active period is used unconditionally, which is
 * what this driver did before: ~50 reads a second on a bus shared with the PMU,
 * the expander, the RTC and the haptic driver. */
#define ACTIVE_MS       20
#define IDLE_MS         50

/* Floor between services when the interrupt keeps firing. The controller only
 * pulses INT when it has something, so this should never bite -- but a stuck
 * or noisy line would otherwise spin this task at hundreds of hertz on core 0,
 * which is the core that owns the radios. */
#define MIN_GAP_MS      5

/* TP_INT, from arduino-esp32's lilygo_twatch_ultra variant and LilyGoLib's
 * own initTouch(), and confirmed in the schematic net list as T_INT------IO12.
 * `touch.int` in the board configuration overrides it; -1 means drive this
 * driver by polling alone, which is the A/B control for the interrupt. */
#define TOUCH_INT       12

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static touch_input_fn s_input;
static bool s_down;              /* pointer button down: only in pass-through */
static bool s_pressed;           /* the controller is reporting a press */

/* The application's own buttons, and which one is currently held. Kept here
 * rather than in the app because the release is synthesised from this driver's
 * own idle timer: the part emits no release event, so only the code that
 * notices the reports stopping can say when a button came up. */
/*
 * Raw-block capture, for the multi-touch question.
 *
 * The buffer is a PSRAM heap allocation rather than a static array: 512 blocks
 * of 16 bytes is 8 KB, and internal DRAM on this board idles near 66 KB free
 * with tdsh's SSH server wanting 65,536 of it (EMU-018). A diagnostic that
 * costs the board a peripheral is not a diagnostic.
 *
 * Written by the poll task, read by the shell task once the flag is cleared --
 * one writer, which is why there is no lock. The reader waits a poll period
 * after clearing the flag so a capture already in flight finishes first.
 */
static uint8_t (*s_dump_buf)[REPORT_LEN];
static volatile int s_dump_n;
static volatile int s_dump_cap;
static volatile bool s_dumping;

static touch_app_touch_fn s_app_touch;
static void *s_app_touch_user;

static bool deliver_app_touch(const touch_frame_t *f);
static bool s_app_active;        /* the app currently has points down */
static int s_last_x = -1, s_last_y = -1;

/* When the last press report arrived. RELEASE_MS is measured against this and
 * not against a count of polls, because the poll period is no longer constant:
 * an interrupt-woken loop runs at whatever rate the controller reports. The
 * previous version counted polls and compared against RELEASE_MS / POLL_MS,
 * which was correct only while every iteration took exactly one fixed poll
 * period -- adding a 250 ms idle sleep to that arithmetic would have made a
 * release take three seconds. */
static int64_t s_last_press_us;
static int64_t s_last_service_us;

/* The interrupt path. -1 when it is not in use, in which case the loop falls
 * back to a fixed 20 ms period. */
static int s_int_pin = -1;
static TaskHandle_t s_task;
static volatile uint32_t s_irqs;

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
static const char *const s_fkey[COL_N] = {
    "\x1b[21~",             /* F10     Start menu */
    "\x1b",                 /* Esc     cancel */
    "\x1b[17~",             /* F6      window switcher */
    "\x1b[27;9;100~",       /* Super+D show desktop */
    "\x1b[23~",             /* F11     full screen */
};

/* The right column, top to bottom: nothing, Left, Right, Up, Down.
 *
 * Arrows, because the other half of the desktop's actions are movement and the
 * margins now form a complete set with no pointer anywhere: the left column is
 * what windows do, this one is where the cursor goes. They reach whatever has
 * the focus -- the icon grid when nothing is open, a list or a form inside a
 * window -- and they are the key a keyboard would send, so nothing here is a
 * private convention.
 *
 * The first button is deliberately empty and is what the operator asked for:
 * a reserved slot. It is painted like the others, so it looks live and does
 * nothing, which is worth knowing before someone presses it and wonders. */
static const char *const s_rkey[COL_N] = {
    NULL,                   /* reserved */
    "\x1b[D",               /* Left */
    "\x1b[C",               /* Right */
    "\x1b[A",               /* Up */
    "\x1b[B",               /* Down */
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

/* What a press at the start of a touch does. Anything not in a margin does
 * nothing: on the desktop the framebuffer is not a touch surface. */
static void edge_press(const edge_hit_t *hit, int y)
{
    const char *key = NULL;

    (void)y;

    /* One short line per edge press, and short on purpose: the Log Viewer is 40
     * columns and truncates silently. This is what distinguishes the three ways
     * a margin button can fail -- no line means the press was not on an edge at
     * all, `e0` means the region was classified but not the button, and a
     * plausible line with the wrong action means the index. */
    ESP_LOGI(TAG, "e%d i%d", (int)hit->region, hit->index);

    /*
     * A SLOT THAT IS NOT DRAWN DOES NOT ACT, and the check is here rather than in
     * every app because this is the one place a margin press becomes a key.
     *
     * The fault it fixes was measured: an app that hid slots left them live, so
     * the glass under a blank slot still sent a key and still buzzed, right beside
     * the controls the operator was testing. `edges_hit()` classifies against
     * COL_N and knows nothing about what the app draws, so the driver has to ask
     * -- and it asks the same predicate the painter uses, so 'not drawn' and 'not
     * live' cannot drift apart.
     */
    switch (hit->region)
    {
    case EDGE_LEFT:
        if (display_margin_slot_live(0, hit->index))
            key = s_fkey[hit->index];
        break;
    case EDGE_RIGHT:
        if (display_margin_slot_live(1, hit->index))
            key = s_rkey[hit->index];
        break;
    case EDGE_BOTTOM:
        if (display_margin_slot_live(2, hit->index))
            key = s_bkey[hit->index];
        break;
    default:
        break;
    }

    if (!key)
        return;                     /* inert: the top margin, or the reserved slot */

    send_str(key);

    /* AFTER the key, so the click coincides with the action rather than
     * preceding it. Only when a key was actually sent: a press on the reserved
     * slot or on inert glass does nothing, and buzzing for it would say
     * something happened when nothing did. */
    haptic_tap();
}

/* Emit a release if a press has stopped reporting.
 *
 * This part has no release event: a lifted finger simply stops being reported,
 * so a held press that goes quiet is a release. Called both when a block is
 * not a press and when a press arrives where nothing is listening, so that the
 * count keeps running in every case -- and so that a finger which slides off
 * the glass does not leave the pointer latched down.
 *
 * MEASURED IN WALL TIME, NOT IN POLLS. The previous version incremented a
 * counter once per iteration and compared it against RELEASE_MS / POLL_MS,
 * which holds only while every iteration takes exactly one poll period. With
 * the interrupt the loop wakes when the controller reports, so an iteration
 * can be any length, and counting iterations would have tied the release to
 * the wake rate instead of to the 250 ms the value means.
 *
 * TWO KINDS OF "NOT A PRESS", AND CONFLATING THEM WAS THE DRAG FAULT.
 *
 * A block that is fifteen 0xFF bytes means nothing is touching the glass: the
 * controller reports idle that way (TCH-012), and that is a FACT about the
 * finger rather than an absence of news. A block whose first byte is ACK_BYTE
 * means the part was acknowledged and had nothing new to say, which says nothing
 * at all about whether a finger is still down.
 *
 * Treating both as "wait RELEASE_MS then release" cost the operator precise
 * movement on Zelda's overworld: he reported the character overshooting and
 * being unable to line up with a cave door. At 60 frames a second a 100 ms late
 * release is SIX FRAMES of travel after the thumb has lifted, and at 100 ms it
 * was fifteen. The timeout exists only to bridge the controller's ~66 ms report
 * interval for a HELD finger -- an all-0xFF block needs no bridging and is
 * released on the next poll, which is 20 ms.
 *
 * `idle_confirmed` is that distinction, and it comes from the block rather than
 * from a judgement here. */
static void release_if_idle(bool idle_confirmed)
{
    if (!s_pressed)
        return;

    if (!idle_confirmed &&
        esp_timer_get_time() - s_last_press_us < (int64_t)RELEASE_MS * 1000)
        return;

    /* Tell the app the glass is clear. One empty frame ends a chord, a hold and
     * a stray press alike, because the frame is state and "nothing down" is a
     * state. Only sent when something was actually reported first, so an idle
     * panel does not wake the app every 250 ms. */
    if (s_app_active)
    {
        touch_frame_t empty;

        memset(&empty, 0, sizeof(empty));
        empty.p[0].region = EDGE_NONE;
        empty.p[1].region = EDGE_NONE;

        s_app_active = false;

        deliver_app_touch(&empty);
    }

    if (s_down)
    {
        int col, row;

        to_cell(s_last_x, s_last_y, &col, &row);
        emit(false, col, row);
        s_down = false;
    }
    s_pressed = false;
}

/* The controller has something to say. THIS DOES NOT TOUCH I2C.
 *
 * LilyGo's own handler does exactly one thing -- set an event-group bit -- and
 * leaves the transfer to the consumer, and that split matters here: an I2C
 * transfer inside an ISR would run with the cache and scheduler in a state the
 * driver does not expect, and this bus is shared with four other parts. The
 * notification coalesces, which is correct: several edges before the task runs
 * still mean one read, and one read returns the controller's newest block. */
static void IRAM_ATTR touch_isr(void *arg)
{
    (void)arg;
    BaseType_t woken = pdFALSE;
    TaskHandle_t t = s_task;

    s_irqs++;
    if (t)
        vTaskNotifyGiveFromISR(t, &woken);
    if (woken)
        portYIELD_FROM_ISR();
}

static void poll_task(void *arg)
{
    uint8_t b[REPORT_LEN];
    bool first = true;

    (void)arg;

    for (;;)
    {
        bool press;

        /* Wait for the controller to say something, or for the period to run
         * out. Both cases matter and neither is redundant:
         *
         *   - with a finger down the period is ACTIVE_MS, so the loop keeps
         *     reading even if the part has stopped pulsing INT for a held
         *     finger, and the release timeout still gets checked;
         *   - with nothing touching it is IDLE_MS, so the interrupt is what
         *     normally wakes us and the poll is only the safety net.
         *
         * A notification that arrives while the task is awake is left pending
         * by ulTaskNotifyTake, so the next iteration returns immediately
         * rather than sleeping through a report. */
        if (s_int_pin >= 0)
        {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(s_pressed ? ACTIVE_MS : IDLE_MS));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(ACTIVE_MS));
        }

        /* Rate floor, see MIN_GAP_MS. Skipped on the first pass so the initial
         * block is read without an artificial delay. */
        if (!first && s_int_pin >= 0)
        {
            int64_t since = esp_timer_get_time() - s_last_service_us;
            if (since < (int64_t)MIN_GAP_MS * 1000)
                vTaskDelay(pdMS_TO_TICKS(MIN_GAP_MS - (int)(since / 1000)));
        }
        first = false;
        s_last_service_us = esp_timer_get_time();

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

        /*
         * IS ANYTHING TOUCHING THE GLASS AT ALL?
         *
         * Fifteen 0xFF bytes is idle, and the controller says so plainly (TCH-012).
         * That is a different statement from a block with ACK_BYTE in byte 0, which
         * only means the part had no new report -- a finger can be held and produce
         * one of those. The release path needs the difference, so it is decided here
         * once, from the block itself, rather than inferred later.
         */
        bool idle_block = true;

        for (int k = 0; k < REPORT_LEN; k++)
            if (b[k] != 0xFF)
                idle_block = false;

        /*
         * Capture before any interpretation, so what lands in the file is what
         * the controller said rather than what this driver understood.
         *
         * IDLE BLOCKS ARE NOT STORED, and that is a correction rather than an
         * optimisation. The buffer is 512 blocks, and with a finger down the
         * poll runs at ACTIVE_MS so 512 blocks is TEN SECONDS -- which silently
         * truncated a thirty-second capture at block 512, after which the file
         * simply stopped while the command still reported success. That cost
         * three captures and, worse, produced a file that looked complete: the
         * gesture in progress at the cutoff was recorded up to its last
         * recorded block and no further, so the release looked absent rather
         * than missing. Storing only real reports makes the depth a count of
         * EVENTS, which is what the buffer is for.
         */
        if (s_dumping && s_dump_n < s_dump_cap)
        {
            if (!idle_block)
            {
                memcpy(s_dump_buf[s_dump_n], b, REPORT_LEN);
                s_dump_n++;
            }
        }

        if (s_logged_blocks < 3)
        {
            log_block("blk", b);
            s_logged_blocks++;
        }
        else if (s_reads % 300 == 0)
        {
            /* Two short fields, because the ring is 40 columns and the window
             * is 38 of them: with the tag and a growing millisecond stamp ahead
             * of it there are about thirteen characters left, which fits
             * `i1488 r3000` and does not fit `irq1488 rd3000 p12`. This line
             * was truncated on its first real use, which is why p is gone --
             * presses are counted by the prs lines now that those are one per
             * touch.
             *
             * i is the interrupt edge count: non-zero and climbing when the
             * glass is touched, and flat when it is not. r is the bus reads,
             * the number the whole change was about. */
            ESP_LOGI(TAG, "i%lu r%lu",
                     (unsigned long)s_irqs, (unsigned long)s_reads);
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

            /*
             * EXACT 0x06, NOT (b[0] & 0x0F) == 0x06, and that is a fix with a
             * measured fault behind it.
             *
             * When a two-finger chord is broken by lifting the FIRST finger,
             * this controller promotes the survivor into the primary slot: its
             * coordinates move to bytes 1..3 and its event code becomes 0x16.
             * Masking the low nibble makes 0x16 look exactly like 0x06, so the
             * stray finger left on the glass after a chord was accepted as a
             * fresh press at its own position -- which means releasing the arm
             * could fire whatever button the other finger was resting on, with
             * the interlock already satisfied and gone. That is precisely the
             * accidental Reset the arm exists to prevent.
             *
             * The capture that proves it (blocks 558..784 of the 2026-11-12
             * lift-order dump) is preserved at
             * ~/twatch-backup/touch-captures/touch7.txt on the development
             * machine and summarised in TCH-012.
             *
             * 0x16 is therefore "a point is present but it is not a new
             * primary press", and the driver treats it as no press at all. */
            press = b[6] == ACK_BYTE && fingers > 0 &&
                    fingers <= MAX_FINGERS && b[0] == EVENT_DOWN;
        }

        /* Log the FIRST report of a touch and nothing while it is held.
         *
         * This used to log every press report, and the controller re-reports a
         * held finger every ~66 ms, so a two-second hold wrote about thirty
         * lines into a 64-line ring and a drag emptied it completely: the Log
         * Viewer became unreadable exactly while something was being
         * diagnosed with it. That is the third time this file records a
         * diagnostic defeating itself, and unlike the 40-column truncations it
         * was not a display limit but the log's own volume.
         *
         * The transition is the interesting event anyway -- the first block
         * carries the position a press started at -- and s_pressed is only
         * cleared by release_if_idle(), so it still holds the previous
         * iteration's state here. */
        if (press && !s_pressed)
            log_block("prs", b);

        if (!press)
        {
            release_if_idle(idle_block);
            continue;
        }

        {
            int x = (b[1] << 4) | (b[3] >> 4);
            int y = (b[2] << 4) | (b[3] & 0x0F);
            edge_hit_t hit;
            bool app = td_wm_app_owns_screen();
            bool edge = !s_pressed;

            s_last_press_us = esp_timer_get_time();

            edges_hit(x, y, &hit);

            /* Who owns the glass decides whether the margins are controls at
             * all. `app` is the desktop's own answer and it is not enough:
             * display_pixels_claim() is the port's, and an application that
             * draws pixels without being a window manager window -- the
             * Tamagotchi emulator -- claims the framebuffer without ever
             * making td_wm_app_owns_screen() true. So a press on Enter while
             * the guest had the screen sent Enter to the hidden shell, which
             * is the fault this line fixes. Ownership is the honest signal and
             * it is already the contract the painter honours. */
            bool owned = display_pixels_claimed();

            s_pressed = true;

            /*
             * A dump must not also ACT. The margins send keys and the
             * framebuffer is a pointer, so a capture taken while the operator
             * touches the glass would otherwise type Enter into the shell, open
             * the Start menu, or close the window that is about to print the
             * result -- and the buttons being characterised are exactly the ones
             * that do it.
             */
            if (s_dumping)
                continue;

            /*
             * A CONSUMER THAT OWNS THE PIXELS GETS STATE, NOT EVENTS.
             *
             * Reported on every block, not only on the first, because the
             * question a chord asks -- "is the arm still held" -- has to be
             * answerable at the moment the action arrives, and an edge-only
             * callback cannot answer it. The buzz fires on the report where the
             * set of points CHANGES, which is the transition a finger feels.
             */
            if (owned)
            {
                touch_frame_t f;

                memset(&f, 0, sizeof(f));
                f.p[0].region = EDGE_NONE;
                f.p[1].region = EDGE_NONE;
                f.n = 1;
                f.p[0].region = hit.region;
                f.p[0].index = hit.index;

                /* The coordinates travel too, and they are the reason this
                 * struct grew: a region and an index describe the margins, and a
                 * press on a control the app drew for itself is inside the
                 * framebuffer, where edges_hit() returns EDGE_NONE. The driver
                 * decoded these two numbers and threw them away. */
                f.p[0].x = x;
                f.p[0].y = y;

                /* The second point, at bytes 8..10 with its event code at byte 7
                 * and its validity marker at byte 12. Same 12-bit encoding as
                 * the first: x = (hi1 << 4) | (lo >> 4), y = (hi2 << 4) | (lo &
                 * 0x0F). Read only when the count says a second point exists,
                 * and only when its own validity marker is present -- a block
                 * claiming two fingers but carrying one valid point would
                 * otherwise decode 0xFF-filled bytes into a position on the
                 * glass and invent a button press at it. */
                if ((b[5] & 0x7F) == 2 && b[12] == ACK_BYTE && b[7] != 0xFF)
                {
                    int x2 = (b[8] << 4) | (b[10] >> 4);
                    int y2 = (b[9] << 4) | (b[10] & 0x0F);
                    edge_hit_t hit2;

                    edges_hit(x2, y2, &hit2);

                    f.p[1].region = hit2.region;
                    f.p[1].index = hit2.index;
                    f.p[1].x = x2;
                    f.p[1].y = y2;
                    f.n = 2;
                }

                s_app_active = true;

                /*
                 * THE BUZZ FOLLOWS CONSUMPTION, NOT THE PRESS EDGE.
                 *
                 * This used to fire `haptic_tap()` on ANY press while an app owned
                 * the screen, so a finger on blank glass, on a margin slot the app
                 * had hidden, or on the padding between two drawn controls all felt
                 * exactly like a real button. The operator reported it as phantom
                 * buttons still vibrating after the slots themselves were removed,
                 * and it made the d-pad impossible to test because every touch
                 * appeared to register.
                 *
                 * The driver cannot answer this alone. Margin-slot liveness is now
                 * knowable (`display_margin_slot_live`), but whether a point INSIDE
                 * the framebuffer hit a control is known only to the app that drew
                 * it, so the answer comes back from the callback.
                 */
                const bool used = deliver_app_touch(&f);

                if (edge && used)
                {
                    ESP_LOGI(TAG, "e%d i%d n%d", (int)hit.region, hit.index, f.n);
                    haptic_tap();
                }

                continue;      /* no keys, and no pointer: the app has it */
            }

            if (!app && edge)
                edge_press(&hit, y);

            /* An application that owns the screen gets the panel as a pointer,
             * margins included -- on those the glass is its, and a game that
             * draws its own controls near the edge needs the whole surface.
             * On the desktop nothing here runs: the framebuffer is not a touch
             * surface, and the margins were already handled above.
             *
             * A CLAIMED framebuffer is not this path, even when a window
             * manager window also says it owns the screen: an app that claimed
             * the pixels has been offered the margins as buttons and expects
             * them as buttons, and sending it pointer reports as well would be
             * two answers to one press. */
            if (!app || owned)
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

#define APP_TOUCH_MAX 4
static touch_app_touch_fn s_app_touch_extra[APP_TOUCH_MAX];
static void *s_app_touch_extra_user[APP_TOUCH_MAX];

void touch_set_app_touch(touch_app_touch_fn fn, void *user)
{
    s_app_touch = fn;
    s_app_touch_user = user;
    s_app_active = false;
}

bool touch_add_app_touch(touch_app_touch_fn fn, void *user)
{
    if (!fn)
        return false;

    for (int i = 0; i < APP_TOUCH_MAX; i++)
    {
        if (s_app_touch_extra[i] == fn)
            return true;                    /* already registered */

        if (!s_app_touch_extra[i])
        {
            s_app_touch_extra[i] = fn;
            s_app_touch_extra_user[i] = user;
            return true;
        }
    }

    return false;
}

/* Every registered consumer, in registration order. Only the one that owns the
 * screen acts; the others check their own running state and return.
 *
 * Returns true if ANY consumer reported that a control fired, which is what the
 * haptic tap follows. Only one app can own the screen at a time, so in practice
 * this is one consumer's answer with the others returning false. */
static bool deliver_app_touch(const touch_frame_t *f)
{
    bool used = false;

    if (s_app_touch && s_app_touch(s_app_touch_user, f))
        used = true;

    for (int i = 0; i < APP_TOUCH_MAX; i++)
        if (s_app_touch_extra[i] && s_app_touch_extra[i](s_app_touch_extra_user[i], f))
            used = true;

    return used;
}

/* --- the raw-block dump -------------------------------------------------- */

#define TOUCH_DUMP_BLOCKS 2048

int touch_dump(int seconds)
{
    if (seconds <= 0)
        seconds = 5;

    if (!s_dump_buf)
    {
        s_dump_buf = heap_caps_malloc((size_t)TOUCH_DUMP_BLOCKS * REPORT_LEN,
                                      MALLOC_CAP_SPIRAM);

        if (!s_dump_buf)
            return -1;
    }

    s_dump_cap = TOUCH_DUMP_BLOCKS;
    s_dump_n = 0;
    s_dumping = true;

    vTaskDelay(pdMS_TO_TICKS(seconds * 1000));

    s_dumping = false;

    /* Let a read already in flight finish: the poll task may be between the
     * memcpy and its own bookkeeping, and reading the count too early would
     * report one block fewer than the file has. */
    vTaskDelay(pdMS_TO_TICKS(ACTIVE_MS * 2));

    int n = s_dump_n;
    int kept = 0;

    FILE *f = fopen("/sd/touch.txt", "w");

    if (!f)
        return -1;

    /* An idle read returns fifteen 0xFF bytes -- MEASURED, and worth stating
     * because it is not what the driver's own comments assume: the "nothing
     * there" case was documented as 0x00 or 0xAB, and it is neither. Idle
     * blocks are excluded at capture time (see the poll loop), so everything
     * here is a real report.
     *
     * What is written is the raw fifteen bytes, contiguous hex so a line fits
     * the Log Viewer's width, in the order the controller sent them -- no
     * decoding, because the decoding is the thing being measured. */
    for (int i = 0; i < n; i++)
    {
        const uint8_t *b = s_dump_buf[i];

        fprintf(f, "%03d ", kept++);
        for (int k = 0; k < REPORT_LEN; k++)
            fprintf(f, "%02x", b[k]);
        fprintf(f, " n%d%s\n", b[5] & 0x7F, b[6] == ACK_BYTE ? " ok" : "");
    }

    fclose(f);
    return kept;
}

static int cmd_touchdump(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    int secs = argc >= 2 ? atoi(argv[1]) : 5;

    if (secs <= 0 || secs > 30)
    {
        printf("seconds 1..30\n");
        return 1;
    }

    printf("capturing %d s -- touch the glass now\n", secs);

    int n = touch_dump(secs);

    if (n < 0)
    {
        printf("no card or no memory\n");
        return 1;
    }

    printf("%d blocks -> /sd/touch.txt\n", n);
    return 0;
}

static const tdsh_command_t s_touchdump_cmd = {
    "touchdump",
    "touchdump [seconds]",
    "Capture raw touch report blocks to /sd/touch.txt",
    cmd_touchdump,
    TDSH_CMD_INTERACTIVE,
};

bool touch_register_command(void)
{
    return tdsh_register_command(&s_touchdump_cmd) == 0;
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

    /* The interrupt is configured before the task exists, so an edge during
     * init has somewhere harmless to land: the handler sees a NULL task handle
     * and does nothing, and the first idle poll catches that press. The task
     * handle is what the handler notifies, so it has to be stored rather than
     * discarded the way the old xTaskCreate call did. */
    s_int_pin = tdsh_board_int("touch.int", TOUCH_INT);
    if (s_int_pin >= 0)
    {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << s_int_pin,
            .mode = GPIO_MODE_INPUT,
            /* Part of the contract rather than a nicety: the controller only
             * pulls this line down and releases it, so without a pull-up the
             * level floats and the edge either never arrives or arrives
             * continuously (TCH-009). LilyGo configures the same pin the same
             * way in its own initTouch(). */
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE,
        };

        esp_err_t err = gpio_config(&io);
        if (err == ESP_OK && !board_gpio_isr_init())
            err = ESP_FAIL;
        if (err == ESP_OK)
            err = gpio_isr_handler_add((gpio_num_t)s_int_pin, touch_isr, NULL);

        if (err != ESP_OK)
        {
            /* Not fatal. Touch still works, at the poll rate it always used. */
            ESP_LOGW(TAG, "no irq, poll only");
            s_int_pin = -1;
        }
    }

    if (xTaskCreatePinnedToCore(poll_task, "tdtouch", 3072, NULL, 3,
                                &s_task, 0) != pdPASS)
    {
        s_int_pin = -1;
        return false;
    }

    /* Short on purpose, and the MODE COMES FIRST: the Log Viewer is 40
     * columns and silently truncates, so "CST9217 up at 0x1A irq 12" loses
     * exactly the words that say whether the interrupt path or the fallback is
     * running. That is the trap this file already records twice -- a diagnostic
     * that gets clipped is not a diagnostic. "irq 12 up" survives. */
    if (s_int_pin >= 0)
        ESP_LOGI(TAG, "irq %d up", s_int_pin);
    else
        ESP_LOGI(TAG, "poll up");
    return true;
}
