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
 * The desktop's size is the panel's, but not by filtering the host: the host
 * terminal's answer is allowed through and then clamped, because the grid is
 * declared at build time (TD_MAX_COLS/TD_MAX_ROWS in the port's CMakeLists,
 * 40x26) and src/td.c clamps whatever any terminal reports into it. So a
 * `screen` session that answers 74x34 shows the same 40x26 desktop as the
 * glass -- the mirror is a second view of one screen, not a second screen.
 * Filtering the host's answer here was tried and removed: it is the wrong
 * layer, it hides the size the terminal really has, and the clamp upstream
 * already does the job.
 *
 * Board configuration keys (see board.example.conf):
 *   lcd.mirror    1 (default): also send the picture to the USB port
 */
#include "link.h"

#include <stdio.h>
#include <string.h>

#include "display.h"
#include "power.h"
#include "rtc.h"
#include "touch.h"
#include "pusher.h"
#include "pwrkey.h"
#include "charge.h"
#include "haptic.h"
#include "sound.h"
#include "hal_mux.h"
#include "tinydesk/td_picture.h"
#include "tinydesk/td_wm.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tdsh_board.h"

#define RX_BUFFER        8192
#define TX_BUFFER        1024
#define WRITE_CHUNK      256
#define MIRROR_BUDGET_MS 15   /* most a frame may spend on the USB mirror */
#define MIRROR_RESYNC_MS 500  /* how often a stale mirror may ask for a frame */
#define INJECT_SIZE      512          /* the panel's answers and touch reports */

static const char *TAG = "link_twatch";
static bool s_mirror = true;
static uint32_t s_mirror_resync_ms;
static bool s_mirror_stale;
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

    /*
     * SEED THE CLOCK FROM THE RTC BEFORE ANYTHING ELSE NEEDS IT.
     *
     * The desktop shows "--:--" and "no clock" until something sets the system
     * clock, and upstream's only source is SNTP, which needs the network. This
     * board has a battery-backed PCF85063A holding the time whether or not
     * Wi-Fi associates, so it is read here, before display_init and before
     * TinyDesk starts. SNTP still overrides it when it arrives, through
     * board_time_set_post().
     */
    if (!rtc_seed_system_clock())
        ESP_LOGW(TAG, "no time from the RTC; the clock will wait for the network");

    s_mirror = tdsh_board_int("lcd.mirror", 1) != 0;

    /*
     * Power the panel BEFORE touching its bus. The AMOLED's rail is switched
     * by XL9555 P07, and on a cold boot the expander has every pin an input,
     * so the panel is unpowered until this runs. Doing it after display_init()
     * would send a full CO5300 init sequence into a dark, unpowered panel and
     * then draw a screen nobody can see.
     */
    if (!board_display_power_on())
        ESP_LOGW(TAG, "panel power did not come on; the screen will be dark");

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

    /* The controller is held in reset by an expander pin until something
     * drives it, so release it before the driver tries to talk. */
    board_touch_reset_release();
    s_touch = touch_init(inject);

    /* The speaker, and the terminal's BEL routed to it. A board with no
     * amplifier answers nothing and the desktop is unaffected, so a failure
     * here is logged and not fatal -- the same shape as the haptics below. */
    if (sound_init())
    {
        /* Also keep the renderer's copy wired: it mirrors the desktop's
         * output, so a BEL that reaches the mirror rather than an app still
         * rings. The path that matters is board_bell(), which the Terminal's
         * own emulator calls through td_sysinfo_t. */
        display_set_bell(sound_bell, NULL);
        ESP_LOGI(TAG, "sound on");
    }
    else
    {
        ESP_LOGE(TAG, "no speaker");
    }

    /* Haptics last, and after its own rail switch. P06 gates the DRV2605's
     * supply and the chip answers I2C without it, so probing before raising
     * the rail reports a healthy part that never moves -- the failure BRD-005
     * records. A missing motor is not fatal: the desktop works without it and
     * says so in the log. */
    /* Before the SD watcher's first mount attempt (it starts from a task, a
     * few hundred ms later), and late enough to stay in the log ring. */
    board_log_rails();

    if (!board_haptic_power_on())
    {
        ESP_LOGE(TAG, "haptic rail failed");
    }
    else if (!haptic_init())
    {
        ESP_LOGE(TAG, "no DRV2605 at 0x5A");
    }
    else
    {
        ESP_LOGI(TAG, "haptics on");
    }

    /* The BOOT pusher, last because it is the only input that can inject a key
     * without the desktop asking. It is not fatal when it fails, and
     * `pusher.pin = -1` turns it off deliberately. */
    pusher_init(inject);

    /* The charge transitions first, then the pin. Order matters: enabling the
     * charger's own interrupts can make a status bit pending, and pwrkey_init()
     * is what clears the status and refuses to arm if the line is still low.
     * Arming first would leave that guard looking at a line the next line of
     * code was about to pull down. */
    charge_watch_enable();

    /* Pusher A, the power key, which is not a GPIO: the AXP2101 reports it on
     * the PMU interrupt line. Failure is not fatal and is the expected outcome
     * if the PMU is holding its IRQ line, which the driver refuses to arm on. */
    pwrkey_init(inject);

    /* What the PMU is currently configured to do about charging. Read only:
     * see charge.c for why the first move on a battery is measurement. */
    charge_report(NULL);

    /* The battery's NTC input, if one is fitted. Read and logged only: this
     * never writes register 0x50, because changing the TS pin's mode changes
     * what the charger uses to decide whether a cell is safe to charge. */
    charge_ts_report(NULL, NULL);

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
        return n;                    /* the panel's own answers come first */
    n = usb_serial_jtag_read_bytes(buf, (uint32_t)cap, 0);
    return n > 0 ? n : 0;
}

int link_write(const uint8_t *buf, int len)
{
    int done = 0;

    /*
     * THE PANEL IS AUTHORITATIVE; THE MIRROR IS A CONVENIENCE.
     *
     * display_write() always gets the whole buffer -- the panel's td_vterm is
     * fast and local, and it must never lose a byte of a frame. The USB mirror
     * is a second view for whoever has a terminal open, and it is allowed to
     * fall behind; it must never be allowed to stall or truncate the desktop.
     *
     * This used to do neither. It waited up to 150 ms per 256-byte chunk and
     * then returned len regardless, claiming a delivery it had not made:
     *
     *   - when nothing was draining the port, every frame stalled the desktop
     *     by up to 450 ms while the panel already had the picture, which is
     *     why a Bluetooth mouse was laggy with the watch plugged into a PC that
     *     had no terminal open, and smooth over a `screen` session;
     *   - usb_serial_jtag_is_connected() only says a USB HOST is present, so
     *     the mirror ran whether or not anything was reading.
     *
     * Now the mirror gets a fixed 15 ms budget per frame, invisible at 60 Hz.
     * len is still returned, because returning less would make the renderer
     * drop bytes that the panel needs as well. When the mirror cannot keep up
     * it is marked stale and the desktop is asked for one full frame, throttled
     * so an undrained port cannot turn that into a redraw every frame; the
     * next complete frame brings the mirror back into step.
     */
    display_write(buf, len);

    if (!s_mirror || !usb_serial_jtag_is_connected())
        return len;                 /* nothing to mirror; the panel has it */

    {
        int64_t t0 = esp_timer_get_time();

        while (done < len)
        {
            int n = len - done > WRITE_CHUNK ? WRITE_CHUNK : len - done;
            int left_ms = MIRROR_BUDGET_MS - (int)((esp_timer_get_time() - t0) / 1000);
            int w;

            if (left_ms <= 0)
                break;              /* spent this frame's budget */
            w = usb_serial_jtag_write_bytes(buf + done, (size_t)n,
                                            pdMS_TO_TICKS(left_ms));
            if (w <= 0)
                break;              /* not draining */
            done += w;
        }

        if (done < len)
        {
            /* The mirror missed part of a frame, so what it shows now has
             * holes in it. Do NOT ask for a repaint here: with nothing reading
             * the port every frame would end like this, and each request is a
             * full compose plus a full panel repaint -- work and power spent
             * on a mirror nobody is looking at. Just remember that it is stale.
             */
            s_mirror_stale = true;
        }
        else if (s_mirror_stale)
        {
            /* A frame got all the way through, so something is reading again.
             * One whole frame fills the holes; without it the mirror would
             * stay partly blank until some window happened to repaint that
             * area. Throttled so a reader that keeps up only pays this once. */
            uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

            s_mirror_stale = false;
            if (now - s_mirror_resync_ms >= MIRROR_RESYNC_MS)
            {
                s_mirror_resync_ms = now;
                td_wm_invalidate();
            }
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
