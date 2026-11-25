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
#include "sdpaths.h"
#include "usbdiag.h"
#include "bleuart.h"
#include "music_player.h"
#include "music_ui.h"
#include "modhost.h"
#include "modload.h"
#include "uac2.h"
#include "usbhost.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "display.h"
#include "power.h"
#include "rtc.h"
#include "touch.h"
#include "memdiag.h"
#include "imu.h"
#include "klio.h"
#include "pusher.h"
#include "pwrkey.h"
#include "charge.h"
#include "haptic.h"
#include "gps.h"
#include "nfc.h"
#include "sound.h"
#include "nes.h"
#include "tama.h"
#include "pmbench.h"
#include "gbcbench.h"
#include "gb.h"
#include "slotb.h"
#include "td_apps.h"
#include "statusbar.h"
#include "pm.h"
#include "hal_mux.h"
#include "tinydesk/td_picture.h"
#include "tinydesk/td_wm.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tdsh_board.h"
#include "tdsh.h"

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

/* ------------------------------------------------------------- play command */

/*
 * `play <path> [stop]` -- because a watch with a speaker and no way to hand it a
 * file is not a testable speaker. Short on output: the Log Viewer is 40 columns
 * and the player itself logs the interesting line.
 */
static int cmd_play(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    if (argc >= 2 && strcmp(argv[1], "stop") == 0)
    {
        sound_stop();
        return 0;
    }
    if (argc < 2)
    {
        printf("usage: play <file.wav> | play stop\n");
        return 2;
    }
    if (sound_is_playing())
    {
        printf("already playing\n");
        return 1;
    }
    if (!sound_play_wav(argv[1]))
    {
        printf("cannot play %s\n", argv[1]);
        return 1;
    }
    printf("playing\n");
    return 0;
}

/*
 * `pattern [0|1|2]` -- three generated images for checking the blit.
 *
 * It lives in the shell rather than on a menu row because it is an instrument,
 * not a feature, and because the question it answers ("is the copy right?") has
 * to be answerable before the thing that will really draw is connected.
 */
/* Print the paint counters and put them on the card.
 *
 * The file is not belt-and-braces: once the panel is owned the USB mirror goes
 * stale, because the desktop's diff stream stops describing what is on the
 * glass. A diagnostic that only exists in the terminal is unreadable exactly
 * when it is wanted. */
static void report_paint(int what)
{
    uint32_t painted = 0, suppressed = 0;
    bool owned = false;

    display_paint_stats(&painted, &suppressed, &owned);

    printf("own=%d painted=%lu skip=%lu\n", owned ? 1 : 0,
           (unsigned long)painted, (unsigned long)suppressed);

    FILE *f = fopen(sd_out("/sd/diag", "pattern.txt"), "w");

    if (f)
    {
        fprintf(f, "own=%d painted=%lu skip=%lu what=%d\n", owned ? 1 : 0,
                (unsigned long)painted, (unsigned long)suppressed, what);
        fclose(f);
    }
}

static int cmd_pattern(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    /* `pattern stats` reads the counters without touching the picture or the
     * ownership -- the only way to ask "is the painter still suppressed?" after
     * waiting. */
    if (argc >= 2 && strcmp(argv[1], "stats") == 0)
    {
        report_paint(-1);
        return 0;
    }

    /* `pattern off` hands the screen back. It is the other half of the test:
     * the bars should stay put while the picture is up and the desktop should
     * come back whole afterwards, and both halves have to be demonstrable. */
    if (argc >= 2 && (strcmp(argv[1], "off") == 0 || strcmp(argv[1], "release") == 0))
    {
        display_pixels_claim(false);
        report_paint(-2);
        return 0;
    }

    int what = argc >= 2 ? atoi(argv[1]) : 0;

    /* Claim before drawing, or the painter repaints the cells it still thinks
     * are on the glass and the console bleeds through the picture in strips. */
    display_pixels_claim(true);

    if (!display_test_pattern(what))
    {
        display_pixels_claim(false);
        printf("no framebuffer (%d)\n", what);
        return 1;
    }
    printf("pattern %d drawn\n", what);
    report_paint(what);
    printf("pattern off to release\n");
    return 0;
}

/*
 * `dcheck` -- is the glass what the model says, and is the model what the
 * painter thinks it drew?
 *
 * Two counts, because "leftover pixels" has two owners: fb_bad is this port's
 * display code, shown_bad is whatever failed to restore the cells. It exists
 * because the Start menu leaves strips behind and guessing between those two
 * causes costs more than measuring them.
 */
static int cmd_dcheck(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;
    (void)argc;
    (void)argv;

    int fb_bad = 0, shown_bad = 0, fc = -1, fr = -1;

    display_check_cells(&fb_bad, &shown_bad, &fc, &fr);

    printf("fb_bad=%d shown_bad=%d\n", fb_bad, shown_bad);
    if (fc >= 0)
        printf("first col=%d row=%d\n", fc, fr);

    /* Also to the card: reading a 40-column terminal is fine when the answer is
     * one line, but this runs while reproducing a rendering fault and the
     * screen may be exactly what is wrong. */
    {
        FILE *f = fopen(sd_out("/sd/diag", "dcheck.txt"), "w");

        if (f)
        {
            fprintf(f, "fb_bad=%d shown_bad=%d first=%d,%d\n",
                    fb_bad, shown_bad, fc, fr);
            fclose(f);
        }
    }

    return (fb_bad || shown_bad) ? 1 : 0;
}

/*
 * `repaint` -- push the whole screen again from the cell grid.
 *
 * The discriminator. If the strips disappear, the framebuffer was right and the
 * painter failed to flush it; if they stay, the pixels are arriving wrong and
 * the fault is in the transfer.
 */
static int cmd_repaint(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;
    (void)argc;
    (void)argv;

    if (!display_repaint_all())
    {
        printf("no framebuffer\n");
        return 1;
    }
    printf("full repaint pushed\n");
    return 0;
}

static const tdsh_command_t s_repaint_cmd = {
    "repaint",
    "repaint",
    "Repaint the whole screen from the cell grid (discriminates strip faults)",
    cmd_repaint,
    TDSH_CMD_INTERACTIVE,
};

static const tdsh_command_t s_dcheck_cmd = {
    "dcheck",
    "dcheck",
    "Compare the panel against the cell model (two fault counts)",
    cmd_dcheck,
    TDSH_CMD_INTERACTIVE,
};


/* --- vbus: is the USB port delivering power? -------------------------------
 *
 * THE QUESTION THIS EXISTS TO ANSWER, AND WHY IT COMES FIRST.
 *
 * A bus-powered USB-C headphone DAC takes its 5 V from its host. If this
 * watch's USB-C port is wired as a device only -- which the board notes suggest,
 * since the port appears under "taken by the board" with nothing said about
 * sourcing -- then a DAC plugged into it is dead on arrival, and no amount of
 * USB host stack changes that. VBUS is therefore the first thing to establish,
 * and it needs no USB host driver, no USB PHY handover and no console
 * switching: the PMU already measures it, and reading a register cannot break
 * anything.
 *
 * `vbus 40` samples the AXP2101 every 250 ms for 40 s into /sd/vbus.log. THE
 * FILE IS THE RECORD AND THE SCREEN IS NOT, because the operator unplugs the
 * watch from the PC partway through and the console goes with it. Registers
 * 0x00..0x02 are the PMU's own status words (VBUS presence among them) and
 * 0x34/0x35 are battery millivolts, logged as a self-check: if the battery
 * reads about 4,170 mV then the I2C path is sound and anything the status
 * words say can be believed.
 *
 * Registers 0x15/0x16 are logged raw because this port has never read them and
 * their meaning on the AXP2101 is not asserted here -- if one of them turns out
 * to be VBUS millivolts, the log will show it rising when a source appears.
 */
static int cmd_vbus(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    int secs = argc >= 2 ? atoi(argv[1]) : 40;

    if (secs < 1)
        secs = 1;
    if (secs > 300)
        secs = 300;

    FILE *f = fopen(sd_out("/sd/diag", "vbus.log"), "w");

    if (!f)
    {
        printf("vbus: cannot open /sd/vbus.log\n");
        return 1;
    }
    fprintf(f, "t,st0,st1,st2,vh,vl,bat_mv\n");
    fclose(f);

    printf("vbus: sampling %d s -> /sd/vbus.log\n", secs);
    printf("vbus: unplug from the PC now, then plug the DAC in\n");

    for (int i = 0; i < secs * 4; i++)
    {
        uint8_t s0 = 0, s1 = 0, s2 = 0, vh = 0, vl = 0, bh = 0, bl = 0;

        board_pmu_read(0x00, &s0);
        board_pmu_read(0x01, &s1);
        board_pmu_read(0x02, &s2);
        board_pmu_read(0x15, &vh);
        board_pmu_read(0x16, &vl);
        board_pmu_read(0x34, &bh);
        board_pmu_read(0x35, &bl);

        const unsigned bat = (unsigned)(((bh & 0x3F) << 8) | bl);

        f = fopen(sd_out("/sd/diag", "vbus.log"), "a");
        if (f)
        {
            fprintf(f, "%d,%u,%u,%u,%u,%u,%u\n", i, (unsigned)s0, (unsigned)s1,
                    (unsigned)s2, (unsigned)vh, (unsigned)vl, bat);
            fclose(f);
        }

        if ((i & 3) == 0)
            printf("vbus %3ds st0=%02X st1=%02X st2=%02X bat=%umV\n",
                   i / 4, (unsigned)s0, (unsigned)s1, (unsigned)s2, bat);

        vTaskDelay(pdMS_TO_TICKS(250));
    }

    printf("vbus: done -- replug the PC and read /sd/vbus.log\n");
    return 0;
}

static const tdsh_command_t s_vbus_cmd = {
    "vbus",
    "vbus [seconds]",
    "Sample AXP2101 VBUS/status into /sd/vbus.log",
    cmd_vbus,
    TDSH_CMD_INTERACTIVE,
};

static const tdsh_command_t s_pattern_cmd = {
    "pattern",
    "pattern [0|1|2|off|stats]",
    "Draw a test pattern (0 bars, 1 grid, 2 frame) or off to release",
    cmd_pattern,
    TDSH_CMD_INTERACTIVE,
};

static const tdsh_command_t s_play_cmd = {
    "play",
    "play <file.wav> | play stop",
    "Play a 16-bit PCM WAV through the speaker",
    cmd_play,
    TDSH_CMD_INTERACTIVE,
};

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
     * THE RGB BOOT BANDS, OFF BY DEFAULT AND RE-ENABLABLE FROM board.conf.
     *
     * They are a bring-up instrument -- three solid colours pushed to the panel
     * prove it is lit before anything else paints, which is how a dark panel is
     * told apart from a controller drawing into memory nobody can see. Three
     * display faults in this project reported success while drawing nothing, so
     * the instrument earned its place.
     *
     * THEY ARE VISIBLE THOUGH, and now that both personalities boot cleanly they
     * are noise on both paths: display_init() pushes them and then blacks the
     * whole panel a moment later, which is about a hundred milliseconds of colour
     * at every boot. Slot B turned them off in entry 6; the operator then saw the
     * same flash coming the other way, when Pusher A returns from a game to the
     * desktop. So the default is now OFF for this board, and a panel fault is
     * diagnosed by putting `lcd.boot_pattern = 1` in board.conf rather than by
     * rebuilding the firmware -- the instrument stays reachable, which is the
     * point of keeping it.
     */
    display_boot_pattern(tdsh_board_int("lcd.boot_pattern", 0) != 0);

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

    /* RGB565 pictures ARE supported now -- display_picture_rgb() copies the
     * bitmap into the framebuffer and flushes it, which is what anything with a
     * frame rather than a terminal needs (the Tamagotchi emulator's output is a
     * 300x308 RGB565 buffer).
     *
     * 8-bit and run-length pictures are still not, and stay NULL: the desktop
     * learns from these three what it can draw, so registering a sink that
     * returns false would be worse than saying plainly that there is none. */
    td_picture_set_sink(NULL, NULL);
    td_picture_set_rgb_sink(display_picture_rgb, NULL);
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

    /* The NFC reader, on the bus the SD card shares. It is not fatal when it
     * does not answer and nothing above this line depends on it; the step is
     * a probe, so what it establishes is the chip's identity and that the
     * shared bus is quiet enough to talk on. Last of the peripherals because
     * it is the only one that takes over a chip select on a bus another
     * driver already owns. */
    if (!nfc_probe())
        ESP_LOGW(TAG, "no NFC reader");

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

    /*
     * THE STATUS STRIP, and it has to be started HERE rather than earlier.
     *
     * `display_init()` above runs `panel_fill()`, which blacks the entire panel
     * including rows 0..43; a strip started before it would be wiped a moment
     * later. Nothing after this point repaints those rows -- the margin painters
     * begin at row 44 -- so once these pixels land they stay until the strip
     * itself changes them.
     *
     * Not fatal when it fails: a watch without a clock in the bezel is a watch,
     * and the desktop's own taskbar clock is unaffected.
     */
    if (!statusbar_start())
        ESP_LOGW(TAG, "status strip did not start");
    /* The play command, registered here because link_init() runs after
     * tdsh_espidf_init() and this port has no earlier hook of its own. */
    if (tdsh_register_command(&s_play_cmd) != 0)
        ESP_LOGW(TAG, "play command did not register");

    if (!nfc_register_command())
        ESP_LOGW(TAG, "nfc command did not register");

    if (!gps_register_command())
        ESP_LOGW(TAG, "gps command did not register");

    if (tdsh_register_command(&s_vbus_cmd) != 0)
        ESP_LOGW(TAG, "vbus command did not register");

    if (!uac2_register_command())
        ESP_LOGW(TAG, "uac2 command did not register");

    if (!modload_register_command())
        ESP_LOGW(TAG, "modtest command did not register");

    /* Registers BOTH `music` and its alias `modflac`: one player, and the
     * decode happens out of the modules partition. */
    if (!music_player_register_commands())
        ESP_LOGW(TAG, "music/modflac command did not register");

    /* The Music window is registered from board_apps_register(), NOT here:
     * registration order is start-menu order, and board init runs before the
     * desktop's own apps, so registering it here put Music at the very top of
     * the menu. See the hook in ports/esp_idf/app/main.c. */

    /* Map the `modules` partition HERE, on the boot task, whose stack is in
     * internal RAM. esp_partition_mmap() disables the flash cache, and PSRAM is
     * reached through that cache, so a PSRAM-stack task cannot do this: the
     * coredump that proved it is quoted in modload.h. */
    {
        char why[96] = "";

        if (modload_prepare(why, sizeof(why)))
            ESP_LOGI(TAG, "module prepared: '%.20s' entry %p data %p",
                     modload_header()->name, modload_entry(), modload_data_base());
        else
            ESP_LOGI(TAG, "no module: %s", why);
    }

    /* Take the module API table once at boot. This is also what keeps its
     * published pointer in the image: `--gc-sections` collects a section
     * nothing references, and a blob build needs the symbol's ADDRESS. */
    {
        const modhost_t *host = modhost_table();

        ESP_LOGI(TAG, "module API v%lu ready (%s)",
                 (unsigned long)(host ? host->abi_version : 0),
                 (host && host->malloc && host->memcpy) ? "populated" : "EMPTY");
    }

    /* DIAGNOSTIC BUILD ONLY: run the module experiment once at boot and leave
     * the verdict in flash, so the result does not have to survive the display
     * mirror. Remove with the define in CMakeLists when the question is
     * answered. */
#ifdef MODTEST_AT_BOOT
    modload_selftest_call();
#endif

#ifdef MUSIC_UI_SELFTEST
    /* DIAGNOSTIC BUILD ONLY: exercise the Music window's scan-and-play path at
     * boot and leave the answer in /fs/root/musicboot.txt. */
    music_ui_selftest();
#endif

    /* The USB-C port's saved role (board keys usb.host, usb.dac), applied now:
     * after the board keys are loaded, and before anything has opened a file on
     * the SD card, because the USB audio stream's DMA memory has to be taken
     * first (reference USB-005). In OTG host mode the PC stops seeing the board
     * from this point; `board set usb.host 0` and `reboot` over BLE undo it. */
    usbhost_boot();

    if (!bleuart_register_command())
        ESP_LOGW(TAG, "bleuart command did not register");

    if (!usbdiag_register_command())
        ESP_LOGW(TAG, "usbdiag command did not register");

    if (tdsh_register_command(&s_pattern_cmd) != 0)
        ESP_LOGW(TAG, "pattern command did not register");

    if (tdsh_register_command(&s_dcheck_cmd) != 0)
        ESP_LOGW(TAG, "dcheck command did not register");

    if (tdsh_register_command(&s_repaint_cmd) != 0)
        ESP_LOGW(TAG, "repaint command did not register");

    if (!tama_register_command())
        ESP_LOGW(TAG, "tama command did not register");

    if (!nes_register_command())
        ESP_LOGW(TAG, "nes command did not register");

    if (!pmbench_register_command())
        ESP_LOGW(TAG, "pmbench command did not register");

    if (!gbcbench_register_command())
        ESP_LOGW(TAG, "gbcbench command did not register");

    if (!gb_register_command())
        ESP_LOGW(TAG, "gb command did not register");

    /* The launcher's command. See slotb.h. */
    slotb_register_command();

    if (!pmini_register_command())
        ESP_LOGW(TAG, "pm command did not register");

    if (!touch_register_command())
        ESP_LOGW(TAG, "touchdump command did not register");

    if (!memdiag_register_command())
        ESP_LOGW(TAG, "mem command did not register");

    if (!imu_register_command())
        ESP_LOGW(TAG, "imu command did not register");

    if (!klio_register_command())
        ESP_LOGW(TAG, "klio command did not register");

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

/*
 * The port's own apps, registered by the desktop AFTER its built-in ones.
 *
 * Registration order is start-menu order. Registering from board init put
 * Music above Terminal; this hook runs after td_apps_register_all(), so a
 * board's apps land at the end of the app list, immediately before the
 * separator and the user's own emulator .tdlink entries.
 */
/* ------------------------------------------------- slot B from the start menu */

/*
 * THE THREE SLOT B APPS ARE NATIVE APPS, NOT CARD-SIDE SCRIPTS.
 *
 * See slotb_launch(): a `.tdlink` naming a Python script is opened by running it
 * in a TERMINAL, so the menu item showed a terminal, the echoed command and the
 * "rebooting into ota_1" line before the reboot. It also lived on the card, so a
 * full flash deleted it. These are registered here instead -- in the image, and
 * silent apart from the panel going black.
 *
 * A FAILED LAUNCH SAYS SO. Reaching this menu with slot B empty is possible after
 * an interrupted install, and a menu item that quietly does nothing is worse than
 * one that explains itself.
 */
static void slotb_launch_or_say(const char *key)
{
    if (!slotb_launch(key))
        td_msgbox("Slot B", "Slot B has no image to boot.", "OK", NULL, NULL);
}

static void launch_slotb_gb(void)  { slotb_launch_or_say("gb"); }
static void launch_slotb_nes(void) { slotb_launch_or_say("nes"); }
static void launch_slotb_pm(void)  { slotb_launch_or_say("pm"); }
static void launch_slotb_tama(void) { slotb_launch_or_say("tama"); }

/*
 * THE ICONS ARE A MOTIF, NOT INITIALS: a filled circle and then a shape.
 *
 * The field is a TWO-CELL glyph and the desktop counts CHARACTERS rather than bytes
 * (`td_textn()` walks UTF-8 in the menu's own drawing path), so two multi-byte symbols
 * fit exactly -- which two letters never did anything with except repeat the name
 * underneath them. "GB", "NE" and "PM" were the initials of the entry sitting beside
 * them, so they spent the two cells saying what the text already said.
 *
 *   ●■  Game Boy Color
 *   ●▲  NES
 *   ●♦  Pokemon Mini
 *   ●☼  Tamagotchi V3
 *
 * THE PREFIX IS SHARED AND THE SUFFIX IS NOT, which is the point: the circle says
 * "console" for every one of them, and the shape says which. That is the reason these
 * are not simply four pretty pairs -- a shared motif makes the menu read as one family,
 * and a distinguishing suffix keeps it from reading as one repeated blob.
 *
 * EVERY GLYPH WAS CHECKED AGAINST THE FONT TABLE BEFORE BEING USED HERE. It carries
 * 799 code points; a symbol it lacks renders as nothing at all, which in a menu looks
 * like a formatting fault rather than a missing glyph. U+25A0, U+25B2, U+2666 and
 * U+263C are all present, and U+25C6 -- the obvious "diamond" answer -- is NOT, which
 * is why Pokemon Mini gets U+2666. The same check is why no heart was used for the
 * Tamagotchi: U+2665 and U+2661 are both absent from the table.
 *
 * TWO FILLED SHAPES SIDE BY SIDE CAN BLOB AT THIS SIZE, so the prefix alternates in
 * kind rather than in weight: a circle then a shape with corners reads as two marks,
 * where two solid squares would read as one smudge.
 *
 * THE TAMAGOTCHI TOOK THE ONLY GLYPH THAT IS NOT GEOMETRY, and it had to be checked
 * rather than chosen: the font has no heart, no star and no hollow circle, so the
 * obvious answers for a virtual pet were all absent and would have rendered as
 * exactly nothing. What it does carry is U+263C, and a sun beside a filled circle
 * is distinct at 8x16 from every suffix already in use (U+25A0, U+25B2, U+2666),
 * which is the whole requirement. The four suffixes are now square, triangle,
 * lozenge and rays -- and no two can be confused at this size.
 *
 * THE NAMES ARE SHORT NOW, which the icons made possible: the icon carries the
 * identity, so the text does not have to spell out a console name in a 21-column menu.
 * "Game Boy Color" was thirteen characters doing the same job as two glyphs.
 *
 * "Pok\xc3\xa9Mini" CARRIES THE ACUTE ACCENT because that is how the product is spelled,
 * and it is written as a UTF-8 escape rather than a literal byte so the file's encoding
 * cannot change what the menu says. U+00E9 is in the port's font -- the table carries
 * the whole Latin-1 Supplement range -- so it renders rather than falling through to a
 * box; a name is not a place to discover a missing glyph.
 *
 * THE "(B)" SUFFIX IS GONE, and the reason it existed is gone with it. It was there
 * because the desktop had its own Pokemon Mini entry -- a `.tdlink` to `/root/Apps/pm.py`
 * -- so two rows read the same name and one needed marking. The operator has removed
 * that entry (it started the emulator INSIDE the desktop, which slot B's picker now
 * does better), so there is one Pokemon Mini in the menu and the suffix had nothing
 * left to disambiguate.
 *
 * WORTH KNOWING IF IT EVER COMES BACK: two rows with the same name and no suffix are
 * indistinguishable at a glance. The suffix was doing real work, not decoration.
 */
static const td_app_t s_app_gb  = { "GBC",            launch_slotb_gb,  "\xe2\x97\x8f\xe2\x96\xa0" };  /* U+25CF U+25A0 */
static const td_app_t s_app_nes = { "NES",            launch_slotb_nes, "\xe2\x97\x8f\xe2\x96\xb2" };  /* U+25CF U+25B2 */
static const td_app_t s_app_pm  = { "Pok\xc3\xa9Mini",    launch_slotb_pm,  "\xe2\x97\x8f\xe2\x99\xa6" };  /* U+25CF U+2666 */
static const td_app_t s_app_tama = { "Tama V3",        launch_slotb_tama, "\xe2\x97\x8f\xe2\x98\xbc" };  /* U+25CF U+263C */

void board_apps_register(void)
{
    music_ui_register();

    /* Registration order is start-menu order, and this hook runs after the
     * built-in apps, so these sit at the end beside Music and the user's own
     * entries -- which is where the card-side shortcuts used to be. */
    td_app_register(&s_app_gb);
    td_app_register(&s_app_nes);
    td_app_register(&s_app_pm);
    td_app_register(&s_app_tama);
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
