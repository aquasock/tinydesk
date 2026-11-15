/*
 * usbdiag.c - what attaches to the ESP32-S3's USB port when it is a HOST.
 *
 * WHY THIS EXISTS. A USB-C headphone DAC is the only usable audio output on a
 * watch whose speaker cannot be used in public, and the question that decides
 * whether one can work here is electrical, not a driver question: the S3's OTG
 * port is FULL SPEED (12 Mbit/s) only -- ESP-IDF's own host documentation says
 * "Supports Full Speed (FS) and Low Speed (LS) Devices" -- while almost every
 * modern USB-C DAC enumerates at High Speed (480 Mbit/s) and offers no
 * full-speed descriptors. A high-speed-only device cannot enumerate on a
 * full-speed port, and no driver can change that. So the first thing to
 * establish is simply whether a given DAC appears at all, and at what speed.
 *
 * THE LOG IS THE INSTRUMENT, NOT THE SCREEN. This board's console is the
 * USB-Serial-JTAG peripheral, and it shares D+/D- with the OTG controller. The
 * moment USB host installs, the console is gone -- that is the same sharing that
 * makes this diagnostic necessary. So everything is written to the SD card,
 * one line at a time with the file reopened per write, exactly as `vbus` does,
 * and the operator carries the card's answer back by replugging the PC.
 *
 * THE SEQUENCE IS THE OPERATOR'S, and it is the only one that works:
 *
 *   1. run `usbdiag 30` with the watch on the PC
 *   2. unplug from the PC, plug the splitter and the DAC in
 *   3. leave it 30 s, unplug the DAC
 *   4. plug the watch back into the PC
 *   5. read /sd/diag/usbdiag.log
 *
 * A few seconds of delay before the host installs gives the operator time to
 * read the instruction and pull the cable; the console survives that long and
 * dies when the host comes up.
 *
 * WHAT IS RECORDED, and why each field earns its place:
 *
 *   speed      the whole question. USB_SPEED_HIGH on this port would mean the
 *              dongle negotiated something the S3 cannot do, which cannot
 *              happen against a full-speed host -- so what this really reports
 *              is whether enumeration succeeded at all.
 *   vid/pid    to identify the part.
 *   bcdUSB     the device's USB version, from its device descriptor.
 *   bcdADC     the audio class version from the class-specific AC header
 *              descriptor, which is what separates UAC1 (1.00) from UAC2
 *              (2.00) -- and therefore whether Espressif's shipped UAC1 driver
 *              can bind it.
 *   endpoints  the streaming interface's transfer type, so an isochronous
 *              endpoint is confirmed rather than assumed.
 *
 * The descriptor walk is done by hand over the raw configuration bytes rather
 * than with the parse helpers, because those stop at the standard descriptors
 * and the audio class descriptors are what this needs. It is a few dozen lines
 * and it cannot be fooled by a vendor-specific layout the way a library's
 * assumptions can.
 */
#include "usbdiag.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdpaths.h"
#include "tdsh.h"
#include "driver/usb_serial_jtag.h"
#include "usb/usb_host.h"

/* Descriptor type bytes this needs. Named rather than magic numbers because a
 * wrong one here produces a log that looks empty and is not. */
#define DESC_DEVICE        0x01
#define DESC_CONFIG        0x02
#define DESC_INTERFACE     0x04
#define DESC_ENDPOINT      0x05
#define DESC_CS_INTERFACE  0x24    /* class-specific interface (audio AC/AS) */

/* Audio class-specific descriptor subtypes used here. */
#define AC_HEADER          0x01

/* USB device addresses are 1..127 (address 0 is the default/unconfigured
 * state), and ESP-IDF 5.3 exports no constant for the limit, so it is stated
 * here. The list handed to the host is smaller than that on purpose: this
 * diagnostic cares whether ONE device enumerated, and a short list keeps the
 * array off a large stack. */
#define USBDIAG_MAX_ADDR   127
#define DIAG_MAX_DEVICES   8

static const char *speed_name(usb_speed_t s)
{
    switch (s)
    {
    case USB_SPEED_LOW:
        return "LOW(1.5M)";
    case USB_SPEED_FULL:
        return "FULL(12M)";
    case USB_SPEED_HIGH:
        return "HIGH(480M)";
    default:
        return "?";
    }
}

/* Append one line to the log. Opened and closed per write on purpose: the board
 * may be unplugged at any moment, and a buffered FILE* would take the last
 * lines with it -- which are exactly the interesting ones. */
static void logf_line(const char *fmt, ...)
{
    char line[256];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    FILE *f = fopen(sd_out("/sd/diag", "usbdiag.log"), "a");

    if (f)
    {
        fprintf(f, "%s\n", line);
        fclose(f);
    }
}

/*
 * Walk a configuration descriptor and report the audio class version and the
 * streaming endpoints.
 *
 * The walk is a flat scan over the buffer, stepping by bLength each time.
 * Anything with a zero length would loop for ever, so it is treated as
 * corruption and the walk stops -- a malformed descriptor set is a finding
 * worth logging, not something to spin on.
 */
static void walk_config(const uint8_t *cfg, int len)
{
    int i = 0;
    int cur_iface = -1;
    int cur_alt = -1;
    int cur_proto = -1;
    int uac_version = -1;

    while (i + 2 <= len)
    {
        const int blen = cfg[i];
        const int btype = cfg[i + 1];

        if (blen < 2 || i + blen > len)
        {
            logf_line("  descriptor walk stopped: bLength=%d type=%d at %d of %d",
                      blen, btype, i, len);
            return;
        }

        if (btype == DESC_INTERFACE && blen >= 9)
        {
            cur_iface = cfg[i + 2];
            cur_alt = cfg[i + 3];
            cur_proto = cfg[i + 7];          /* bInterfaceProtocol */

            const int cls = cfg[i + 5], sub = cfg[i + 6];

            /* Audio class only: 0x01 is the audio class, subclass 1 is
             * AudioControl and 2 is AudioStreaming. Interface protocol 0x20 is
             * the UAC2 marker and 0x00 the UAC1 one, which is a second,
             * redundant statement of the version recorded below. */
            if (cls == 0x01)
                logf_line("  iface %d alt %d: audio subclass %d protocol 0x%02X%s",
                          cur_iface, cur_alt, sub, cur_proto,
                          cur_proto == 0x20 ? " (UAC2)" : cur_proto == 0 ? " (UAC1)" : "");
        }
        else if (btype == DESC_CS_INTERFACE && blen >= 3)
        {
            /* An AudioControl HEADER carries bcdADC at offset 4..5. An
             * AudioStreaming GENERAL carries bTerminalLink instead, so the
             * subtype is what distinguishes them -- reading bcdADC by offset
             * without checking the subtype would report the terminal link as a
             * version. */
            if (cfg[i + 2] == AC_HEADER && blen >= 7)
            {
                uac_version = (cfg[i + 4] << 8) | cfg[i + 5];
                logf_line("  AC header: bcdADC = 0x%04X%s", uac_version,
                          uac_version == 0x0200 ? " (UAC 2.0)" :
                          uac_version == 0x0100 ? " (UAC 1.0)" : "");
            }
        }
        else if (btype == DESC_ENDPOINT && blen >= 7)
        {
            const int addr = cfg[i + 2];
            const int attr = cfg[i + 3];
            const int xfer = attr & 0x03;
            const int sync = (attr >> 2) & 0x03;
            const int mps = cfg[i + 4] | (cfg[i + 5] << 8);
            const int interval = cfg[i + 6];

            static const char *const XFER[] = {"control", "isochronous", "bulk", "interrupt"};
            static const char *const SYNC[] = {"none", "async", "adaptive", "sync"};

            logf_line("  iface %d alt %d ep 0x%02X: %s %s%s mps=%d interval=%d",
                      cur_iface, cur_alt, addr,
                      XFER[xfer], SYNC[sync],
                      addr & 0x80 ? " IN" : " OUT", mps, interval);
        }

        i += blen;
    }
}

/* True once this device address has already been reported, so the poll loop
 * logs each attachment once rather than every 200 ms. */
static bool s_seen[USBDIAG_MAX_ADDR + 1];
static volatile bool s_diag_running;

/*
 * The client event callback. Required even for a synchronous client (see the
 * comment at registration), and it is where a device's arrival is first
 * visible.
 */
static void on_client_event(const usb_host_client_event_msg_t *msg, void *arg)
{
    (void)arg;

    if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV)
        logf_line("# client event: NEW_DEV address %d", (int)msg->new_dev.address);
}

/* The host library requires a CLIENT to open a device -- usb_host_device_open()
 * takes a client handle, not a bare address. A SYNCHRONOUS client is the right
 * shape here: this command wants to block on its own task and ask questions in
 * order, not react to events on a callback. */
static usb_host_client_handle_t s_client;

static void report_device(uint8_t addr)
{
    usb_device_handle_t dev = NULL;

    if (usb_host_device_open(s_client, addr, &dev) != ESP_OK || !dev)
    {
        logf_line("device %d: open FAILED", addr);
        return;
    }

    usb_device_info_t info;

    if (usb_host_device_info(dev, &info) == ESP_OK)
    {
        /* wData is UTF-16LE, not a C string: printing it with %s reads the low
         * byte of each code unit and then walks off into the high bytes, which
         * produces plausible-looking garbage. Each unit is narrowed here, and
         * anything outside ASCII becomes '.', which is enough to recognise a
         * product name and cannot mislead. */
        char product[40];
        int np = 0;

        if (info.str_desc_product)
        {
            const usb_str_desc_t *p = info.str_desc_product;
            const int units = p->bLength >= 2 ? (p->bLength - 2) / 2 : 0;

            for (int i = 0; i < units && np < (int)sizeof(product) - 1; i++)
            {
                const unsigned ch = p->wData[i];

                product[np++] = (ch >= 0x20 && ch < 0x7F) ? (char)ch : '.';
            }
        }
        product[np] = '\0';

        logf_line("device %d: speed=%s EP0max=%d config=%d product='%s'",
                  addr, speed_name(info.speed), info.bMaxPacketSize0,
                  info.bConfigurationValue, product);
    }
    else
    {
        logf_line("device %d: device_info FAILED", addr);
    }

    const usb_device_desc_t *dd = NULL;

    if (usb_host_get_device_descriptor(dev, &dd) == ESP_OK && dd)
        logf_line("  id %04X:%04X bcdUSB=0x%04X class=%d/%d/%d",
                  dd->idVendor, dd->idProduct, dd->bcdUSB,
                  dd->bDeviceClass, dd->bDeviceSubClass, dd->bDeviceProtocol);
    else
        logf_line("  device descriptor FAILED");

    const usb_config_desc_t *cd = NULL;

    if (usb_host_get_active_config_descriptor(dev, &cd) == ESP_OK && cd)
        walk_config((const uint8_t *)cd, cd->wTotalLength);
    else
        logf_line("  config descriptor FAILED");

    logf_line("  ---- end of device %d ----", addr);

    usb_host_device_close(s_client, dev);
}

/*
 * `usbdiag [seconds]` -- bring up USB host, log whatever attaches, and stop.
 *
 * Returns as soon as the host is installed and the operator has been told what
 * to do; the polling runs on this task, so the shell is blocked for the
 * duration. That is deliberate: the console is about to disappear anyway, and a
 * command that returned while a background task quietly waited would let the
 * operator do the physical steps before the log existed.
 */
/*
 * THE POLLING RUNS ON ITS OWN TASK, NOT THE SHELL'S.
 *
 * It used to run inline, and the shell task crashed -- the coredump named it
 * outright ("Crashed task handle: ... name: 'tdsh'") with a null frame at #0,
 * which is stack corruption. usb_host_lib_handle_events() and
 * usb_host_client_handle_events() dispatch the host stack's own callbacks, and
 * report_device() adds descriptor buffers on top of that; all of it landed on a
 * task that was already holding the shell's parser and its output buffers.
 *
 * A dedicated 8 KB task is the shape this deserves: the shell returns to its
 * prompt, the operator can keep using it, and the polling has room.
 */
static void usbdiag_task(void *arg)
{
    const int secs = (int)(intptr_t)arg;

    /* Long enough for the operator to read the steps and pull the PC cable,
     * during which the USB console still works. */
    vTaskDelay(pdMS_TO_TICKS(12000));

    logf_line("# host install begins");

    /*
     * THE INTERRUPT LEVEL IS WHAT WAS WRONG, and the first run's error code named
     * it: usb_host_install() returned ESP_ERR_NOT_FOUND, which is not a return
     * value anywhere in its own path. It comes from esp_intr_alloc(), which
     * returns exactly that with "No free interrupt inputs for USB interrupt
     * (flags 0x%X)" when no slot matching the requested flags is free.
     *
     * The HCD asks for ESP_INTR_FLAG_LEVEL1. On this board WiFi and BLE are both
     * running by the time this runs, and the level-1 slots are taken. LEVEL3
     * works -- verified on hardware.
     */
    static const struct { int flags; const char *name; } levels[] = {
        { ESP_INTR_FLAG_LEVEL3, "LEVEL3" },
        { ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_LEVEL3, "LEVEL2|LEVEL3" },
        { ESP_INTR_FLAG_LEVEL1, "LEVEL1" },
        { 0, "default(0)" },
    };

    esp_err_t err = ESP_FAIL;

    for (size_t i = 0; i < sizeof(levels) / sizeof(levels[0]); i++)
    {
        const usb_host_config_t host = {
            .skip_phy_setup = false,
            .intr_flags = levels[i].flags,
        };

        err = usb_host_install(&host);

        logf_line("# usb_host_install(%s flags=0x%X) -> %s (0x%X)",
                  levels[i].name, (unsigned)levels[i].flags,
                  esp_err_to_name(err), (unsigned)err);

        if (err == ESP_OK)
            break;
    }

    if (err != ESP_OK)
    {
        logf_line("# host install failed; stopping");
        vTaskDelete(NULL);
        return;
    }

    memset(s_seen, 0, sizeof(s_seen));

    /*
     * A SYNCHRONOUS CLIENT STILL NEEDS AN EVENT CALLBACK, AND ITS ABSENCE WAS A
     * NULL-POINTER JUMP THAT REBOOTED THE BOARD.
     *
     * usb_host_client_register() validates async.client_event_callback ONLY when
     * is_synchronous is false (usb_host.c:667), but
     * usb_host_client_handle_events() invokes it unconditionally whenever an
     * event message is queued (usb_host.c:809). A synchronous client that omits
     * the callback therefore registers fine and then calls address zero the
     * moment a device is attached:
     *
     *   exccause 0x14 InstFetchProhibitedCause
     *   pc       0x0
     *   Crashed task: 'usbdiag'
     *
     * THIS COST FOUR RUNS AND NEARLY CONDEMNED THE USB PORT. A run with nothing
     * attached completed and logged "# done: 0 device(s)"; every run with a
     * device -- a UAC2 DAC and an 8BitDo controller, two completely different
     * devices -- crashed at the same point. Two devices failing identically is
     * not a device problem, it is a harness problem, and that is the shape the
     * evidence had from the second run.
     *
     * is_synchronous only means the client's TRANSFERS block; events still
     * arrive, so a callback is required either way.
     */
    const usb_host_client_config_t client_cfg = {
        .is_synchronous = true,
        .max_num_event_msg = 4,
        .async = { .client_event_callback = on_client_event, .callback_arg = NULL },
    };
    err = usb_host_client_register(&client_cfg, &s_client);

    logf_line("# usb_host_client_register -> %s (0x%X)",
              esp_err_to_name(err), (unsigned)err);

    if (err != ESP_OK)
    {
        usb_host_uninstall();
        vTaskDelete(NULL);
        return;
    }

    const int64_t end = xTaskGetTickCount() + pdMS_TO_TICKS(secs * 1000);

    while (xTaskGetTickCount() < end)
    {
        uint32_t flags = 0;

        usb_host_lib_handle_events(pdMS_TO_TICKS(100), &flags);
        usb_host_client_handle_events(s_client, pdMS_TO_TICKS(100));

        uint8_t list[DIAG_MAX_DEVICES];
        int n = 0;

        if (usb_host_device_addr_list_fill(DIAG_MAX_DEVICES, list, &n) == ESP_OK)
        {
            for (int i = 0; i < n && i < DIAG_MAX_DEVICES; i++)
            {
                const uint8_t a = list[i];

                if (a == 0 || a > USBDIAG_MAX_ADDR || s_seen[a])
                    continue;

                s_seen[a] = true;
                logf_line("# attached (t=%lums)", (unsigned long)esp_log_timestamp());
                report_device(a);
            }
        }
    }

    int total = 0;

    for (int i = 1; i <= USBDIAG_MAX_ADDR; i++)
        if (s_seen[i])
            total++;

    logf_line("# done: %d device(s) enumerated in %d s", total, secs);
    logf_line("# %s", total ? "port works as a host and enumerated something"
                            : "NOTHING enumerated -- FS/HS or CC role is the question");

    usb_host_client_deregister(s_client);
    usb_host_uninstall();

    s_diag_running = false;
    vTaskDelete(NULL);
}

static int cmd_usbdiag(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    int secs = argc >= 2 ? atoi(argv[1]) : 30;

    if (secs < 5)
        secs = 5;
    if (secs > 300)
        secs = 300;

    FILE *f = fopen(sd_out("/sd/diag", "usbdiag.log"), "w");

    if (!f)
    {
        printf("usbdiag: cannot write /sd/diag/usbdiag.log -- is the card in?\n");
        return 1;
    }

    fprintf(f, "# usbdiag: what attaches to the S3 as a USB HOST\n");
    fprintf(f, "# console and USB host share D+/D-, so the console dies when host installs\n");

    /* WHY THE PREVIOUS RUN ENDED, recorded at the START of this one.
     *
     * A run that stops without its "# done" line has reset the board somehow,
     * and neither the log nor the coredump partition can say how: a coredump is
     * only written for an exception, so a watchdog, a brownout and an orderly
     * esp_restart() all leave an empty log and an empty dump. That ambiguity
     * cost a full round trip -- the run after the SPI fix left no "# done" and
     * the only way to tell "it panicked again" from "power blipped" was to go
     * looking for a coredump that was not there.
     *
     * esp_reset_reason() answers it in one word at the top of the next log.
     */
    static const char *const RST[] = {
        "unknown", "power-on", "external-pin", "software", "panic",
        "interrupt-watchdog", "task-watchdog", "watchdog", "deep-sleep",
        "brownout", "sdio", "usb",
    };
    const esp_reset_reason_t rr = esp_reset_reason();

    fprintf(f, "# previous run ended by: %s (code %d)\n",
            (unsigned)rr < sizeof(RST) / sizeof(RST[0]) ? RST[rr] : "?",
            (int)rr);
    fprintf(f, "# free internal: %u KB\n",
            (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024u));
    fclose(f);

    /*
     * THE OPERATOR'S STEPS, PRINTED IN ORDER. The watch cannot be a USB host
     * and a USB device at the same time -- its single port is either plugged
     * into the PC (device, console works) or into the splitter (host, console
     * dead) -- so this test REQUIRES unplugging from the PC. That is not an
     * inconvenience to be designed away; it is what "be a host" means here.
     *
     * The splitter supplies power on one branch and carries the DAC on the
     * other, which is why the board keeps running with the PC unplugged.
     */
    printf("usbdiag: %d seconds, log -> /sd/diag/usbdiag.log\n", secs);
    printf("usbdiag: NOW: 1. unplug this cable from the PC\n");
    printf("usbdiag:       2. plug in the splitter, with power AND the DAC\n");
    printf("usbdiag: wait, then: 3. unplug everything, replug the PC cable\n");
    printf("usbdiag: the console goes away while it is a host -- expected\n");

    /*
     * THE OPERATOR'S STEPS, AND WHY THEY CANNOT BE DESIGNED AWAY.
     *
     * The watch has one USB-C port. It is either a DEVICE (plugged into the PC,
     * console alive, cannot be a host) or a HOST (plugged into the splitter,
     * console dead). There is no third state, so a host test requires unplugging
     * from the PC. The splitter supplies power on one branch and carries the DAC
     * on the other, which is why the board keeps running with the PC unplugged.
     */
    printf("usbdiag: %d seconds, log -> /sd/diag/usbdiag.log\n", secs);
    printf("usbdiag: NOW: 1. unplug this cable from the PC\n");
    printf("usbdiag:       2. plug in the splitter, with power AND the DAC\n");
    printf("usbdiag: wait, then: 3. unplug everything, replug the PC cable\n");
    printf("usbdiag: polling now runs on its own task; this shell stays usable\n");

    if (s_diag_running)
    {
        printf("usbdiag: already running\n");
        return 1;
    }

    s_diag_running = true;

    /*
     * THE OPERATOR GETS A HEAD START, NOT A BLOCKING PAUSE.
     *
     * The task is started immediately so the countdown runs in parallel with the
     * operator pulling the cable. Its first act is to wait 12 seconds, which is
     * long enough to read the four steps and unplug; during that window the USB
     * console still works, so the shell can return its prompt.
     */
    if (xTaskCreate(usbdiag_task, "usbdiag", 8192, (void *)(intptr_t)secs, 3, NULL) != pdPASS)
    {
        s_diag_running = false;
        printf("usbdiag: no task\n");
        return 1;
    }

    printf("usbdiag: task started; unplug from the PC within 12 s\n");
    return 0;
}

static const tdsh_command_t s_usbdiag_cmd = {
    "usbdiag",
    "usbdiag [seconds]",
    "Bring up USB host and log what attaches to /sd/diag/usbdiag.log",
    cmd_usbdiag,
    TDSH_CMD_INTERACTIVE,
};

bool usbdiag_register_command(void)
{
    return tdsh_register_command(&s_usbdiag_cmd) == 0;
}
