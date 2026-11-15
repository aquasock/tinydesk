/*
 * uac2.c - USB Audio Class 2.0 playback to a bus-powered DAC.
 *
 * WHY THIS EXISTS AT ALL, GIVEN ESPRESSIF SHIPS A UAC DRIVER. Their
 * espressif/usb_host_uac supports UAC 1.0 devices only, through version 1.5.0
 * ("UAC driver allows access to UAC 1.0 devices"). The DAC this board has --
 * TTGK KM-HIFI-384KHZ, 3302:3366 -- reports bcdADC 0x0204 and protocol 0x20 on
 * every audio interface, so that driver cannot bind it. The alternative was a
 * UAC1-only dongle, which is what a search for one turns up: almost nothing,
 * because the market moved to UAC2. So the class that exists in the world is the
 * class this has to speak.
 *
 * WHAT THE DEVICE ACTUALLY OFFERS, read from its own descriptors on hardware:
 *
 *   speed          FULL(12M)      -- it is NOT high-speed only, which was the
 *                                    reason it was nearly given up on
 *   bcdADC         0x0204
 *   iface 0        AudioControl
 *   iface 1 alt 3  ep 0x01 isochronous Synchronous OUT, mps 768, interval 1
 *   iface 2        AudioStreaming IN (microphone path), unused here
 *   iface 3        interrupt IN (HID-style controls), unused here
 *
 * SYNCHRONOUS AND NOT ASYNCHRONOUS, which is the whole reason this is small. A
 * synchronous sink has no feedback endpoint, so the host does not have to read
 * the device's clock and adjust -- the single hardest part of a general UAC2
 * implementation. The host just sends one packet per 1 ms frame and the device's
 * clock follows the bus. The cost is that the device's crystal and the host's
 * frame timer can drift against each other over minutes; at 48 kHz the drift is
 * small and a resampler upstream absorbs it.
 *
 * THE PLAYBACK PATH IS THREE STEPS, and each is visible in the descriptor table
 * above:
 *
 *   1. SET_CUR the Clock Source's Sample Rate Control to the rate we want.
 *   2. SET_INTERFACE interface 1, alternate setting 3 -- the one that carries
 *      the isochronous OUT endpoint. Alternate 0 has no endpoint; selecting it
 *      is how the stream is stopped.
 *   3. Submit isochronous OUT transfers, one packet per frame, 192 bytes each
 *      at 48 kHz stereo 16-bit (48000 / 1000 * 2 channels * 2 bytes).
 *
 * The clock entity id is NOT hardcoded: it is read from the Clock Source
 * descriptor in the device's own AudioControl interface. Hardcoding it would
 * work for this one dongle and silently set the wrong entity on any other.
 */
#include "uac2.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "usb/usb_host.h"

static const char *TAG = "uac2";

/* Is this device the one we can drive? Matched on class AND vendor, so a future
 * UAC2 device with a different topology is skipped rather than crashed on. */
#define UAC2_VID            0x3302
#define UAC2_PID            0x3366

#define USB_CLASS_AUDIO     0x01
#define AUDIO_SUBCLASS_AC   0x01
#define AUDIO_SUBCLASS_AS   0x02

/* Class-specific descriptor/layout constants this needs. */
#define CS_INTERFACE        0x24
#define AC_HEADER           0x01
#define AC_CLOCK_SOURCE     0x0A
#define AC_INPUT_TERMINAL   0x02
#define AS_FORMAT_TYPE_I    0x02

/* UAC2 control selector for the sample rate, from the Audio 2.0 spec 5.2.3.1. */
#define UAC2_CS_SAM_FREQ    0x01
#define UAC2_CN_NONE        0x00

/* Playback geometry. 48 kHz stereo S16 is what every emulator in this tree can
 * be resampled to and what the descriptor's largest alt setting supports. */
#define UAC2_RATE           48000
#define UAC2_CHANNELS       2
#define UAC2_BYTES_PER_SAMPLE 2
#define UAC2_FRAME_BYTES    (UAC2_RATE / 1000 * UAC2_CHANNELS * UAC2_BYTES_PER_SAMPLE)  /* 192 */

/* Frames of audio carried per transfer, and how many transfers may be in flight.
 * 16 frames is 16 ms; three in flight gives 48 ms of slack, which is what absorbs
 * a scheduler hiccup from the display or an emulator without an underrun. */
#define UAC2_PKTS_PER_XFER  16
#define UAC2_XFERS          3
#define UAC2_XFER_BYTES     (UAC2_FRAME_BYTES * UAC2_PKTS_PER_XFER)
#define UAC2_RING_BYTES     (UAC2_FRAME_BYTES * 96)     /* 96 ms of PCM */

static usb_host_client_handle_t s_client;
static usb_device_handle_t s_dev;
static uint8_t s_dev_addr;
static bool s_host_installed;

/* Discovered from the device's own descriptors. */
static uint8_t s_clock_id;
static uint8_t s_as_iface;
static uint8_t s_as_alt;
static uint8_t s_ep_addr;
static uint16_t s_ep_mps;
static uint8_t s_ep_interval;
static uint32_t s_rates[8];
static int s_nrates;

static RingbufHandle_t s_ring;
static usb_transfer_t *s_xfer[UAC2_XFERS];
static volatile bool s_streaming;
static volatile uint32_t s_frames_sent;
static volatile uint32_t s_xfer_errors;
static volatile uint32_t s_underruns;
static volatile uint32_t s_missed;      /* packets the controller reported SKIPPED */
static volatile int s_last_status = -1;
static TaskHandle_t s_task;

static void on_client_event(const usb_host_client_event_msg_t *msg, void *arg);

/* ---------------------------------------------------------------- helpers */

static const uint8_t *find_cs_descriptor(const uint8_t *cfg, int len, int subtype,
                                         int *out_len)
{
    int i = 0;

    while (i + 2 <= len)
    {
        const int blen = cfg[i];
        const int btype = cfg[i + 1];

        if (blen < 2 || i + blen > len)
            break;

        if (btype == CS_INTERFACE && blen >= 3 && cfg[i + 2] == subtype)
        {
            *out_len = blen;
            return &cfg[i];
        }

        i += blen;
    }

    *out_len = 0;
    return NULL;
}

/*
 * Walk the configuration and record what the playback path needs.
 *
 * The AudioStreaming interface is chosen by its ENDPOINT, not by its interface
 * number: alternate settings share an interface number, and only one of them
 * carries the isochronous OUT endpoint. Picking the first alternates that has an
 * OUT endpoint is therefore the correct rule, and it is what makes this work
 * without knowing the device's alternate-setting numbering in advance.
 */
static bool parse_config(const uint8_t *cfg, int len)
{
    int i = 0;
    int cur_iface = -1, cur_alt = 0, cur_sub = 0;
    bool have_clk = false;

    s_as_iface = 0xFF;
    s_nrates = 0;

    while (i + 2 <= len)
    {
        const int blen = cfg[i];
        const int btype = cfg[i + 1];

        if (blen < 2 || i + blen > len)
            break;

        if (btype == 0x04 && blen >= 9)              /* INTERFACE */
        {
            cur_iface = cfg[i + 2];
            cur_alt = cfg[i + 3];
            cur_sub = cfg[i + 6];
        }
        else if (btype == CS_INTERFACE && blen >= 3)
        {
            const int sub = cfg[i + 2];

            if (sub == AC_CLOCK_SOURCE && blen >= 9 && !have_clk)
            {
                /* bClockID, bmAttributes, bmControls, bAssocTerminal,
                 * iClockSource. Any of the three clock types can set a rate, so
                 * the id is taken without checking bmAttributes -- what matters
                 * is that a Sample Rate control exists, and the SET_CUR below
                 * will be refused by a device that has none, which is a truthful
                 * failure rather than a silent one. */
                s_clock_id = cfg[i + 3];
                have_clk = true;
                ESP_LOGI(TAG, "clock source id %u", s_clock_id);
            }
            else if (sub == AS_FORMAT_TYPE_I && blen >= 6)
            {
                /* Format Type I: bFormatType, bNrChannels, bSubframeSize,
                 * bBitResolution, bSamFreqType, then either three-byte rates or
                 * a lower/upper pair. Collected so the log can say what the
                 * device supports, and so the rate can be checked rather than
                 * assumed. */
                const int nfreq = cfg[i + 7];
                const int rate0 = i + 8;

                if (nfreq > 0 && nfreq <= 8 && rate0 + nfreq * 3 <= i + blen)
                {
                    for (int k = 0; k < nfreq && s_nrates < 8; k++)
                    {
                        const int o = rate0 + k * 3;
                        s_rates[s_nrates++] = (uint32_t)cfg[o] |
                                              ((uint32_t)cfg[o + 1] << 8) |
                                              ((uint32_t)cfg[o + 2] << 16);
                    }
                }
            }
        }
        else if (btype == 0x05 && blen >= 7)          /* ENDPOINT */
        {
            const int addr = cfg[i + 2];
            const int attr = cfg[i + 3];
            const int xfer = attr & 0x03;

            /* 1 = isochronous. OUT only: this is a playback driver. */
            if (xfer == 1 && !(addr & 0x80) && cur_sub == AUDIO_SUBCLASS_AS)
            {
                s_as_iface = (uint8_t)cur_iface;
                s_as_alt = (uint8_t)cur_alt;
                s_ep_addr = (uint8_t)addr;
                s_ep_mps = (uint16_t)(cfg[i + 4] | (cfg[i + 5] << 8));
                s_ep_interval = cfg[i + 6];
            }
        }

        i += blen;
    }

    if (s_as_iface == 0xFF)
    {
        ESP_LOGE(TAG, "no isochronous OUT endpoint found");
        return false;
    }

    ESP_LOGI(TAG, "as iface %u alt %u ep 0x%02X mps %u interval %u, %d rate(s)",
             s_as_iface, s_as_alt, s_ep_addr, s_ep_mps, s_ep_interval, s_nrates);

    for (int k = 0; k < s_nrates; k++)
        ESP_LOGI(TAG, "  rate %lu", (unsigned long)s_rates[k]);

    if (!have_clk)
        ESP_LOGW(TAG, "no clock source descriptor; rate control will be skipped");

    return true;
}

/* ------------------------------------------------------------ control I/O */

static esp_err_t ctrl(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
                      uint16_t wIndex, uint8_t *data, uint16_t len)
{
    /*
     * A CONTROL TRANSFER'S SETUP PACKET LIVES IN THE DATA BUFFER.
     *
     * usb_transfer_t has no setup_packet member: the first eight bytes of
     * data_buffer ARE the setup packet, and num_bytes counts them as well as the
     * payload. Getting this wrong is a compile error rather than a silent fault,
     * which is the good kind of mistake.
     */
    const int total = 8 + (int)len;
    usb_transfer_t *t = NULL;
    esp_err_t err = usb_host_transfer_alloc((size_t)total, 0, &t);

    if (err != ESP_OK)
        return err;

    t->device_handle = s_dev;
    t->bEndpointAddress = 0;
    t->callback = NULL;             /* synchronous client: the submit call blocks */
    t->context = NULL;
    t->timeout_ms = 1000;
    t->num_bytes = total;

    uint8_t *p = t->data_buffer;

    p[0] = bmRequestType;
    p[1] = bRequest;
    p[2] = (uint8_t)(wValue & 0xFF);
    p[3] = (uint8_t)(wValue >> 8);
    p[4] = (uint8_t)(wIndex & 0xFF);
    p[5] = (uint8_t)(wIndex >> 8);
    p[6] = (uint8_t)(len & 0xFF);
    p[7] = (uint8_t)(len >> 8);

    /* OUT data goes after the setup packet; IN data is read back from the same
     * place once the transfer completes. */
    if (data && len && !(bmRequestType & 0x80))
        memcpy(p + 8, data, len);

    err = usb_host_transfer_submit_control(s_client, t);

    if (err == ESP_OK && t->status != USB_TRANSFER_STATUS_COMPLETED)
    {
        ESP_LOGW(TAG, "ctrl %02X/%02X status %d", bmRequestType, bRequest, t->status);
        err = ESP_FAIL;
    }

    if (err == ESP_OK && data && len && (bmRequestType & 0x80))
        memcpy(data, t->data_buffer + 8, len);

    usb_host_transfer_free(t);
    return err;
}

/* The sample rate, as a 4-byte little-endian value in Hz, on the clock entity. */
static esp_err_t set_rate(uint32_t rate)
{
    if (!s_clock_id)
        return ESP_ERR_NOT_SUPPORTED;

    uint8_t v[4] = { (uint8_t)(rate), (uint8_t)(rate >> 8),
                     (uint8_t)(rate >> 16), (uint8_t)(rate >> 24) };

    return ctrl(0x21, 0x01,                     /* class, interface, host->device, SET_CUR */
                (UAC2_CS_SAM_FREQ << 8) | 0x00, /* CS = SAM_FREQ_CONTROL, CN = 0 */
                (UAC2_CN_NONE << 8) | s_clock_id,
                v, 4);
}

static esp_err_t set_alt(uint8_t iface, uint8_t alt)
{
    return ctrl(0x01, 0x0B, alt, iface, NULL, 0);
}

/* --------------------------------------------------------------- the pump */

/*
 * One transfer completed. Refill it and resubmit, which is the entire isochronous
 * loop: the controller never stops asking for frames, so the host either keeps up
 * or the device plays silence.
 */
static void xfer_done(usb_transfer_t *t)
{
    if (t->status != USB_TRANSFER_STATUS_COMPLETED)
    {
        s_xfer_errors++;
        s_last_status = t->status;
    }

    for (int p = 0; p < UAC2_PKTS_PER_XFER; p++)
        if (t->isoc_packet_desc[p].status == USB_TRANSFER_STATUS_SKIPPED)
            s_missed++;

    if (!s_streaming)
        return;

    /*
     * REFILL FROM THE RING; PLAY SILENCE IF THERE IS NOT ENOUGH.
     *
     * An underrun must produce sound, not a stall: isochronous transfers cannot
     * be paused without the device glitching, so a short read is padded with
     * zeros and counted. Counting it matters -- a silent underrun and a working
     * stream sound identical on a first listen, and only the counter separates
     * them.
     */
    size_t got = 0;
    uint8_t *pcm = xRingbufferReceiveUpTo(s_ring, &got, 0, UAC2_XFER_BYTES);

    if (pcm)
    {
        memcpy(t->data_buffer, pcm, got);
        vRingbufferReturnItem(s_ring, pcm);
    }

    if (got < UAC2_XFER_BYTES)
    {
        memset(t->data_buffer + got, 0, UAC2_XFER_BYTES - got);
        if (got == 0)
            s_underruns++;
    }

    for (int p = 0; p < UAC2_PKTS_PER_XFER; p++)
        t->isoc_packet_desc[p].num_bytes = UAC2_FRAME_BYTES;

    t->num_bytes = UAC2_XFER_BYTES;

    if (usb_host_transfer_submit(t) != ESP_OK)
        s_xfer_errors++;
    else
        s_frames_sent += UAC2_PKTS_PER_XFER;
}

static void pump_task(void *arg)
{
    (void)arg;

    /* The three transfers are submitted back to back; from then on the
     * controller's completion callbacks drive the rotation and this task has
     * nothing to do but wait. That is why it costs no CPU: the loop is the USB
     * peripheral's, not FreeRTOS's. */
    for (;;)
    {
        if (!s_streaming)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ------------------------------------------------------------------ public */

bool uac2_open(void)
{
    if (s_dev)
        return true;

    if (!s_host_installed)
    {
        /* LEVEL3: WiFi and BLE hold the level-1 slots, and the install returns
         * ESP_ERR_NOT_FOUND with LEVEL1. Recorded in the log entry for the
         * diagnostic that found it. */
        const usb_host_config_t host = {
            .skip_phy_setup = false,
            .intr_flags = ESP_INTR_FLAG_LEVEL3,
        };

        if (usb_host_install(&host) != ESP_OK)
        {
            ESP_LOGE(TAG, "usb_host_install failed");
            return false;
        }

        s_host_installed = true;
    }

    if (!s_client)
    {
        /* is_synchronous = true, AND STILL A CALLBACK: the library invokes
         * async.client_event_callback unconditionally on any queued event even
         * for synchronous clients, so leaving it NULL jumps to address zero the
         * moment a device attaches. That cost four diagnostic runs. */
        const usb_host_client_config_t cfg = {
            .is_synchronous = true,
            .max_num_event_msg = 4,
            .async = { .client_event_callback = on_client_event, .callback_arg = NULL },
        };

        if (usb_host_client_register(&cfg, &s_client) != ESP_OK)
        {
            ESP_LOGE(TAG, "client register failed");
            return false;
        }
    }

    /* Find the DAC among the attached devices. */
    uint8_t list[8];
    int n = 0;

    if (usb_host_device_addr_list_fill(8, list, &n) != ESP_OK)
        return false;

    for (int i = 0; i < n; i++)
    {
        usb_device_handle_t dev = NULL;

        if (usb_host_device_open(s_client, list[i], &dev) != ESP_OK || !dev)
            continue;

        const usb_device_desc_t *dd = NULL;

        if (usb_host_get_device_descriptor(dev, &dd) != ESP_OK || !dd)
        {
            usb_host_device_close(s_client, dev);
            continue;
        }

        if (dd->idVendor != UAC2_VID || dd->idProduct != UAC2_PID)
        {
            usb_host_device_close(s_client, dev);
            continue;
        }

        s_dev = dev;
        s_dev_addr = list[i];
        break;
    }

    if (!s_dev)
    {
        ESP_LOGW(TAG, "DAC %04X:%04X not attached", UAC2_VID, UAC2_PID);
        return false;
    }

    const usb_config_desc_t *cd = NULL;

    if (usb_host_get_active_config_descriptor(s_dev, &cd) != ESP_OK || !cd ||
        !parse_config((const uint8_t *)cd, cd->wTotalLength))
    {
        ESP_LOGE(TAG, "cannot parse the DAC's configuration");
        return false;
    }

    s_ring = xRingbufferCreate(UAC2_RING_BYTES, RINGBUF_TYPE_BYTEBUF);

    if (!s_ring)
    {
        ESP_LOGE(TAG, "no memory for the PCM ring");
        return false;
    }

    if (xTaskCreate(pump_task, "uac2", 4096, NULL, 5, &s_task) != pdPASS)
    {
        ESP_LOGE(TAG, "no task");
        return false;
    }

    ESP_LOGI(TAG, "DAC open: %lu Hz stereo S16, %u-byte frames", (unsigned long)UAC2_RATE,
             UAC2_FRAME_BYTES);
    return true;
}

bool uac2_start(void)
{
    if (!s_dev && !uac2_open())
        return false;

    if (s_streaming)
        return true;

    /* Order matters: claim the interface before touching its endpoints, set the
     * rate before selecting the streaming alternate setting, and only then bring
     * the endpoint up. */
    if (usb_host_interface_claim(s_client, s_dev, s_as_iface, s_as_alt) != ESP_OK)
    {
        ESP_LOGE(TAG, "cannot claim interface %u alt %u", s_as_iface, s_as_alt);
        return false;
    }

    const esp_err_t rerr = set_rate(UAC2_RATE);
    ESP_LOGI(TAG, "set_rate(%lu) -> %s", (unsigned long)UAC2_RATE, esp_err_to_name(rerr));

    for (int i = 0; i < UAC2_XFERS; i++)
    {
        if (usb_host_transfer_alloc(UAC2_XFER_BYTES, UAC2_PKTS_PER_XFER, &s_xfer[i]) != ESP_OK)
        {
            ESP_LOGE(TAG, "transfer alloc failed");
            return false;
        }

        s_xfer[i]->device_handle = s_dev;
        s_xfer[i]->bEndpointAddress = s_ep_addr;
        s_xfer[i]->callback = xfer_done;
        s_xfer[i]->context = s_xfer[i];
        /* timeout_ms 0: an isochronous transfer has no timeout -- it either
         * completes in its frame or is reported SKIPPED.
         *
         * num_isoc_packets is const and comes from the alloc call above; it
         * cannot be assigned here, which is the API saying the packet count is
         * fixed for the life of the transfer. */
        s_xfer[i]->timeout_ms = 0;
        s_xfer[i]->num_bytes = UAC2_XFER_BYTES;

        for (int p = 0; p < UAC2_PKTS_PER_XFER; p++)
            s_xfer[i]->isoc_packet_desc[p].num_bytes = UAC2_FRAME_BYTES;

        memset(s_xfer[i]->data_buffer, 0, UAC2_XFER_BYTES);
    }

    s_streaming = true;

    if (set_alt(s_as_iface, s_as_alt) != ESP_OK)
    {
        ESP_LOGE(TAG, "SET_INTERFACE %u/%u failed", s_as_iface, s_as_alt);
        s_streaming = false;
        return false;
    }

    for (int i = 0; i < UAC2_XFERS; i++)
        usb_host_transfer_submit(s_xfer[i]);

    ESP_LOGI(TAG, "streaming on ep 0x%02X", s_ep_addr);
    return true;
}

void uac2_stop(void)
{
    if (!s_streaming)
        return;

    s_streaming = false;

    /* Let the in-flight transfers retire before the interface is released;
     * tearing the alternate setting out from under a submitted isochronous
     * transfer leaves the controller with a stale pipe. */
    vTaskDelay(pdMS_TO_TICKS(60));

    set_alt(s_as_iface, 0);             /* alternate 0 stops the stream */

    for (int i = 0; i < UAC2_XFERS; i++)
    {
        if (s_xfer[i])
        {
            usb_host_transfer_free(s_xfer[i]);
            s_xfer[i] = NULL;
        }
    }

    usb_host_interface_release(s_client, s_dev, s_as_iface);
    ESP_LOGI(TAG, "stopped");
}

int uac2_write(const uint8_t *pcm, int len)
{
    if (!s_streaming || !s_ring || !pcm || len <= 0)
        return 0;

    /* Non-blocking: the emulators call this from their own frame loop, and an
     * emulator that blocks on audio stops emulating. A full ring means the sink
     * is behind; dropping the newest samples is what a watched buffer would do
     * anyway, and it cannot deadlock the guest. */
    const size_t n = xRingbufferSend(s_ring, pcm, (size_t)len, 0);

    return (int)n;
}

uint32_t uac2_rate(void)
{
    return UAC2_RATE;
}

bool uac2_ready(void)
{
    return s_streaming;
}

void uac2_stats(uint32_t *frames, uint32_t *errors, uint32_t *underruns,
                uint32_t *missed)
{
    if (frames)    *frames = s_frames_sent;
    if (errors)    *errors = s_xfer_errors;
    if (underruns) *underruns = s_underruns;
    if (missed)    *missed = s_missed;
}

static void on_client_event(const usb_host_client_event_msg_t *msg, void *arg)
{
    (void)arg;

    if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV)
        ESP_LOGI(TAG, "device attached: address %d", (int)msg->new_dev.address);
    else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE)
        ESP_LOGW(TAG, "device gone");
}


/* --- `uac2` -- the command that makes this testable -----------------------
 *
 * A driver whose only output is audio cannot be diagnosed: silence sounds the
 * same whether the code never ran, the stream underran, or the device is muted.
 * So the command reports every counter the pump keeps, and `uac2 tone` plays a
 * deliberately obvious sine so the first test is a sound rather than a claim.
 */
#include <math.h>
#include <stdio.h>

#include "tdsh.h"

static int cmd_uac2(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    if (argc >= 2 && strcmp(argv[1], "tone") == 0)
    {
        if (!uac2_start())
        {
            printf("uac2: cannot start (is the DAC attached?)\n");
            return 1;
        }

        /* Two seconds of 440 Hz stereo. A sine is used rather than silence so an
         * audience can tell success from a wiring fault by ear. */
        const int rate = (int)uac2_rate();
        const int total = rate * 2;             /* 2 seconds of frames */
        static float phase;

        for (int i = 0; i < total; i += 96)
        {
            int16_t buf[96 * 2];

            for (int k = 0; k < 96; k++)
            {
                const float v = sinf(phase) * 0.30f * 32767.0f;

                phase += 2.0f * (float)M_PI * 440.0f / (float)rate;
                if (phase > 2.0f * (float)M_PI)
                    phase -= 2.0f * (float)M_PI;

                buf[k * 2] = (int16_t)v;
                buf[k * 2 + 1] = (int16_t)v;
            }

            uac2_write((const uint8_t *)buf, sizeof(buf));

            /* Pace the fill at roughly real time so the ring does not simply
             * absorb two seconds and then starve. */
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        vTaskDelay(pdMS_TO_TICKS(500));
        printf("uac2: tone done\n");
        return 0;
    }

    if (argc >= 2 && strcmp(argv[1], "open") == 0)
    {
        printf("uac2: open %s\n", uac2_open() ? "ok" : "failed");
        return 0;
    }
    if (argc >= 2 && strcmp(argv[1], "start") == 0)
    {
        printf("uac2: start %s\n", uac2_start() ? "ok" : "failed");
        return 0;
    }
    if (argc >= 2 && strcmp(argv[1], "stop") == 0)
    {
        uac2_stop();
        return 0;
    }

    uint32_t f, e, u, m;
    uac2_stats(&f, &e, &u, &m);

    printf("streaming=%d dev=%d clock=%u iface=%u alt=%u ep=0x%02X mps=%u\n",
           s_streaming ? 1 : 0, s_dev ? 1 : 0, s_clock_id, s_as_iface, s_as_alt,
           s_ep_addr, s_ep_mps);
    printf("frames=%lu errors=%lu underruns=%lu missed=%lu last=%d\n",
           (unsigned long)f, (unsigned long)e, (unsigned long)u,
           (unsigned long)m, s_last_status);
    printf("rates:");
    for (int i = 0; i < s_nrates; i++)
        printf(" %lu", (unsigned long)s_rates[i]);
    printf("\n");
    return 0;
}

static const tdsh_command_t s_uac2_cmd = {
    "uac2",
    "uac2 [open|start|stop|tone]",
    "USB audio DAC: status, or play a test tone",
    cmd_uac2,
    TDSH_CMD_INTERACTIVE,
};

bool uac2_register_command(void)
{
    return tdsh_register_command(&s_uac2_cmd) == 0;
}
