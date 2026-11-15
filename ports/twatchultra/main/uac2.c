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
 *   iface 1        ep 0x01 isochronous Synchronous OUT, interval 1, three alts:
 *                  alt 1  2 ch S16  mps 384   <- the one this driver uses
 *                  alt 2  2 ch S24  mps 576
 *                  alt 3  2 ch S32  mps 768
 *   clock 9        reached via input terminal 1; RANGE reports 8000 16000
 *                  32000 44100 48000 96000 Hz
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
 *   2. SET_INTERFACE interface 1 to the 2-channel 16-bit alternate (alt 1 on
 *      this DAC), chosen from the format descriptors rather than assumed.
 *      Alternate 0 has no endpoint; selecting it is how the stream is stopped.
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
#include "freertos/semphr.h"
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
#define AC_INPUT_TERMINAL   0x02    /* inside the AudioControl interface */
#define AS_GENERAL          0x01    /* inside an AudioStreaming interface */
#define AS_FORMAT_TYPE_I    0x02    /* inside an AudioStreaming interface */

/* UAC2 control selector for the sample rate, from the Audio 2.0 spec 5.2.3.1. */
#define UAC2_CS_SAM_FREQ    0x01
#define UAC2_CN_NONE        0x00

/* UAC2 class requests (Audio 2.0 spec A.14). */
#define UAC2_REQ_CUR        0x01
#define UAC2_REQ_RANGE      0x02

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
static uint8_t s_ac_iface;      /* AudioControl interface: addresses its entities */
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
static TaskHandle_t s_task;             /* the client's event loop */
static TaskHandle_t s_lib_task;         /* the host library's event loop */
static SemaphoreHandle_t s_ctrl_done;   /* a control transfer has completed */

/* How long uac2_open() waits for the DAC to attach and enumerate after the host
 * comes up. Enumeration of this dongle measured well under a second. */
#define UAC2_ATTACH_WAIT_MS 2000

static void on_client_event(const usb_host_client_event_msg_t *msg, void *arg);

/* ---------------------------------------------------------------- helpers */

/*
 * Walk the configuration and record what the playback path needs.
 *
 * CLASS-SPECIFIC SUBTYPES MEAN DIFFERENT THINGS IN DIFFERENT INTERFACES, which
 * is what the first version of this got wrong: subtype 0x02 is an Input Terminal
 * inside the AudioControl interface and a Format Type descriptor inside an
 * AudioStreaming one. So every CS_INTERFACE descriptor is read in the context of
 * the interface descriptor that precedes it.
 *
 * THE UAC2 LAYOUTS (Audio 2.0 spec, 4.7.2.1 / 4.7.2.4 / 4.9.2 and Frmts 2.0
 * 2.3.1.6), all of which the first version read with UAC1 offsets:
 *
 *   Clock Source     8 bytes:  bClockID at +3
 *   Input Terminal  17 bytes:  bTerminalID at +3, bCSourceID at +7
 *   AS General      16 bytes:  bTerminalLink at +3, bNrChannels at +10
 *   Format Type I    6 bytes:  bSubslotSize at +4, bBitResolution at +5
 *
 * UAC2 format descriptors carry NO sample rates: a rate list comes from a RANGE
 * request on the clock entity, which uac2_open() makes once the device is open.
 *
 * Every playback alternate is recorded, and the one chosen is the 2-channel,
 * 2-byte-subslot one, because that is what this driver sends. A device with no
 * such alternate is refused with its formats logged, rather than fed 16-bit
 * frames it would play as noise.
 */
#define UAC2_MAX_ALTS   8
#define UAC2_MAX_TERMS  8

typedef struct
{
    uint8_t alt, ep, interval, term, channels, subslot, bits;
    uint16_t mps;
} as_alt_t;

static bool parse_config(const uint8_t *cfg, int len)
{
    as_alt_t alts[UAC2_MAX_ALTS];
    int nalts = 0;
    uint8_t term_id[UAC2_MAX_TERMS], term_clk[UAC2_MAX_TERMS];
    int nterms = 0;
    uint8_t first_clock = 0;

    int cur_iface = -1, cur_alt = 0, cur_class = 0, cur_sub = 0;
    as_alt_t cur = { 0 };
    int as_iface = -1;

    s_as_iface = 0xFF;
    s_ac_iface = 0;
    s_clock_id = 0;
    s_nrates = 0;

    for (int i = 0; i + 2 <= len; )
    {
        const int blen = cfg[i];
        const int btype = cfg[i + 1];

        if (blen < 2 || i + blen > len)
            break;

        if (btype == 0x04 && blen >= 9)                 /* INTERFACE */
        {
            cur_iface = cfg[i + 2];
            cur_alt = cfg[i + 3];
            cur_class = cfg[i + 5];
            cur_sub = cfg[i + 6];
            memset(&cur, 0, sizeof(cur));
            cur.alt = (uint8_t)cur_alt;

            if (cur_class == USB_CLASS_AUDIO && cur_sub == AUDIO_SUBCLASS_AC)
                s_ac_iface = (uint8_t)cur_iface;
        }
        else if (btype == CS_INTERFACE && blen >= 3 && cur_class == USB_CLASS_AUDIO)
        {
            const int sub = cfg[i + 2];

            if (cur_sub == AUDIO_SUBCLASS_AC)
            {
                if (sub == AC_CLOCK_SOURCE && blen >= 8 && !first_clock)
                    first_clock = cfg[i + 3];
                else if (sub == AC_INPUT_TERMINAL && blen >= 17 && nterms < UAC2_MAX_TERMS)
                {
                    term_id[nterms] = cfg[i + 3];
                    term_clk[nterms] = cfg[i + 7];
                    nterms++;
                }
            }
            else if (cur_sub == AUDIO_SUBCLASS_AS)
            {
                if (sub == AS_GENERAL && blen >= 16)
                {
                    cur.term = cfg[i + 3];
                    cur.channels = cfg[i + 10];
                }
                else if (sub == AS_FORMAT_TYPE_I && blen >= 6)
                {
                    cur.subslot = cfg[i + 4];
                    cur.bits = cfg[i + 5];
                }
            }
        }
        else if (btype == 0x05 && blen >= 7)            /* ENDPOINT */
        {
            const int addr = cfg[i + 2];
            const int xfer = cfg[i + 3] & 0x03;

            /* 1 = isochronous. OUT only: this is a playback driver. */
            if (xfer == 1 && !(addr & 0x80) && cur_class == USB_CLASS_AUDIO &&
                cur_sub == AUDIO_SUBCLASS_AS && nalts < UAC2_MAX_ALTS)
            {
                cur.ep = (uint8_t)addr;
                cur.mps = (uint16_t)(cfg[i + 4] | (cfg[i + 5] << 8));
                cur.interval = cfg[i + 6];
                alts[nalts++] = cur;
                as_iface = cur_iface;
            }
        }

        i += blen;
    }

    if (nalts == 0)
    {
        ESP_LOGE(TAG, "no isochronous OUT endpoint found");
        return false;
    }

    int pick = -1;

    for (int k = 0; k < nalts; k++)
    {
        ESP_LOGI(TAG, "iface %d alt %u: %u ch, %u-byte slots, %u bits, ep 0x%02X mps %u",
                 as_iface, alts[k].alt, alts[k].channels, alts[k].subslot, alts[k].bits,
                 alts[k].ep, alts[k].mps);

        if (pick < 0 && alts[k].channels == UAC2_CHANNELS &&
            alts[k].subslot == UAC2_BYTES_PER_SAMPLE && alts[k].mps >= UAC2_FRAME_BYTES)
            pick = k;
    }

    if (pick < 0)
    {
        ESP_LOGE(TAG, "no %d-channel %d-byte alternate: this driver sends only that",
                 UAC2_CHANNELS, UAC2_BYTES_PER_SAMPLE);
        return false;
    }

    s_as_iface = (uint8_t)as_iface;
    s_as_alt = alts[pick].alt;
    s_ep_addr = alts[pick].ep;
    s_ep_mps = alts[pick].mps;
    s_ep_interval = alts[pick].interval;

    /* The clock that drives THIS stream: the alternate links to an input
     * terminal, and the terminal names its clock. This DAC has two clock
     * sources, so taking the first one found would be a coin toss. */
    for (int t = 0; t < nterms; t++)
        if (term_id[t] == alts[pick].term)
            s_clock_id = term_clk[t];

    if (!s_clock_id)
        s_clock_id = first_clock;

    ESP_LOGI(TAG, "chose alt %u (ep 0x%02X mps %u), terminal %u, clock %u, AC iface %u",
             s_as_alt, s_ep_addr, s_ep_mps, alts[pick].term, s_clock_id, s_ac_iface);

    if (!s_clock_id)
        ESP_LOGW(TAG, "no clock source descriptor; rate control will be skipped");

    return true;
}

/* ------------------------------------------------------------ control I/O */

/* Completion of a control transfer, run from the client task's
 * usb_host_client_handle_events(). */
static void ctrl_cb(usb_transfer_t *t)
{
    (void)t;
    xSemaphoreGive(s_ctrl_done);
}

/*
 * SUBMIT, THEN WAIT FOR THE COMPLETION CALLBACK.
 *
 * usb_host_transfer_submit_control() does not block (IDF 5.3.1 usb_host.c:1446):
 * it queues the URB and returns, and the transfer's callback runs later from the
 * client's usb_host_client_handle_events() -- which calls it unconditionally
 * (usb_host.c:798), so a NULL callback is a jump to address zero. The first
 * version of this assumed a "synchronous client" made submit block; IDF's own
 * header says is_synchronous must be false. So the callback signals a semaphore
 * and this waits on it. Never call this from the client task itself: that task is
 * the one that delivers the completion.
 */
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
    t->callback = ctrl_cb;
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

    xSemaphoreTake(s_ctrl_done, 0);         /* clear any stale completion */

    err = usb_host_transfer_submit_control(s_client, t);

    if (err == ESP_OK && xSemaphoreTake(s_ctrl_done, pdMS_TO_TICKS(1500)) != pdTRUE)
    {
        /* Still in flight: freeing it now would hand the stack a dangling URB,
         * so it is leaked instead. A control transfer that never completes means
         * the device or the client loop is gone, and is reported as such. */
        ESP_LOGE(TAG, "ctrl %02X/%02X: no completion", bmRequestType, bRequest);
        return ESP_ERR_TIMEOUT;
    }

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

/*
 * ENTITY REQUESTS ADDRESS wIndex = (entity id << 8) | AudioControl interface
 * (Audio 2.0 spec 5.2.2). The first version had the two bytes swapped -- entity
 * in the low byte, zero in the high -- so it addressed interface <clock id> and
 * entity 0, which is not the clock.
 */
#define UAC2_CLOCK_WINDEX   ((uint16_t)((s_clock_id << 8) | s_ac_iface))
#define UAC2_SAM_FREQ_WVAL  ((uint16_t)((UAC2_CS_SAM_FREQ << 8) | UAC2_CN_NONE))

/* The sample rate, as a 4-byte little-endian value in Hz, on the clock entity. */
static esp_err_t set_rate(uint32_t rate)
{
    if (!s_clock_id)
        return ESP_ERR_NOT_SUPPORTED;

    uint8_t v[4] = { (uint8_t)(rate), (uint8_t)(rate >> 8),
                     (uint8_t)(rate >> 16), (uint8_t)(rate >> 24) };

    return ctrl(0x21, UAC2_REQ_CUR,             /* class, interface, host->device */
                UAC2_SAM_FREQ_WVAL, UAC2_CLOCK_WINDEX, v, 4);
}

/*
 * The rates the clock supports, from a RANGE request: wNumSubRanges (2 bytes)
 * then 12 bytes per subrange, dMIN / dMAX / dRES, each 4 bytes little-endian.
 * A discrete-rate clock reports each rate as a subrange with dMIN == dMAX, which
 * is what s_rates records; a continuous subrange records its two ends.
 */
static esp_err_t get_rates(void)
{
    uint8_t r[2 + 12 * 8];

    s_nrates = 0;
    if (!s_clock_id)
        return ESP_ERR_NOT_SUPPORTED;

    memset(r, 0, sizeof(r));

    const esp_err_t err = ctrl(0xA1, UAC2_REQ_RANGE,    /* class, interface, device->host */
                               UAC2_SAM_FREQ_WVAL, UAC2_CLOCK_WINDEX, r, sizeof(r));
    if (err != ESP_OK)
        return err;

    int n = r[0] | (r[1] << 8);

    if (n > 8)
        n = 8;

    for (int k = 0; k < n && s_nrates < 8; k++)
    {
        const uint8_t *s = &r[2 + 12 * k];
        const uint32_t lo = s[0] | (s[1] << 8) | (s[2] << 16) | ((uint32_t)s[3] << 24);
        const uint32_t hi = s[4] | (s[5] << 8) | (s[6] << 16) | ((uint32_t)s[7] << 24);

        s_rates[s_nrates++] = lo;
        if (hi != lo && s_nrates < 8)
            s_rates[s_nrates++] = hi;
    }

    return ESP_OK;
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

/*
 * THE TWO EVENT LOOPS IDF REQUIRES, AND WHY NEITHER IS OPTIONAL.
 *
 * The host LIBRARY processes attach, enumeration and detach only inside
 * usb_host_lib_handle_events(); with nobody calling it a device is never
 * enumerated, and usb_host_device_addr_list_fill() stays empty. That is exactly
 * what the first version showed on hardware: `uac2` reported dev=0 with the DAC
 * plugged in, while `usbdiag` -- which does run this loop -- enumerated it.
 *
 * The CLIENT receives transfer completions only inside
 * usb_host_client_handle_events() (usb_host.h: "the transfer's callback will be
 * called from the client's usb_host_client_handle_events()"). So xfer_done(),
 * which refills and resubmits every isochronous transfer, and ctrl_cb() both
 * run on this task. The first version had an idle task here and assumed the
 * controller drove the rotation by itself; it does not.
 */
static void lib_task(void *arg)
{
    (void)arg;

    for (;;)
    {
        uint32_t flags = 0;

        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
            usb_host_device_free_all();
    }
}

static void client_task(void *arg)
{
    (void)arg;

    for (;;)
        usb_host_client_handle_events(s_client, portMAX_DELAY);
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

    if (!s_lib_task &&
        xTaskCreate(lib_task, "uac2_lib", 4096, NULL, 4, &s_lib_task) != pdPASS)
    {
        ESP_LOGE(TAG, "no host library task");
        return false;
    }

    if (!s_ctrl_done && !(s_ctrl_done = xSemaphoreCreateBinary()))
        return false;

    if (!s_client)
    {
        /* is_synchronous = false: IDF 5.3.1's header says "Set to false for now",
         * and the async event callback is mandatory -- the library calls it on
         * every queued event, so a NULL one jumps to address zero the moment a
         * device attaches. That cost four diagnostic runs in usbdiag. */
        const usb_host_client_config_t cfg = {
            .is_synchronous = false,
            .max_num_event_msg = 4,
            .async = { .client_event_callback = on_client_event, .callback_arg = NULL },
        };

        if (usb_host_client_register(&cfg, &s_client) != ESP_OK)
        {
            ESP_LOGE(TAG, "client register failed");
            return false;
        }
    }

    /* Higher than the library task: this one refills the isochronous stream. */
    if (!s_task && xTaskCreate(client_task, "uac2", 4096, NULL, 5, &s_task) != pdPASS)
    {
        ESP_LOGE(TAG, "no client task");
        return false;
    }

    /* Find the DAC among the attached devices. A host that has only just been
     * installed has not enumerated anything yet, so look again for a while
     * rather than once. */
    uint8_t list[8];
    int n = 0;

    for (int waited = 0; ; waited += 100)
    {
        if (usb_host_device_addr_list_fill(8, list, &n) != ESP_OK)
            return false;
        if (n > 0 || waited >= UAC2_ATTACH_WAIT_MS)
            break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

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

    const esp_err_t rerr = get_rates();

    ESP_LOGI(TAG, "clock %u RANGE -> %s, %d rate(s)", s_clock_id, esp_err_to_name(rerr),
             s_nrates);
    for (int k = 0; k < s_nrates; k++)
        ESP_LOGI(TAG, "  rate %lu", (unsigned long)s_rates[k]);

    if (!s_ring)
        s_ring = xRingbufferCreate(UAC2_RING_BYTES, RINGBUF_TYPE_BYTEBUF);

    if (!s_ring)
    {
        ESP_LOGE(TAG, "no memory for the PCM ring");
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
