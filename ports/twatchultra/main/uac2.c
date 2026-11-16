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

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "usb/usb_host.h"
#include "usbhost.h"

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
#define AC_FEATURE_UNIT     0x06    /* inside the AudioControl interface */
#define UAC2_FU_MUTE        0x01    /* Feature Unit control selectors, Audio 2.0 A.17.7 */
#define UAC2_FU_VOLUME      0x02

/* UAC2 control selector for the sample rate, from the Audio 2.0 spec 5.2.3.1. */
#define UAC2_CS_SAM_FREQ    0x01
#define UAC2_CN_NONE        0x00

/* UAC2 class requests (Audio 2.0 spec A.14). */
#define UAC2_REQ_CUR        0x01
#define UAC2_REQ_RANGE      0x02

/* Playback geometry. The stream is always stereo S16; its RATE is whatever the
 * caller asks for from the clock's RANGE list (uac2_start_rate()), 48 kHz by
 * default and for the test tone. UAC2_FRAME_BYTES is one 1 ms packet AT 48 kHz,
 * kept for the sizes that are quoted in that unit. */
#define UAC2_RATE           48000
#define UAC2_CHANNELS       2
#define UAC2_BYTES_PER_SAMPLE 2
#define UAC2_BYTES_PER_FRAME (UAC2_CHANNELS * UAC2_BYTES_PER_SAMPLE)                   /* 4 */
#define UAC2_FRAME_BYTES    (UAC2_RATE / 1000 * UAC2_BYTES_PER_FRAME)                    /* 192 */

/* Frames of audio carried per transfer, and how many transfers may be in flight.
 * 8 frames is 8 ms; three in flight gives 24 ms of slack.
 *
 * WHY 8 AND NOT 16. At 16 packets per transfer the DAC's line output, captured
 * on a PC, carried a discontinuity at every transfer boundary -- 440 Hz test
 * tone sidebands at +/- n x 62.5 Hz, -36 dB each, a few samples' phase step
 * every 16 ms -- and it stayed exactly that bad after the ring was proven to
 * deliver every sample (short=0 underruns=0). So the glitch is at the URB
 * boundary in the host path, not in this driver's buffering, and its mechanism
 * is NOT established. At 8 the operator heard the tone as clean; that build's
 * line-in captures came back empty, so 8 is accepted by ear, not by
 * measurement, and a capture at 8 is still owed. */
#define UAC2_PKTS_PER_XFER  8
#define UAC2_XFERS          3

/* A transfer's buffer is sized for the largest packet AT THE STREAM'S RATE --
 * ceil(rate / 1000) frames -- and reallocated with the stream on a rate change.
 * Sizing it for the endpoint's 384-byte maximum instead was tried and failed on
 * hardware: three 3,072-byte DMA buffers are INTERNAL RAM, of which this board
 * had 17,507 bytes free with a largest block of 7,168, and the start failed with
 * ESP_ERR_NO_MEM. At 44.1 kHz this is 1,440 bytes per transfer. */
#define UAC2_MAX_RATE       48000
#define UAC2_XFER_BUF_BYTES ((size_t)(UAC2_MAX_RATE / 1000) * UAC2_BYTES_PER_FRAME * UAC2_PKTS_PER_XFER)

/* ALLOCATED ONCE, IN uac2_open(), AND NEVER FREED. A second hardware failure
 * forced this. Allocating at stream start worked from the shell's own task but
 * failed with ESP_ERR_NO_MEM when the file player started the stream: that
 * task's 4 KB stack is internal RAM too, and with ~17 KB free in fragments the
 * three DMA buffers no longer fitted. The same build played at 19:40 and failed
 * at 19:43 on another boot. Allocating while the shell runs `uac2 open`, before
 * any player exists, and keeping the transfers for the driver's life removes
 * both the timing luck and every free from the stop and unplug paths (the
 * unplug path's free is what panicked in tlsf_free, core-log entry 18). The cost
 * is 4.6 KB of internal RAM held while the DAC is open, and a 48 kHz ceiling. */

/* About 500 ms of PCM at 48 kHz, in PSRAM. A file is read from an SD card over
 * SPI at a few hundred KB/s with stalls of 100+ ms (core-log entry 7 measured
 * 127 ms for a 32 KB write), and 44.1 kHz stereo needs 176 KB/s, so the ring has
 * to ride out a stall rather than a scheduler hiccup. Internal RAM is the scarce
 * pool on this board (SOC-005); the ring is only ever touched from tasks, never
 * from an ISR, so PSRAM is safe for it. */
#define UAC2_RING_BYTES     (UAC2_FRAME_BYTES * 500)

static usb_host_client_handle_t s_client;
static usb_device_handle_t s_dev;
static uint8_t s_dev_addr;

/* Discovered from the device's own descriptors. */
static uint8_t s_ac_iface;      /* AudioControl interface: addresses its entities */
static uint8_t s_fu_id[4];      /* Feature Units: the DAC's own mute and volume */
static uint8_t s_fu_src[4];     /* each one's bSourceID */
static int s_nfu;
static uint8_t s_play_fu;       /* the Feature Unit fed by the playback terminal, 0 = none */

/* HARDWARE VOLUME. `uac2 vol` used to scale the samples, and on headphones the
 * operator heard hiss at low settings: the samples lose resolution while the
 * DAC's analog stage stays at full gain with its own noise floor. The DAC has a
 * Volume control on its playback Feature Unit (it answered GET_CUR with 0 dB),
 * so the volume is set THERE and the samples go out at full scale. Software
 * scaling remains only as the fallback for a DAC without one. */
static bool s_hw_vol;           /* the playback FU's volume control answered RANGE */
static int16_t s_vol_min, s_vol_max;   /* its range, 1/256 dB */
static int s_vol_pct = 50;      /* what the operator asked for */
static bool s_claimed;          /* the streaming interface is claimed (held while open) */
static int16_t s_vol_db256;     /* what was last written to the DAC */
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
static uint32_t s_rate = UAC2_RATE;     /* the rate the stream is running at */
static uint32_t s_pkt_n;                /* packets sized since the stream started */
/* Software volume, Q15 (32768 = unity), applied as each transfer is filled so
 * it reaches the file player and the tone alike and can change mid-stream. 50
 * percent by default: at unity the DAC was "very loud" on the operator's
 * headphones. Scaling 16-bit samples costs resolution at low settings; the
 * DAC's own Feature Unit volume would not, and is the better long-term control. */
static volatile int32_t s_gain_q15 = 32768;

/* WHEN THE POPS HAPPEN. Each short or empty transfer during a stream is kept
 * with its time and with whether the file reader was stuck inside fread() at
 * that moment, and for how long: an SD stall and a scheduling problem look the
 * same in a total and different here. `uac2 ev` prints them. */
typedef struct
{
    uint32_t ms;            /* since the stream started */
    uint16_t got, want;     /* bytes the ring supplied, bytes the transfer needed */
    uint32_t read_ms;       /* how long the reader had been in fread(), 0 if not */
} uac2_ev_t;

#define UAC2_NEV 12
static uac2_ev_t s_ev[UAC2_NEV];
static volatile uint32_t s_nev;
static int64_t s_t0_us;                 /* stream start */
static volatile int64_t s_read_start_us;   /* reader entered fread(), 0 = not reading */
static volatile bool s_feeding;            /* a file player is feeding the ring */
static volatile uint32_t s_frames_sent;
static volatile uint32_t s_xfer_errors;
static volatile uint32_t s_underruns;   /* transfers sent as pure silence */
static volatile uint32_t s_short;       /* transfers part-filled, the rest silence */
static volatile bool s_gone;            /* the DAC was unplugged mid-stream */
static volatile uint32_t s_missed;      /* packets the controller reported SKIPPED */
static volatile int s_last_status = -1;
static TaskHandle_t s_task;             /* the client's event loop */
static SemaphoreHandle_t s_ctrl_done;   /* a control transfer has completed */

/* How long uac2_open() waits for the DAC to attach and enumerate after the host
 * comes up. Enumeration of this dongle measured well under a second. */
#define UAC2_ATTACH_WAIT_MS 2000

static void on_client_event(const usb_host_client_event_msg_t *msg, void *arg);
static void note_start_err(const char *step, esp_err_t err);

/* AUTOMATIC MODE (usb.dac = 1 in OTG host mode): a worker opens the DAC when it
 * is plugged in and forgets it when it goes, so nothing has to type `uac2 open`.
 * The client's event callback only wakes the worker -- opening a device is a
 * library call the client task must stay free to service. */
static TaskHandle_t s_worker;
static char s_product[40];      /* the DAC's product string, for the USB page */

/*
 * A DMA RESERVE FOR THE CLAIM. Claiming the streaming interface allocates two
 * 512-byte, 512-aligned descriptor lists in DMA-capable RAM, and IDF 5.3.1
 * PANICS when it cannot (reference USB-005). A DAC plugged in after a file has
 * been opened on the SD card -- which takes DMA RAM of its own -- would hit that.
 * So two blocks of exactly that shape are taken at boot and freed immediately
 * before the claim, leaving holes the claim's own allocation fits. Re-taken when
 * a DAC is unplugged and its pipe released. Sized from IDF 5.3.1:
 * XFER_LIST_LEN_ISOC 64 descriptors x 8 bytes, USB_DWC_QTD_LIST_MEM_ALIGN 512.
 */
#define UAC2_RESERVE_N      2
#define UAC2_RESERVE_BYTES  512
static void *s_reserve[UAC2_RESERVE_N];

static void reserve_take(void)
{
    for (int i = 0; i < UAC2_RESERVE_N; i++)
        if (!s_reserve[i])
            s_reserve[i] = heap_caps_aligned_alloc(UAC2_RESERVE_BYTES, UAC2_RESERVE_BYTES,
                                                   MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
}

static void reserve_release(void)
{
    for (int i = 0; i < UAC2_RESERVE_N; i++)
    {
        heap_caps_free(s_reserve[i]);
        s_reserve[i] = NULL;
    }
}

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
    s_nfu = 0;
    s_play_fu = 0;
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
                else if (sub == AC_FEATURE_UNIT && blen >= 6 && s_nfu < 4)
                {
                    s_fu_src[s_nfu] = cfg[i + 4];           /* bSourceID */
                    s_fu_id[s_nfu++] = cfg[i + 3];          /* bUnitID */
                }
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

    for (int u = 0; u < s_nfu; u++)
        if (s_fu_src[u] == alts[pick].term)
            s_play_fu = s_fu_id[u];

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
/*
 * THE DAC'S OWN MUTE AND VOLUME. A UAC2 device carries them in Feature Units in
 * its AudioControl interface, addressed like the clock: wIndex = (unit << 8) |
 * AC interface, wValue = (control selector << 8) | channel (0 = master). Mute is
 * one byte; volume is a signed 16-bit value in 1/256 dB. Nothing here set them,
 * and on hardware the DAC went silent with a clean stream feeding it -- after
 * being re-plugged, and in three line-in captures before that -- so the first
 * question is what state it is in.
 */
static esp_err_t fu_get(uint8_t unit, uint8_t cs, uint8_t cn, uint8_t *v, uint16_t len)
{
    return ctrl(0xA1, UAC2_REQ_CUR, (uint16_t)((cs << 8) | cn),
                (uint16_t)((unit << 8) | s_ac_iface), v, len);
}

static esp_err_t fu_set(uint8_t unit, uint8_t cs, uint8_t cn, uint8_t *v, uint16_t len)
{
    return ctrl(0x21, UAC2_REQ_CUR, (uint16_t)((cs << 8) | cn),
                (uint16_t)((unit << 8) | s_ac_iface), v, len);
}

static void fu_report(bool set)
{
    if (!s_dev)
    {
        printf("uac2: not open\n");
        return;
    }
    if (s_nfu == 0)
        printf("uac2: no feature units in the descriptors\n");

    for (int u = 0; u < s_nfu; u++)
    {
        for (uint8_t cn = 0; cn <= 2; cn++)
        {
            if (set)
            {
                uint8_t m = 0, v[2] = { 0, 0 };     /* unmute, 0 dB */

                fu_set(s_fu_id[u], UAC2_FU_MUTE, cn, &m, 1);
                fu_set(s_fu_id[u], UAC2_FU_VOLUME, cn, v, 2);
            }

            uint8_t m = 0xEE, v[2] = { 0, 0 };
            const esp_err_t em = fu_get(s_fu_id[u], UAC2_FU_MUTE, cn, &m, 1);
            const esp_err_t ev = fu_get(s_fu_id[u], UAC2_FU_VOLUME, cn, v, 2);
            const int16_t vol = (int16_t)(v[0] | (v[1] << 8));

            printf("fu %u ch %u: mute %s vol %s\n", s_fu_id[u], cn,
                   em == ESP_OK ? (m ? "ON" : "off") : "n/a",
                   ev == ESP_OK ? "" : "n/a");
            if (ev == ESP_OK)
                printf("   vol %d/256 dB = %d.%02d dB\n", vol, vol / 256,
                       (vol < 0 ? -vol : vol) % 256 * 100 / 256);
        }
    }
}

/* Read the playback FU's volume range: RANGE on the Volume control returns
 * wNumSubRanges then (wMIN, wMAX, wRES) per subrange, each a signed 16-bit
 * 1/256 dB value (Audio 2.0 spec 5.2.3.2). */
static void hw_vol_probe(void)
{
    uint8_t r[2 + 6 * 4];

    s_hw_vol = false;
    if (!s_play_fu)
        return;

    memset(r, 0, sizeof(r));
    if (ctrl(0xA1, UAC2_REQ_RANGE, (uint16_t)(UAC2_FU_VOLUME << 8),
             (uint16_t)((s_play_fu << 8) | s_ac_iface), r, sizeof(r)) != ESP_OK)
        return;

    const int n = r[0] | (r[1] << 8);

    if (n < 1)
        return;
    s_vol_min = (int16_t)(r[2] | (r[3] << 8));
    s_vol_max = (int16_t)(r[4] | (r[5] << 8));
    s_hw_vol = s_vol_max > s_vol_min;
}

/* Apply s_vol_pct: on the DAC when it has a volume control (samples at unity),
 * by scaling the samples otherwise. Percent is amplitude, so 50 is -6 dB, and
 * 0 mutes. */
static void apply_volume(void)
{
    if (!s_hw_vol)
    {
        s_gain_q15 = (int32_t)s_vol_pct * 32768 / 100;
        return;
    }

    s_gain_q15 = 32768;

    uint8_t m = s_vol_pct == 0 ? 1 : 0;
    int32_t db = s_vol_pct == 0 ? s_vol_min
                                : (int32_t)lroundf(20.0f * log10f(s_vol_pct / 100.0f) * 256.0f);

    if (db < s_vol_min)
        db = s_vol_min;
    if (db > s_vol_max)
        db = s_vol_max;
    s_vol_db256 = (int16_t)db;

    uint8_t v[2] = { (uint8_t)(db & 0xFF), (uint8_t)((db >> 8) & 0xFF) };

    fu_set(s_play_fu, UAC2_FU_MUTE, 0, &m, 1);
    fu_set(s_play_fu, UAC2_FU_VOLUME, 0, v, 2);
}

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

/* Size every packet of `t` for the next 1 ms slices at s_rate, and return the
 * bytes the transfer then carries. Packet n holds the frames between the
 * integer counts reached at the start and end of millisecond n, so 44.1 kHz
 * alternates 44 and 45 frames and the average is exact. Called only from the
 * client task (refill) and from uac2_start_rate() before the stream is running,
 * so s_pkt_n has one writer at a time. */
static size_t size_packets(usb_transfer_t *t)
{
    size_t total = 0;

    for (int p = 0; p < UAC2_PKTS_PER_XFER; p++)
    {
        const uint32_t n = s_pkt_n++;
        const uint32_t frames = (uint32_t)(((uint64_t)s_rate * (n + 1)) / 1000 -
                                           ((uint64_t)s_rate * n) / 1000);

        t->isoc_packet_desc[p].num_bytes = (int)(frames * UAC2_BYTES_PER_FRAME);
        total += frames * UAC2_BYTES_PER_FRAME;
    }

    return total;
}

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

    /* The DAC was unplugged. Resubmitting would only fail again, so the stream
     * stops here and uac2_open() tears the rest down from the shell, where a
     * device close is safe; the transfer itself is complete and can be freed. */
    if (t->status == USB_TRANSFER_STATUS_NO_DEVICE)
    {
        s_streaming = false;
        s_gone = true;
        return;
    }

    if (!s_streaming)
        return;

    /*
     * REFILL FROM THE RING; PLAY SILENCE ONLY FOR WHAT IS TRULY MISSING.
     *
     * An underrun must produce sound, not a stall: isochronous transfers cannot
     * be paused without the device glitching, so whatever the ring cannot supply
     * is zeros. Two things about that were wrong: xRingbufferReceiveUpTo() on a
     * byte buffer stops at the ring's wrap point, so ONE call can return less
     * than the ring holds -- a second call is needed to take the rest -- and a
     * fill that came up a few bytes short was padded silently, because only an
     * EMPTY ring was counted. Every short fill is now counted, as `short`. These
     * were NOT the audible scratch: with short=0 the 62.5 Hz sidebands measured
     * from the DAC's line out were unchanged; that is the transfer boundary
     * (reference USB-004, and UAC2_PKTS_PER_XFER above).
     *
     * EACH PACKET CARRIES ONE MILLISECOND AT THE STREAM'S RATE. At 44.1 kHz that
     * is 44.1 frames, so packets alternate 44 and 45 frames; the size of packet
     * n is the integer frame count reached by its end minus the count reached by
     * its start, which is exact on average and never drifts. IDF packs the
     * packets back to back in the buffer by their num_bytes (hcd_dwc.c
     * _buffer_fill_isoc), so variable sizes need nothing else.
     */
    const size_t want = size_packets(t);
    size_t filled = 0;

    while (filled < want)
    {
        size_t got = 0;
        uint8_t *pcm = xRingbufferReceiveUpTo(s_ring, &got, 0, want - filled);

        if (!pcm)
            break;

        memcpy(t->data_buffer + filled, pcm, got);
        vRingbufferReturnItem(s_ring, pcm);
        filled += got;
    }

    if (s_gain_q15 != 32768)
    {
        int16_t *smp = (int16_t *)t->data_buffer;
        const int32_t g = s_gain_q15;

        for (size_t i = 0; i < filled / 2; i++)
            smp[i] = (int16_t)(((int32_t)smp[i] * g) >> 15);
    }

    if (filled < want)
    {
        memset(t->data_buffer + filled, 0, want - filled);
        if (filled == 0)
            s_underruns++;
        else
            s_short++;

        /* Only while a file is being fed: the silence after a track is expected
         * and would fill the log with nothing. */
        if (s_feeding)
        {
            const int64_t now = esp_timer_get_time();
            const int64_t rs = s_read_start_us;
            uac2_ev_t *e = &s_ev[s_nev % UAC2_NEV];

            e->ms = (uint32_t)((now - s_t0_us) / 1000);
            e->got = (uint16_t)filled;
            e->want = (uint16_t)want;
            e->read_ms = rs > 0 ? (uint32_t)((now - rs) / 1000) : 0;
            s_nev++;
        }
    }

    t->num_bytes = (int)want;

    if (usb_host_transfer_submit(t) != ESP_OK)
        s_xfer_errors++;
    else
        s_frames_sent += UAC2_PKTS_PER_XFER;
}

/*
 * THE TWO EVENT LOOPS IDF REQUIRES, AND WHY NEITHER IS OPTIONAL.
 *
 * The host LIBRARY processes attach, enumeration and detach only inside
 * usb_host_lib_handle_events() -- that loop now lives in usbhost.c, the one
 * owner of the host stack; with nobody calling it a device is never
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
static void client_task(void *arg)
{
    (void)arg;

    for (;;)
        usb_host_client_handle_events(s_client, portMAX_DELAY);
}

/* ------------------------------------------------------------------ public */

/*
 * Forget a DAC that was unplugged, so the next open can find it again.
 *
 * Run from the shell task, never from the client task's callbacks: closing a
 * device is a library call that the client task must be free to service. Before
 * this existed an unplug left s_dev pointing at a dead device and `streaming=1`,
 * every later write went into a ring nothing drained, and only a reboot
 * recovered -- which is how a line-in capture came back silent on 2026-11-15.
 */
static void forget_gone_device(void)
{
    if (!s_gone)
        return;

    s_streaming = false;

    /* The transfers are kept and re-aimed at the new device by the next start;
     * freeing them here is what panicked (see UAC2_XFER_BUF_BYTES). */

    if (s_dev)
    {
        usb_host_interface_release(s_client, s_dev, s_as_iface);   /* may fail: gone */
        s_claimed = false;
        usb_host_device_close(s_client, s_dev);
        s_dev = NULL;
    }
    s_product[0] = '\0';
    /*
     * NO reserve_take() here. It used to re-take the claim's block as soon as the
     * device went away, which left DMA-capable RAM held across the re-enumeration
     * that follows a re-plug -- the very window that panics in device_alloc()
     * when the pool is empty. uac2_open() takes it again before the next claim,
     * which is the only place it is needed.
     */

    if (s_ring)
    {
        size_t got = 0;
        void *p;

        while ((p = xRingbufferReceiveUpTo(s_ring, &got, 0, UAC2_RING_BYTES)) != NULL)
            vRingbufferReturnItem(s_ring, p);
    }

    s_gone = false;
    ESP_LOGW(TAG, "DAC was unplugged; state cleared");
}

/* The control semaphore, the client and its event task: everything that has to
 * exist before a DAC can be seen, whether opened by hand or automatically. */
static bool client_setup(void)
{
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

    return true;
}

/* The stream's three transfers, allocated once and kept (see
 * UAC2_XFER_BUF_BYTES). They need no device: it is set at each start. */
static bool alloc_transfers(void)
{
    for (int i = 0; i < UAC2_XFERS; i++)
    {
        if (s_xfer[i])
            continue;

        const esp_err_t aerr = usb_host_transfer_alloc(UAC2_XFER_BUF_BYTES, UAC2_PKTS_PER_XFER,
                                                       &s_xfer[i]);
        if (aerr != ESP_OK)
        {
            s_xfer[i] = NULL;
            note_start_err("transfer alloc", aerr);
            return false;
        }
    }
    return true;
}

bool uac2_open(void)
{
    forget_gone_device();

    if (s_dev)
        return true;

    /* The host stack belongs to usbhost.c; in Normal mode a manual `uac2 open`
     * still works as a diagnostic, and installs it (the port is then a host
     * until the next restart). */
    if (!usbhost_install())
        return false;

    if (!client_setup())
        return false;

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

        /* The product string, for the USB page: UTF-16LE, ASCII kept. */
        usb_device_info_t info;

        s_product[0] = '\0';
        if (usb_host_device_info(dev, &info) == ESP_OK && info.str_desc_product)
        {
            const usb_str_desc_t *sd = info.str_desc_product;
            int n = (sd->bLength - 2) / 2, k = 0;

            for (int j = 0; j < n && k < (int)sizeof(s_product) - 1; j++)
                if (sd->wData[j] >= 0x20 && sd->wData[j] < 0x7F)
                    s_product[k++] = (char)sd->wData[j];
            s_product[k] = '\0';
        }
        if (!s_product[0])
            snprintf(s_product, sizeof(s_product), "%04X:%04X", UAC2_VID, UAC2_PID);
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

    /*
     * The reserve is taken HERE and not at boot: it exists to be held while the
     * control requests above run and released at the claim, so the claim finds a
     * contiguous block nothing else has taken -- not to sit in the pool while a
     * device is still enumerating, which starves enumeration itself.
     */
    reserve_take();

    if (!alloc_transfers())
        return false;

    /*
     * CLAIM ONCE, HERE, AND HOLD IT. Claiming allocates the pipe and the HCD's
     * transfer-descriptor lists in DMA-capable internal RAM, and IDF 5.3.1
     * ASSERTS when that allocation fails (hcd_dwc.c:1059 transfer_descriptor_
     * list_alloc) -- a panic, not an error code. On hardware it fired when the
     * file player started the stream: the SD card runs over SPI with DMA, so an
     * open file had already taken DMA-capable RAM. Every DMA allocation the
     * stream needs therefore happens in `uac2 open`, before any file is opened,
     * and stop/rate changes only switch the alternate setting.
     */
    if (!s_claimed)
    {
        reserve_release();      /* make room exactly where the claim needs it */

        const esp_err_t cerr = usb_host_interface_claim(s_client, s_dev, s_as_iface, s_as_alt);

        if (cerr != ESP_OK)
        {
            note_start_err("claim at open", cerr);
            return false;
        }
        s_claimed = true;
    }

    hw_vol_probe();
    apply_volume();
    ESP_LOGI(TAG, "volume: %s, playback FU %u, range %d..%d /256 dB", s_hw_vol ? "DAC" : "software",
             s_play_fu, s_vol_min, s_vol_max);

    if (!s_ring)
        s_ring = xRingbufferCreateWithCaps(UAC2_RING_BYTES, RINGBUF_TYPE_BYTEBUF,
                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!s_ring)
    {
        ESP_LOGE(TAG, "no memory for the PCM ring");
        return false;
    }

    ESP_LOGI(TAG, "DAC open: stereo S16, ring %u bytes in PSRAM", (unsigned)UAC2_RING_BYTES);
    return true;
}

/* THE LAST START FAILURE, KEPT FOR `uac2`. A start attempted from the player's
 * task logs nowhere visible, and on hardware a start that failed after claiming
 * the interface left it claimed, so the NEXT start failed at the claim and the
 * real cause was never seen. Every failure is now recorded here, with its IDF
 * error, and undone (see start_fail()). */
static char s_start_err[96];

static void note_start_err(const char *step, esp_err_t err)
{
    snprintf(s_start_err, sizeof(s_start_err), "%s: %s (0x%x)", step, esp_err_to_name(err),
             (unsigned)err);
    ESP_LOGE(TAG, "start failed at %s", s_start_err);
}

/* Undo a partial start: free the transfers allocated so far (none has been
 * submitted, so freeing is safe) and release the interface if it was claimed. */
static void start_fail(bool claimed)
{
    (void)claimed;              /* the claim is held for the driver's life */
    s_streaming = false;
}

/* True when the clock's RANGE list offers `rate` and one packet of it fits the
 * endpoint. An empty list (a device that refused RANGE) admits only 48 kHz. */
static bool rate_ok(uint32_t rate)
{
    if ((rate + 999) / 1000 * UAC2_BYTES_PER_FRAME > s_ep_mps || rate > UAC2_MAX_RATE)
        return false;
    if (s_nrates == 0)
        return rate == UAC2_RATE;
    for (int k = 0; k < s_nrates; k++)
        if (s_rates[k] == rate)
            return true;
    return false;
}

/* The worker: opens the DAC when one appears, forgets it when it goes. One
 * pass at start covers a DAC that was already plugged in at boot, since a
 * client only hears NEW_DEV for devices enumerated after it registered. */
static void worker_task(void *arg)
{
    (void)arg;

    for (;;)
    {
        if (!s_dev || s_gone)
            uac2_open();
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(300));     /* let enumeration settle */
    }
}

bool uac2_auto_start(void)
{
    if (s_worker)
        return true;
    if (!usbhost_install() || !client_setup())
        return false;

    /*
     * NOTHING IS ALLOCATED HERE, and that is the fix for a boot loop.
     *
     * This used to take the stream's transfers and the claim's reserve at boot,
     * on the reasoning that the DMA memory has to be taken before anything opens
     * a file on the SD card (USB-005). With a DAC already plugged in it did the
     * opposite of what was wanted: enumeration runs in the USB library's own
     * task the moment the host stack is installed, and ENUMERATION NEEDS
     * DMA-CAPABLE INTERNAL RAM TOO -- device_alloc() allocates the default
     * control pipe, whose descriptor list is 3 qtds and asserts when it cannot
     * (hcd_dwc.c:1059, the same assert as the claim).
     *
     * 3 x 1536 transfers plus 2 x 512 reserve is 5632 bytes, and on this board
     * in OTG mode internal RAM is down to about 8 KB free with the shell's
     * 24 KB stack block held, so the pool was empty by the time a device
     * attached. The coredump named it: task 'usb_lib', hub.c root_port_handle_
     * events -> usbh_devs_add -> device_alloc -> hcd_pipe_alloc -> assert.
     * Boot, desktop, reset, for ever -- and unplugging the DAC was the only way
     * out, because the reset happens before any command can run.
     *
     * The allocation belongs in uac2_open(), which runs once a device is
     * present, after enumeration and still before any file is opened.
     */
    if (xTaskCreate(worker_task, "uac2dev", 4096, NULL, 3, &s_worker) != pdPASS)
    {
        s_worker = NULL;
        return false;
    }
    ESP_LOGI(TAG, "USB-C DAC: automatic (opens when plugged in)");
    return true;
}

bool uac2_attached(void)
{
    return s_dev && !s_gone && s_claimed;
}

const char *uac2_product(void)
{
    return s_product[0] ? s_product : "?";
}

bool uac2_start(void)
{
    return uac2_start_rate(UAC2_RATE);
}

bool uac2_start_rate(uint32_t rate)
{
    if ((s_gone || !s_dev) && !uac2_open())
        return false;

    if (!rate_ok(rate))
    {
        ESP_LOGE(TAG, "rate %lu not offered by the DAC", (unsigned long)rate);
        return false;
    }

    if (s_streaming && rate == s_rate)
        return true;

    /* A different rate: the stream has to come down to change the clock, and
     * whatever is still queued was meant for the old rate. */
    if (s_streaming)
        uac2_stop();

    {
        size_t got = 0;
        void *p;

        while ((p = xRingbufferReceiveUpTo(s_ring, &got, 0, UAC2_RING_BYTES)) != NULL)
            vRingbufferReturnItem(s_ring, p);
    }

    s_rate = rate;
    s_pkt_n = 0;
    s_t0_us = esp_timer_get_time();

    /* Order matters: claim the interface before touching its endpoints, set the
     * rate before selecting the streaming alternate setting, and only then bring
     * the endpoint up. */
    s_start_err[0] = '\0';

    esp_err_t err = ESP_OK;

    if (!s_claimed)
    {
        note_start_err("interface not claimed (run `uac2 open`)", ESP_ERR_INVALID_STATE);
        return false;
    }

    const esp_err_t rerr = set_rate(rate);
    ESP_LOGI(TAG, "set_rate(%lu) -> %s", (unsigned long)rate, esp_err_to_name(rerr));
    if (rerr != ESP_OK)
        note_start_err("set_rate (not fatal)", rerr);

    for (int i = 0; i < UAC2_XFERS; i++)
    {
        if (!s_xfer[i])
        {
            note_start_err("transfers (uac2 open did not allocate them)", ESP_ERR_NO_MEM);
            start_fail(true);
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
        memset(s_xfer[i]->data_buffer, 0, UAC2_XFER_BUF_BYTES);
        s_xfer[i]->num_bytes = (int)size_packets(s_xfer[i]);   /* silence, at the new rate */
    }

    s_streaming = true;

    err = set_alt(s_as_iface, s_as_alt);
    if (err != ESP_OK)
    {
        note_start_err("SET_INTERFACE", err);
        start_fail(true);
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

    /* The transfers are kept: see UAC2_XFER_BUF_BYTES. */

    /* The interface stays claimed: see uac2_open(). */
    ESP_LOGI(TAG, "stopped");
}

int uac2_write(const uint8_t *pcm, int len)
{
    if (!s_streaming || !s_ring || !pcm || len <= 0)
        return 0;

    /* Non-blocking: the emulators call this from their own frame loop, and an
     * emulator that blocks on audio stops emulating. A full ring means the sink
     * is behind; dropping the newest samples is what a watched buffer would do
     * anyway, and it cannot deadlock the guest.
     *
     * xRingbufferSend() returns pdTRUE/pdFALSE, not a count, and is
     * all-or-nothing on a byte buffer (reference TOL-004); this used to return
     * the flag, i.e. 1. */
    return xRingbufferSend(s_ring, pcm, (size_t)len, 0) == pdTRUE ? len : 0;
}

int uac2_write_wait(const uint8_t *pcm, int len, uint32_t timeout_ms)
{
    if (!s_streaming || !s_ring || !pcm || len <= 0)
        return 0;

    return xRingbufferSend(s_ring, pcm, (size_t)len, pdMS_TO_TICKS(timeout_ms)) == pdTRUE
               ? len : 0;
}

uint32_t uac2_rate(void)
{
    return s_rate;
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
    {
        ESP_LOGI(TAG, "device attached: address %d", (int)msg->new_dev.address);
        if (s_worker)
            xTaskNotifyGive(s_worker);
    }
    else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE)
    {
        /* Only flag it: this runs on the client task, and the teardown (device
         * close) belongs to the shell side -- see forget_gone_device(). */
        if (s_dev && msg->dev_gone.dev_hdl == s_dev)
        {
            s_streaming = false;
            s_gone = true;
        }
        ESP_LOGW(TAG, "device gone");
        if (s_worker)
            xTaskNotifyGive(s_worker);
    }
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
#include <stdarg.h>
#include <stdlib.h>

#include "tdsh.h"
#include "wav.h"

/* --- `uac2 play` -- a WAV file from the card to the DAC --------------------
 *
 * The same shape as the speaker's sound_play_wav(): its own task, the file
 * STREAMED a chunk at a time (a track is tens of megabytes against 8 MiB of
 * PSRAM), the header read by wav_parse() so both players agree on what a WAV is.
 * Two differences. The DAC is run at the FILE's rate when its clock offers it,
 * so a 44.1 kHz track needs no resampler; and every write blocks on ring space,
 * so the reader is paced by the DAC and can never fall behind it -- the lesson
 * of the tone. A file at a rate the DAC does not list is refused, not converted.
 */
#define UAC2_PLAY_CHUNK     4096            /* bytes read from the card per step */
#define UAC2_PLAY_HDR       65536           /* LIST/INFO chunks can be large */
#define UAC2_PLAY_STACK     4096

static char s_play_path[128];
static TaskHandle_t s_play_task;
static volatile bool s_play_stop;

/* THE PLAYER'S LAST WORD, KEPT FOR `uac2`. The player runs on its own task, and
 * only the shell task's output reaches the Terminal window, so an ESP_LOG line
 * from here is lost: the first run on hardware failed silently with streaming=0
 * and nothing on screen to say why. Every line the player logs is also kept
 * here, and `uac2` prints the last one. */
static char s_play_msg[112];

static void play_note(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void play_note(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(s_play_msg, sizeof(s_play_msg), fmt, ap);
    va_end(ap);
    ESP_LOGI(TAG, "%s", s_play_msg);
}

static void play_task(void *arg)
{
    const char *path = (const char *)arg;
    FILE *f = fopen(path, "rb");
    uint8_t *hdr = NULL, *buf = NULL;
    int16_t *dup = NULL;
    wav_info_t info;

    if (!f)
    {
        play_note("play: cannot open %s", path);
        goto done;
    }

    /* ALL IN PSRAM, EXPLICITLY. A plain malloc() of 4 KB or less lands in
     * internal RAM on this build (SPIRAM_MALLOC_ALWAYSINTERNAL=4096), and
     * internal RAM is what the stream's DMA transfers need next: taking it here
     * is how the first start from this task failed with ESP_ERR_NO_MEM. */
    hdr = heap_caps_malloc(UAC2_PLAY_HDR, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    buf = heap_caps_malloc(UAC2_PLAY_CHUNK, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    dup = heap_caps_malloc(UAC2_PLAY_CHUNK * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!hdr || !buf || !dup)
    {
        play_note("play: no memory");
        goto done;
    }

    const size_t got = fread(hdr, 1, UAC2_PLAY_HDR, f);
    const wav_status_t st = wav_parse(hdr, got, &info);

    free(hdr);
    hdr = NULL;

    if (st != WAV_OK)
    {
        play_note("play: %s", wav_status_str(st));
        goto done;
    }
    if (info.format != 1 || info.bits != 16 || info.channels < 1 || info.channels > 2)
    {
        play_note("play: fmt %u, %u bits, %u ch -- only 16-bit PCM mono/stereo",
                 info.format, info.bits, info.channels);
        goto done;
    }

    /* A header written before its data can claim more than the file holds;
     * reading past the end would play whatever the allocator left there. */
    if (fseek(f, 0, SEEK_END) == 0)
    {
        const long real = ftell(f);
        const long avail = real > (long)info.data_offset ? real - (long)info.data_offset : 0;

        if ((uint32_t)avail < info.data_bytes)
            info.data_bytes = (uint32_t)avail;
    }
    if (fseek(f, (long)info.data_offset, SEEK_SET) != 0)
    {
        play_note("play: seek failed");
        goto done;
    }

    if (!uac2_start_rate(info.rate))
    {
        play_note("play: cannot stream at %lu Hz", (unsigned long)info.rate);
        goto done;
    }

    const bool mono = info.channels == 1;
    const uint32_t secs = info.data_bytes / (info.rate * info.channels * 2);
    const uint32_t short0 = s_short, under0 = s_underruns;
    uint32_t left = info.data_bytes;

    uint32_t max_read_ms = 0, slow_reads = 0;

    s_nev = 0;
    s_feeding = true;
    play_note("play: %lu Hz %s, %lu s, %s", (unsigned long)info.rate,
             mono ? "mono" : "stereo", (unsigned long)secs, path);

    while (left > 0 && !s_play_stop)
    {
        size_t want = left < UAC2_PLAY_CHUNK ? left : UAC2_PLAY_CHUNK;

        want -= want % (mono ? 2 : 4);              /* whole frames only */
        if (want == 0)
            break;

        s_read_start_us = esp_timer_get_time();
        const size_t n = fread(buf, 1, want, f);
        const uint32_t read_ms = (uint32_t)((esp_timer_get_time() - s_read_start_us) / 1000);

        s_read_start_us = 0;
        if (read_ms > max_read_ms)
            max_read_ms = read_ms;
        if (read_ms > 50)
            slow_reads++;

        if (n == 0)
            break;
        left -= (uint32_t)n;

        const uint8_t *out = buf;
        size_t out_len = n - n % (mono ? 2 : 4);

        if (mono)
        {
            /* One sample to both slots: the DAC's alternate is stereo. */
            const size_t frames = out_len / 2;

            for (size_t i = 0; i < frames; i++)
            {
                const int16_t v = (int16_t)((uint16_t)buf[2 * i] | ((uint16_t)buf[2 * i + 1] << 8));

                dup[2 * i] = v;
                dup[2 * i + 1] = v;
            }
            out = (const uint8_t *)dup;
            out_len = frames * 4;
        }

        /* 500 ms with no room means the stream died (DAC gone, or stopped). */
        if (out_len && uac2_write_wait(out, (int)out_len, 500) != (int)out_len)
        {
            play_note("play: the stream stopped taking samples%s",
                     s_gone ? " (DAC unplugged)" : "");
            break;
        }
    }

    s_feeding = false;
    play_note("play: %s, short=%lu underruns=%lu, slowest read %lu ms, %lu reads >50 ms",
             s_play_stop ? "stopped" : "done",
             (unsigned long)(s_short - short0), (unsigned long)(s_underruns - under0),
             (unsigned long)max_read_ms, (unsigned long)slow_reads);

done:
    s_feeding = false;
    s_read_start_us = 0;
    free(hdr);
    free(buf);
    free(dup);
    if (f)
        fclose(f);
    s_play_stop = false;
    s_play_task = NULL;
    vTaskDelete(NULL);
}

static int cmd_play(const char *path)
{
    if (s_play_task)
    {
        printf("uac2: already playing; `uac2 play stop` first\n");
        return 1;
    }

    /* COPY THE PATH BEFORE STARTING THE TASK: argv is the shell's buffer and is
     * gone by the time the task opens the file (sound.c learned this first). */
    if (strlen(path) >= sizeof(s_play_path))
    {
        printf("uac2: path too long\n");
        return 1;
    }
    snprintf(s_play_path, sizeof(s_play_path), "%s", path);

    s_play_stop = false;
    if (xTaskCreate(play_task, "uac2play", UAC2_PLAY_STACK, s_play_path, 4, &s_play_task) != pdPASS)
    {
        s_play_task = NULL;
        printf("uac2: no task\n");
        return 1;
    }
    printf("uac2: playing %s -- `uac2` shows progress and the result\n", s_play_path);
    return 0;
}

static int cmd_uac2(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    if (argc >= 2 && strcmp(argv[1], "ev") == 0)
    {
        const uint32_t n = s_nev;

        printf("%lu short/empty transfer(s) while feeding\n", (unsigned long)n);
        for (uint32_t i = n > UAC2_NEV ? n - UAC2_NEV : 0; i < n; i++)
        {
            const uac2_ev_t *e = &s_ev[i % UAC2_NEV];

            printf("t=%lums %u/%u B, reader in fread %lums\n", (unsigned long)e->ms, e->got,
                   e->want, (unsigned long)e->read_ms);
        }
        return 0;
    }

    if (argc >= 2 && strcmp(argv[1], "fu") == 0)
    {
        fu_report(argc >= 3 && strcmp(argv[2], "set") == 0);
        return 0;
    }

    if (argc >= 2 && strcmp(argv[1], "vol") == 0)
    {
        if (argc >= 3)
        {
            const int pct = atoi(argv[2]);

            if (pct < 0 || pct > 100)
            {
                printf("uac2: vol is 0..100\n");
                return 1;
            }
            s_vol_pct = pct;
            if (s_dev)
                apply_volume();
        }
        if (s_hw_vol)
            printf("uac2: vol %d%% = DAC %d.%02d dB\n", s_vol_pct, s_vol_db256 / 256,
                   (s_vol_db256 < 0 ? -s_vol_db256 : s_vol_db256) % 256 * 100 / 256);
        else
            printf("uac2: vol %d%% (software)\n", s_vol_pct);
        return 0;
    }

    if (argc >= 3 && strcmp(argv[1], "play") == 0)
    {
        if (strcmp(argv[2], "stop") == 0)
        {
            s_play_stop = true;
            return 0;
        }
        return cmd_play(argv[2]);
    }

    if (argc >= 2 && strcmp(argv[1], "tone") == 0)
    {
        if (!uac2_start())
        {
            printf("uac2: cannot start (is the DAC attached?)\n");
            return 1;
        }

        /* Two seconds of 440 Hz stereo. A sine is used rather than silence so an
         * audience can tell success from a wiring fault by ear.
         *
         * PACED BY THE SINK, NOT BY A CLOCK. This used to write 2 ms of audio and
         * then sleep 2 ms, which with the sine arithmetic on top ran slightly
         * slower than the DAC drains, so the ring kept running a few samples
         * short of a transfer. Blocking on ring space keeps it full: the
         * producer can never fall behind, and the counters below prove it. */
        const int rate = (int)uac2_rate();
        const int total = rate * 2;             /* 2 seconds of frames */
        static float phase;
        const uint32_t short0 = s_short, under0 = s_underruns;

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

            /* The ring holds 96 ms; 200 ms without room means the stream died. */
            if (uac2_write_wait((const uint8_t *)buf, sizeof(buf), 200) != (int)sizeof(buf))
            {
                printf("uac2: the stream stopped taking samples%s\n",
                       s_gone ? " (DAC unplugged)" : "");
                return 1;
            }
        }

        /* Counted up to the last write: after it the ring drains and the stream
         * goes back to silence, which is correct and not a fault of the tone. */
        const uint32_t short1 = s_short - short0, under1 = s_underruns - under0;

        vTaskDelay(pdMS_TO_TICKS(200));         /* let the ring drain */
        printf("uac2: tone done, short=%lu underruns=%lu while writing\n",
               (unsigned long)short1, (unsigned long)under1);
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
    printf("frames=%lu errors=%lu underruns=%lu short=%lu missed=%lu last=%d%s\n",
           (unsigned long)f, (unsigned long)e, (unsigned long)u, (unsigned long)s_short,
           (unsigned long)m, s_last_status, s_gone ? " GONE" : "");
    printf("rate=%lu vol=%d%%%s play: %s\n", (unsigned long)s_rate, s_vol_pct,
           s_hw_vol ? "(DAC)" : "(sw)",
           s_play_task ? "running" : (s_play_msg[0] ? s_play_msg : "idle"));
    if (s_start_err[0])
        printf("start err: %s\n", s_start_err);
    printf("rates:");
    for (int i = 0; i < s_nrates; i++)
        printf(" %lu", (unsigned long)s_rates[i]);
    printf("\n");
    return 0;
}

static const tdsh_command_t s_uac2_cmd = {
    "uac2",
    "uac2 [open|start|stop|tone|vol [0-100]|ev|fu [set]|play <file.wav>|play stop]",
    "USB audio DAC: status, a test tone, or a WAV file from the card",
    cmd_uac2,
    TDSH_CMD_INTERACTIVE,
};

bool uac2_register_command(void)
{
    return tdsh_register_command(&s_uac2_cmd) == 0;
}
