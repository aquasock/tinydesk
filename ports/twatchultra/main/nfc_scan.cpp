/*
 * nfc_scan.cpp - the ST25R3916 spoken to through ST's own stack.
 *
 * This is the layer between the port and the vendor library in
 * components/st25r3916: it owns the two C++ objects (the RF driver and the NFC
 * activity layer), starts a discovery loop and turns an activated card into a
 * plain C struct. Everything protocol-shaped -- the analog configuration, the
 * ISO14443A short frame, the anticollision loop, the SAK lookup -- is the
 * vendor's code and is not touched.
 *
 * WHY A DISCOVERY LOOP RATHER THAN AN INTERRUPT. RFAL wants rfalNfcWorker()
 * called in a loop; the worker is cheap when nothing is happening because it
 * returns immediately unless there is work. This driver is polled from its own
 * task at about 1 kHz, which is fast enough that the reader's interrupt line
 * never has to be read at all, and it means no SPI transaction is ever started
 * from an interrupt context.
 *
 * THE CARD IS DEACTIVATED AS SOON AS IT IS READ, and the field goes off with
 * it. That is not laziness: holding a card active means holding the field on,
 * which is tens of milliamps on a watch, and this port has no use for a
 * persistent tag session yet. A cooldown stops the same card being reported
 * twice when a reader is still resting on the antenna.
 */

#include <cstdio>
#include <cstring>
#include <new>

#include "Arduino.h"
#include "SPI.h"

#include "rfal_nfc.h"
#include "rfal_rfst25r3916.h"
#include "ndef_class.h"

#include "esp_log.h"

#include "nfc.h"

/* A card is reported at most once per this long, so a tag left on the watch
 * does not fill the log with the same UID. */
#define REPORT_COOLDOWN_MS 2000

static RfalRfST25R3916Class *s_hw;
static RfalNfcClass *s_rf;
static NdefClass *s_ndef;
static bool s_running;
static bool s_handled;
static int s_cs = -1;
static uint32_t s_last_report_ms;
static char s_last_uid[33];

/* --- helpers -------------------------------------------------------------- */

static const char *tech_name(rfalNfcDevType t)
{
    switch (t)
    {
    case RFAL_NFC_LISTEN_TYPE_NFCA:
    case RFAL_NFC_POLL_TYPE_NFCA:
        return "A";
    case RFAL_NFC_LISTEN_TYPE_NFCB:
    case RFAL_NFC_POLL_TYPE_NFCB:
        return "B";
    case RFAL_NFC_LISTEN_TYPE_NFCF:
    case RFAL_NFC_POLL_TYPE_NFCF:
        return "F";
    case RFAL_NFC_LISTEN_TYPE_NFCV:
    case RFAL_NFC_POLL_TYPE_NFCV:
        return "V";
    case RFAL_NFC_LISTEN_TYPE_ST25TB:
        return "TB";
    case RFAL_NFC_LISTEN_TYPE_AP2P:
    case RFAL_NFC_POLL_TYPE_AP2P:
        return "P2P";
    default:
        return "?";
    }
}

static const char *nfca_tag_name(rfalNfcaListenDeviceType t)
{
    switch (t)
    {
    case RFAL_NFCA_T1T:
        return "T1T";
    case RFAL_NFCA_T2T:
        return "T2T";
    case RFAL_NFCA_T4T:
    case RFAL_NFCA_T4T_NFCDEP:
        return "T4T";
    case RFAL_NFCA_NFCDEP:
        return "P2P";
    default:
        return "";
    }
}

static void hex(const uint8_t *b, int n, char *out, size_t cap)
{
    static const char d[] = "0123456789abcdef";
    size_t o = 0;
    for (int i = 0; i < n && o + 2 < cap; i++)
    {
        out[o++] = d[(b[i] >> 4) & 0xF];
        out[o++] = d[b[i] & 0xF];
    }
    out[o] = '\0';
}

/* --- discovery ------------------------------------------------------------ */

static void begin_discovery(void)
{
    rfalNfcDiscoverParam p;
    memset(&p, 0, sizeof(p));

    p.compMode = RFAL_COMPLIANCE_MODE_NFC;
    p.devLimit = 1;
    p.techs2Find = RFAL_NFC_POLL_TECH_A | RFAL_NFC_POLL_TECH_B |
                   RFAL_NFC_POLL_TECH_F | RFAL_NFC_POLL_TECH_V;
    p.totalDuration = 1000;
    p.nfcfBR = RFAL_BR_212;
    p.ap2pBR = RFAL_BR_424;
    p.GBLen = 0;
    p.notifyCb = NULL;      /* the worker loop notices the state change */
    p.wakeupEnabled = false;

    ReturnCode err = s_rf->rfalNfcDiscover(&p);
    if (err != ST_ERR_NONE)
        ESP_LOGW("twatch-nfc", "discover %04x", err);
}

/* Fill the C struct from an activated device and work out what it is. */
static void describe(rfalNfcDevice *dev, nfc_card_t *out)
{
    memset(out, 0, sizeof(*out));
    out->ndef = -1;
    out->sak = -1;

    snprintf(out->tech, sizeof(out->tech), "%s", tech_name(dev->type));

    int len = dev->nfcidLen;
    if (len > (int)sizeof(out->uid) / 2 - 1)
        len = (int)sizeof(out->uid) / 2 - 1;
    if (dev->nfcid && len > 0)
    {
        hex(dev->nfcid, len, out->uid, sizeof(out->uid));
        out->uid_len = len;
    }

    if (dev->type == RFAL_NFC_LISTEN_TYPE_NFCA || dev->type == RFAL_NFC_POLL_TYPE_NFCA)
    {
        out->sak = dev->dev.nfca.selRes.sak;
        snprintf(out->tag, sizeof(out->tag), "%s", nfca_tag_name(dev->dev.nfca.type));
    }

    /* NDEF, attempted only where RFAL has a poller for the tag type. A blank
     * tag answering "no NDEF" is a result, not a failure, and is worth telling
     * apart from "this technology has no NDEF poller at all". */
    if (out->tag[0] != '\0')
    {
        ndefInfo info;
        memset(&info, 0, sizeof(info));

        ReturnCode err = s_ndef->ndefPollerContextInitialization(dev);
        if (err == ST_ERR_NONE)
            err = s_ndef->ndefPollerNdefDetect(&info);

        if (err != ST_ERR_NONE)
        {
            out->ndef = 0;
        }
        else if (info.state == NDEF_STATE_READWRITE || info.state == NDEF_STATE_READONLY)
        {
            out->ndef = info.messageLen > 0 ? 1 : 0;
            out->ndef_len = (int)info.messageLen;

            /* Read the message itself, not just its length. A tag that answers
             * "19 bytes" and a tag that answers "0 bytes" are different
             * objects, and the difference is only visible if the bytes are
             * shown. Truncated to the struct's buffer: the length above still
             * reports the real size. */
            if (out->ndef)
            {
                uint32_t got = 0;
                if (s_ndef->ndefPollerReadRawMessage(out->ndef_raw,
                                                     sizeof(out->ndef_raw),
                                                     &got) == ST_ERR_NONE)
                    out->ndef_raw_len = (int)got;
            }
        }
        else
        {
            out->ndef = 0;
        }
    }
}

/* --- public --------------------------------------------------------------- */

extern "C" bool nfc_scan_start(int host, int sck, int miso, int mosi, int cs, int irq)
{
    if (s_running)
        return true;

    if (sck < 0 || miso < 0 || mosi < 0 || cs < 0)
        return false;

    s_cs = cs;

    /* Chip select is this driver's to drive. It must NOT be handed to the SPI
     * peripheral: the stack pulls it low by hand around the command byte and
     * its data, and a pin routed to an SPI output signal ignores gpio_set_level
     * -- which fails as "the chip never answers", the same shape as a dead
     * part. */
    SPI.configure(host, 5000000);
    SPI.begin(sck, miso, mosi, -1);

    pinMode((uint8_t)cs, OUTPUT);
    digitalWrite((uint8_t)cs, HIGH);

    s_hw = new (std::nothrow) RfalRfST25R3916Class(&SPI, cs, irq, 5000000);
    if (!s_hw)
        return false;

    s_rf = new (std::nothrow) RfalNfcClass(s_hw);
    if (!s_rf)
        return false;

    s_ndef = new (std::nothrow) NdefClass(s_rf);
    if (!s_ndef)
        return false;

    ReturnCode err = s_rf->rfalNfcInitialize();
    if (err != ST_ERR_NONE)
    {
        ESP_LOGE("twatch-nfc", "rfal init %04x", err);
        return false;
    }

    begin_discovery();
    s_running = true;

    ESP_LOGI("twatch-nfc", "rfal up, polling");
    s_handled = false;
    s_last_report_ms = 0;
    s_last_uid[0] = '\0';
    return true;
}

extern "C" int nfc_scan_step(nfc_card_t *out)
{
    if (!s_running)
        return -1;

    s_rf->rfalNfcWorker();

    rfalNfcState st = s_rf->rfalNfcGetState();

    if (st == RFAL_NFC_STATE_ACTIVATED)
    {
        if (s_handled)
            return 0;
        s_handled = true;

        rfalNfcDevice *dev = NULL;
        if (s_rf->rfalNfcGetActiveDevice(&dev) == ST_ERR_NONE && dev)
        {
            describe(dev, out);

            /* The field comes off with the card: holding it on is tens of
             * milliamps, and there is no session to keep. */
            s_rf->rfalNfcDeactivate(true);

            uint32_t now = millis();
            bool repeat = now - s_last_report_ms < REPORT_COOLDOWN_MS &&
                          strcmp(s_last_uid, out->uid) == 0;

            s_last_report_ms = now;
            snprintf(s_last_uid, sizeof(s_last_uid), "%s", out->uid);

            s_handled = false;
            return repeat ? 0 : 1;
        }

        s_handled = false;
        s_rf->rfalNfcDeactivate(true);
        return 0;
    }

    /* Discovery finished without a card, or never started. Either way, poll
     * again: the point of this loop is that a tag can appear at any time. */
    if (st == RFAL_NFC_STATE_IDLE)
    {
        s_handled = false;
        begin_discovery();
    }

    return 0;
}

extern "C" void nfc_scan_stop(void)
{
    /* Drop the field rather than only forgetting the loop: the reader stays
     * powered and its antenna stays cycling until it is told to stop, and the
     * whole point of not running a background poller is that the antenna is
     * quiet when nobody is asking. */
    if (s_rf && s_running)
        s_rf->rfalNfcDeactivate(false);

    s_running = false;
}
