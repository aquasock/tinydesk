/*
 * bleuart.c - a serial console over Bluetooth LE, in the Nordic UART shape.
 *
 * WHY THIS EXISTS. Every other way into this watch competes with the thing being
 * worked on. The USB port is the console *and* the OTG host, and those cannot
 * run at once -- the USB-Serial-JTAG peripheral resets the chip when the host
 * controller drives the same D+/D- lines a device is talking on. WiFi works but
 * wants most of a depleted internal heap: the SSH server refuses to start below
 * 65,536 bytes free and this board has measured 41 KB with the USB host stack
 * linked in. The BLE radio, by contrast, is on core 0, on its own pins, and is
 * already running for the keyboard. A console there is reachable while the USB
 * port is a host, which is exactly when it is needed.
 *
 * THE SERVICE IS NORDIC'S, DELIBERATELY. 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * with a write characteristic in and a notify characteristic out is what every
 * off-the-shelf BLE serial client speaks -- nRF Connect, the Serial Bluetooth
 * Terminal app, the many "BLE UART" bridges. Inventing a private UUID would mean
 * writing a custom client before anything could be tested, which is the wrong
 * order.
 *
 * ONE NOTIFICATION IN FLIGHT AT A TIME. BLE-002 in the reference records the
 * lesson this is built around: an unbounded stream of GATT writes empties the
 * mbuf pool, and the peripheral then drops the link with HCI 0x16 -- a failure
 * that looks like a bad cable. So a single notify is outstanding at a time, the
 * next waits for BLE_GAP_EVENT_NOTIFY_TX, and the wait is bounded so a client
 * that stops accepting cannot wedge the console task.
 *
 * THE RADIO IS SHARED. This registers a GATT service on the host bt_hid.c
 * already brought up, and advertises alongside its central role: the keyboard
 * is a device this watch connects to, and the console is a service this watch
 * offers. Both directions at once is what NimBLE's peripheral role added.
 */
#include "bleuart.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_hs_adv.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "bleuart";

/* Nordic UART Service, byte-reversed as NimBLE's BLE_UUID128_INIT expects. */
static const ble_uuid128_t s_svc_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
static const ble_uuid128_t s_rx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
static const ble_uuid128_t s_tx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

/*
 * RING SIZES ARE A MEMORY BUDGET, NOT A PREFERENCE.
 *
 * This board reports 31 KB of free internal RAM with the USB host stack linked
 * in. The rings plus the mbuf pool were consuming 28 KB of it -- 16 KB tx,
 * 4 KB rx, 8 KB of mbufs -- which leaves nothing for the display, the shell or
 * WiFi. A sustained transfer that crashes after about fifteen seconds while a
 * short burst succeeds is what starvation looks like.
 *
 * 4 KB of tx is twenty 200-byte notifications, which is ample: the consumer is
 * the notify task draining at the connection interval, not a slow human.
 */
#define BLEUART_RX_RING 2048    /* client -> shell; absorbs a write burst */

/* Flow control. The shell drains one byte per poll, so a client that writes a
 * burst can outrun it. Rather than drop silently -- which corrupted a file
 * transfer on the first attempt, 800 of 1000 bytes arriving -- the ring has
 * watermarks: crossing the high one sends XOFF on the notify characteristic and
 * crossing the low one sends XON, which is what a serial console has always
 * done. A client that ignores them still cannot lose data quietly, because a
 * full ring now REFUSES the write instead of accepting and discarding it. */
#define BLEUART_XOFF_AT (BLEUART_RX_RING * 3 / 4)
#define BLEUART_XON_AT  (BLEUART_RX_RING * 1 / 4)
#define BLEUART_XOFF    0x13    /* Ctrl-S */
#define BLEUART_XON     0x11    /* Ctrl-Q */
#define BLEUART_TX_RING 4096     /* shell -> client; 20 notifications */
#define BLEUART_ADV_NAME "TinyDesk"
/*
 * HOW LONG TO WAIT FOR A NOTIFICATION TO LEAVE BEFORE SENDING THE NEXT.
 *
 * This was 200 and was lowered to 40 in an attempt to speed the console up,
 * which was a mistake with a measured consequence: at 40 ms this task produces
 * notifications faster than a 20 ms connection interval can carry them, the
 * mbuf pool empties, and notify starts failing with BLE_HS_ENOMEM (6). The
 * damage is not confined to output -- with no free mbufs the stack cannot
 * RECEIVE either, so a client's writes are dropped before they reach the GATT
 * callback. Observed exactly that: `tx failed=14 lasterr=6` beside `in=0`, and
 * the same write landing (`in=10 out=10`) when sent before any output piled up.
 *
 * This is BLE-002's failure mode, reached from the other direction. The wait is
 * generous on purpose: output a few milliseconds late is invisible, and an
 * empty mbuf pool silently breaks the console in both directions.
 */
#define BLEUART_NOTIFY_WAIT_MS 200

static RingbufHandle_t s_rx;         /* bytes the client sent          */
static RingbufHandle_t s_tx;         /* bytes to notify to the client  */
static SemaphoreHandle_t s_notify_done;
static uint16_t s_tx_val_handle;
static uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;
static uint8_t s_addr_type;
static bool s_up;
static bool s_registered;    /* the GATT service is in the attribute table */
static bool s_tx_task;
static volatile bool s_xoff_sent;     /* the client has been told to pause */
static volatile uint32_t s_rx_refused; /* writes refused for a full ring */
static volatile uint32_t s_rx_bytes;   /* bytes accepted into the ring */
static volatile uint32_t s_rx_out;     /* bytes actually handed to the reader */
static volatile uint32_t s_tx_sent;    /* bytes successfully notified */
static volatile uint32_t s_tx_failed;  /* notifications the stack refused */
static volatile uint16_t s_mtu = 23;   /* negotiated ATT MTU */
static volatile int s_last_nrc;        /* last notify error code */

/* Set while `bleuart fetch` is streaming a file.
 *
 * The display mirror and a file transfer share ONE 4 KB TX ring, and the
 * desktop repaints far faster than the link drains. Letting them compete
 * starves the file -- and because the fetch loop waited on that ring, it spun
 * forever and took the shell task with it: every command typed afterwards did
 * nothing, which reads as "the console died".
 *
 * So the mirror yields for the duration. Losing a few frames of desktop costs
 * nothing here: a client collecting a file is not watching the desktop. */
static volatile bool s_fetching;

/*
 * THE WRITE PATH, which is the only way a byte enters the shell.
 *
 * ctxt->om is an os_mbuf chain and may be longer than one buffer, so the copy is
 * by pktlen and copydata rather than assuming a single flat buffer -- a client
 * that writes a full MTU in one go exercises exactly that.
 */
static int gatt_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt,
                       void *arg)
{
    (void)conn;
    (void)attr;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR)
        return BLE_ATT_ERR_READ_NOT_PERMITTED;

    const uint16_t len = OS_MBUF_PKTLEN(ctxt->om);

    if (len == 0)
        return 0;

    uint8_t buf[256];

    if (len > sizeof(buf))
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    if (os_mbuf_copydata(ctxt->om, 0, len, buf) != 0)
        return BLE_ATT_ERR_UNLIKELY;

    /*
     * A FULL RING REFUSES THE WRITE; IT DOES NOT SWALLOW IT.
     *
     * This used to accept the bytes and drop them with a log line, which is
     * silent data loss: a 1000-byte file transfer arrived as 800 bytes and the
     * only trace was a warning nobody was reading. Returning
     * BLE_ATT_ERR_INSUFFICIENT_RES makes a write-with-response client's stack
     * report a failure, so it can retry -- and the ATT layer only acknowledges
     * after this callback returns, so nothing has been committed yet.
     *
     * A write-WITHOUT-response client gets no error, which is why XOFF exists:
     * crossing the high watermark sends Ctrl-S on the notify characteristic, as
     * a serial console always has. A client that honours it never reaches the
     * refusal. A client that honours neither is the one case that can still lose
     * data, and that is stated in the header rather than discovered.
     *
     * Blocking here instead would be simpler and is the wrong trade: this runs in
     * NimBLE's host task, and BLE-004 records what starving that task costs.
     */
    if (xRingbufferSend(s_rx, buf, len, 0) != pdTRUE)
    {
        s_rx_refused++;
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    s_rx_bytes += len;

    size_t held = 0;
    vRingbufferGetInfo(s_rx, NULL, NULL, NULL, NULL, &held);

    if (held >= BLEUART_XOFF_AT && !s_xoff_sent)
    {
        s_xoff_sent = true;
        const uint8_t c = BLEUART_XOFF;
        xRingbufferSend(s_tx, &c, 1, 0);
    }

    return 0;
}

static const struct ble_gatt_svc_def s_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                /* RX: the client writes here. Write-without-response is what
                 * serial clients use by default; with-response is accepted too
                 * so a client that asks for an ack still works. */
                .uuid = &s_rx_uuid.u,
                .access_cb = gatt_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                /* TX: the shell's output, notified. */
                .uuid = &s_tx_uuid.u,
                .access_cb = gatt_access,
                .val_handle = &s_tx_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 },
        },
    },
    { 0 },
};

static void advertise(void);

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0)
        {
            s_conn = event->connect.conn_handle;
            s_mtu = ble_att_mtu(s_conn);    /* whatever this link agreed */
            ESP_LOGI(TAG, "console client connected (handle %d, mtu %u)",
                     s_conn, (unsigned)s_mtu);
        }
        else
        {
            advertise();        /* the attempt failed: keep offering */
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "console client gone (reason %d)", event->disconnect.reason);
        s_conn = BLE_HS_CONN_HANDLE_NONE;
        advertise();            /* always be reachable again */
        return 0;

    case BLE_GAP_EVENT_NOTIFY_TX:
        /* The in-flight notification has left. Release the sender. */
        xSemaphoreGive(s_notify_done);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertise();
        return 0;

    case BLE_GAP_EVENT_MTU:
        s_mtu = event->mtu.value;
        ESP_LOGI(TAG, "mtu now %u", (unsigned)s_mtu);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "client subscribed to the console output: %d",
                 event->subscribe.cur_notify);
        return 0;

    default:
        return 0;
    }
}

static void advertise(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* THE NAME HAS TO BE IN THE ADVERTISEMENT ITSELF.
     *
     * ble_svc_gap_device_name_set() sets the GAP characteristic's value, which a
     * client can only read AFTER connecting -- it does not put the name on air.
     * Without these two lines the watch advertises correctly and is invisible to
     * every scan-by-name, which reads as "the radio is not advertising" and cost
     * a debug round trip.
     */
    fields.name = (const uint8_t *)BLEUART_ADV_NAME;
    fields.name_len = sizeof(BLEUART_ADV_NAME) - 1;
    fields.name_is_complete = 1;

    /* The full 128-bit service UUID in the advertisement, so a client that
     * scans for the Nordic UART service finds this without being told the name
     * -- and so the name is not the only way to identify it. */
    fields.uuids128 = (ble_uuid128_t *)&s_svc_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);

    if (rc != 0)
    {
        ESP_LOGE(TAG, "adv fields: %d", rc);
        return;
    }

    struct ble_gap_adv_params params;
    memset(&params, 0, sizeof(params));
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_addr_type, NULL, BLE_HS_FOREVER, &params, gap_event, NULL);

    /* EALREADY means an advertisement is already running, which after a
     * disconnect is the normal state rather than an error. */
    if (rc != 0 && rc != BLE_HS_EALREADY)
        ESP_LOGW(TAG, "adv start: %d", rc);
}

/*
 * Drain the TX ring to the client, one notification at a time.
 *
 * Runs as its own task so the shell's console task never blocks on the radio,
 * and so a client that stops reading costs a bounded wait rather than a stalled
 * shell. When no client is connected the bytes are discarded rather than queued:
 * a console that accumulates output for a client that is not there would fill
 * the ring and then start refusing, and the ones that matter are the next ones.
 */
static void tx_task(void *arg)
{
    (void)arg;

    for (;;)
    {
        /*
         * CHUNK BY THE NEGOTIATED MTU, NOT BY A GUESS.
         *
         * This asked for up to 240 bytes per notification regardless of what the
         * link had agreed. A notification larger than MTU-3 cannot be sent: the
         * stack refuses it, this task logged a warning and DROPPED the bytes, and
         * the client saw a short file. That is why reading a 276-byte log
         * produced fragments while a 4,000-byte WRITE -- which is chunked by the
         * client's own MTU handling -- was byte-exact.
         *
         * s_mtu is updated on connect and on BLE_GAP_EVENT_MTU.
         */
        const uint16_t mtu = s_mtu > 23 ? s_mtu : 23;
        size_t chunk_max = mtu > 3 ? (size_t)(mtu - 3) : 20;
        if (chunk_max > 240)
            chunk_max = 240;        /* no point making notifications huge */

        size_t len = 0;
        uint8_t *p = xRingbufferReceiveUpTo(s_tx, &len, pdMS_TO_TICKS(1000), chunk_max);

        if (!p)
            continue;

        if (s_conn == BLE_HS_CONN_HANDLE_NONE)
        {
            vRingbufferReturnItem(s_tx, p);
            continue;
        }

        struct os_mbuf *om = ble_hs_mbuf_from_flat(p, (uint16_t)len);

        vRingbufferReturnItem(s_tx, p);

        if (!om)
        {
            ESP_LOGW(TAG, "no mbuf for %u bytes", (unsigned)len);
            continue;
        }

        xSemaphoreTake(s_notify_done, 0);       /* clear any stale signal */

        const int rc = ble_gatts_notify_custom(s_conn, s_tx_val_handle, om);

        if (rc != 0)
        {
            /* om is consumed on success only. Counted rather than only logged:
             * this is silent data loss on the read path and it has to be
             * visible in `bleuart`, not buried in a log nobody can read. */
            s_tx_failed++;
            s_last_nrc = rc;

            /* OUT OF MBUFS MEANS PAUSE, AND NOTHING ELSE.
             *
             * THIS MBUF MUST NOT BE FREED HERE. ble_gatts_notify_custom() sets
             * txom = NULL unconditionally after calling ble_att_clt_tx_notify(),
             * so NimBLE takes ownership of the mbuf whether the notify succeeds
             * or fails -- and frees it itself on the failure path.
             *
             * An earlier revision freed it on BLE_HS_ENOMEM, which is a double
             * free. The visible symptom was not a double-free report but pool
             * CORRUPTION: the next allocation returned a block whose om_omp
             * pointer was garbage, and ble_hs_mbuf_gen_pkt() dereferenced it:
             *
             *   Crashed task: 'bleuart'
             *   #0 ble_hs_mbuf_gen_pkt  ble_hs_mbuf.c:42
             *   #3 tx_task              bleuart.c:353
             *
             * It appeared under load, which is why a 4 KB transfer at gentle
             * pacing passed for hours and typing into minicom killed the board in
             * seconds. The pause below is the whole backoff; ownership stays with
             * NimBLE. */
            if (rc == BLE_HS_ENOMEM)
                vTaskDelay(pdMS_TO_TICKS(20));

            continue;
        }

        s_tx_sent += len;

        /* BLE-002: exactly one notification outstanding, so the mbuf pool cannot
         * be drained by this path. A client that never acknowledges costs this
         * task a bounded delay per chunk, not the link. */
        xSemaphoreTake(s_notify_done, pdMS_TO_TICKS(BLEUART_NOTIFY_WAIT_MS));
    }
}

bool bleuart_register(void)
{
    if (s_registered)
        return true;

    s_rx = xRingbufferCreate(BLEUART_RX_RING, RINGBUF_TYPE_BYTEBUF);
    s_tx = xRingbufferCreate(BLEUART_TX_RING, RINGBUF_TYPE_BYTEBUF);
    s_notify_done = xSemaphoreCreateBinary();

    if (!s_rx || !s_tx || !s_notify_done)
    {
        ESP_LOGE(TAG, "no memory for the rings");
        return false;
    }

    /*
     * THE STANDARD SERVICES FIRST, AND ONLY HERE.
     *
     * ble_svc_gap_init() and ble_svc_gatt_init() assert SYSINIT_ASSERT_ACTIVE(),
     * which is true only between nimble_port_init() and the host task starting.
     * Calling them from the sync callback panics the board -- the backtrace from
     * the coredump named it in one run after two wrong guesses. So registration
     * is split from advertising: this half runs before the host starts, and only
     * bleuart_start()'s advertising half runs after the sync.
     */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_svcs);

    if (rc == 0)
        rc = ble_gatts_add_svcs(s_svcs);

    if (rc != 0)
    {
        ESP_LOGE(TAG, "gatt service: %d", rc);
        return false;
    }

    ble_svc_gap_device_name_set(BLEUART_ADV_NAME);

    s_registered = true;
    return true;
}

bool bleuart_start(void)
{
    if (s_up)
        return true;

    if (!s_registered)
    {
        ESP_LOGE(TAG, "started before the service was registered");
        return false;
    }

    if (ble_hs_id_infer_auto(0, &s_addr_type) != 0)
    {
        ESP_LOGE(TAG, "no address for advertising");
        return false;
    }

    if (!s_tx_task &&
        xTaskCreate(tx_task, "bleuart", 3072, NULL, 3, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "no task");
        return false;
    }
    s_tx_task = true;

    advertise();

    s_up = true;
    ESP_LOGI(TAG, "console over BLE: service 6E400001, advertising as \"%s\"",
             BLEUART_ADV_NAME);
    return true;
}

bool bleuart_ready(void)
{
    return s_up;
}

int bleuart_read(void)
{
    if (!s_up)
        return -1;

    size_t len = 0;
    uint8_t *p = xRingbufferReceiveUpTo(s_rx, &len, 0, 1);

    if (!p)
        return -1;

    const int b = p[0];
    s_rx_out++;
    vRingbufferReturnItem(s_rx, p);

    /* Release a paused client once the ring has drained enough. */
    if (s_xoff_sent)
    {
        size_t held = 0;
        vRingbufferGetInfo(s_rx, NULL, NULL, NULL, NULL, &held);

        if (held <= BLEUART_XON_AT)
        {
            s_xoff_sent = false;
            const uint8_t c = BLEUART_XON;
            xRingbufferSend(s_tx, &c, 1, 0);
        }
    }

    return b;
}

/*
 * OUTPUT IS NON-BLOCKING, AND THAT IS DELIBERATE.
 *
 * Blocking here was tried and rejected: this is called from the console and
 * renderer path, so stalling it for a slow client stalls the desktop, which then
 * repaints -- reading a 4,000-byte file back produced 17,700 bytes of screen
 * traffic with not one of the file's 100 line markers intact. Backpressure
 * belongs on the INPUT side (where a lost byte corrupts a file quietly) and must
 * not be added here, where a lost byte only shortens what a diagnostic prints.
 *
 * So a full ring returns a short count. The ring is large enough for a burst of
 * ordinary command output, and a client that reads promptly never sees a drop.
 *
 * THE RETURN VALUE IS A BYTE COUNT, AND xRingbufferSend() DOES NOT RETURN ONE.
 * It returns pdTRUE/pdFALSE, and on a byte buffer it is all-or-nothing. This
 * function used to return that flag, so every successful write reported ONE
 * byte. td_render_flush() then resent out+1, out+2, ... each time accepted in
 * full, until the ring filled; the next write returned 0, the renderer dropped
 * the frame, marked the link down and repainted the whole screen 500 ms later,
 * for ever. Captured on 2026-11-15: every BLE repaint was one 1,024-byte render
 * buffer followed by copies of it shifted by 1, 2 and 3 bytes, 4,090 bytes in
 * all, ~24 KB/s on an idle desktop. The "17,700 bytes" above is the same fault.
 * So: send what fits, and report exactly that.
 */
int bleuart_write(const uint8_t *buf, int len)
{
    if (!s_up || !buf || len <= 0)
        return 0;

    size_t n = xRingbufferGetCurFreeSize(s_tx);

    if (n > (size_t)len)
        n = (size_t)len;

    if (n == 0 || xRingbufferSend(s_tx, buf, n, 0) != pdTRUE)
        return 0;

    return (int)n;
}

/* Blocking variant, used only by `fetch`: wait for ring space instead of
 * dropping. The console path needs the non-blocking write; a file transfer
 * needs the opposite, because a dropped chunk is a hole in the file.
 *
 * Returns the bytes queued (all of them: a byte-buffer send is all-or-nothing),
 * or 0 on timeout, so the caller can bound its wait. len must not exceed the
 * ring, which `fetch`'s 256-byte chunks never do. */
static int bleuart_write_wait(const uint8_t *buf, int len, TickType_t ticks)
{
    if (!s_up || !buf || len <= 0)
        return 0;

    return xRingbufferSend(s_tx, buf, (size_t)len, ticks) == pdTRUE ? len : 0;
}

/* --- the hooks the shared layer calls ------------------------------------
 *
 * Strong definitions for the weak defaults: bt_hid.c's sync callback brings the
 * service up, and hal_mux.c's alternate serial-2 transport is this stream. The
 * shared files have no idea any of this exists, which is the point -- another
 * port supplies its own or links the empty defaults.
 */

void board_ble_uart_register(void)
{
    bleuart_register();
}

void board_ble_uart_on_sync(void)
{
    bleuart_start();
}

bool board_serial2_alt_open(void)
{
    /* Not started here: the service comes up with the radio, in bt_hid.c's sync
     * callback, because a GATT service registered before the host syncs has no
     * attribute table to go into. This reports whether it is there. */
    if (!bleuart_ready())
    {
        ESP_LOGW(TAG, "display 2 wants BLE but the radio is not up "
                      "(set bt.enable = 1)");
        return false;
    }
    return true;
}

int board_serial2_alt_read(void)
{
    return bleuart_read();
}

int board_serial2_alt_write(const uint8_t *buf, int len)
{
    /* Yield the TX ring to an in-progress file transfer. Reporting len keeps the
     * caller's "accepted" contract intact; these are mirror bytes, not data. */
    if (s_fetching)
        return len;

    return bleuart_write(buf, len);
}

/* --- `bleuart` -- what the console service believes -----------------------
 *
 * The same lesson the status strip taught: a feature whose only output is a log
 * line the console cannot show is a feature nobody can diagnose. This reports
 * the state that decides whether a client can connect, and retries advertising,
 * because "it is up but not advertising" and "it never started" look identical
 * from a PC that cannot see it.
 */
#include <stdio.h>
#include <stdlib.h>

#include "tdsh.h"

static int cmd_bleuart(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    if (argc >= 2 && strcmp(argv[1], "adv") == 0)
    {
        advertise();
        printf("bleuart: advertising restarted\n");
        return 0;
    }

    /*
     * `bleuart drop` -- terminate the current link, whoever owns it.
     *
     * THIS EXISTS BECAUSE A DEAD CLIENT IS INVISIBLE FROM HERE AND EXPENSIVE
     * FROM THERE. When a client disconnects without a clean shutdown the watch
     * holds the link until the BLE supervision timeout (about 20 seconds), and
     * for that whole window it neither advertises nor accepts a new connection.
     * From the PC that is indistinguishable from a dead watch, and the only cure
     * was a reboot -- which happened four times in one session.
     *
     * Terminating from this side ends it immediately and re-advertises, so a new
     * client can attach at once.
     */
    /*
     * `bleuart fetch <path>` -- stream a file over the link, bypassing the
     * terminal.
     *
     * WHY THIS EXISTS. Reading a file back with `cat` does not work, and the
     * reason is not bandwidth: the console is a DISPLAY SOURCE, so the shell's
     * output is rendered into a 40x26 terminal and the desktop's mirror sends
     * only cells that changed. A file scrolls far faster than that diff can
     * travel, so the top of it is gone before it is transmitted -- measured as
     * 11 of 20 lines arriving, every one of them from the END of the output.
     *
     * This writes the file's bytes straight into the notification ring instead:
     * no screen, no wrapping, no diff. The client receives the exact file.
     *
     * It prints nothing to the terminal on purpose -- progress messages would
     * interleave with the data. The client knows how many bytes it asked for;
     * the console's own `tx sent` counter confirms delivery.
     */
    if (argc >= 3 && strcmp(argv[1], "fetch") == 0)
    {
        FILE *f = fopen(argv[2], "rb");

        if (!f)
        {
            printf("bleuart: cannot open %s\n", argv[2]);
            return 1;
        }

        unsigned long total = 0;
        bool stuck = false;

        s_fetching = true;

        while (!stuck)
        {
            uint8_t chunk[256];
            const size_t n = fread(chunk, 1, sizeof(chunk), f);

            if (n == 0)
                break;

            size_t off = 0;
            int stalls = 0;

            /* BLOCK FOR RING SPACE, WITH A CEILING.
             *
             * The old loop polled every 5 ms and never gave up. When the ring
             * stayed full it looped forever and the shell never returned to a
             * prompt, so every later command was silently swallowed. A
             * transfer that cannot make progress now ends and says so instead
             * of hanging the only channel available to report it. */
            while (off < n)
            {
                const int w = bleuart_write_wait(chunk + off, (int)(n - off),
                                                 pdMS_TO_TICKS(500));

                if (w > 0)
                {
                    off += (size_t)w;
                    stalls = 0;
                    continue;
                }

                if (++stalls > 4)
                {
                    stuck = true;
                    break;
                }
            }

            if (stuck)
                break;

            total += n;
        }

        fclose(f);

        /* Let the notify task empty the ring before the shell prints anything
         * that would interleave with the last of the data. */
        {
            int i;

            for (i = 0; i < 40; i++)
            {
                size_t held = 0;

                vRingbufferGetInfo(s_tx, NULL, NULL, NULL, NULL, &held);

                if (held == 0)
                    break;

                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }

        s_fetching = false;

        printf("bleuart: fetched %lu bytes from %s%s\n", total, argv[2],
               stuck ? " (client stopped reading)" : "");
        return 0;
    }

    if (argc >= 2 && strcmp(argv[1], "drop") == 0)
    {
        if (s_conn == BLE_HS_CONN_HANDLE_NONE)
        {
            printf("bleuart: no connection to drop\n");
            advertise();
            return 0;
        }

        const int rc = ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);

        printf("bleuart: terminate(%d) -> %d\n", (int)s_conn, rc);
        s_conn = BLE_HS_CONN_HANDLE_NONE;
        advertise();
        return 0;
    }

    printf("started=%d conn=%d\n", s_up ? 1 : 0, (int)s_conn);
    printf("tx_handle=%u addr_type=%u\n", (unsigned)s_tx_val_handle, (unsigned)s_addr_type);

    if (s_rx)
    {
        size_t used = 0;
        UBaseType_t items = 0;
        vRingbufferGetInfo(s_rx, NULL, NULL, NULL, NULL, &used);
        (void)items;
        printf("rx ring used=%u refused=%lu in=%lu out=%lu\n",
           (unsigned)used, (unsigned long)s_rx_refused,
           (unsigned long)s_rx_bytes, (unsigned long)s_rx_out);
    printf("tx sent=%lu failed=%lu lasterr=%d mtu=%u\n",
           (unsigned long)s_tx_sent, (unsigned long)s_tx_failed,
           s_last_nrc, (unsigned)s_mtu);
    }
    if (s_tx)
    {
        size_t used = 0;
        vRingbufferGetInfo(s_tx, NULL, NULL, NULL, NULL, &used);
        printf("tx ring used=%u\n", (unsigned)used);
    }

    return 0;
}

static const tdsh_command_t s_bleuart_cmd = {
    "bleuart",
    "bleuart [adv|drop|fetch <path>]",
    "BLE serial console: state, restart advertising, drop link, send a file",
    cmd_bleuart,
    TDSH_CMD_INTERACTIVE,
};

bool bleuart_register_command(void)
{
    return tdsh_register_command(&s_bleuart_cmd) == 0;
}

/* The radio has a reason to run that is not a keyboard: this console is a
 * peripheral the watch OFFERS, so it must be up at boot whether or not a
 * keyboard is paired. Without this the radio stayed off until someone opened
 * the Bluetooth app, and the console was invisible until they did. */
bool board_bt_want_radio(void)
{
    return true;
}
