/*
 * nfc.c - the ST25R3916 NFC reader: its rail, its bus, and its identity.
 *
 * See nfc.h for the wiring. This file is the first step on the part and stops
 * deliberately short of protocol work: it is the answer to "is the chip there
 * and can we talk to it on the bus the SD card shares", which every later step
 * depends on and which is worth one flash cycle on its own.
 *
 * THE WIRE PROTOCOL, from ST's driver (the RFAL stack LilyGo's firmware uses):
 *
 *   write register : (reg & 0x3F)          then data
 *   read register  : (reg & 0x3F) | 0x40   then one dummy byte, then data
 *   FIFO load      : 0x80                  then data
 *   FIFO read      : 0xBF                  then data
 *   direct command : 0xC0 | cmd
 *
 * SPI mode 1, MSB first, up to 5 MHz. The one-byte read shape is why a
 * register read is two bytes wide: the chip cannot drive MISO while it is
 * still receiving the command, so the first byte clocked back is meaningless
 * and the value is the second.
 */

#include "nfc.h"
#include "power.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tdsh.h"
#include "tdsh_board.h"

#define TAG "twatch-nfc"

/* Pins, from the board's own Arduino variant (pins_arduino.h): NFC_CS 4,
 * NFC_INT 5, and MOSI/MISO/SCK shared with the SD card and the LoRa radio. */
#define NFC_CS_DEFAULT   4
#define NFC_INT_DEFAULT  5
#define NFC_HOST_DEFAULT 2   /* the second SPI bus, same default as sd.spi_host */
#define NFC_HZ           5000000

/* AXP2101. DLDO1 feeds the reader (BRD-003). Its voltage is register 0x99 in
 * 100 mV steps from 500 mV, and its enable is bit 7 of 0x90 -- the same
 * register that carries ALDO1..4 in bits 0..3 and BLDO2 in bit 5. Both are
 * read-modify-written: 0x90 is not this module's register to assign. */
#define PMU_DLDO1_VOLT   0x99
#define PMU_LDO_EN       0x90
#define PMU_DLDO1_EN_BIT 7
#define PMU_DLDO1_3300MV 28

#define ST25_CMD_SET_DEFAULT 0xC1
#define ST25_REG_IO_CONF1    0x00
#define ST25_REG_IO_CONF2    0x01
#define ST25_REG_OP_CONTROL  0x02
#define ST25_REG_IC_IDENTITY 0x3F

#define ST25_IO_CONF2_io_18   (1u << 2)
#define ST25_IO_CONF2_miso_pd1 (1u << 3)
#define ST25_IO_CONF2_miso_pd2 (1u << 4)
#define ST25_REG_IC_IDENTITY_ic_type_mask (0x1Fu << 3)
#define ST25_IC_ST25R3916     (5u << 3)
#define ST25_IC_ST25R3916B    (6u << 3)

static spi_device_handle_t s_nfc;
static int s_cs = -1;
static int s_int = -1;
static int s_host = -1;
static int s_sck = -1;
static int s_mosi = -1;
static int s_miso = -1;

/* What the probe run from link_init() found.
 *
 * Kept because the boot log line is the only record of it and the Log Viewer
 * is forty columns wide, pins itself to the newest line, and sits behind an
 * app that has to be opened to read at all. If the reader works when asked
 * from the shell but not at boot, that difference is the finding -- and it is
 * invisible unless the two answers are kept apart. */
static bool s_boot_ran;
static bool s_boot_ok;

/* --- the rail ------------------------------------------------------------ */

bool board_nfc_rail_on(void)
{
    uint8_t v = 0, en = 0;

    if (!board_pmu_read(PMU_DLDO1_VOLT, &v) || !board_pmu_read(PMU_LDO_EN, &en))
    {
        ESP_LOGE(TAG, "no PMU for the nfc rail");
        return false;
    }

    if ((v & 0x1F) != PMU_DLDO1_3300MV &&
        !board_pmu_write(PMU_DLDO1_VOLT, (uint8_t)((v & 0xE0) | PMU_DLDO1_3300MV)))
    {
        ESP_LOGE(TAG, "could not set DLDO1 voltage");
        return false;
    }

    if (!(en & (1u << PMU_DLDO1_EN_BIT)) &&
        !board_pmu_write(PMU_LDO_EN, (uint8_t)(en | (1u << PMU_DLDO1_EN_BIT))))
    {
        ESP_LOGE(TAG, "could not enable DLDO1");
        return false;
    }

    return true;
}

/* --- the bus ------------------------------------------------------------
 *
 * The SD watcher owns this bus in the ordinary case and may not have started
 * yet, so this initializes it too and treats "already initialized" as success.
 * The pins come from the sd.* keys rather than being repeated, because it is
 * one bus described once -- and a port that hardcodes the same pin twice is a
 * port that will disagree with itself eventually.
 */
static bool nfc_bus(void)
{
    if (s_nfc)
        return true;

    int host = tdsh_board_int("nfc.spi_host", NFC_HOST_DEFAULT);
    int sclk = tdsh_board_int("sd.sclk", 35);
    int mosi = tdsh_board_int("sd.mosi", 34);
    int miso = tdsh_board_int("sd.miso", 33);
    s_cs = tdsh_board_int("nfc.cs", NFC_CS_DEFAULT);
    s_int = tdsh_board_int("nfc.int", NFC_INT_DEFAULT);
    s_host = host;
    s_sck = sclk;
    s_mosi = mosi;
    s_miso = miso;

    if (host < 0 || sclk < 0 || mosi < 0 || miso < 0 || s_cs < 0)
    {
        ESP_LOGE(TAG, "nfc bus pins are not configured");
        return false;
    }

    spi_bus_config_t bus = {
        .mosi_io_num = mosi,
        .miso_io_num = miso,
        .sclk_io_num = sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    esp_err_t err = spi_bus_initialize((spi_host_device_t)host, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "shared spi bus did not start: %s", esp_err_to_name(err));
        return false;
    }

    /* CHIP SELECT IS DRIVEN BY HAND, NOT BY THE SPI PERIPHERAL.
     *
     * This is not a style choice. A pin handed to the SPI driver as its CS is
     * routed to a peripheral output signal through the GPIO matrix, and
     * gpio_set_level() has no effect on such a pin. ST's stack pulls CS low
     * itself around the command byte and its data (digitalWrite(cs, LOW) ...
     * digitalWrite(cs, HIGH)), so the pin has to stay a plain GPIO -- and the
     * failure if it does not is that the chip never answers, which is
     * indistinguishable from a dead part or a wrong mode. */
    spi_device_interface_config_t dev = {
        .mode = 1,                 /* CPOL 0, CPHA 1 -- the part's only mode */
        .clock_speed_hz = NFC_HZ,
        .spics_io_num = -1,
        .queue_size = 1,
    };

    gpio_config_t cs_out = {
        .pin_bit_mask = (1ULL << s_cs),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cs_out);
    gpio_set_level((gpio_num_t)s_cs, 1);

    err = spi_bus_add_device((spi_host_device_t)host, &dev, &s_nfc);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "could not add nfc cs %d: %s", s_cs, esp_err_to_name(err));
        s_nfc = NULL;
        return false;
    }

    return true;
}

/* One SPI transaction: command byte and data, full duplex.
 *
 * Short transfers use the transaction's own four-byte registers, which the
 * driver accepts without a DMA-capable buffer; longer ones get a heap
 * allocation flagged MALLOC_CAP_DMA, because with DMA enabled the driver will
 * not silently copy a stack buffer. Written for both sizes now so the protocol
 * code above this does not have to care which it is. */
static bool xfer(const uint8_t *tx, uint8_t *rx, size_t n)
{
    if (!s_nfc || n == 0)
        return false;

    spi_transaction_t t = { 0 };
    t.length = n * 8;

    uint8_t *tbuf = NULL;
    uint8_t *rbuf = NULL;

    if (n <= sizeof(t.tx_data))
    {
        t.flags |= SPI_TRANS_USE_TXDATA;
        if (tx)
            memcpy(t.tx_data, tx, n);
        if (rx)
            t.flags |= SPI_TRANS_USE_RXDATA;
    }
    else
    {
        tbuf = heap_caps_malloc(n, MALLOC_CAP_DMA);
        if (!tbuf)
        {
            ESP_LOGE(TAG, "no dma buffer for %u bytes", (unsigned)n);
            return false;
        }
        memset(tbuf, 0, n);
        if (tx)
            memcpy(tbuf, tx, n);
        t.tx_buffer = tbuf;

        if (rx)
        {
            rbuf = heap_caps_malloc(n, MALLOC_CAP_DMA);
            if (!rbuf)
            {
                heap_caps_free(tbuf);
                return false;
            }
            t.rx_buffer = rbuf;
        }
    }

    gpio_set_level((gpio_num_t)s_cs, 0);
    esp_err_t err = spi_device_transmit(s_nfc, &t);
    gpio_set_level((gpio_num_t)s_cs, 1);

    if (err == ESP_OK && rx)
    {
        if (n <= sizeof(t.rx_data))
            memcpy(rx, t.rx_data, n);
        else
            memcpy(rx, rbuf, n);
    }

    if (tbuf)
        heap_caps_free(tbuf);
    if (rbuf)
        heap_caps_free(rbuf);

    return err == ESP_OK;
}

static bool nfc_cmd(uint8_t cmd)
{
    uint8_t b = (uint8_t)(0xC0 | (cmd & 0x3F));
    return xfer(&b, NULL, 1);
}

bool nfc_read_reg(uint8_t reg, uint8_t *v)
{
    uint8_t tx[2] = { (uint8_t)((reg & 0x3F) | 0x40), 0x00 };
    uint8_t rx[2] = { 0 };

    if (!xfer(tx, rx, sizeof(tx)))
        return false;

    *v = rx[1];
    return true;
}

bool nfc_write_reg(uint8_t reg, uint8_t v)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x3F), v };
    return xfer(tx, NULL, sizeof(tx));
}

/* --- the identity --------------------------------------------------------
 *
 * Separate from the logging because the shell command asks the same question
 * and prints more of the answer than a log line has room for.
 */
struct nfc_ident
{
    uint8_t id;      /* register 0x3F as read */
    uint8_t io2;     /* IO_CONF2, read back to prove writes land */
    uint8_t op;      /* OP_CONTROL, 0x00 after SET_DEFAULT */
    int irq;         /* the interrupt pin's level, or -1 if not configured */
    bool known;      /* the identity is an ST25R3916 family part */
    bool is_b;       /* B silicon: different analog table, same registers */
    unsigned rev;
    bool rail;       /* DLDO1 came up */
    bool bus;        /* the SPI device was added */
    bool answered;   /* SET_DEFAULT and a stable identity read came back */
};

static bool nfc_identify(struct nfc_ident *out)
{
    uint8_t id = 0, id2 = 0;

    memset(out, 0, sizeof(*out));
    out->irq = -1;

    if (!board_nfc_rail_on())
        return false;
    out->rail = true;

    /* DLDO1 is a switched regulator, not a tie to VBAT: the chip is not
     * addressable until the rail has settled. */
    vTaskDelay(pdMS_TO_TICKS(20));

    if (!nfc_bus())
        return false;
    out->bus = true;

    if (!nfc_cmd(ST25_CMD_SET_DEFAULT))
    {
        ESP_LOGE(TAG, "nfc cs %d: no answer", s_cs);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(3));

    /* Raise the MISO drive and enable both pull-downs before trusting a read.
     * ST's own init does exactly this, in this order. On a bus whose MISO is
     * shared with two other parts, an undriven line reads as whatever the
     * parking state says -- which looks like the wrong chip rather than the
     * wrong line. */
    if (!nfc_write_reg(ST25_REG_IO_CONF2, ST25_IO_CONF2_io_18))
    {
        ESP_LOGE(TAG, "nfc io_conf2 write failed");
        return false;
    }
    (void)nfc_read_reg(ST25_REG_IO_CONF2, &out->io2);

    /* Two reads across two transactions: one value can be a coincidence, a
     * repeated one cannot. */
    if (!nfc_read_reg(ST25_REG_IC_IDENTITY, &id) ||
        !nfc_read_reg(ST25_REG_IC_IDENTITY, &id2))
    {
        ESP_LOGE(TAG, "nfc identity read failed");
        return false;
    }

    (void)nfc_read_reg(ST25_REG_OP_CONTROL, &out->op);

    if (s_int >= 0)
    {
        gpio_config_t in = {
            .pin_bit_mask = (1ULL << s_int),
            .mode = GPIO_MODE_INPUT,
        };

        /* gpio_config() logs the whole pin state at INFO. That is three lines
         * of the log ring's sixty-four, for a pin whose level is printed on
         * the next line anyway. Quieter while it runs, then put back. */
        esp_log_level_t lvl = esp_log_level_get("gpio");
        esp_log_level_set("gpio", ESP_LOG_WARN);
        esp_err_t gerr = gpio_config(&in);
        esp_log_level_set("gpio", lvl);

        if (gerr == ESP_OK)
            out->irq = gpio_get_level((gpio_num_t)s_int);
    }

    if (id != id2)
    {
        ESP_LOGE(TAG, "nfc unstable id %02x/%02x", id, id2);
        return false;
    }

    out->id = id;
    out->rev = id & 0x07;
    out->known = (id & ST25_REG_IC_IDENTITY_ic_type_mask) == ST25_IC_ST25R3916 ||
                 (id & ST25_REG_IC_IDENTITY_ic_type_mask) == ST25_IC_ST25R3916B;
    out->is_b = (id & ST25_REG_IC_IDENTITY_ic_type_mask) == ST25_IC_ST25R3916B;
    out->answered = true;
    return out->known;
}

bool nfc_probe(void)
{
    struct nfc_ident n;

    s_boot_ran = true;
    s_boot_ok = false;

    if (!nfc_identify(&n))
    {
        /* 0x00 and 0xFF are what a floating MISO produces, so the value is
         * worth printing: it separates "the chip is not there" from "the chip
         * is there and the line is not". */
        if (n.rail && n.bus && !n.answered)
            ESP_LOGE(TAG, "nfc no id");
        return false;
    }

    ESP_LOGI(TAG, "nfc up id %02x %s rev%u irq%d op%02x",
             n.id, n.is_b ? "3916B" : "3916", n.rev, n.irq, n.op);
    s_boot_ok = true;
    return true;
}

/* --- the shell command --------------------------------------------------- */

/* One card, in two short lines. Forty columns is the desktop terminal's whole
 * width, so the UID goes on its own line rather than trailing a description
 * that would push it off the edge. */
/* The message as hex, eight bytes to a line, so it fits the forty columns
 * without wrapping. A length above the buffer's size is called out rather than
 * silently truncated -- a tag that holds more than is shown is a different
 * fact from a tag that holds exactly what is shown. */
static void print_ndef_hex(const nfc_card_t *c)
{
    char line[40];

    for (int i = 0; i < c->ndef_raw_len; i++)
    {
        int o = (i % 8) * 2;
        if (o == 0)
            line[0] = '\0';
        line[o] = "0123456789abcdef"[(c->ndef_raw[i] >> 4) & 0xF];
        line[o + 1] = "0123456789abcdef"[c->ndef_raw[i] & 0xF];
        line[o + 2] = '\0';

        if (i % 8 == 7 || i == c->ndef_raw_len - 1)
            printf("  %s\n", line);
    }

    if (c->ndef_len > c->ndef_raw_len && c->ndef_raw_len > 0)
        printf("  ...of %d bytes\n", c->ndef_len);
}

/*
 * A single well-known URI or Text record, decoded; nothing else.
 *
 * This is not an NDEF parser and does not pretend to be one. A tag written by
 * a phone is nearly always one of these two record types, and reading one in
 * the shell is what makes a tag's presence checkable rather than merely
 * detectable. Anything else prints as hex and stops, which is honest.
 */
static void print_ndef_text(const nfc_card_t *c)
{
    static const char *const URI_PREFIX[] = {
        "", "http://www.", "https://www.", "http://", "https://", "tel:",
        "mailto:", "ftp://anonymous:anonymous@", "ftp://ftp.", "ftps://",
        "sftp://", "smb://", "nfs://", "ftp://", "dav://", "news:",
        "telnet://", "imap:", "rtsp://", "urn:", "pop:", "sip:", "sips:",
        "tftp:", "btspp://", "btl2cap://", "btgoep://", "tcpobex://",
        "irdaobex://", "file://", "urn:epc:id:", "urn:epc:tag:",
        "urn:epc:pat:", "urn:epc:raw:", "urn:epc:", "urn:nfc:",
    };

    int n = c->ndef_raw_len;
    if (n < 3)
        return;

    const uint8_t *b = c->ndef_raw;
    if ((b[0] & 0x10) == 0)      /* not a short record: length is 4 bytes */
        return;
    if ((b[0] & 0x07) != 0x01)   /* not TNF well-known */
        return;

    int type_len = b[1];
    int pay_len = b[2];
    int type_at = 3;
    int pay_at = type_at + type_len;
    if (type_len != 1 || pay_at >= n || pay_at + pay_len > n || pay_len < 1)
        return;

    char out[48];
    int o = 0;

    if (b[type_at] == 'U')
    {
        unsigned pfx = b[pay_at];
        const char *p = pfx < sizeof(URI_PREFIX) / sizeof(URI_PREFIX[0])
                            ? URI_PREFIX[pfx]
                            : "";
        o = snprintf(out, sizeof(out), "%s", p);
        for (int i = pay_at + 1; i < pay_at + pay_len && o < (int)sizeof(out) - 1; i++)
            out[o++] = (b[i] >= 0x20 && b[i] < 0x7F) ? (char)b[i] : '?';
        out[o] = '\0';
    }
    else if (b[type_at] == 'T')
    {
        /* Text: status byte, then a language code, then the text itself. The
         * status byte's low six bits are the language length. */
        unsigned status = b[pay_at];
        int lang = (int)(status & 0x3F);
        int at = pay_at + 1 + lang;
        for (int i = at; i < pay_at + pay_len && o < (int)sizeof(out) - 1; i++)
            out[o++] = (b[i] >= 0x20 && b[i] < 0x7F) ? (char)b[i] : '?';
        out[o] = '\0';
    }
    else
    {
        return;
    }

    if (o > 0)
        printf("  %c %s\n", b[type_at], out);
}

static void print_card(const nfc_card_t *c)
{
    if (c->uid_len > 0)
        printf("card %s uid %s\n", c->tech, c->uid);
    else
        printf("card %s (no uid)\n", c->tech);

    printf("  ");
    if (c->sak >= 0)
        printf("sak %02x ", c->sak);
    if (c->tag[0])
        printf("%s ", c->tag);
    if (c->ndef < 0)
        printf("ndef n/a");
    else if (c->ndef)
        printf("ndef %d bytes", c->ndef_len);
    else
        printf("ndef none");
    printf("\n");

    if (c->ndef_raw_len > 0)
    {
        print_ndef_text(c);
        print_ndef_hex(c);
    }
}

/*
 * `nfc scan [seconds]` -- the field on, for a while, printing whatever answers.
 *
 * The loop is here rather than in a task because the reader's supply is the
 * battery: a background poller would keep the antenna cycling for ever and show
 * up in the milliamps, and nothing above this needs a tag to be live. Two
 * milliseconds per pass is well inside the reader's framing, and the worker
 * returns immediately when there is nothing to do.
 */
static int cmd_nfc_scan(int argc, char **argv)
{
    int secs = argc >= 3 ? atoi(argv[2]) : 20;
    if (secs <= 0)
        secs = 20;

    if (!board_nfc_rail_on())
    {
        printf("nfc rail FAILED\n");
        return 1;
    }

    if (!nfc_bus())
        return 1;

    if (!nfc_scan_start(s_host, s_sck, s_miso, s_mosi, s_cs, s_int))
    {
        printf("scanner did not start\n");
        return 1;
    }

    printf("scanning %d s -- tap a tag\n", secs);

    int64_t end = esp_timer_get_time() + (int64_t)secs * 1000000;
    int seen = 0;

    while (esp_timer_get_time() < end)
    {
        nfc_card_t c;
        int r = nfc_scan_step(&c);

        if (r > 0)
        {
            print_card(&c);
            seen++;
        }
        else if (r < 0)
        {
            printf("scanner stopped\n");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    nfc_scan_stop();
    printf("%d card%s\n", seen, seen == 1 ? "" : "s");
    return seen > 0 ? 0 : 1;
}

static int cmd_nfc(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    if (argc >= 2 && strcmp(argv[1], "scan") == 0)
        return cmd_nfc_scan(argc, argv);

    if (argc >= 2 && strcmp(argv[1], "reg") == 0)
    {
        if (argc < 3)
        {
            printf("usage: nfc reg <hex>\n");
            return 2;
        }
        uint8_t v = 0;
        unsigned reg = (unsigned)strtoul(argv[2], NULL, 16);
        if (!nfc_bus())
        {
            printf("no bus\n");
            return 1;
        }
        if (!nfc_read_reg((uint8_t)reg, &v))
        {
            printf("reg %02x did not answer\n", reg);
            return 1;
        }
        printf("reg %02x = %02x\n", reg, v);
        return 0;
    }

    if (argc >= 2)
    {
        printf("usage: nfc | nfc scan [s] | nfc reg <hex>\n");
        return 2;
    }

    struct nfc_ident n;
    bool ok = nfc_identify(&n);

    printf("rail  %s\n", n.rail ? "up" : "FAILED");
    printf("bus   %s cs %d int %d\n", n.bus ? "up" : "FAILED", s_cs, s_int);
    if (!n.answered)
    {
        printf("id    no answer\n");
        return 1;
    }
    printf("id    %02x type %s rev %u\n", n.id,
           n.known ? (n.is_b ? "ST25R3916B" : "ST25R3916") : "unknown", n.rev);
    printf("io2   %02x\n", n.io2);
    printf("op    %02x\n", n.op);
    printf("irq   %d\n", n.irq);
    printf("boot  %s\n", !s_boot_ran ? "not run"
                          : s_boot_ok  ? "ok"
                                       : "no answer at boot");

    return ok ? 0 : 1;
}

static const tdsh_command_t s_nfc_cmd = {
    "nfc",
    "nfc | nfc scan [s] | nfc reg <hex>",
    "Probe the ST25R3916 reader, scan for tags, or read a register",
    cmd_nfc,
    TDSH_CMD_INTERACTIVE,
};

bool nfc_register_command(void)
{
    return tdsh_register_command(&s_nfc_cmd) == 0;
}
