/*
 * display.c - the T-Watch Ultra's CO5300 AMOLED as a terminal.
 *
 * The desktop's output is parsed into a td_vterm and a task paints the cells
 * that changed. This file is the only one in the port that is NOT modelled on
 * ports/crowpanel7, because the two panels have nothing in common:
 *
 *   CrowPanel   800x480 RGB parallel, driven by esp_lcd's RGB panel driver,
 *               which scans a framebuffer continuously and needs no writes.
 *   T-Watch     410x502 AMOLED, CO5300 controller on a four-lane QSPI bus,
 *               write-only, with no scanout of its own.
 *
 * ESP-IDF 5.3's esp_lcd panel IO has no quad mode, so the bus is driven with
 * the SPI master driver directly and the CO5300 command set by hand. The
 * sequences below come from the Escapement project's working Linux driver
 * (patches/kernel/display/final/co5300.c), which in turn took them from the
 * watch's factory firmware at file offset 0x130dd4 -- they are the ones that
 * actually bring this panel up, not guesses from a datasheet.
 *
 * PIXELS ARE SWAPPED. The CO5300 takes the low byte of an RGB565 word first,
 * so every pixel written into the line buffer is byte-swapped. Black is
 * identical either way, which is exactly why a missing swap looks like a
 * working driver until something coloured is drawn.
 */

#include "display.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "font8x16.h"
#include "tinydesk/td_vterm.h"

#define TAG "twatch-display"

/* ---- pins, from board.example.conf ------------------------------------- */
#define LCD_SCK   40
#define LCD_D0    38
#define LCD_D1    39
#define LCD_D2    42
#define LCD_D3    45
#define LCD_CS    41
#define LCD_RST   37
#define LCD_SPI_HZ (40 * 1000 * 1000)

/* The panel's own coordinate space. The glass is 410x502; DISPLAY_W/H in
 * display.h is the 320x400 rectangle inside it that the desktop uses. The
 * offset centres one in the other. */
#define PANEL_W   410
#define PANEL_H   502

#define CELL_W    8
#define CELL_H    16

/* A CO5300 command on the QSPI bus: one command byte, then parameters. The
 * controller treats a CS edge as end-of-burst, so a command and its payload
 * must travel in one transaction. */
static spi_device_handle_t s_spi;
static SemaphoreHandle_t s_lock;      /* the bus, between painter and init */
static td_vterm_t *s_vt;              /* in PSRAM */
static td_vcell_t *s_shown;           /* what the panel is displaying */
static int s_cols, s_rows;
static TaskHandle_t s_paint_task;
static display_reply_fn s_reply;

/* One line of pixels, in panel byte order, composed a row at a time. */
static uint16_t s_line[DISPLAY_W];

/* Palette: td_vterm gives a 256-colour index. Above 15 the desktop only uses
 * indices it knows are approximated, so a 16-colour table is enough and the
 * upper half is folded down by the terminal itself. */
static const uint16_t s_palette[16] = {
    0x0000, 0xF800, 0x07E0, 0xFFE0, 0x001F, 0xF81F, 0x07FF, 0xFFFF,
    0x8410, 0xF810, 0x87E0, 0xFF90, 0x841F, 0xF81F, 0x87FF, 0xFFFF,
};

static inline uint16_t swap16(uint16_t v)
{
    return (uint16_t)((v >> 8) | (v << 8));
}

/* ---- low level --------------------------------------------------------- */

/* Send one command plus its parameters in a single transaction. */
static void co5300_cmd(uint8_t op, const uint8_t *data, int len)
{
    spi_transaction_ext_t t = { 0 };

    spi_device_acquire_bus(s_spi, portMAX_DELAY);
    t.base.flags = SPI_TRANS_MODE_QIO;
    t.base.cmd = op;
    t.base.length = 8 * len;
    t.base.tx_buffer = data;
    /* Command and address in QIO, data in QIO, so the whole burst stays on
     * all four lanes. One command byte, one dummy "address" byte, then the
     * payload -- which is the framing the CO5300 expects and what the Linux
     * driver's spi-mem op encodes. */
    t.address_bits = 8;
    t.address = 0x00;
    t.command_bits = 8;
    if (len)
        spi_device_polling_transmit(s_spi, (spi_transaction_t *)&t);
    else
        spi_device_polling_transmit(s_spi, (spi_transaction_t *)&t);
    spi_device_release_bus(s_spi);
}

static void co5300_window(int x, int y, int w, int h)
{
    uint8_t cas[4] = { (uint8_t)(x >> 8), (uint8_t)x,
                       (uint8_t)((x + w - 1) >> 8), (uint8_t)(x + w - 1) };
    uint8_t ras[4] = { (uint8_t)(y >> 8), (uint8_t)y,
                       (uint8_t)((y + h - 1) >> 8), (uint8_t)(y + h - 1) };

    co5300_cmd(0x2A, cas, 4);
    co5300_cmd(0x2B, ras, 4);
    co5300_cmd(0x2C, NULL, 0);        /* memory write follows */
}

/* The eleven entries that bring this panel up. */
static const struct {
    uint8_t op;
    uint8_t data[4];
    uint8_t len;                      /* low 5 bits: parameters, 0x80: settle */
} s_init[] = {
    { 0xFE, { 0x00 }, 0x01 },
    { 0xC4, { 0x80 }, 0x01 },
    { 0x3A, { 0x55 }, 0x01 },         /* RGB565 */
    { 0x35, { 0x00 }, 0x01 },
    { 0x53, { 0x20 }, 0x01 },
    { 0x63, { 0xFF }, 0x01 },
    { 0x2A, { 0x00, 0x16, 0x01, 0xAF }, 0x04 },
    { 0x2B, { 0x00, 0x00, 0x01, 0xF5 }, 0x04 },
    { 0x11, { 0x00 }, 0x80 },         /* sleep out */
    { 0x29, { 0x00 }, 0x80 },         /* display on */
    { 0x51, { 0x00 }, 0x01 },         /* brightness; 0xFF follows */
};

static bool panel_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << LCD_RST,
        .mode = GPIO_MODE_OUTPUT,
    };
    size_t i;
    uint8_t bright = 0xFF;

    gpio_config(&io);
    gpio_set_level(LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    for (i = 0; i < sizeof(s_init) / sizeof(s_init[0]); i++) {
        co5300_cmd(s_init[i].op, s_init[i].data, s_init[i].len & 0x1F);
        if (s_init[i].len & 0x80)
            vTaskDelay(pdMS_TO_TICKS(120));
    }
    co5300_cmd(0x51, &bright, 1);
    return true;
}

/* ---- painting ---------------------------------------------------------- */

static void paint_cell(int col, int row, td_vcell_t c, bool cursor)
{
    const uint8_t *glyph = font8x16_glyph(c.ch);
    uint16_t fg = s_palette[c.fg & 0x0F];
    uint16_t bg = s_palette[c.bg & 0x0F];
    int x0 = col * CELL_W;
    int y, bit;

    if (cursor)
    {
        uint16_t t = fg;
        fg = bg;
        bg = t;
    }
    if (fg == bg)
    {
        /* Nothing to compose: one fill instead of 16 glyph rows. */
        for (y = 0; y < CELL_H; y++)
            for (bit = 0; bit < CELL_W; bit++)
                s_line[x0 + bit] = swap16(bg);
        return;
    }

    for (y = 0; y < CELL_H; y++)
    {
        uint8_t bits = glyph ? glyph[y] : 0;

        for (bit = 0; bit < CELL_W; bit++)
            s_line[x0 + bit] = swap16((bits & (0x80 >> bit)) ? fg : bg);

        co5300_window(x0, row * CELL_H + y, CELL_W, 1);
        spi_device_acquire_bus(s_spi, portMAX_DELAY);
        {
            spi_transaction_ext_t t = { 0 };
            t.base.flags = SPI_TRANS_MODE_QIO;
            t.base.length = 8 * CELL_W * 2;
            t.base.tx_buffer = s_line + x0;
            spi_device_polling_transmit(s_spi, (spi_transaction_t *)&t);
        }
        spi_device_release_bus(s_spi);
    }
}

static void paint_task(void *arg)
{
    (void)arg;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        xSemaphoreTake(s_lock, portMAX_DELAY);
        for (int r = 0; r < s_rows; r++)
        {
            for (int c = 0; c < s_cols; c++)
            {
                td_vcell_t want;
                td_vcell_t *have = &s_shown[r * TD_VT_MAX_COLS + c];
                bool cur = s_vt->cursor_visible &&
                           s_vt->cx == c && s_vt->cy == r;

                want = s_vt->cells[r * TD_VT_MAX_COLS + c];
                if (want.ch == have->ch && want.fg == have->fg &&
                    want.bg == have->bg && !cur)
                    continue;
                paint_cell(c, r, want, cur);
                *have = want;
            }
        }
        s_vt->dirty = false;
        xSemaphoreGive(s_lock);
    }
}

/* ---- td_vterm plumbing ------------------------------------------------- */

static void vt_reply(void *user, const char *data, int len)
{
    (void)user;
    if (s_reply)
        s_reply(data, len);
}

int display_cols(void) { return s_cols; }
int display_rows(void) { return s_rows; }

bool display_init(display_reply_fn reply)
{
    spi_bus_config_t bus = {
        .mosi_io_num = LCD_D0,
        .miso_io_num = LCD_D1,
        .sclk_io_num = LCD_SCK,
        .quadwp_io_num = LCD_D2,
        .quadhd_io_num = LCD_D3,
        .max_transfer_sz = 8 * DISPLAY_W * 2,
    };
    spi_device_interface_config_t dev = {
        .clock_speed_hz = LCD_SPI_HZ,
        .mode = 0,
        .spics_io_num = LCD_CS,
        .queue_size = 1,
    };

    if (spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI2 bus did not start");
        return false;
    }
    if (spi_bus_add_device(SPI2_HOST, &dev, &s_spi) != ESP_OK)
    {
        ESP_LOGE(TAG, "the panel device did not register");
        return false;
    }

    s_lock = xSemaphoreCreateMutex();
    s_reply = reply;
    if (!panel_init())
        return false;

    s_vt = heap_caps_calloc(1, sizeof(*s_vt),
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_shown = heap_caps_calloc(TD_VT_MAX_ROWS * TD_VT_MAX_COLS,
                               sizeof(*s_shown),
                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_vt || !s_shown)
    {
        ESP_LOGE(TAG, "no PSRAM for the terminal");
        return false;
    }

    s_cols = DISPLAY_MAX_COLS;
    s_rows = DISPLAY_MAX_ROWS;
    td_vterm_init(s_vt, s_cols, s_rows);
    /* The reply hook is a field set after init, not an init argument: the
     * terminal answers ESC [ 6 n and friends through it, and those answers
     * have to reach the desktop as input. */
    s_vt->reply = vt_reply;
    s_vt->reply_user = NULL;
    s_vt->cell_w = CELL_W;
    s_vt->cell_h = CELL_H;

    /* A cell that has never been drawn must not match anything, so the shown
     * grid starts as a value no real cell can hold. */
    memset(s_shown, 0xFF, TD_VT_MAX_ROWS * TD_VT_MAX_COLS * sizeof(*s_shown));

    if (xTaskCreatePinnedToCore(paint_task, "tdpaint", 4096, NULL, 4,
                                &s_paint_task, 1) != pdPASS)
    {
        ESP_LOGE(TAG, "the painter did not start");
        return false;
    }
    return true;
}

void display_write(const uint8_t *buf, int len)
{
    if (!s_vt)
        return;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    td_vterm_write(s_vt, buf, len);
    xSemaphoreGive(s_lock);

    xTaskNotifyGive(s_paint_task);
}

/* ---- pictures: not supported ------------------------------------------- */

bool display_picture(void *user, int x, int y, const uint8_t *pixels, int w, int h)
{
    (void)user; (void)x; (void)y; (void)pixels; (void)w; (void)h;
    return false;
}
bool display_picture_rgb(void *user, int x, int y, const uint16_t *pixels, int w, int h)
{
    (void)user; (void)x; (void)y; (void)pixels; (void)w; (void)h;
    return false;
}
bool display_picture_runs(void *user, int x, int y, const uint16_t *runs, size_t words, int w, int h)
{
    (void)user; (void)x; (void)y; (void)runs; (void)words; (void)w; (void)h;
    return false;
}
void display_scanout_snapshot(uint32_t out[12]) { memset(out, 0, 12 * sizeof(uint32_t)); }
void display_scanout_info(uint32_t out[8]) { memset(out, 0, 8 * sizeof(uint32_t)); }
