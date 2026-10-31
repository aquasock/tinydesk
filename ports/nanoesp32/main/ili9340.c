/*
 * ili9340.c - SPI driver for the ILI9340 on the Adafruit 2.2" PiTFT HAT.
 *
 * 4-wire SPI, no framebuffer in RAM: the controller owns its GRAM, so a
 * changed rectangle is pushed straight to it. Read-back (0x04) is used once
 * at init as a wiring check, which is why MISO is wired.
 */
#include "ili9340.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ili9340";

/* ILI9340 command set, only what this driver needs. */
#define CMD_SWRESET 0x01
#define CMD_RDDID   0x04
#define CMD_SLPOUT  0x11
#define CMD_DISPON  0x29
#define CMD_CASET   0x2A
#define CMD_RASET   0x2B
#define CMD_RAMWR   0x2C
#define CMD_MADCTL  0x36
#define CMD_COLMOD  0x3A

#define SPI_HOST_USED SPI2_HOST
#define CHUNK_PX      512

static spi_device_handle_t s_spi;
static bool s_ready;

/* ---- low level ---------------------------------------------------- */

static void dc(int level)
{
    gpio_set_level(LCD_PIN_DC, level);
}

static void cmd(uint8_t c)
{
    dc(0);
    spi_transaction_t t = {.length = 8, .tx_buffer = &c};
    spi_device_polling_transmit(s_spi, &t);
    dc(1);
}

static void data(const uint8_t *buf, size_t n)
{
    if (!n)
        return;
    spi_transaction_t t = {.length = n * 8, .tx_buffer = buf};
    spi_device_polling_transmit(s_spi, &t);
}

static void data8(uint8_t v)
{
    data(&v, 1);
}

/* ---- panel setup -------------------------------------------------- */

bool ili9340_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << LCD_PIN_DC,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    dc(1);

    /* Backlight on, if it is wired to a pin. */
#if LCD_PIN_BL >= 0
    {
        gpio_config_t bl = {
            .pin_bit_mask = 1ULL << LCD_PIN_BL,
            .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config(&bl);
        gpio_set_level(LCD_PIN_BL, 1);
    }
#endif

#if LCD_PIN_RST >= 0
    {
        gpio_config_t rst = {
            .pin_bit_mask = 1ULL << LCD_PIN_RST,
            .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config(&rst);
        gpio_set_level(LCD_PIN_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(LCD_PIN_RST, 1);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
#else
    /* The HAT's reset supervisor holds the panel in reset and releases it
     * itself, so the host only has to wait it out before talking. */
    vTaskDelay(pdMS_TO_TICKS(150));
#endif

    spi_bus_config_t bus = {
        .mosi_io_num = LCD_PIN_MOSI,
        .miso_io_num = LCD_PIN_MISO,
        .sclk_io_num = LCD_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = CHUNK_PX * 2,
    };
    esp_err_t err = spi_bus_initialize(SPI_HOST_USED, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
        return false;
    }

    spi_device_interface_config_t dev = {
        .clock_speed_hz = LCD_SPI_HZ,
        .mode = 0,
        .spics_io_num = LCD_PIN_CS,
        .queue_size = 4,
    };
    err = spi_bus_add_device(SPI_HOST_USED, &dev, &s_spi);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(err));
        return false;
    }

    cmd(CMD_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(120));

    cmd(CMD_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(120));
    cmd(CMD_COLMOD);
    data8(0x55);                       /* 16 bits per pixel */
    cmd(CMD_MADCTL);
    data8(0x28);                       /* MV|BGR: landscape, BGR order */
    vTaskDelay(pdMS_TO_TICKS(10));
    cmd(CMD_DISPON);
    vTaskDelay(pdMS_TO_TICKS(20));

    s_ready = true;
    return true;
}

uint32_t ili9340_read_id(void)
{
    if (!s_ready)
        return 0;

    /* Full duplex read: send a command byte, then clock N dummy bytes and
     * keep what comes back. length and rxlength must match -- an earlier
     * version asked for 8 bits out and 32 in one transaction, which ESP-IDF
     * cannot express, so nothing was ever read. */
    uint8_t cmd_byte = CMD_RDDID;
    dc(0);
    spi_transaction_t t = {.length = 8, .tx_buffer = &cmd_byte};
    spi_device_polling_transmit(s_spi, &t);
    dc(1);

    uint8_t rx[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t tx[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    spi_transaction_t r = {
        .length = 32,
        .rxlength = 32,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    if (spi_device_polling_transmit(s_spi, &r) != ESP_OK)
        return 0;

    ESP_LOGI(TAG, "raw id bytes: %02x %02x %02x %02x",
             rx[0], rx[1], rx[2], rx[3]);
    return ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) |
           ((uint32_t)rx[2] << 8) | rx[3];
}

void ili9340_window(int x, int y, int w, int h)
{
    uint8_t b[4];
    cmd(CMD_CASET);
    b[0] = (uint8_t)(x >> 8);   b[1] = (uint8_t)x;
    b[2] = (uint8_t)((x + w - 1) >> 8); b[3] = (uint8_t)(x + w - 1);
    data(b, 4);
    cmd(CMD_RASET);
    b[0] = (uint8_t)(y >> 8);   b[1] = (uint8_t)y;
    b[2] = (uint8_t)((y + h - 1) >> 8); b[3] = (uint8_t)(y + h - 1);
    data(b, 4);
    cmd(CMD_RAMWR);
}

void ili9340_pixel(uint16_t c)
{
    uint8_t b[2] = {(uint8_t)(c >> 8), (uint8_t)c};
    data(b, 2);
}

void ili9340_blit(int x, int y, int w, int h, const uint16_t *px)
{
    if (!s_ready || w <= 0 || h <= 0)
        return;
    if (x < 0 || y < 0 || x + w > LCD_W || y + h > LCD_H)
        return;

    ili9340_window(x, y, w, h);

    /* The panel takes big-endian pixels; the SPI peripheral sends bytes in
     * order, so a buffer of uint16_t needs its halves swapped first. */
    static uint16_t line[CHUNK_PX];
    int total = w * h;
    int done = 0;
    while (done < total)
    {
        int n = total - done;
        if (n > CHUNK_PX)
            n = CHUNK_PX;
        for (int i = 0; i < n; i++)
        {
            uint16_t c = px[done + i];
            line[i] = (uint16_t)((c >> 8) | (c << 8));
        }
        data((const uint8_t *)line, (size_t)n * 2);
        done += n;
    }
}

void ili9340_fill(uint16_t colour)
{
    if (!s_ready)
        return;
    ili9340_window(0, 0, LCD_W, LCD_H);
    static uint16_t line[CHUNK_PX];
    for (int i = 0; i < CHUNK_PX; i++)
        line[i] = (uint16_t)((colour >> 8) | (colour << 8));
    int total = LCD_W * LCD_H;
    while (total > 0)
    {
        int n = total > CHUNK_PX ? CHUNK_PX : total;
        data((const uint8_t *)line, (size_t)n * 2);
        total -= n;
    }
}
