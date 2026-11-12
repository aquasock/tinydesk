/*
 * arduino_shim.cpp - what the Arduino core was doing, in ESP-IDF terms.
 *
 * Three globals (Serial, Wire, SPI) and the handful of functions declared in
 * Arduino.h. See those headers for why each shape exists; this file is the
 * implementation and the comments here are about the two places where ESP-IDF
 * differs from Arduino in a way that changes behaviour.
 */

#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

ArduinoSerial Serial;
TwoWire Wire;
SPIClass SPI;

static const char *TAG = "nfc-shim";

/* --- time ---------------------------------------------------------------- */

void delay(uint32_t ms)
{
    /* Arduino's delay(0) returns immediately on some cores and yields on
     * others. vTaskDelay(0) is a yield, which is the safer of the two here:
     * the stack calls delay() inside polling loops and would otherwise be a
     * busy wait on a single-core-critical path. */
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void delayMicroseconds(uint32_t us)
{
    esp_rom_delay_us(us);
}

uint32_t millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

uint32_t micros(void)
{
    return (uint32_t)esp_timer_get_time();
}

/* --- gpio ---------------------------------------------------------------- */

void pinMode(uint8_t pin, uint8_t mode)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << pin),
        .mode = (mode == OUTPUT) ? GPIO_MODE_OUTPUT
                                 : (mode == INPUT_PULLUP ? GPIO_MODE_INPUT : GPIO_MODE_INPUT),
        .pull_up_en = (mode == INPUT_PULLUP) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    /* gpio_config() logs the entire pin state at INFO. That is three lines of
     * the desktop's 64-line log ring per pin, for a pin whose level the NFC
     * driver prints anyway. Quiet while it runs, then put back. */
    esp_log_level_t lvl = esp_log_level_get("gpio");
    esp_log_level_set("gpio", ESP_LOG_WARN);
    gpio_config(&cfg);
    esp_log_level_set("gpio", lvl);
}

void digitalWrite(uint8_t pin, uint8_t value)
{
    gpio_set_level((gpio_num_t)pin, value ? 1 : 0);
}

int digitalRead(uint8_t pin)
{
    return gpio_get_level((gpio_num_t)pin);
}

/*
 * No-op by design -- see Arduino.h. The pin is left as an input, which is what
 * the stack's fallback path reads, so a raised interrupt line is still noticed
 * on the next worker call.
 */
void attachInterrupt(uint8_t pin, void (*fn)(void), int mode)
{
    (void)pin;
    (void)fn;
    (void)mode;
}

void detachInterrupt(uint8_t pin)
{
    (void)pin;
}

/* --- SPI ----------------------------------------------------------------- */

#define SHIM_XFER_MAX 1024

static spi_device_handle_t s_dev;
static int s_host = 2;
static uint32_t s_hz = 5000000;
static uint8_t *s_tx;
static uint8_t *s_rx;

void SPIClass::configure(int host, uint32_t hz)
{
    s_host = host;
    s_hz = hz;
}

bool SPIClass::ready(void) const
{
    return s_dev != NULL;
}

void SPIClass::begin(int sck, int miso, int mosi, int ss)
{
    (void)ss; /* the stack drives chip select itself */
    if (s_dev)
        return;
    start_bus(sck, miso, mosi);
}

void SPIClass::end(void)
{
    if (s_dev)
    {
        spi_bus_remove_device(s_dev);
        s_dev = NULL;
    }
}

bool SPIClass::start_bus(int sck, int miso, int mosi)
{
    spi_bus_config_t bus = {
        .mosi_io_num = mosi,
        .miso_io_num = miso,
        .sclk_io_num = sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SHIM_XFER_MAX,
    };

    /* The SD card's watcher may already own this bus, and it may not have
     * started yet. Same tolerance as the port's own nfc_bus(): an already
     * initialized bus is success, not an error.
     *
     * The tag is quieted around the call because ESP-IDF logs
     * "SPI bus already initialized" at ERROR level before returning
     * ESP_ERR_INVALID_STATE -- an expected outcome reaching the log as a fault.
     * tdsh_sdcard.c already does this for the same reason. */
    esp_log_level_t spi_lvl = esp_log_level_get("spi");
    esp_log_level_set("spi", ESP_LOG_NONE);
    esp_err_t err = spi_bus_initialize((spi_host_device_t)s_host, &bus, SPI_DMA_CH_AUTO);
    esp_log_level_set("spi", spi_lvl);

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "spi bus did not start: %s", esp_err_to_name(err));
        return false;
    }

    spi_device_interface_config_t dev = {
        .mode = 1,               /* CPOL 0, CPHA 1 */
        .clock_speed_hz = (int)s_hz,
        .spics_io_num = -1,      /* the stack toggles GPIO 4 itself */
        .queue_size = 1,
    };

    err = spi_bus_add_device((spi_host_device_t)s_host, &dev, &s_dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "could not add the nfc spi device: %s", esp_err_to_name(err));
        s_dev = NULL;
        return false;
    }

    if (!s_tx)
    {
        s_tx = (uint8_t *)heap_caps_malloc(SHIM_XFER_MAX, MALLOC_CAP_DMA);
        s_rx = (uint8_t *)heap_caps_malloc(SHIM_XFER_MAX, MALLOC_CAP_DMA);
        if (!s_tx || !s_rx)
        {
            ESP_LOGE(TAG, "no dma buffers");
            heap_caps_free(s_tx);
            heap_caps_free(s_rx);
            s_tx = s_rx = NULL;
            spi_bus_remove_device(s_dev);
            s_dev = NULL;
            return false;
        }
    }

    return true;
}

/*
 * The clock is set once, at device creation, from the speed the stack passes to
 * its own constructor. beginTransaction() therefore does not reconfigure
 * anything: the stack asks for the same speed on every transaction, and
 * ESP-IDF can only change a device's clock by removing and re-adding it, which
 * is not something to do in the middle of a polling loop.
 */
void SPIClass::beginTransaction(SPISettings s)
{
    if (s.clock != s_hz)
        ESP_LOGD(TAG, "ignoring a %u Hz request; the bus is at %u",
                 (unsigned)s.clock, (unsigned)s_hz);
}

void SPIClass::endTransaction(void) {}

uint8_t SPIClass::transfer(uint8_t v)
{
    if (!s_dev)
        return 0;

    spi_transaction_t t = { 0 };
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 8;
    t.tx_data[0] = v;

    if (spi_device_transmit(s_dev, &t) != ESP_OK)
        return 0;

    return t.rx_data[0];
}

void SPIClass::transfer(void *buf, size_t count)
{
    if (!s_dev || !buf || count == 0)
        return;

    if (count > SHIM_XFER_MAX)
    {
        ESP_LOGE(TAG, "transfer of %u bytes is over the buffer", (unsigned)count);
        return;
    }

    /* Full duplex, and the caller's buffer is both the source and the
     * destination -- that is Arduino's contract and the stack depends on it
     * for register reads. Two separate buffers because ESP-IDF's DMA path
     * wants them distinct. */
    memcpy(s_tx, buf, count);

    spi_transaction_t t = { 0 };
    t.length = count * 8;
    t.tx_buffer = s_tx;
    t.rx_buffer = s_rx;

    if (spi_device_transmit(s_dev, &t) != ESP_OK)
        return;

    memcpy(buf, s_rx, count);
}
