/*
 * power.c - the shared I2C bus and the AMOLED's switched rail.
 *
 * See power.h for why this module exists. The short version: the panel is
 * dark unless XL9555 P07 is driven high, and nothing else on the board does
 * that.
 */

#include "power.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define TAG "twatch-power"

#define BOARD_SDA   3
#define BOARD_SCL   2
#define BOARD_HZ    400000

#define XL9555_ADDR 0x20
/* PCA9555 register map. Inputs are 0x00/0x01, outputs 0x02/0x03, the
 * polarity inversions 0x04/0x05, and the configuration (direction)
 * registers 0x06/0x07. A set bit in a configuration register means INPUT. */
#define XL9555_OUTPUT0  0x02
#define XL9555_CONFIG0  0x06

#define DISP_EN_BIT     7       /* P07: the display rail's switch */

static i2c_master_bus_handle_t s_bus;
static bool s_bus_tried;
static i2c_master_dev_handle_t s_xl;

i2c_master_bus_handle_t board_i2c(void)
{
    if (s_bus || s_bus_tried)
        return s_bus;

    s_bus_tried = true;

    i2c_master_bus_config_t bus = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = BOARD_SDA,
        .scl_io_num = BOARD_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    if (i2c_new_master_bus(&bus, &s_bus) != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C bus did not start");
        s_bus = NULL;
    }
    return s_bus;
}

static i2c_master_dev_handle_t xl9555(void)
{
    if (s_xl)
        return s_xl;

    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = XL9555_ADDR,
        .scl_speed_hz = BOARD_HZ,
    };

    if (!board_i2c() || i2c_master_bus_add_device(board_i2c(), &dev, &s_xl) != ESP_OK)
    {
        ESP_LOGE(TAG, "expander 0x%02X did not answer", XL9555_ADDR);
        s_xl = NULL;
    }
    return s_xl;
}

static bool reg_read(uint8_t reg, uint8_t *v)
{
    i2c_master_dev_handle_t d = xl9555();

    if (!d)
        return false;
    return i2c_master_transmit_receive(d, &reg, 1, v, 1, 100) == ESP_OK;
}

static bool reg_write(uint8_t reg, uint8_t v)
{
    i2c_master_dev_handle_t d = xl9555();
    uint8_t b[2] = { reg, v };

    if (!d)
        return false;
    return i2c_master_transmit(d, b, sizeof(b), 100) == ESP_OK;
}

void board_dump_expander(void)
{
    uint8_t in0 = 0, in1 = 0, out0 = 0, out1 = 0, cfg0 = 0, cfg1 = 0;

    if (!reg_read(0x00, &in0) || !reg_read(0x01, &in1) ||
        !reg_read(0x02, &out0) || !reg_read(0x03, &out1) ||
        !reg_read(0x06, &cfg0) || !reg_read(0x07, &cfg1))
    {
        ESP_LOGW(TAG, "XL9555 did not answer");
        return;
    }
    ESP_LOGI(TAG, "XL9555 in=%02x/%02x out=%02x/%02x cfg=%02x/%02x",
             in0, in1, out0, out1, cfg0, cfg1);
}

bool board_display_power_on(void)
{
    uint8_t cfg, out;

    if (!reg_read(XL9555_CONFIG0, &cfg) || !reg_read(XL9555_OUTPUT0, &out))
    {
        ESP_LOGE(TAG, "cannot reach the expander to power the panel");
        return false;
    }

    /* P07 must be an OUTPUT before its latch means anything. */
    if (cfg & (1u << DISP_EN_BIT))
    {
        if (!reg_write(XL9555_CONFIG0, (uint8_t)(cfg & ~(1u << DISP_EN_BIT))))
        {
            ESP_LOGE(TAG, "could not make P07 an output");
            return false;
        }
    }
    if (!reg_write(XL9555_OUTPUT0, (uint8_t)(out | (1u << DISP_EN_BIT))))
    {
        ESP_LOGE(TAG, "could not raise P07");
        return false;
    }

    board_dump_expander();

    /* Latch, then let the panel's own supply settle before it is touched. */
    vTaskDelay(pdMS_TO_TICKS(50));
    return true;
}
