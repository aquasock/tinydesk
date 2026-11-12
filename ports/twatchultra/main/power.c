/*
 * power.c - the shared I2C bus and the AMOLED's switched rail.
 *
 * See power.h for why this module exists. The short version: the panel is
 * dark unless XL9555 P07 is driven high, and nothing else on the board does
 * that.
 */

#include "power.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "tinydesk/td_sysinfo.h"
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
#define DRV_EN_BIT      6       /* P06: the haptic driver's rail */

/*
 * THE HAPTIC RAIL IS A SEPARATE SWITCH AND IT IS OFF AT BOOT.
 *
 * P06 on the same expander gates the DRV2605's supply, and a PCA9555 comes up
 * with every pin an input, so P06 floats until something drives it. The
 * consequence is silent in the worst way: the chip still answers every I2C
 * transaction, so a driver that only probes it reports success, and the motor
 * does not move because it has no power. That is BRD-005's second silent
 * failure -- "an ERM given the LRA library buzzes weakly or not at all" is the
 * other -- and the two are easy to confuse with a dead motor.
 *
 * Two independent sources agree the bit is 6: LilyGoLib's
 * POWER_HAPTIC_DRIVER case drives EXPANDS_DRV_EN, and this project's reference
 * cites the same value from LilyGo's TinyPetz. Unlike the touch reset line,
 * there is no disagreement to hedge around here. */


/*
 * TOUCH RESET. Two sources disagreed on which expander pin it is, and the
 * board's own Arduino variant settles it:
 *
 *   Escapement .ai/core-reference.md TCH-005     "P08, port 1 bit 0"
 *   arduino-esp32 lilygo_twatch_ultra variant    EXPANDS_TOUCH_RST = 8
 *
 * LilyGo's prose hardware page said GPIO10, which is EXPANDS_SD_DET -- the SD
 * card's insert switch. That is why the pin was wrong, and driving it as an
 * output fought a physical switch.
 */
#define TOUCH_RST_LINEAR_A  8    /* P08: EXPANDS_TOUCH_RST in the board variant */

/* --- AXP2101 PMU, at 0x34 ------------------------------------------------
 *
 * ALDO2 feeds the display rail, and XL9555 P07 switches it. Both have to be
 * on: P07 alone switches a rail that is not generating, and ALDO2 alone is
 * disconnected by the switch. Registers and bit layout are from the mainline
 * AXP2101 driver this project carries in patches/kernel/axp2101 -- the
 * voltage field is bits 4:0 at 100 mV per step above 500 mV, and the enables
 * for ALDO1..4 are bits 0..3 of 0x90.
 */
#define PMU_ADDR            0x34
#define PMU_ALDO1_VOLT      0x92
#define PMU_ALDO2_VOLT      0x93
#define PMU_BLDO2_VOLT      0x97
#define PMU_LDO_EN          0x90
#define PMU_ALDO1_EN_BIT    0
#define PMU_ALDO2_EN_BIT    1
#define PMU_BLDO2_EN_BIT    5
/* 3.3 V: (3300 - 500) / 100 = 28 */
#define PMU_ALDO1_3300MV    28
#define PMU_ALDO2_3300MV    28
#define PMU_BLDO2_3300MV    28

/* The gauge. Registers from the AXP2101 driver this project carries in
 * patches/kernel/axp2101, which measured them on this board (PMU-001):
 * capacity is 0xa4 bits 6:0 and anything above 100 means the gauge has not
 * settled, voltage is 0x34 bits 5:0 above 0x35 giving 14 bits already in mV,
 * and the charge state is 0x01 bits 2:0. */
#define PMU_STATUS1         0x00
#define PMU_STATUS2         0x01
#define PMU_BATT_VOLT_H     0x34
#define PMU_BATT_VOLT_L     0x35
#define PMU_BATT_PERCENT    0xa4
#define PMU_BATT_PRESENT    0x08        /* status1 bit 3 */
#define PMU_CHRG_STATE_MASK 0x07        /* status2 bits 2:0 */

static i2c_master_dev_handle_t s_pmu;

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

static i2c_master_dev_handle_t pmu(void)
{
    if (s_pmu)
        return s_pmu;

    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PMU_ADDR,
        .scl_speed_hz = BOARD_HZ,
    };

    if (!board_i2c() || i2c_master_bus_add_device(board_i2c(), &dev, &s_pmu) != ESP_OK)
    {
        ESP_LOGE(TAG, "PMU 0x%02X did not answer", PMU_ADDR);
        s_pmu = NULL;
    }
    return s_pmu;
}

static bool pmu_r(uint8_t reg, uint8_t *v)
{
    i2c_master_dev_handle_t d = pmu();
    return d && i2c_master_transmit_receive(d, &reg, 1, v, 1, 100) == ESP_OK;
}

static bool pmu_w(uint8_t reg, uint8_t v)
{
    i2c_master_dev_handle_t d = pmu();
    uint8_t b[2] = { reg, v };
    return d && i2c_master_transmit(d, b, sizeof(b), 100) == ESP_OK;
}

/*
 * THE SD CARD SHARES ITS SPI BUS, AND THE OTHER TWO CHIPS HAVE TO BE SILENT.
 *
 * MISO/MOSI/SCLK are shared with the LoRa radio (CS 36) and the NFC reader
 * (CS 4), per the board's Arduino variant. A chip select that floats low leaves
 * that device driving MISO, and a second device on MISO corrupts every SD
 * transaction -- the handshake fails at CMD8 with ESP_ERR_INVALID_RESPONSE,
 * which reads like a wiring fault or a bad card and is neither.
 *
 * This board has already shown that signature once: BRD-004 records the NFC
 * probe returning nothing "with MISO clamped low by something external", and
 * that was on the Linux spi-mem stack rather than this one, so it says nothing
 * about the cause -- only that a held MISO is a thing this hardware does.
 *
 * LilyGo's own bring-up, LilyGoUltra::initShareSPIPins(), drives exactly these
 * pins high before the bus is used. This port never did, because until the SD
 * card there was nothing in it on that bus.
 */
#define SHARED_SPI_CS_NFC   4
#define SHARED_SPI_CS_LORA  36
#define SHARED_SPI_CS_SD    21

/* Raise ALDO1 to 3.3 V and enable it: the SD card's supply.
 *
 * BRD-003 records ALDO1 as the card's rail, and LilyGo's own PMU bring-up sets
 * it to 3300 mV and enables it right beside the display's ALDO2, so the card is
 * powered before anything tries to talk to it.
 *
 * Same silent failure as the haptic rail: an unpowered card does not answer, so
 * a mount attempt fails with a timeout that reads as a wiring problem. This is
 * called before the SD stack is started for that reason, not because the SD
 * code needs it in any specific order.
 *
 * Returns false only if the PMU does not answer; a rail that is already up is a
 * success. */
bool board_sd_rail_on(void)
{
    uint8_t v, en;

    if (!pmu_r(PMU_ALDO1_VOLT, &v) || !pmu_r(PMU_LDO_EN, &en))
    {
        ESP_LOGE(TAG, "PMU did not answer");
        return false;
    }

    if ((v & 0x1F) != PMU_ALDO1_3300MV)
    {
        if (!pmu_w(PMU_ALDO1_VOLT, (uint8_t)((v & 0xE0) | PMU_ALDO1_3300MV)))
        {
            ESP_LOGE(TAG, "could not set ALDO1 voltage");
            return false;
        }
    }
    if (!(en & (1u << PMU_ALDO1_EN_BIT)))
    {
        if (!pmu_w(PMU_LDO_EN, (uint8_t)(en | (1u << PMU_ALDO1_EN_BIT))))
        {
            ESP_LOGE(TAG, "could not enable ALDO1");
            return false;
        }
    }

    ESP_LOGI(TAG, "PMU aldo1_v=%02x ldo_en=%02x", v, en);
    vTaskDelay(pdMS_TO_TICKS(20));
    return true;
}

/* Raise BLDO2 to 3.3 V and enable it: the MAX98357A speaker amplifier's supply.
 *
 * BRD-003 assigns BLDO2 to the speaker, and LilyGo's PMU bring-up sets it to
 * 3300 mV and enables it beside the display's ALDO2 and the card's ALDO1.
 *
 * The amplifier is a class-D I2S DAC: with its rail down it accepts every I2S
 * clock and produces silence, which is the same silent-stub shape as the
 * haptic chip with P06 low and the SD card with ALDO1 low. That is why this is
 * called before the I2S channel is created rather than after.
 *
 * Returns false only if the PMU does not answer; a rail that is already up is a
 * success. */
bool board_speaker_rail_on(void)
{
    uint8_t v, en;

    if (!pmu_r(PMU_BLDO2_VOLT, &v) || !pmu_r(PMU_LDO_EN, &en))
    {
        ESP_LOGE(TAG, "PMU did not answer");
        return false;
    }

    if ((v & 0x1F) != PMU_BLDO2_3300MV)
    {
        if (!pmu_w(PMU_BLDO2_VOLT, (uint8_t)((v & 0xE0) | PMU_BLDO2_3300MV)))
        {
            ESP_LOGE(TAG, "could not set BLDO2 voltage");
            return false;
        }
    }
    if (!(en & (1u << PMU_BLDO2_EN_BIT)))
    {
        if (!pmu_w(PMU_LDO_EN, (uint8_t)(en | (1u << PMU_BLDO2_EN_BIT))))
        {
            ESP_LOGE(TAG, "could not enable BLDO2");
            return false;
        }
    }

    /* READ BACK, do not report the pre-write values. `v` and `en` above were
     * read before the writes, so logging them says what the rail looked like
     * and nothing about whether the write landed -- which is exactly the
     * question a failure here would need answered. */
    vTaskDelay(pdMS_TO_TICKS(20));
    {
        uint8_t v2 = 0, en2 = 0;

        if (!pmu_r(PMU_BLDO2_VOLT, &v2) || !pmu_r(PMU_LDO_EN, &en2))
        {
            ESP_LOGE(TAG, "bldo2 readback failed");
            return false;
        }

        /* Compact for the Log Viewer's 40 columns: voltage in mV, and the
         * enable byte, which bit 5 of is this rail. */
        ESP_LOGI(TAG, "bldo2 %dmV en%02x bit5=%d",
                 (int)(v2 & 0x1F) * 100 + 500, en2,
                 (en2 >> PMU_BLDO2_EN_BIT) & 1);

        if (!(en2 & (1u << PMU_BLDO2_EN_BIT)))
        {
            ESP_LOGE(TAG, "BLDO2 did not stay enabled");
            return false;
        }
    }
    return true;
}

/* Raise ALDO2 to 3.3 V and enable it. Returns false only if the PMU does not
 * answer; a rail that is already up is a success. */
bool board_display_rail_on(void)
{
    uint8_t v, en;

    if (!pmu_r(PMU_ALDO2_VOLT, &v) || !pmu_r(PMU_LDO_EN, &en))
    {
        ESP_LOGE(TAG, "PMU did not answer");
        return false;
    }

    if ((v & 0x1F) != PMU_ALDO2_3300MV)
    {
        if (!pmu_w(PMU_ALDO2_VOLT, (uint8_t)((v & 0xE0) | PMU_ALDO2_3300MV)))
        {
            ESP_LOGE(TAG, "could not set ALDO2 voltage");
            return false;
        }
    }
    if (!(en & (1u << PMU_ALDO2_EN_BIT)))
    {
        if (!pmu_w(PMU_LDO_EN, (uint8_t)(en | (1u << PMU_ALDO2_EN_BIT))))
        {
            ESP_LOGE(TAG, "could not enable ALDO2");
            return false;
        }
    }

    ESP_LOGI(TAG, "PMU aldo2_v=%02x ldo_en=%02x", v, en);
    vTaskDelay(pdMS_TO_TICKS(20));
    return true;
}

/* The board's pre-init hook: raise the SD card's rail before the shell's SD
 * stack tries to mount anything.
 *
 * This overrides the weak default in ports/esp_idf/app/main.c, which is called
 * from app_main before tdsh_espidf_init(). The port's own link_init() runs
 * later, from the UI task, so anywhere in there would be too late. */
void board_power_rails_preinit(void)
{
    gpio_config_t park = {
        .pin_bit_mask = (1ULL << SHARED_SPI_CS_NFC) |
                        (1ULL << SHARED_SPI_CS_LORA) |
                        (1ULL << SHARED_SPI_CS_SD),
        .mode = GPIO_MODE_OUTPUT,
    };

    if (!board_sd_rail_on())
        ESP_LOGW(TAG, "SD rail did not come up; a card will not mount");

    /* Park the other chips on the SD card's bus. See the note above. */
    if (gpio_config(&park) == ESP_OK)
    {
        gpio_set_level(SHARED_SPI_CS_NFC, 1);
        gpio_set_level(SHARED_SPI_CS_LORA, 1);
        gpio_set_level(SHARED_SPI_CS_SD, 1);
        ESP_LOGI(TAG, "spi cs parked");
    }
    else
    {
        ESP_LOGW(TAG, "could not park the shared spi chips");
    }
}

/* Log the rails this port owns, by reading them back.
 *
 * Written because the same information was already being logged at preinit and
 * that line was never once read: the desktop's log ring holds 64 lines and the
 * preinit line is emitted before the panel has even been initialised, so it is
 * always gone by the time anyone can open the Log Viewer. A diagnostic nobody
 * can read is not a diagnostic.
 *
 * This runs from link_init, which is late enough to survive in the ring and
 * still early enough to precede the SD watcher's first mount attempt.
 * Compact so it fits the Log Viewer's 40 columns. */
void board_log_rails(void)
{
    uint8_t a1 = 0xEE, a2 = 0xEE, en = 0xEE;

    if (!pmu_r(PMU_ALDO1_VOLT, &a1) || !pmu_r(PMU_ALDO2_VOLT, &a2) ||
        !pmu_r(PMU_LDO_EN, &en))
    {
        ESP_LOGW(TAG, "pmu silent");
        return;
    }
    /* The enable byte on its own line: it carries the haptic rail (P06 is the
     * expander, not this) and ALDO1, ALDO2 and BLDO2, and putting it last on
     * the previous line meant the Log Viewer's 40 columns cut it off -- which
     * is how a rail state went unread earlier in this port's life. */
    ESP_LOGI(TAG, "en %02x ald1 %02x ald2 %02x", en, a1, a2);
}

/* The battery, for System Monitor.
 *
 * Reads the registers the kernel driver measured on this board rather than
 * inferring anything: capacity straight from the gauge, voltage as a 14-bit
 * millivolt value, and the charge state from the same bits mainline decodes.
 *
 * A GAUGE ABOVE 100 IS NOT A PERCENTAGE. PMU-001 records that a freshly
 * powered AXP2101 reports more than 100 until its fuel gauge settles, and the
 * kernel driver returns -ENODATA rather than reporting it. Same here: percent
 * is -1, which System Monitor prints as "--%", because a wrong number on a
 * battery is worse than no number.
 *
 * Cheap by design -- five register reads on a bus this port already owns, no
 * allocation, no delay -- because System Monitor calls it twice a second. */
int board_battery_read(td_battery_t *out, int max)
{
    uint8_t st1 = 0, st2 = 0, hi = 0, lo = 0, pct = 0;
    int mv;
    int state;

    if (!out || max < 1)
        return 0;
    if (!pmu_r(PMU_STATUS1, &st1) || !pmu_r(PMU_STATUS2, &st2))
        return 0;
    if (!(st1 & PMU_BATT_PRESENT))
        return 0;                      /* no cell, or not detected */

    memset(out, 0, sizeof(*out));
    snprintf(out->label, sizeof(out->label), "Battery");

    out->percent = -1;
    if (pmu_r(PMU_BATT_PERCENT, &pct))
    {
        int p = pct & 0x7F;

        if (p <= 100)
            out->percent = p;
    }

    out->millivolts = 0;
    if (pmu_r(PMU_BATT_VOLT_H, &hi) && pmu_r(PMU_BATT_VOLT_L, &lo))
    {
        mv = ((hi & 0x3F) << 8) | lo;
        if (mv > 0)
            out->millivolts = mv;
    }

    switch (st2 & PMU_CHRG_STATE_MASK)
    {
    case 1:                            /* precharge */
    case 2:                            /* constant current */
    case 3:                            /* constant voltage */
        state = TD_BATT_CHARGING;
        break;
    case 4:                            /* done */
        state = TD_BATT_FULL;
        break;
    default:                           /* trickle, or nothing running */
        state = TD_BATT_DISCHARGING;
        break;
    }
    out->state = state;
    return 1;
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

/* Configure one linear expander pin as an output and drive it to `level`.
 * The configuration bit must be cleared first: a set bit means INPUT, and an
 * output register written while the pin is still an input changes nothing. */
static bool xl_pin_set(int linear, bool level)
{
    int port = linear / 8;
    int bit = linear % 8;
    uint8_t cfg_reg = (uint8_t)(XL9555_CONFIG0 + port);
    uint8_t out_reg = (uint8_t)(XL9555_OUTPUT0 + port);
    uint8_t cfg, out;

    if (!reg_read(cfg_reg, &cfg) || !reg_read(out_reg, &out))
        return false;

    if (cfg & (1u << bit))
        if (!reg_write(cfg_reg, (uint8_t)(cfg & ~(1u << bit))))
            return false;

    uint8_t want = level ? (uint8_t)(out | (1u << bit))
                         : (uint8_t)(out & ~(1u << bit));

    if (want != out && !reg_write(out_reg, want))
        return false;
    return true;
}

static bool xl_pin_high(int linear)
{
    return xl_pin_set(linear, true);
}

/* Pulse the touch controller's reset line, then release it.
 *
 * A level is not a pulse. The measured driver asserts the line (active low)
 * for 20 ms and then releases it for 60 ms before the first I2C transfer, and
 * on a cold boot the expander has every pin configured as an input, so the
 * line's state before this runs is the board's pull-up and not something this
 * code chose. Pulsing makes the starting state irrelevant.
 *
 * ONE PIN, and the second one this used to drive was actively harmful. The
 * hedge existed because two sources disagreed, but the board's own Arduino
 * variant settles it: EXPANDS_TOUCH_RST is expander pin 8. The other candidate,
 * pin 10, is EXPANDS_SD_DET -- the SD card's insert switch -- so driving it as
 * an output fought a physical switch and told anything that reads it that no
 * card was present. It is left alone now. */
/* Install the GPIO interrupt service once for every driver that wants one.
 *
 * See power.h for why this is idempotent: `gpio_install_isr_service()` reports
 * "GPIO isr service already installed" through GPIO_CHECK, which logs at ERROR
 * level, so a second caller puts an E line in the log at every boot for a
 * condition that is exactly what was wanted. The static flag makes the second
 * and later calls silent no-ops. */
bool board_gpio_isr_init(void)
{
    static bool s_installed;
    static bool s_failed;

    if (s_installed)
        return true;
    if (s_failed)
        return false;

    esp_err_t err = gpio_install_isr_service(0);
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE)
    {
        s_installed = true;
        return true;
    }

    ESP_LOGE(TAG, "isr service: %s", esp_err_to_name(err));
    s_failed = true;
    return false;
}

/* See power.h: the power key needs the PMU's interrupt status, which means the
 * register file the rest of this file keeps to itself. */
bool board_pmu_read(uint8_t reg, uint8_t *v)
{
    return pmu_r(reg, v);
}

bool board_pmu_write(uint8_t reg, uint8_t v)
{
    return pmu_w(reg, v);
}

bool board_touch_reset_release(void)
{
    bool ok = xl_pin_set(TOUCH_RST_LINEAR_A, false);

    vTaskDelay(pdMS_TO_TICKS(20));
    ok = xl_pin_high(TOUCH_RST_LINEAR_A) && ok;

    /* Compact so the line fits the Log Viewer's 40 columns. */
    ESP_LOGI(TAG, "rst %d=%d", TOUCH_RST_LINEAR_A, ok ? 1 : 0);
    vTaskDelay(pdMS_TO_TICKS(60));
    return ok;
}

/* Power the haptic amplifier by raising XL9555 P06.
 *
 * Same two-step as the display's switch: configure the pin as an output, then
 * drive the latch. Idempotent, so calling it again after a reset cannot hurt,
 * and it leaves the expander's other pins alone -- P07 in particular, which the
 * display is running from. */
bool board_haptic_power_on(void)
{
    bool ok = xl_pin_high(DRV_EN_BIT);

    ESP_LOGI(TAG, "haptic rail P%02d %s", DRV_EN_BIT, ok ? "on" : "FAILED");
    return ok;
}

bool board_display_power_on(void)
{
    uint8_t cfg, out;

    /* Rail first, then the switch. */
    if (!board_display_rail_on())
        ESP_LOGW(TAG, "PMU rail did not come up; continuing anyway");

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
