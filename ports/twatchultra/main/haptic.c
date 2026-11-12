/*
 * haptic.c - the DRV2605 haptic amplifier at I2C 0x5A, driving an ERM motor.
 *
 * Taken from LilyGo's own driver rather than inferred from the datasheet,
 * because the two things that go wrong here are both silent and both are
 * settled by their code instead of by the register map.
 *
 * WHAT LILYGO DOES (Xinyuan-LilyGO/LilyGoLib, LilyGoWatchUltra.cpp):
 *
 *   initDrv()     drv.begin(Wire, DRV2605_SLAVE_ADDRESS)
 *                 drv.selectLibrary(1)
 *                 drv.setMode(HapticMode::INTERNAL_TRIGGER)
 *                 drv.setWaveform(0, _effects)     _effects defaults to 1
 *                 drv.setWaveform(1, 0)
 *                 drv.run()
 *
 *   vibrator()    drv.setWaveform(0, _effects)
 *                 drv.setWaveform(1, 0)
 *                 drv.run()
 *
 * So the library and the mode are set ONCE, at init, and a tap only rewrites
 * the waveform and fires GO. That split is the design below. The underlying
 * register class is lewisxhe/SensorLib, src/haptic/HapticDriver_DRV2605.cpp,
 * where selectLibrary() writes 0x03, setMode(INTERNAL_TRIGGER) writes 0x00 to
 * 0x01, setWaveform(n) writes 0x04+n, and run() writes 1 to GO at 0x0C.
 *
 * THE EFFECT LIBRARY IS THE FIRST SILENT FAILURE. Library 1 is the ERM set and
 * library 6 is the LRA set; this board's FEEDBACK register (0x1A) reads 0x36,
 * whose bit 7 is 0, so the motor is ERM (BRD-005). Give an ERM the LRA library
 * and it buzzes weakly or not at all, with no error anywhere.
 *
 * THE RAIL IS THE SECOND, and it is the one that looks like dead hardware: the
 * chip answers every I2C transaction with its supply switched off, so a driver
 * that only probes reports success and then nothing moves. board_haptic_power_on()
 * raises XL9555 P06 for exactly that reason and must run first.
 *
 * THE THIRD IS THE SEQUENCE TERMINATOR. WAVESEQ2 must be written to 0, because
 * the sequence ends at the first zero; without it the chip replays whatever was
 * already in the wave registers, which is how a "short click" becomes a second
 * of buzzing. Both wave slots are rewritten on every tap for that reason, even
 * though only slot 0's value changes -- the terminator is part of the payload,
 * not part of the configuration.
 *
 * Effect 1 in library 1 is Strong Click at 100 percent, which is what a button
 * should feel like and what LilyGo's default is.
 */

#include "haptic.h"

#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "power.h"

#define TAG "twatch-haptic"

#define HAPTIC_ADDR 0x5A
#define HAPTIC_HZ   400000

/* DRV2605 register map, the subset this port uses. */
#define REG_STATUS   0x00
#define REG_FEEDBACK 0x1A        /* bit 7: 0 = ERM, 1 = LRA */
#define REG_MODE     0x01
#define REG_LIBRARY  0x03
#define REG_WAVESEQ1 0x04
#define REG_GO       0x0C

#define MODE_INTERNAL_TRIGGER 0x00
#define LIBRARY_ERM           0x01
#define EFFECT_STRONG_CLICK   0x01

/* STATUS bits 7:5 are DEVICE_ID and 0b111 is the DRV2605. */
#define DEVICE_ID_MASK 0xE0
#define DEVICE_ID_DRV2605 0xE0

static i2c_master_dev_handle_t s_dev;

static bool wr(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };

    return s_dev && i2c_master_transmit(s_dev, buf, sizeof(buf), 100) == ESP_OK;
}

static bool rd(uint8_t reg, uint8_t *val)
{
    return s_dev && i2c_master_transmit_receive(s_dev, &reg, 1, val, 1, 100) == ESP_OK;
}

bool haptic_init(void)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = HAPTIC_ADDR,
        .scl_speed_hz = HAPTIC_HZ,
    };
    uint8_t status = 0, feedback = 0;

    /* The bus belongs to power.c: every peripheral here shares it. */
    if (!board_i2c())
        return false;
    if (i2c_master_bus_add_device(board_i2c(), &cfg, &s_dev) != ESP_OK)
    {
        ESP_LOGE(TAG, "no device at 0x%02X", HAPTIC_ADDR);
        s_dev = NULL;
        return false;
    }

    /* Identify it before configuring it. 0x5A is also the address a CST92xx
     * family header lists for touch, so a chip answering at this address is not
     * by itself evidence of which part it is (TCH-001); DEVICE_ID is. */
    if (!rd(REG_STATUS, &status) || (status & DEVICE_ID_MASK) != DEVICE_ID_DRV2605)
    {
        ESP_LOGE(TAG, "status %02x is not a DRV2605", status);
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return false;
    }
    if (!rd(REG_FEEDBACK, &feedback))
        feedback = 0;

    /* Internal trigger and the ERM library, once. A tap relies on both. */
    if (!wr(REG_MODE, MODE_INTERNAL_TRIGGER) ||
        !wr(REG_LIBRARY, LIBRARY_ERM))
    {
        ESP_LOGE(TAG, "could not configure the chip");
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return false;
    }

    /* Leave a silent sequence loaded, so a stray GO cannot replay an effect
     * from whatever the previous firmware left in these registers. */
    (void)wr(REG_WAVESEQ1, 0x00);
    (void)wr(REG_WAVESEQ1 + 1, 0x00);

    /* Short enough for the Log Viewer's 40 columns. */
    ESP_LOGI(TAG, "DRV2605 lib%d %s", LIBRARY_ERM,
             (feedback & 0x80) ? "LRA?" : "ERM");
    return true;
}

void haptic_tap(void)
{
    if (!s_dev)
        return;

    /* Waveform and terminator on every tap: see the note at the top about GO
     * replaying stale registers. */
    if (!wr(REG_WAVESEQ1, EFFECT_STRONG_CLICK) ||
        !wr(REG_WAVESEQ1 + 1, 0x00) ||
        !wr(REG_GO, 0x01))
    {
        ESP_LOGW(TAG, "tap failed");
        return;
    }
    /* Nothing is polled: the chip plays the effect and stops by itself. */
}
