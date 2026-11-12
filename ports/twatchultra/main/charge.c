/*
 * charge.c - what the AXP2101 is doing about charging, read and reported.
 *
 * WHY THIS IS READ-ONLY ON ITS FIRST BUILD. Charging is the one subsystem on
 * this board where a wrong write damages hardware that is not easily replaced,
 * and the two obvious sources disagree. LilyGo's own init calls
 * `setChargeVoltage(4288)`, and SensorLib's setChargeVoltage() accepts only
 * 5000, 4000, 4100, 4200, 4350 and 4400 -- 4288 falls through to
 * `default: return false`, so the vendor's CV call does nothing at all and the
 * register keeps whatever the part was born with. A project that copied that
 * line would believe it had set a voltage it had not set, which is the worst of
 * both outcomes: no effect, and false confidence.
 *
 * So the first question is not "what should the voltage be" but "what is it
 * now". This file answers that by reading the five registers and decoding them.
 * Nothing is written until the measured values are known and the cell's own
 * spec has been checked against them.
 *
 * THE REGISTER MAP, from lewisxhe/SensorLib's AXP2101Charger, which is the
 * driver LilyGo's own firmware uses:
 *
 *   0x18  module enable   bit 1 = cell charger, bit 3 = fuel gauge
 *   0x30  ADC enable      bit 1 = TS pin (NTC battery temperature)
 *   0x61  precharge       low nibble * 25 mA
 *   0x62  fast charge     bits 4:0; <= 8 is code * 25 mA, above 8 is
 *                         200 + (code - 8) * 100 mA
 *   0x63  termination     low nibble * 25 mA
 *   0x64  CV voltage      bits 2:0 -> 5000, 4000, 4100, 4200, 4350, 4400 mV
 *
 * THE PART'S LIMITS ARE NOT THIS PROJECT'S LIMITS. The values above are the
 * decoder's; what this board should actually use is a separate decision, and
 * LilyGo documents 1100 mAh at 3.7 V with "use a charging current below 500 mA
 * to prevent the PMU from overheating" and "do not charge at a rate greater
 * than 0.5 C". 0.5 C of 1100 mAh is 550 mA, so their own 512 mA default sits
 * between their two statements, which is a reason to pick deliberately rather
 * than to copy.
 *
 * THE LOG LINES ARE SPLIT ON PURPOSE. The Log Viewer is 40 columns and clips,
 * and this file records three separate faults from diagnostics that were cut
 * off. So the raw bytes go on two short lines and the two decoded values that
 * matter -- CV and fast charge -- go on a third.
 */

#include "charge.h"

#include "esp_log.h"

#include "power.h"

#define TAG "twatch-chg"

/* Register addresses, from SensorLib's axp2101_regs. */
#define REG_MODULE_EN   0x18
#define REG_ADC_EN      0x30
#define REG_IPRECHG     0x61
#define REG_ICC         0x62
#define REG_ITERM       0x63
#define REG_CV          0x64

#define EN_CELL_CHARGE  (1u << 1)
#define EN_TS_MEASURE   (1u << 1)

/*
 * WHAT THIS BOARD SHOULD BE DOING, so that the values are checked rather than
 * assumed. These are the AXP2101's own power-on defaults, measured on this
 * watch on 2026-11-12 -- and they happen to be right for the fitted cell, which
 * is a 1100 mAh 3.7 V pack according to LilyGo:
 *
 *   4200 mV  is the standard 4.2 V full-charge voltage for a 3.7 V Li-ion cell.
 *            NOT 4350 or 4400, which are for high-voltage cells, and NOT 5000,
 *            which is the USB setting and would damage this pack.
 *   500 mA   is 0.45 C. LilyGo's hardware page says "use a charging current
 *            below 500 mA to prevent the PMU from overheating" and "do not
 *            charge at a rate greater than 0.5 C", which for 1100 mAh is
 *            550 mA. Their own firmware asks for 512, which the driver encodes
 *            to the same register code as 500, so all three agree in practice.
 *   125 mA   precharge, for a deeply discharged cell.
 *   125 mA   termination, about 11 percent of capacity, which is the usual
 *            shape and not a mis-set default.
 *
 * A mismatch is reported as a warning rather than corrected, because correcting
 * it would mean writing a full-charge voltage on the strength of a table this
 * file read out of someone else's driver. Changing the cell means changing
 * these constants deliberately, with the new cell's spec in hand.
 */
#define WANT_CV_MV          4200
#define WANT_FAST_MA        500
#define WANT_PRECHARGE_MA   125
#define WANT_TERMINATION_MA 125

static int decode_fast_ma(uint8_t reg)
{
    uint8_t code = reg & 0x1F;

    if (code <= 8)
        return code * 25;
    return 200 + (code - 8) * 100;
}

static int decode_cv_mv(uint8_t reg)
{
    switch (reg & 0x07)
    {
    case 0: return 5000;
    case 1: return 4000;
    case 2: return 4100;
    case 3: return 4200;
    case 4: return 4350;
    case 5: return 4400;
    default: return 0;          /* 6 and 7 are not defined by the part */
    }
}

bool charge_report(charge_cfg_t *out)
{
    charge_cfg_t c;

    if (!board_pmu_read(REG_MODULE_EN, &c.raw_18) ||
        !board_pmu_read(REG_ADC_EN, &c.raw_30) ||
        !board_pmu_read(REG_IPRECHG, &c.raw_61) ||
        !board_pmu_read(REG_ICC, &c.raw_62) ||
        !board_pmu_read(REG_ITERM, &c.raw_63) ||
        !board_pmu_read(REG_CV, &c.raw_64))
    {
        ESP_LOGW(TAG, "pmu silent");
        return false;
    }

    c.cell_charge_enabled = (c.raw_18 & EN_CELL_CHARGE) != 0;
    c.ts_measure_enabled = (c.raw_30 & EN_TS_MEASURE) != 0;
    c.precharge_ma = (c.raw_61 & 0x0F) * 25;
    c.fast_charge_ma = decode_fast_ma(c.raw_62);
    c.termination_ma = (c.raw_63 & 0x0F) * 25;
    c.cv_mv = decode_cv_mv(c.raw_64);

    /* Three short lines rather than one long one: the ring clips at 40 columns
     * and a truncated voltage report is worse than none. */
    ESP_LOGI(TAG, "18%02x 30%02x 61%02x", c.raw_18, c.raw_30, c.raw_61);
    ESP_LOGI(TAG, "62%02x 63%02x 64%02x", c.raw_62, c.raw_63, c.raw_64);
    ESP_LOGI(TAG, "cv%d ic%d", c.cv_mv, c.fast_charge_ma);

    /* The check. Everything above is a report; this is the part that makes the
     * report mean something on the next boot too. One line, and it only speaks
     * when something is wrong -- a boot that says nothing here was correct. */
    if (!c.cell_charge_enabled)
        ESP_LOGW(TAG, "cell charge is OFF");
    if (c.cv_mv != WANT_CV_MV)
        ESP_LOGW(TAG, "cv %d want %d", c.cv_mv, WANT_CV_MV);
    if (c.fast_charge_ma != WANT_FAST_MA)
        ESP_LOGW(TAG, "ic %d want %d", c.fast_charge_ma, WANT_FAST_MA);
    if (c.precharge_ma != WANT_PRECHARGE_MA)
        ESP_LOGW(TAG, "ip %d want %d", c.precharge_ma, WANT_PRECHARGE_MA);
    if (c.termination_ma != WANT_TERMINATION_MA)
        ESP_LOGW(TAG, "it %d want %d", c.termination_ma, WANT_TERMINATION_MA);

    if (out)
        *out = c;
    return true;
}
