/*
 * charge.h - what the AXP2101 is doing about charging, read and reported.
 *
 * This exists because the PMU's charge configuration has never been touched by
 * this port, and the right first move on a battery is to find out what the part
 * is already doing rather than to write a value and hope. Everything here is a
 * read except the one documented call in the header.
 */
#ifndef TWATCH_CHARGE_H
#define TWATCH_CHARGE_H

#include <stdbool.h>
#include <stdint.h>

/* What the part is configured to do, decoded from its own registers. */
typedef struct
{
    bool cell_charge_enabled;   /* REG 0x18 bit 1 */
    bool ts_measure_enabled;    /* REG 0x30 bit 1: NTC temperature sensing */
    int precharge_ma;           /* REG 0x61 low nibble * 25 */
    int fast_charge_ma;         /* REG 0x62 bits 4:0, two-part scale */
    int termination_ma;         /* REG 0x63 low nibble * 25 */
    int cv_mv;                  /* REG 0x64 bits 2:0, six fixed steps */
    uint8_t raw_18, raw_30, raw_61, raw_62, raw_63, raw_64;
} charge_cfg_t;

/*
 * Read the charge configuration and log it. Nothing is written.
 *
 * Returns false only when the PMU does not answer. The values it fills in are
 * whatever the part currently holds, which on a board that has never configured
 * charging means its power-on defaults -- and those are what this project
 * needs to know before it decides whether anything should change.
 */
bool charge_report(charge_cfg_t *out);

/*
 * Enable the charging interrupts and act on them, called once at start-up.
 *
 * `vbus in` alone only says a cable arrived. The AXP2101 also reports when the
 * cell actually starts taking current and when it has finished, on bank 2 of
 * the same status block, and those are the transitions worth reacting to: a
 * cable that is plugged into a dead charger, or a charge that stops early,
 * looks identical to a working one if only VBUS is watched.
 *
 * Idempotent and safe to call before pwrkey_init(); it only writes two enable
 * bits and leaves the rest alone.
 */
void charge_watch_enable(void);

/*
 * Handle one interrupt's worth of status bytes.
 *
 * Called by the power-key driver, which owns the GPIO and clears the status.
 * This function does not touch the PMU; it only decides what the bits mean and
 * logs the transitions, so the charge knowledge stays in one file and the
 * button driver stays a button driver.
 */
void charge_on_irq(const uint8_t status[3]);

/*
 * Read the TS pin and report what it says about a battery thermistor.
 *
 * THE TS PIN IS THE BATTERY'S NTC INPUT, not an ambient sensor: it exists so
 * the charger can refuse to charge a cell that is too hot or too cold, and its
 * reading is a VOLTAGE across an external resistor divider, not a temperature
 * the part computes. So the raw millivolts are logged alongside any decoded
 * temperature, because the decode depends on an NTC part number this project
 * does not know -- the schematic marks the NTC position "NC".
 *
 * An absent thermistor is not an error to the part. A floating TS pin still
 * produces a plausible number, so the only way to tell a fitted NTC from an
 * empty pad is to look at what the pin actually reads and whether it moves.
 *
 * Returns false when the PMU does not answer; the pin's state is reported in
 * *ts_mv and *ctrl regardless.
 */
bool charge_ts_report(int *ts_mv, uint8_t *ctrl);

/*
 * The TS pin in millivolts, or -1 when the PMU does not answer. No logging and
 * no side effects, so a caller on a tick can use it -- System Monitor asks
 * twice a second and must not fill the log ring with it.
 */
int charge_ts_mv(void);

#endif
