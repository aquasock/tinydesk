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

#endif
