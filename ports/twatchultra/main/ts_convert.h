/*
 * ts_convert.h - the TS thermistor's voltage to temperature, with no hardware.
 *
 * Deliberately free of ESP-IDF: this is pure arithmetic on numbers a caller has
 * already read off the bus, so it can be compiled and swept on a host. The
 * alternative -- leaving it inside charge.c next to the I2C calls -- would put
 * the one piece of this feature that can be wrong in a plausible way beyond the
 * reach of any test.
 *
 * THE PART DATA, from PMU-007: the AXP2101 sources a selectable constant
 * current through an NTC between TS and GND, nominally 10 kohm at 25 C, and the
 * datasheet (X-Powers V1.4, section 6.7.4.4, Table 6-3) tabulates the result
 * for the Mitsubishi TH11-3H103F reference part. This file reproduces that
 * table rather than curve-fitting it, because the Steinhart-Hart coefficients
 * that circulate in public drivers disagree with the vendor's own table by
 * about 2.6 C.
 */
#ifndef TWATCH_TS_CONVERT_H
#define TWATCH_TS_CONVERT_H

#include <stdint.h>

/*
 * The TS pin's control register, reg50H, decoded:
 *   [4]    0 = battery thermistor feeding the charger, 1 = fixed external input
 *   [3:0]   bits 3:2 enable the current source, bits 1:0 select it
 */
int ts_current_ua(uint8_t ctrl);

/*
 * Convert a TS reading to tenths of a degree Celsius.
 *
 * `mv` is the 14-bit ADC value scaled at 0.5 mV per count, as the register pair
 * 0x36/0x37 presents it; `ctrl` is reg50H.
 *
 * Returns TD_NTC_NONE when the reading is the part's open-pin code, TD_NTC_NOT_TEMP
 * when the pin is not in thermistor mode, and clamps at the table's ends rather
 * than extrapolating -- beyond -20 or +60 C the part's own accuracy is not
 * specified, so a number there would be invented.
 *
 * The current is taken from `ctrl` and the reading scaled to the table's 50 uA
 * first. That is exact rather than approximate: the thermistor's resistance does
 * not depend on the current, and the voltage across a resistor is proportional
 * to it.
 */
int ts_tenths_c(int mv, uint8_t ctrl);

#endif
