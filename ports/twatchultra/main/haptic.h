/*
 * haptic.h - the DRV2605 haptic amplifier, so a press can be felt.
 *
 * The chip is at I2C 0x5A on the shared bus and drives an ERM motor. Its
 * supply is gated by XL9555 P06, so board_haptic_power_on() has to run first;
 * see the note there for why that failure is silent.
 */
#ifndef TWATCH_HAPTIC_H
#define TWATCH_HAPTIC_H

#include <stdbool.h>

/*
 * Probe and configure the chip; false when it does not answer or turns out to
 * be something else.
 *
 * This sets the effect library and the trigger mode once, and leaves a silent
 * waveform loaded. It deliberately plays nothing: bring-up should not buzz.
 */
bool haptic_init(void);

/*
 * Play one short click.
 *
 * Safe to call before haptic_init() and safe to call when the chip is absent:
 * it returns quietly. Feedback is a nicety, never a thing whose failure
 * matters, so no caller checks its result and nothing blocks on it.
 */
void haptic_tap(void);

#endif
