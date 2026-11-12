/*
 * pwrkey.h - Pusher A, the watch's power key, as a key.
 *
 * Pusher A is not a GPIO. It reaches the SoC only as the AXP2101's PEK
 * (power-key) input, which the PMU reports through its own interrupt line on
 * GPIO 7. So this driver is an I2C driver with a GPIO wake-up rather than a
 * button driver, and it is the reason the port can now report VBUS plug and
 * removal as well: the same line carries both.
 */
#ifndef TWATCH_PWRKEY_H
#define TWATCH_PWRKEY_H

#include <stdbool.h>

/* Called with the bytes the key stands for, the same way the touch panel's
 * margin buttons and the BOOT pusher deliver theirs. */
typedef void (*pwrkey_input_fn)(const char *data, int len);

/*
 * Clear anything the PMU is holding, enable the short-press interrupt, and
 * start watching GPIO 7; false when the PMU does not answer or the line could
 * not be made to work.
 *
 * IT REFUSES TO ENABLE THE INTERRUPT WHEN THE LINE READS LOW at start-up, and
 * that guard is the whole point of this function's shape. BRD-002 records this
 * exact pin storming and hanging a boot on the Linux side: the AXP2101 holds
 * its IRQ output asserted until the status bits are written back, so a pending
 * event that nobody clears is a permanently low line, and a level-triggered
 * handler on a permanently low line never returns. Refusing to arm in that case
 * leaves the board usable at the cost of a button, which is the right trade.
 *
 * `pwrkey.pin = -1` turns the key off; the interrupt still reports VBUS, which
 * is how to tell a key fault from a PMU fault.
 */
bool pwrkey_init(pwrkey_input_fn input);

#endif
