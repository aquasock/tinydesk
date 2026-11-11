/*
 * power.h - the T-Watch Ultra's switched rails, and the shared I2C bus.
 *
 * Every peripheral on this board is on ONE I2C bus (SDA 3, SCL 2): the PMU at
 * 0x34, the XL9555 expander at 0x20, the RTC at 0x51, the haptic driver at
 * 0x5A and the touch controller at 0x1A. This module owns that bus so the
 * display power and the touch driver do not each install their own.
 */
#ifndef TWATCH_POWER_H
#define TWATCH_POWER_H

#include <stdbool.h>

#include "driver/i2c_master.h"

/* The board's single I2C bus, created on first use. NULL on failure. */
i2c_master_bus_handle_t board_i2c(void);

/*
 * Power the AMOLED.
 *
 * THE PANEL'S RAIL IS ENABLED BY XL9555 P07 AND NOTHING ELSE. The AXP2101's
 * ALDO2 feeds the display rail, but the switch that actually connects it is
 * P07 on the expander at 0x20. On a cold boot the PCA9555 comes up with every
 * pin configured as an input, so P07 floats and the panel stays dark however
 * correct the QSPI traffic is.
 *
 * This cost a flash cycle to find: the port was written, built and flashed
 * with no power enable at all, because the display looked like a pure SPI
 * problem. It is not.
 *
 * Both registers are read-modify-written rather than assigned, because the
 * same port carries the haptic rail (P06) and other pins this port does not
 * own, and clobbering them would break hardware that otherwise works.
 */
bool board_display_power_on(void);

/*
 * Raise the AXP2101's ALDO2 and enable it. This is the rail the display runs
 * from; P07 above only switches it. Order matters: the rail has to be
 * generating before the switch closes, or the panel sees an unpowered line
 * and stays dark with no signal that anything is wrong.
 */
bool board_display_rail_on(void);

/* Log the expander's configuration and output registers. */
void board_dump_expander(void);

#endif
