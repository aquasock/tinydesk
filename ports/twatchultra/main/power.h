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
#include "tinydesk/td_sysinfo.h"

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

/* Raise the AXP2101's ALDO1 to 3.3 V and enable it: the SD card's supply.
 *
 * MUST RUN BEFORE THE SD STACK IS STARTED. An unpowered card does not answer,
 * so a mount fails with a timeout that reads as a wiring fault -- the same
 * silent shape as the haptic rail, which answers I2C with its supply off. */
bool board_sd_rail_on(void);

/* Raise the AXP2101's BLDO2 to 3.3 V and enable it: the speaker amp's supply.
 *
 * MUST RUN BEFORE THE I2S CHANNEL IS CREATED. With the rail down the MAX98357A
 * clocks in every sample and outputs silence, so the failure reads as a broken
 * I2S configuration rather than as an unpowered amplifier. */
bool board_speaker_rail_on(void);

/* Power the DRV2605 haptic amplifier by raising XL9555 P06.
 *
 * MUST RUN BEFORE THE HAPTIC DRIVER IS PROBED. The chip answers I2C with its
 * supply switched off, so a driver that skips this reads a healthy device and
 * then produces no movement -- a failure that looks exactly like a dead motor
 * or a wrong effect library. */
bool board_haptic_power_on(void);

/* Release the touch controller's reset, which lives on the expander, not on
 * an SoC GPIO. Must run before touch_init() talks to 0x1A. */
bool board_touch_reset_release(void);

/* Install the GPIO interrupt service, once, for every driver that wants one.
 *
 * IDEMPOTENT, AND THAT IS THE POINT. `gpio_install_isr_service()` logs at ERROR
 * level when the service is already installed -- GPIO_CHECK reports "GPIO isr
 * service already installed" and returns ESP_ERR_INVALID_STATE. With the touch
 * panel and the BOOT pusher both wanting an interrupt, the second caller was
 * putting an E line in the log at every boot, which looks like a fault and is
 * not. One guarded installer removes both the error and the ordering question:
 * any driver may call this and only the first call does anything. */
bool board_gpio_isr_init(void);

/* The battery, for System Monitor, from the AXP2101's fuel gauge.
 *
 * Overrides the weak board_battery_read() in the shared ESP-IDF app. Returns
 * the number of batteries filled in (0 when there is no cell or no gauge), and
 * reports percent as -1 rather than a wrong number while the gauge settles. */
int board_battery_read(td_battery_t *out, int max);

/* Log ALDO1/ALDO2 and the LDO enable register, read back from the PMU.
 *
 * Call late enough to survive in the desktop's 64-line log ring: the same
 * values logged at preinit are always scrolled away before anyone can look. */
void board_log_rails(void);

/* Log the expander's configuration and output registers. */
void board_dump_expander(void);

#endif
