/*
 * usbdiag.h - a diagnostic for what attaches to the ESP32-S3 as a USB HOST.
 *
 * See usbdiag.c. The screen cannot be used as the output: this board's console
 * shares D+/D- with the OTG controller, so installing the host takes the
 * console away. Everything goes to /sd/diag/usbdiag.log instead.
 */
#ifndef TWATCH_USBDIAG_H
#define TWATCH_USBDIAG_H

#include <stdbool.h>

/* Register the `usbdiag` shell command. */
bool usbdiag_register_command(void);

#endif
