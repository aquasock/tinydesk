/*
 * usbhost.h - the one owner of the USB host stack on this board (see usbhost.c).
 */
#ifndef TWATCH_USBHOST_H
#define TWATCH_USBHOST_H

#include <stdbool.h>

/* Install the host stack and its library event loop, once. Idempotent; false
 * when the install fails. Every class driver calls this instead of installing
 * the host itself. */
bool usbhost_install(void);

/* True once the host stack is installed (the port is then a host until the
 * next restart). */
bool usbhost_installed(void);

/* At boot: apply the saved role (board key usb.host) and start the enabled
 * device classes (usb.dac). Call once, after the board keys are loaded and
 * before anything opens a file on the SD card -- the USB stream's DMA memory
 * has to be taken first (reference USB-005). */
void usbhost_boot(void);

/* True once the TinyDesk Shell's task exists (board_shell_task_ready(), called
 * by the shell bridge after xTaskCreate). The USB audio worker waits on this
 * before its first open, so the open cannot contend with the shell's 24 KB
 * stack allocation. */
bool usbhost_shell_ready(void);

#endif
