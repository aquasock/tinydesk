/*
 * bleuart.h - a serial console over Bluetooth LE (see bleuart.c).
 *
 * The service is Nordic's UART UUID pair, so ordinary BLE serial clients find
 * and use it without a custom app. The radio must already be up: this registers
 * onto the NimBLE host that bt_hid.c starts, which requires bt.enable = 1.
 */
#ifndef TWATCH_BLEUART_H
#define TWATCH_BLEUART_H

#include <stdbool.h>
#include <stdint.h>

/* Register the GATT service and start advertising. Idempotent. Called from
 * bt_hid.c's sync callback through board_ble_uart_on_sync(). */
bool bleuart_register(void);   /* before the NimBLE host starts */

bool bleuart_start(void);      /* after it syncs: advertise   */

/* True once the service is registered and advertising. */
bool bleuart_ready(void);

/* One byte the client sent, or -1. Non-blocking; this is a display source's
 * read, which the shell calls in its own loop. */
int bleuart_read(void);

/* Send bytes to the client. Best-effort and non-blocking: a full ring drops
 * output rather than stalling the shell. Returns the bytes accepted. */
int bleuart_write(const uint8_t *buf, int len);

/* The `bleuart` shell command. */
bool bleuart_register_command(void);

#endif
