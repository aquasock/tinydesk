/*
 * imu.h - the Bosch BHI260AP smart sensor (imu.c).
 *
 * The part is on the shared I2C bus at 0x28 and is powered by AXP2101 ALDO4 at
 * 1.8 V, which power.c raises at bring-up. Its ~121 KB firmware is a blob in the
 * `moddata` partition and is streamed into the part on FIRST USE, not at boot.
 */
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Upload the firmware and boot the part if that has not happened yet. False
 * with imu_why() set: no rail, no device, no firmware blob, or the part
 * refused. */
bool imu_start(void);

/* Turn the accelerometer off but leave the firmware loaded -- the upload is the
 * expensive part and the sensor idles at microamps. */
void imu_stop(void);

bool imu_ready(void);

/* Why imu_start() failed, in words. Never NULL, "" before any attempt. */
const char *imu_why(void);

/* Kernel version into `out`, e.g. "BHI260AP fw kernel 1.0". False when not
 * started. */
bool imu_version(char *out, size_t cap);

/* Turn the accelerometer on at `hz`, or off at 0. Idempotent. */
bool imu_accel_enable(float hz);

/* Drain the FIFO and dispatch to the sensor callbacks. One I2C read when
 * nothing is waiting. */
bool imu_poll(void);

/* The latest acceleration in g, and how many samples have arrived since start.
 * False when no sample has been seen yet. */
bool imu_accel(float *x, float *y, float *z, uint32_t *count);

bool imu_sensor_on(void);

bool imu_register_command(void);

#endif
