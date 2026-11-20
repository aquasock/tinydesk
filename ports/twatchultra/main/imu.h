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
#include "tinydesk/td_sysinfo.h"
#include <stddef.h>
#include <stdint.h>

/* Declared, not included: this header must stay free of the BHY2 SDK so the
 * desktop apps can include it without the whole protocol. */
struct bhy2_dev;

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

/* The BHY2 device, for companion instruments (klio.c) that speak the same
 * protocol. NULL until imu_start() succeeds. */
struct bhy2_dev *imu_dev(void);

/* The table the desktop's IMU app reads (td_imu_ops_t). */
const td_imu_ops_t *board_imu_ops(void);

/* Turn acceleration and orientation on at `hz`. Cheap when already on. */
bool imu_stream_on(float hz);

/* "FACE UP", "TILT UP" and the rest -- the posture word the window shows, for
 * any acceleration. "--" when there is no valid reading. */
const char *imu_posture(const td_imu_t *r);

/* The latest orientation in degrees, false until the part has reported one. */
bool imu_orientation(float *heading, float *pitch, float *roll);

/* ---- data injection, for experiments that feed the part synthetic motion ----
 *
 * Not a feature: this exists so klio.c can ask whether a classifier on the
 * Fuser core will learn from host-supplied samples. The command is present in
 * this firmware (entry 54); whether the data reaches the ALGORITHMS is the open
 * question these three calls exist to answer.
 *
 * imu_inject_begin() sets read-time or step-by-step mode and puts the async
 * status bit back, which the SDK's own helper clears and never restores.
 * imu_inject_frame() writes one BHY2_CMD_INJECT_DATA packet with the payload
 * exactly as given -- no padding, because a FIFO-format frame is parsed by
 * walking sensor ids and a trailing zero byte is a byte the part did not send.
 * imu_inject_end() returns the part to normal mode.
 */
/* Stop the physical sensors and empty the FIFO, then drain repeatedly until it
 * stays empty. Pattern-table work (read_pattern, write_pattern,
 * set_pattern_states, similarity) is pure computation on the part and does not
 * need the sensors streaming -- but leaving them streaming while several
 * blocking I2C calls run overflows the FIFO, and a dropped packet during those
 * calls is indistinguishable from a corrupt result. */
void imu_quiesce(void);

/* Drain the FIFO regardless of whether a sensor is nominally on. The normal
 * imu_poll() refuses when s_sensor_on is false, which is exactly the state a
 * quiesce leaves behind. */
void imu_drain_hard(void);

bool imu_inject_begin(int mode);
bool imu_inject_frame(const uint8_t *payload, uint16_t len);
void imu_inject_end(void);

/* ---- what has to be put back after the part reboots ----------------------
 *
 * The BHI260AP's firmware is uploaded into its RAM by this host, so its pattern
 * table and any other RAM-resident state start empty on every boot. Anything
 * that needs to outlive a reset registers here, and imu_start() calls it once the
 * firmware is up.
 *
 * WEAK ON PURPOSE: imu.c must not depend on klio.c, and a build without it must
 * still link. imu.c defines an empty default; klio.c's strong definition wins.
 */
void klio_after_boot(void);

#endif
