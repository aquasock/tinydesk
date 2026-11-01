/*
 * sensors.h - the four fitted parts, read into one snapshot.
 *
 * All four share the I2C bus on A4/A5 and are confirmed present and
 * identifying correctly (see core-reference SEN-006, SEN-007 and GPS-005):
 *
 *   0x10  PA1010D  GPS       NMEA over I2C, padded with 0x0A when idle
 *   0x1c  LIS3MDL  magnetometer  WHO_AM_I 0x3d
 *   0x44  SHT45    temperature and humidity, no id register
 *   0x47  BMP581   pressure and temperature, chip id 0x50
 *
 * The reading maths come from the standalone TinyNode drivers rather than
 * being re-derived: the SHT45 conversion, its CRC-8 and the LIS3MDL's +-4
 * gauss scaling were all measured on this hardware and are unchanged by the
 * move to a different firmware.
 */
#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>
#include <stdint.h>

struct sensor_snapshot
{
    /* Validity per group, so a part that stops answering is reported as
     * absent rather than as a stale or zero reading. */
    bool have_env;         /* SHT45 */
    bool have_pressure;    /* BMP581 */
    bool have_mag;         /* LIS3MDL */
    bool have_gps;         /* PA1010D */

    double temperature_c;  /* SHT45 */
    double humidity_pct;   /* SHT45 */

    double pressure_hpa;   /* BMP581 */
    double pressure_temp_c;

    double mag_x_ut, mag_y_ut, mag_z_ut;   /* LIS3MDL, microtesla */

    /* GPS: a fix and the position it gave. */
    bool gps_fix;
    uint8_t gps_sats;
    double gps_lat, gps_lon, gps_alt_m;
    uint32_t gps_unix;     /* UTC seconds from the last valid RMC, 0 if none */
};

/* Configure the three register-based parts. The GPS needs no setup. Returns
 * how many of the four answered. */
int sensors_begin(void);

/* Read all four into a snapshot. Parts that fail leave their have_* flag
 * false and their fields untouched. */
void sensors_read(struct sensor_snapshot *out);

/* Poll the GPS only. The receiver streams NMEA continuously at about
 * 480 bytes/s, so it has to be drained far more often than the other three
 * are read: at one 32-byte chunk per 2 s the module's buffer overruns and the
 * sentences parsed are tens of seconds old. Call this from the main loop. */
void sensors_poll_gps(void);

/* Re-read all four into the shared snapshot. Called from the node task on a
 * timer so a telemetry request never waits on a conversion. */
void sensors_refresh(void);

/* The most recent snapshot, refreshed in the background at a fixed interval
 * so a telemetry request never blocks on an 8.3 ms humidity conversion. */
const struct sensor_snapshot *sensors_latest(void);

#endif
