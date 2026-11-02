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
#include <stddef.h>
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

    /* Motion and quality. Speed is metres per second over ground, course is
     * degrees true, and accuracy is metres, derived from HDOP. Each has its
     * own flag because a stationary receiver reports no course rather than a
     * course of zero, and reporting the two as the same thing would turn
     * "unknown" into a confident north. */
    double gps_speed_mps;
    bool gps_have_speed;
    double gps_course_deg;
    bool gps_have_course;
    double gps_accuracy_m;
    bool gps_have_accuracy;
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

/* ---- magnetometer hard-iron calibration ------------------------------
 *
 * Offsets are learned by tumbling the board so the field sweeps a sphere; the
 * midpoint of each axis's range is the offset the surrounding metal adds. They
 * describe the metal around the sensor, so moving the board to a different
 * mount invalidates them.
 *
 * Sampling runs in the node task (mag_cal_poll), not in whatever command
 * started it, because that task owns the I2C bus and a command must not read
 * the sensor concurrently. A command starts the run, then polls
 * mag_cal_samples() and mag_cal_spans() for progress while the user tumbles.
 */
struct mag_offsets
{
    int16_t x, y, z;
};

bool     mag_cal_begin(void);          /* false if absent or already running */
void     mag_cal_poll(void);           /* from the node task, every tick */
bool     mag_cal_active(void);
uint32_t mag_cal_samples(void);
void     mag_cal_spans(int32_t out[3]);
/* Fitted sphere radius in uT so far; 0 when not solvable. This is the run's
 * progress: it approaches the local field only once the samples span enough
 * of the sphere. */
double   mag_cal_fit_radius_ut(void);
/* Print the sample cloud: centre, per-axis spread, octant coverage, duplicate
 * runs. Explains a bad fit instead of leaving it to guesswork. */
void     mag_cal_diagnose(void);
/* Fraction of samples in the least-populated octant about their centre.
 * 0.125 is even; below ~0.06 the board never saw that side. */
double   mag_cal_coverage(void);
bool     mag_cal_finish(struct mag_offsets *out, char *report, size_t n);

/* Centre of the sphere the samples lie on, in counts: the bias still present.
 * Adding it to the stored offsets removes the rest. */
bool     mag_cal_residual(int32_t out[3], double *radius_ut, double *scatter_ut);
/* Largest axis spread among the samples, in uT. Small means the board was
 * never turned and no fit can be trusted. */
double   mag_cal_rotation_ut(void);
/* Fit hard AND soft iron from the collected samples: a 3x3 correction that
 * reshapes the ellipsoid back to a sphere. `target_ut` is the local field
 * magnitude, since the shape is scale-free. */
bool     mag_cal_fit_ellipsoid(double target_ut, char *report, size_t n);
bool     mag_cal_using_matrix(void);
void     mag_cal_clearmatrix(void);
/* Copy the stored correction out, for display. False on the offsets-only path. */
bool     mag_cal_matrix(double M[3][3], double centre[3]);
/* Rescale the stored correction so a field measuring `measured_ut` reads
 * `target_ut`. The fit fixes the shape of the correction but not its absolute
 * size -- that comes from how much of the ellipsoid the samples covered, and a
 * partial tumble leaves the surface small so every reading comes out high.
 * A uniform factor changes magnitude only, leaving bearing and dip untouched;
 * scaling per axis would destroy the soft-iron information the fit found. */
bool     mag_cal_rescale(double measured_ut, double target_ut);
void     mag_cal_abort(void);
void     mag_cal_set_offsets(const struct mag_offsets *o);
struct mag_offsets mag_cal_offsets(void);
bool     mag_cal_save(void);           /* to /fs/mag_cal */
bool     mag_cal_load(void);           /* from /fs/mag_cal */


/* The most recent snapshot, refreshed in the background at a fixed interval
 * so a telemetry request never blocks on an 8.3 ms humidity conversion. */
const struct sensor_snapshot *sensors_latest(void);

#endif
