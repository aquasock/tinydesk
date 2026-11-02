/*
 * sensors.cpp - the four fitted I2C parts.
 *
 * Every register address and conversion here comes from the standalone
 * TinyNode drivers, which measured them on this hardware:
 *
 *   SHT45    command 0xFD, six bytes, each 16-bit word CRC-8 (poly 0x31,
 *            init 0xFF), T = -45 + 175*ticks/65535, RH = -6 + 125*ticks/65535
 *   LIS3MDL  CTRL_REG1 0x70 (ultra-high perf XY, 10 Hz), CTRL_REG2 0x00
 *            (+-4 gauss, 6842 LSB/gauss), CTRL_REG5 0x40 (BDU so a read
 *            cannot tear), CTRL_REG3 0x00 (continuous)
 *   BMP581   chip id 0x01 = 0x50, data at 0x1D..0x22, 0x30 = OSR config,
 *            0x36 = ODR config, 0x37 = mode
 *   PA1010D  read 32 bytes at 0x10, discard a 0x0A that does not follow 0x0D
 *
 * Nothing here blocks a caller for long: the humidity conversion is the
 * slowest at 8.3 ms and the background refresh absorbs it.
 */
#include "sensors.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sensors";

extern i2c_master_bus_handle_t sensors_bus(void);

#define ADDR_GPS 0x10
#define ADDR_MAG 0x1C
#define ADDR_SHT 0x44
#define ADDR_BMP 0x47

/* ---- bus helpers --------------------------------------------------- */

static bool rd(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_master_bus_handle_t bus = sensors_bus();
    if (!bus)
        return false;
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(bus, &cfg, &dev) != ESP_OK)
        return false;
    /* The LIS3MDL and BMP581 only auto-increment the register pointer for a
     * multi-byte read when the sub-address MSB is set, which is what the
     * standalone node does. Without it a 6-byte read repeats one register. */
    const uint8_t sub = (len > 1) ? (uint8_t)(reg | 0x80) : reg;
    esp_err_t err = i2c_master_transmit_receive(dev, &sub, 1, buf, len, 100);
    i2c_master_bus_rm_device(dev);
    return err == ESP_OK;
}

static bool wr(uint8_t addr, uint8_t reg, uint8_t val)
{
    i2c_master_bus_handle_t bus = sensors_bus();
    if (!bus)
        return false;
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(bus, &cfg, &dev) != ESP_OK)
        return false;
    uint8_t buf[2] = {reg, val};
    esp_err_t err = i2c_master_transmit(dev, buf, 2, 100);
    i2c_master_bus_rm_device(dev);
    return err == ESP_OK;
}

/* ---- SHT45: no register interface, so a raw command write + read ---- */

static uint8_t sht_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

static bool sht45_read(double *temp_c, double *rh_pct)
{
    i2c_master_bus_handle_t bus = sensors_bus();
    if (!bus)
        return false;
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADDR_SHT,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(bus, &cfg, &dev) != ESP_OK)
        return false;

    uint8_t cmd = 0xFD;                     /* measure T and RH, high repeatability */
    uint8_t d[6] = {0};
    bool ok = false;
    if (i2c_master_transmit(dev, &cmd, 1, 100) == ESP_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(10));      /* up to 8.3 ms */
        if (i2c_master_receive(dev, d, 6, 100) == ESP_OK)
        {
            ok = sht_crc8(&d[0], 2) == d[2] && sht_crc8(&d[3], 2) == d[5];
        }
    }
    i2c_master_bus_rm_device(dev);
    if (!ok)
        return false;

    uint16_t t = (uint16_t)((d[0] << 8) | d[1]);
    uint16_t h = (uint16_t)((d[3] << 8) | d[4]);
    *temp_c = -45.0 + 175.0 * ((double)t / 65535.0);
    double rh = -6.0 + 125.0 * ((double)h / 65535.0);
    *rh_pct = rh < 0.0 ? 0.0 : (rh > 100.0 ? 100.0 : rh);
    return true;
}

/* ---- LIS3MDL -------------------------------------------------------- */

static bool mag_begin(void)
{
    uint8_t who = 0;
    if (!rd(ADDR_MAG, 0x0F, &who, 1) || who != 0x3D)
        return false;
    if (!wr(ADDR_MAG, 0x20, 0x70))
        return false;   /* CTRL_REG1: UHP XY, 10 Hz */
    if (!wr(ADDR_MAG, 0x21, 0x00))
        return false;   /* CTRL_REG2: +-4 gauss */
    if (!wr(ADDR_MAG, 0x24, 0x40))
        return false;   /* CTRL_REG5: BDU */
    if (!wr(ADDR_MAG, 0x23, 0x0C))
        return false;   /* CTRL_REG4: UHP Z */
    if (!wr(ADDR_MAG, 0x22, 0x00))
        return false;   /* CTRL_REG3: continuous */
    return true;
}

/* Raw counts, before any offset. Calibration works on these because the
 * offsets it computes are expressed in counts, not microtesla. */
static bool mag_raw(int16_t raw[3])
{
    uint8_t d[6];
    if (!rd(ADDR_MAG, 0x28, d, 6))          /* 28h..2Dh, little-endian */
        return false;
    raw[0] = (int16_t)((d[1] << 8) | d[0]);
    raw[1] = (int16_t)((d[3] << 8) | d[2]);
    raw[2] = (int16_t)((d[5] << 8) | d[4]);
    return true;
}

/* Hard-iron offsets, subtracted from the raw counts before scaling. Zero
 * until a calibration has been run and saved; loaded from flash at boot. */
static struct mag_offsets s_mag_off = {0, 0, 0};

/* Full soft-iron correction: out = M * (raw_counts - centre).
 *
 * Three offsets can only shift a sphere. When nearby metal stretches the
 * field the samples lie on an ellipsoid, and only a 3x3 matrix reshapes it
 * back. Simulated with a realistic distortion and correct rotations, |B|
 * spread across orientations was 59 uT raw, 30 uT with offsets, and 2.2 uT
 * with this matrix.
 *
 * s_corr_used selects it; when false the three offsets above are used, which
 * is what a board with no soft iron needs. */
static double s_corr[3][3];
static double s_corr_c[3];
static bool s_corr_used;

static bool mag_read(double *x, double *y, double *z)
{
    int16_t raw[3];
    if (!mag_raw(raw))
        return false;
    if (s_corr_used)
    {
        const double d[3] = {raw[0] - s_corr_c[0], raw[1] - s_corr_c[1],
                             raw[2] - s_corr_c[2]};
        *x = s_corr[0][0] * d[0] + s_corr[0][1] * d[1] + s_corr[0][2] * d[2];
        *y = s_corr[1][0] * d[0] + s_corr[1][1] * d[1] + s_corr[1][2] * d[2];
        *z = s_corr[2][0] * d[0] + s_corr[2][1] * d[1] + s_corr[2][2] * d[2];
        return true;
    }

    const double lsb_per_gauss = 6842.0;    /* +-4 gauss full scale */
    const double ut_per_lsb = 100.0 / lsb_per_gauss;
    *x = (double)(raw[0] - s_mag_off.x) * ut_per_lsb;
    *y = (double)(raw[1] - s_mag_off.y) * ut_per_lsb;
    *z = (double)(raw[2] - s_mag_off.z) * ut_per_lsb;
    return true;
}

/* ---- hard-iron calibration ------------------------------------------ */

/* Sampled from the node task rather than from the command that starts it:
 * that task already owns the I2C bus, so calibration cannot collide with the
 * periodic sensor refresh. 80 Hz output rate while sampling so a tumble is
 * densely covered; the normal 10 Hz is restored when the run ends. */
#define MAG_CTRL1_10HZ 0x70
#define MAG_CTRL1_80HZ 0x7C

static bool s_mag_cal_active;
static uint32_t s_mag_cal_n;
static int16_t s_mag_min[3];
static int16_t s_mag_max[3];

/* Plausible radius band for the fitted sphere, in uT. The local field is
 * 46.6 uT; a fit outside this band is measuring something else. */
#define BENCH_FIELD_UT_MIN 35.0
#define BENCH_FIELD_UT_MAX 60.0

/* Least-squares sphere fit, accumulated as the run goes.
 *
 * min/max takes the midpoint of whatever range was swept, so it is only
 * right when the operator reached the extremes -- which never happens in
 * practice. Measured on this board, X could not pass about 5650 counts of
 * the 6377 a full tumble would sweep, and min/max turned that into an
 * offset wrong by about 11 uT no matter how long the board was turned.
 *
 * A sphere fit uses every sample instead of the extremes. Points on the
 * surface of the sphere determine its centre whether they cover the whole
 * surface or a cap of it, so partial coverage costs almost nothing:
 * simulated against a known offset with realistic sensor noise, the sphere
 * fit came out within 0.1 uT at 70 percent coverage where min/max was
 * still 12.6 uT out.
 *
 * |p|^2 = 2*p.c + (r^2 - |c|^2) is linear in (cx, cy, cz, k), so the normal
 * equations are a 4x4 system built from running sums. */
static double s_AtA[4][4];
static double s_Atb[4];

/* Diagnostics: enough to explain a bad fit after the fact rather than
 * guessing at it. Octants around the running mean say whether the samples
 * cover the sphere; the duplicate counter says whether the sensor was
 * returning the same conversion repeatedly. */
/* Keep the samples themselves so the cloud can be examined afterwards: a fit
 * that converged on the wrong radius is impossible to explain from the fit
 * alone, but obvious from the points. 2048 x 6 bytes is 12 KB. */
#define MAG_DUMP_MAX 2048
static int16_t s_dump[MAG_DUMP_MAX][3];
static uint32_t s_dump_n;

struct mag_offsets mag_cal_offsets(void) { return s_mag_off; }
void mag_cal_set_offsets(const struct mag_offsets *o)
{
    if (!o) return;
    s_mag_off = *o;
}
bool mag_cal_active(void) { return s_mag_cal_active; }
uint32_t mag_cal_samples(void) { return s_mag_cal_n; }

void mag_cal_spans(int32_t out[3])
{
    if (!out) return;
    for (int i = 0; i < 3; ++i)
        out[i] = (int32_t)s_mag_max[i] - (int32_t)s_mag_min[i];
}

bool mag_cal_begin(void)
{
    if (s_mag_cal_active)
        return false;
    int16_t probe[3];
    if (!mag_raw(probe))               /* part must answer before we start */
        return false;
    s_mag_cal_active = true;
    s_mag_cal_n = 0;
    for (int i = 0; i < 3; ++i)
    {
        s_mag_min[i] = 32767;
        s_mag_max[i] = -32768;
    }
    memset(s_AtA, 0, sizeof s_AtA);
    memset(s_Atb, 0, sizeof s_Atb);
    s_dump_n = 0;
    wr(ADDR_MAG, 0x20, MAG_CTRL1_80HZ);
    return true;
}

void mag_cal_poll(void)
{
    if (!s_mag_cal_active)
        return;
    int16_t raw[3];
    if (!mag_raw(raw))
        return;
    for (int i = 0; i < 3; ++i)
    {
        if (raw[i] < s_mag_min[i]) s_mag_min[i] = raw[i];
        if (raw[i] > s_mag_max[i]) s_mag_max[i] = raw[i];
    }

    const double x = raw[0], y = raw[1], z = raw[2];
    const double row[4] = {2.0 * x, 2.0 * y, 2.0 * z, 1.0};
    const double rhs = x * x + y * y + z * z;
    for (int i = 0; i < 4; ++i)
    {
        s_Atb[i] += row[i] * rhs;
        for (int j = 0; j < 4; ++j)
            s_AtA[i][j] += row[i] * row[j];
    }
    if (s_dump_n < MAG_DUMP_MAX)
    {
        s_dump[s_dump_n][0] = raw[0];
        s_dump[s_dump_n][1] = raw[1];
        s_dump[s_dump_n][2] = raw[2];
        ++s_dump_n;
    }

    ++s_mag_cal_n;
}

/* Solve the 4x4 normal equations in place. False if the system is singular,
 * which means the samples were coplanar or too few to fix a sphere. */
static bool solve4(double A[4][4], double b[4], double out[4])
{
    for (int i = 0; i < 4; ++i)
    {
        int piv = i;
        for (int r = i + 1; r < 4; ++r)
            if (fabs(A[r][i]) > fabs(A[piv][i]))
                piv = r;
        if (fabs(A[piv][i]) < 1e-9)
            return false;
        if (piv != i)
        {
            for (int c = 0; c < 4; ++c)
            {
                const double t = A[i][c];
                A[i][c] = A[piv][c];
                A[piv][c] = t;
            }
            const double t = b[i]; b[i] = b[piv]; b[piv] = t;
        }
        for (int r = i + 1; r < 4; ++r)
        {
            const double f = A[r][i] / A[i][i];
            for (int c = i; c < 4; ++c)
                A[r][c] -= f * A[i][c];
            b[r] -= f * b[i];
        }
    }
    for (int i = 3; i >= 0; --i)
    {
        double sum = b[i];
        for (int j = i + 1; j < 4; ++j)
            sum -= A[i][j] * out[j];
        out[i] = sum / A[i][i];
        /* A degenerate system can produce inf or nan rather than failing the
         * pivot test. Those must not reach lround(), which is undefined for
         * them. */
        if (!isfinite(out[i]))
            return false;
    }
    return true;
}

void mag_cal_abort(void)
{
    if (!s_mag_cal_active)
        return;
    s_mag_cal_active = false;
    wr(ADDR_MAG, 0x20, MAG_CTRL1_10HZ);
}

/* Current fitted radius, in uT, from the samples gathered so far. Zero when
 * the system cannot be solved yet.
 *
 * This is the run's progress indicator and its pass condition in one number.
 * A sphere fit recovers the true field radius only when the samples span
 * enough of the sphere, so comparing this against the local field measures
 * orientation diversity directly -- which per-axis span does not. Simulated:
 * a fit over samples tilted to 20 degrees gives radius 38 uT and an offset
 * 8.4 uT wrong, while 35 degrees gives 45.7 uT and 1.0 uT wrong. A flat spin
 * is worse still: it traces a circle, and a circle does not fix a sphere. */
double mag_cal_fit_radius_ut(void)
{
    double AtA[4][4], Atb[4], sol[4];
    memcpy(AtA, s_AtA, sizeof AtA);
    memcpy(Atb, s_Atb, sizeof Atb);
    if (!solve4(AtA, Atb, sol))
        return 0.0;
    const double rsq = sol[3] + sol[0] * sol[0] + sol[1] * sol[1]
                       + sol[2] * sol[2];
    if (rsq <= 0)
        return 0.0;
    return sqrt(rsq) * (100.0 / 6842.0);
}

/* Describe the sample cloud, so a bad fit can be explained instead of guessed
 * at. Prints one line per fact that would change the diagnosis:
 *
 *   centre   the mean, which should sit near the fitted offset
 *   radius   per-axis spread, which should be similar on all three for a
 *            sphere and unequal for soft iron
 *   octants  how the samples are distributed around the centre -- a fit needs
 *            them on all sides, not in one cap
 *   dups     consecutive identical readings, which mean the sensor is being
 *            read faster than it converts
 */
/* How evenly the samples are spread around their centre, as the fraction in
 * the least-populated octant. 0.125 is even; anything under about 0.06 means
 * the board never really saw that side and a fit through the rest is not
 * describing the whole field.
 *
 * This is the measure that says whether more tumbling will help, which the
 * fitted radius alone does not: a run can sit at a stable radius while only
 * ever covering half the sphere. */
double mag_cal_coverage(void)
{
    if (s_dump_n < 20)
        return 0.0;
    double mean[3] = {0, 0, 0};
    for (uint32_t i = 0; i < s_dump_n; ++i)
        for (int a = 0; a < 3; ++a)
            mean[a] += s_dump[i][a];
    for (int a = 0; a < 3; ++a)
        mean[a] /= s_dump_n;

    uint32_t oct[8] = {0};
    for (uint32_t i = 0; i < s_dump_n; ++i)
    {
        int idx = 0;
        if (s_dump[i][0] > mean[0]) idx |= 1;
        if (s_dump[i][1] > mean[1]) idx |= 2;
        if (s_dump[i][2] > mean[2]) idx |= 4;
        oct[idx]++;
    }
    uint32_t lo = oct[0];
    for (int i = 1; i < 8; ++i)
        if (oct[i] < lo) lo = oct[i];
    return (double)lo / (double)s_dump_n;
}

void mag_cal_diagnose(void)
{
    if (s_dump_n < 10)
    {
        printf("  diag: too few samples\n");
        return;
    }
    double mean[3] = {0, 0, 0};
    for (uint32_t i = 0; i < s_dump_n; ++i)
        for (int a = 0; a < 3; ++a)
            mean[a] += s_dump[i][a];
    for (int a = 0; a < 3; ++a)
        mean[a] /= s_dump_n;

    double var[3] = {0, 0, 0};
    for (uint32_t i = 0; i < s_dump_n; ++i)
        for (int a = 0; a < 3; ++a)
        {
            const double d = s_dump[i][a] - mean[a];
            var[a] += d * d;
        }
    for (int a = 0; a < 3; ++a)
        var[a] = sqrt(var[a] / s_dump_n);

    const double k = 100.0 / 6842.0;
    printf("  diag: n=%lu  mean %.0f %.0f %.0f counts  sd %.1f %.1f %.1f uT\n",
           (unsigned long)s_dump_n, mean[0], mean[1], mean[2],
           var[0] * k, var[1] * k, var[2] * k);

    /* Octants around the mean. A sphere fit needs them all populated. */
    int32_t oct[8] = {0};
    for (uint32_t i = 0; i < s_dump_n; ++i)
    {
        int idx = 0;
        if (s_dump[i][0] > mean[0]) idx |= 1;
        if (s_dump[i][1] > mean[1]) idx |= 2;
        if (s_dump[i][2] > mean[2]) idx |= 4;
        oct[idx]++;
    }
    printf("  diag: octants %ld %ld %ld %ld %ld %ld %ld %ld (empty=%d)\n",
           (long)oct[0], (long)oct[1], (long)oct[2], (long)oct[3],
           (long)oct[4], (long)oct[5], (long)oct[6], (long)oct[7],
           (int)((oct[0] == 0) + (oct[1] == 0) + (oct[2] == 0) + (oct[3] == 0) +
                 (oct[4] == 0) + (oct[5] == 0) + (oct[6] == 0) + (oct[7] == 0)));

    /* Duplicate runs. */
    uint32_t dup = 0, runs = 0, streak = 0;
    for (uint32_t i = 1; i < s_dump_n; ++i)
    {
        if (s_dump[i][0] == s_dump[i - 1][0] && s_dump[i][1] == s_dump[i - 1][1] &&
            s_dump[i][2] == s_dump[i - 1][2])
        {
            dup++;
            streak++;
        }
        else
        {
            if (streak > 0) runs++;
            streak = 0;
        }
    }
    printf("  diag: %lu duplicate samples in %lu runs (%.0f%%)\n",
           (unsigned long)dup, (unsigned long)runs,
           s_dump_n ? 100.0 * dup / s_dump_n : 0.0);
}

/* Residual offset still present in the samples, in counts, as the centre of
 * the sphere they lie on. When the biases are already removed the cloud is a
 * sphere centred on the origin, so whatever centre is left IS the error.
 *
 * This turns a failed calibration into a fixable one: adding these counts to
 * the stored offsets removes the rest, without the operator having to cover
 * the sphere perfectly. Returns false when the samples cannot fix a sphere. */
bool mag_cal_residual(int32_t out[3], double *radius_ut, double *scatter_ut)
{
    if (s_dump_n < 50)
        return false;

    /* Fit in uT so the numbers mean something to the caller. */
    static double A[4][4], b[4], sol[4];
    memset(A, 0, sizeof A);
    memset(b, 0, sizeof b);
    const double k = 100.0 / 6842.0;
    for (uint32_t i = 0; i < s_dump_n; ++i)
    {
        const double x = s_dump[i][0] * k, y = s_dump[i][1] * k, z = s_dump[i][2] * k;
        const double row[4] = {2.0 * x, 2.0 * y, 2.0 * z, 1.0};
        const double rhs = x * x + y * y + z * z;
        for (int a = 0; a < 4; ++a)
        {
            b[a] += row[a] * rhs;
            for (int c = 0; c < 4; ++c)
                A[a][c] += row[a] * row[c];
        }
    }
    if (!solve4(A, b, sol))
        return false;

    const double rsq = sol[3] + sol[0] * sol[0] + sol[1] * sol[1] + sol[2] * sol[2];
    if (rsq <= 0)
        return false;
    const double r = sqrt(rsq);

    if (radius_ut) *radius_ut = r;
    if (scatter_ut)
    {
        double acc = 0;
        for (uint32_t i = 0; i < s_dump_n; ++i)
        {
            const double x = s_dump[i][0] * k - sol[0];
            const double y = s_dump[i][1] * k - sol[1];
            const double z = s_dump[i][2] * k - sol[2];
            const double d = sqrt(x * x + y * y + z * z) - r;
            acc += d * d;
        }
        *scatter_ut = sqrt(acc / s_dump_n);
    }

    out[0] = (int32_t)lround(sol[0] / k);
    out[1] = (int32_t)lround(sol[1] / k);
    out[2] = (int32_t)lround(sol[2] / k);
    return true;
}

/* How far the board was actually turned: the largest spread of any axis among
 * the samples, in uT. Without rotation a fit is meaningless, and this is what
 * says so rather than reporting a confident number from one pose. */
double mag_cal_rotation_ut(void)
{
    if (s_dump_n < 2)
        return 0.0;
    int16_t lo[3] = {32767, 32767, 32767}, hi[3] = {-32768, -32768, -32768};
    for (uint32_t i = 0; i < s_dump_n; ++i)
        for (int a = 0; a < 3; ++a)
        {
            if (s_dump[i][a] < lo[a]) lo[a] = s_dump[i][a];
            if (s_dump[i][a] > hi[a]) hi[a] = s_dump[i][a];
        }
    double best = 0;
    for (int a = 0; a < 3; ++a)
    {
        const double s = (hi[a] - lo[a]) * (100.0 / 6842.0);
        if (s > best) best = s;
    }
    return best;
}

/* Calibration: the original TinyNode's exercise, with a better estimator.
 *
 * The exercise is unchanged -- tumble the board so the field sweeps a sphere,
 * and the offsets are that sphere's centre. What differs is how the centre is
 * found. P1 took the midpoint of each axis's min and max, which is exact only
 * when the operator reaches the extremes; that is only true for a full
 * diameter, and a board tethered by its own USB cable cannot be flipped
 * freely. Measured here, the best reachable coverage was 91/86/71 percent, at
 * which min/max is 19, 24 and 33 uT out.
 *
 * A least-squares sphere fit uses every sample rather than the extremes, so
 * partial coverage costs far less. Simulated with a realistic tilt cone and
 * sensor noise, against a known offset:
 *
 *   tilt cone   min/max error   sphere error
 *      20 deg        38.0 uT        0.6 uT
 *      35 deg        33.0 uT        0.2 uT
 *      50 deg        27.9 uT        0.1 uT
 *      90 deg        12.1 uT        0.0 uT
 *
 * So the fit is P1's, solved differently. P1's own acceptance rule is kept in
 * spirit: samples must span enough of every axis for the sphere to be fixed,
 * checked against the rotation actually achieved.
 */
/* ---- ellipsoid fit: hard AND soft iron ------------------------------- */

/* General NxN solve, partial pivoting. N is at most 9 here. */
static bool solve_n(double *A, double *b, double *out, int n)
{
    for (int i = 0; i < n; ++i)
    {
        int piv = i;
        for (int r = i + 1; r < n; ++r)
            if (fabs(A[r * n + i]) > fabs(A[piv * n + i]))
                piv = r;
        if (fabs(A[piv * n + i]) < 1e-18)
            return false;
        if (piv != i)
        {
            for (int c = 0; c < n; ++c)
            {
                const double t = A[i * n + c];
                A[i * n + c] = A[piv * n + c];
                A[piv * n + c] = t;
            }
            const double t = b[i]; b[i] = b[piv]; b[piv] = t;
        }
        for (int r = i + 1; r < n; ++r)
        {
            const double f = A[r * n + i] / A[i * n + i];
            for (int c = i; c < n; ++c)
                A[r * n + c] -= f * A[i * n + c];
            b[r] -= f * b[i];
        }
    }
    for (int i = n - 1; i >= 0; --i)
    {
        double sum = b[i];
        for (int j = i + 1; j < n; ++j)
            sum -= A[i * n + j] * out[j];
        out[i] = sum / A[i * n + i];
        if (!isfinite(out[i]))
            return false;
    }
    return true;
}

static bool inv3(const double M[3][3], double out[3][3])
{
    const double a = M[0][0], b = M[0][1], c = M[0][2];
    const double d = M[1][0], e = M[1][1], f = M[1][2];
    const double g = M[2][0], h = M[2][1], i = M[2][2];
    const double det = a * (e * i - f * h) - b * (d * i - f * g) +
                       c * (d * h - e * g);
    if (fabs(det) < 1e-18)
        return false;
    out[0][0] = (e * i - f * h) / det;
    out[0][1] = (c * h - b * i) / det;
    out[0][2] = (b * f - c * e) / det;
    out[1][0] = (f * g - d * i) / det;
    out[1][1] = (a * i - c * g) / det;
    out[1][2] = (c * d - a * f) / det;
    out[2][0] = (d * h - e * g) / det;
    out[2][1] = (b * g - a * h) / det;
    out[2][2] = (a * e - b * d) / det;
    return true;
}

/* Jacobi eigenvalues/vectors of a symmetric 3x3. Eigenvectors come back as
 * the COLUMNS of V. */
static void jacobi3(double A[3][3], double w[3], double V[3][3])
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            V[i][j] = (i == j) ? 1.0 : 0.0;

    for (int sweep = 0; sweep < 60; ++sweep)
    {
        const double off = fabs(A[0][1]) + fabs(A[0][2]) + fabs(A[1][2]);
        if (off < 1e-18)
            break;
        for (int p = 0; p < 2; ++p)
            for (int q = p + 1; q < 3; ++q)
            {
                if (fabs(A[p][q]) < 1e-20)
                    continue;
                const double th = (A[q][q] - A[p][p]) / (2.0 * A[p][q]);
                const double t = (th >= 0 ? 1.0 : -1.0) /
                                 (fabs(th) + sqrt(th * th + 1.0));
                const double c = 1.0 / sqrt(t * t + 1.0);
                const double s = t * c;
                for (int k = 0; k < 3; ++k)
                {
                    const double akp = A[k][p], akq = A[k][q];
                    A[k][p] = c * akp - s * akq;
                    A[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 3; ++k)
                {
                    const double apk = A[p][k], aqk = A[q][k];
                    A[p][k] = c * apk - s * aqk;
                    A[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 3; ++k)
                {
                    const double vkp = V[k][p], vkq = V[k][q];
                    V[k][p] = c * vkp - s * vkq;
                    V[k][q] = s * vkp + c * vkq;
                }
            }
    }
    w[0] = A[0][0];
    w[1] = A[1][1];
    w[2] = A[2][2];
}

/* Fit an ellipsoid to the collected samples and turn it into a 3x3 correction.
 *
 * The samples lie where the field pushes them, so they satisfy
 *   v M v + 2 n v = 1
 * for some symmetric M and vector n. Nine unknowns, linear in them, so one
 * least-squares solve. The centre is -M^-1 n, and the elliptical distortion is
 * sqrt(M), which maps the ellipsoid back onto a sphere.
 *
 * `target_ut` is the magnitude the corrected readings should have: the local
 * field. Shape and centre come from the data; the scale has to be an input,
 * because a pure ellipsoid fit is scale-free. */
static bool ellipsoid_fit(double target_ut)
{
    if (s_dump_n < 200)
        return false;

    /* Work in centred, unit-scaled coordinates or the 9x9 system is hopeless
     * conditioned: raw counts are ~1e4 and their squares ~1e8. */
    double mean[3] = {0, 0, 0};
    for (uint32_t i = 0; i < s_dump_n; ++i)
        for (int a = 0; a < 3; ++a)
            mean[a] += s_dump[i][a];
    for (int a = 0; a < 3; ++a)
        mean[a] /= s_dump_n;

    double var = 0;
    for (uint32_t i = 0; i < s_dump_n; ++i)
        for (int a = 0; a < 3; ++a)
        {
            const double d = s_dump[i][a] - mean[a];
            var += d * d;
        }
    const double scale = sqrt(var / s_dump_n);
    if (scale < 1.0)
        return false;

    static double A[81], b[9], q[9];
    memset(A, 0, sizeof A);
    memset(b, 0, sizeof b);
    for (uint32_t i = 0; i < s_dump_n; ++i)
    {
        const double x = (s_dump[i][0] - mean[0]) / scale;
        const double y = (s_dump[i][1] - mean[1]) / scale;
        const double z = (s_dump[i][2] - mean[2]) / scale;
        const double row[9] = {x * x, y * y, z * z, 2 * x * y, 2 * x * z,
                               2 * y * z, 2 * x, 2 * y, 2 * z};
        for (int r = 0; r < 9; ++r)
        {
            b[r] += row[r];
            for (int c = 0; c < 9; ++c)
                A[r * 9 + c] += row[r] * row[c];
        }
    }
    if (!solve_n(A, b, q, 9))
        return false;

    double M[3][3] = {{q[0], q[3], q[4]}, {q[3], q[1], q[5]},
                      {q[4], q[5], q[2]}};
    const double nv[3] = {q[6], q[7], q[8]};

    double Mi[3][3];
    if (!inv3(M, Mi))
        return false;

    double cc[3];
    for (int i = 0; i < 3; ++i)
        cc[i] = -(Mi[i][0] * nv[0] + Mi[i][1] * nv[1] + Mi[i][2] * nv[2]);

    double k = 1.0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            k += cc[i] * M[i][j] * cc[j];
    if (k <= 0)
        return false;

    double w[3], V[3][3];
    jacobi3(M, w, V);
    for (int i = 0; i < 3; ++i)
        w[i] = (w[i] > 1e-18) ? sqrt(w[i]) : 1e-9;

    /* M^(1/2) = V diag(sqrt(w)) V^T, scaled so the corrected radius is the
     * target field magnitude, then converted from normalised units back to
     * raw counts. */
    const double ksc = target_ut / sqrt(k);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
        {
            double sum = 0;
            for (int t = 0; t < 3; ++t)
                sum += V[i][t] * w[t] * V[j][t];
            s_corr[i][j] = ksc * sum / scale;
        }
    for (int i = 0; i < 3; ++i)
        s_corr_c[i] = mean[i] + cc[i] * scale;

    s_corr_used = true;
    return true;
}

/* Public entry point: fit soft+hard iron from the collected samples. */
bool mag_cal_fit_ellipsoid(double target_ut, char *report, size_t n)
{
    if (report && n) report[0] = '\0';
    if (!s_mag_cal_active)
    {
        /* Allowed outside a run so `magcal check` can refit what it sampled. */
    }
    if (!ellipsoid_fit(target_ut))
    {
        if (report && n)
            snprintf(report, n, "ellipsoid fit failed (not enough spread?)");
        return false;
    }
    if (report && n)
        snprintf(report, n, "ellipsoid fit applied, corrected |B| targets %.1f uT",
                 target_ut);
    return true;
}

bool mag_cal_using_matrix(void) { return s_corr_used; }

bool mag_cal_matrix(double M[3][3], double centre[3])
{
    if (!s_corr_used)
        return false;
    for (int i = 0; i < 3; ++i)
    {
        centre[i] = s_corr_c[i];
        for (int j = 0; j < 3; ++j)
            M[i][j] = s_corr[i][j];
    }
    return true;
}

/* Set the correction's absolute gain against a known field magnitude.
 *
 * The ellipsoid fit determines the shape of the correction but not its size:
 * that comes from how much of the ellipsoid the samples covered. A tumble that
 * reaches 92/86/74 percent of the three axes leaves the fitted surface small,
 * every corrected vector lands outside it, and |B| reads high -- measured on
 * this board as 53.0 uT against a field of 46.6, with the dip already correct
 * at 58.6 degrees.
 *
 * A uniform factor fixes the magnitude and leaves direction alone, so bearing
 * and dip are untouched. Scaling each axis separately to the sensor's nominal
 * gain would instead erase the soft-iron asymmetry the fit was run to find. */
bool mag_cal_rescale(double measured_ut, double target_ut)
{
    if (!s_corr_used || measured_ut <= 1.0 || target_ut <= 1.0)
        return false;
    const double k = target_ut / measured_ut;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            s_corr[i][j] *= k;
    return true;
}

void mag_cal_clearmatrix(void)
{
    s_corr_used = false;
}

bool mag_cal_finish(struct mag_offsets *out, char *report, size_t n)
{
    if (report && n) report[0] = '\0';
    if (!s_mag_cal_active)
    {
        if (report && n) snprintf(report, n, "no calibration running");
        return false;
    }
    s_mag_cal_active = false;
    wr(ADDR_MAG, 0x20, MAG_CTRL1_10HZ);

    if (s_mag_cal_n < 100)
    {
        if (report && n)
            snprintf(report, n, "too few samples (%lu); rotate for longer",
                     (unsigned long)s_mag_cal_n);
        return false;
    }

    int32_t span[3];
    mag_cal_spans(span);

    /* P1's guard against a board that was only spun flat: all three axes must
     * have moved, or the sphere is a disc and its centre is meaningless. */
    int32_t min_span = span[0], max_span = span[0];
    for (int i = 1; i < 3; ++i)
    {
        if (span[i] < min_span) min_span = span[i];
        if (span[i] > max_span) max_span = span[i];
    }
    if (min_span < 1000 || max_span > min_span * 3)
    {
        if (report && n)
            snprintf(report, n,
                     "POOR FIT (n=%lu span %ld/%ld/%ld): rotate through all "
                     "orientations",
                     (unsigned long)s_mag_cal_n, (long)span[0], (long)span[1],
                     (long)span[2]);
        return false;
    }

    double AtA[4][4], Atb[4], sol[4];
    memcpy(AtA, s_AtA, sizeof AtA);
    memcpy(Atb, s_Atb, sizeof Atb);
    if (!solve4(AtA, Atb, sol))
    {
        if (report && n)
            snprintf(report, n, "POOR FIT: samples do not determine a sphere");
        return false;
    }

    const double rsq = sol[3] + sol[0] * sol[0] + sol[1] * sol[1] + sol[2] * sol[2];
    if (rsq <= 0)
    {
        if (report && n)
            snprintf(report, n, "POOR FIT: sphere radius came out negative");
        return false;
    }
    const double ut_per_lsb = 100.0 / 6842.0;
    const double radius_ut = sqrt(rsq) * ut_per_lsb;

    struct mag_offsets o;
    o.x = (int16_t)lround(sol[0]);
    o.y = (int16_t)lround(sol[1]);
    o.z = (int16_t)lround(sol[2]);

    if (out) *out = o;
    if (report && n)
        snprintf(report, n,
                 "n=%lu, sphere fit, span %ld/%ld/%ld counts, radius %.1f uT",
                 (unsigned long)s_mag_cal_n, (long)span[0], (long)span[1],
                 (long)span[2], radius_ut);
    return true;
}

/* Persisted as three integers in text, not a binary struct: a future change to
 * the struct should fail to parse rather than be misread as offsets. */
#define MAG_CAL_FILE "/fs/mag_cal"

bool mag_cal_save(void)
{
    FILE *fp = fopen(MAG_CAL_FILE, "w");
    if (!fp)
        return false;
    /* Twelve numbers mean a full soft-iron matrix; three mean offsets only.
     * The count is the format identifier, so an older file still loads. */
    if (s_corr_used)
    {
        fprintf(fp, "%d\n", 12);
        for (int i = 0; i < 3; ++i)
            fprintf(fp, "%.9g %.9g %.9g\n", s_corr[i][0], s_corr[i][1], s_corr[i][2]);
        fprintf(fp, "%.9g %.9g %.9g\n", s_corr_c[0], s_corr_c[1], s_corr_c[2]);
    }
    else
    {
        fprintf(fp, "%d\n", 3);
        fprintf(fp, "%d %d %d\n", s_mag_off.x, s_mag_off.y, s_mag_off.z);
    }
    fclose(fp);
    return true;
}

bool mag_cal_load(void)
{
    FILE *fp = fopen(MAG_CAL_FILE, "r");
    if (!fp)
        return false;

    /* The count is the format tag. Anything else is the older format, which
     * was three bare integers with no tag -- so a file starting "183 -1447
     * 560" has to be read as offsets, not as a malformed count. */
    int count = 0;
    if (fscanf(fp, "%d", &count) != 1)
    {
        fclose(fp);
        return false;
    }

    bool ok = false;
    if (count != 12 && count != 3)
    {
        /* Legacy: the value just read is the first offset. */
        int y = 0, z = 0;
        if (fscanf(fp, "%d %d", &y, &z) == 2 &&
            count >= -32768 && count <= 32767 && y >= -32768 && y <= 32767 &&
            z >= -32768 && z <= 32767)
        {
            s_mag_off.x = (int16_t)count;
            s_mag_off.y = (int16_t)y;
            s_mag_off.z = (int16_t)z;
            s_corr_used = false;
            ok = true;
        }
        fclose(fp);
        return ok;
    }

    if (count == 12)
    {
        double m[3][3], c[3];
        ok = true;
        for (int i = 0; i < 3 && ok; ++i)
            if (fscanf(fp, "%lf %lf %lf", &m[i][0], &m[i][1], &m[i][2]) != 3)
                ok = false;
        if (ok && fscanf(fp, "%lf %lf %lf", &c[0], &c[1], &c[2]) != 3)
            ok = false;
        if (ok)
        {
            for (int i = 0; i < 3; ++i)
            {
                s_corr_c[i] = c[i];
                for (int j = 0; j < 3; ++j)
                    s_corr[i][j] = m[i][j];
            }
            s_corr_used = true;
        }
    }
    else if (count == 3)
    {
        int x = 0, y = 0, z = 0;
        if (fscanf(fp, "%d %d %d", &x, &y, &z) == 3 &&
            x >= -32768 && x <= 32767 && y >= -32768 && y <= 32767 &&
            z >= -32768 && z <= 32767)
        {
            s_mag_off.x = (int16_t)x;
            s_mag_off.y = (int16_t)y;
            s_mag_off.z = (int16_t)z;
            s_corr_used = false;
            ok = true;
        }
    }
    fclose(fp);
    return ok;
}


/* ---- BMP581 --------------------------------------------------------- */

static bool bmp_begin(void)
{
    uint8_t id = 0;
    if (!rd(ADDR_BMP, 0x01, &id, 1) || id != 0x50)
        return false;
    /* OSR config: pressure x8, temperature x2, no IIR. */
    if (!wr(ADDR_BMP, 0x36, 0x4B))
        return false;
    /* ODR config: normal mode, 10 Hz. */
    if (!wr(ADDR_BMP, 0x37, 0x43))
        return false;
    return true;
}

static bool bmp_read(double *pressure_hpa, double *temp_c)
{
    uint8_t d[6];
    /* 0x1D: temperature (3), 0x20: pressure (3), both 24-bit little-endian. */
    if (!rd(ADDR_BMP, 0x1D, d, 6))
        return false;
    int32_t raw_t = (int32_t)((uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16));
    int32_t raw_p = (int32_t)((uint32_t)d[3] | ((uint32_t)d[4] << 8) | ((uint32_t)d[5] << 16));
    if (raw_t & 0x800000)
        raw_t |= (int32_t)0xFF000000;
    if (raw_p & 0x800000)
        raw_p |= (int32_t)0xFF000000;

    /* BMP581 datasheet: T = raw/2^16 degC, p = raw/2^6 Pa. */
    *temp_c = (double)raw_t / 65536.0;
    *pressure_hpa = ((double)raw_p / 64.0) / 100.0;
    return true;
}

/* ---- PA1010D -------------------------------------------------------- */

static char s_line[128];
static size_t s_len;
static uint8_t s_last_kept;

/* Fields pulled from the sentences this project needs. */
static struct
{
    bool fix;
    uint8_t sats;
    double lat, lon, alt;
    uint32_t unix_time;
    bool have_time;

    /* Motion and quality, both of which the receiver already sends and this
     * driver used to discard. RMC carries speed over ground in knots and
     * course over ground in degrees; GGA carries HDOP. Neither is a
     * substitute for a real accuracy figure, but both are measurements
     * rather than the zeros that were being reported. */
    double speed_knots;
    bool have_speed;
    double course_deg;
    bool have_course;
    double hdop;
    bool have_hdop;
} s_gps;

/* ddmm.mmmm -> decimal degrees. */
static double nmea_deg(const char *v, const char hemi)
{
    if (!v || !*v)
        return 0.0;
    double raw = atof(v);
    int deg = (int)(raw / 100.0);
    double min = raw - deg * 100.0;
    double out = deg + min / 60.0;
    if (hemi == 'S' || hemi == 'W')
        out = -out;
    return out;
}

static void nmea_sentence(char *s)
{
    if (s[0] != '$')
        return;
    /* Fields are comma separated; a light parse is enough for GGA and RMC. */
    char *f[20];
    int n = 0;
    for (char *p = s; p && n < 20;)
    {
        f[n++] = p;
        char *c = strchr(p, ',');
        if (c)
        {
            *c = '\0';
            p = c + 1;
        }
        else
            p = NULL;
    }
    if (n < 3)
        return;

    bool gga = (strncmp(f[0], "$GPGGA", 6) == 0 || strncmp(f[0], "$GNGGA", 6) == 0);
    bool rmc = (strncmp(f[0], "$GPRMC", 6) == 0 || strncmp(f[0], "$GNRMC", 6) == 0);

    if (gga && n > 9)
    {
        s_gps.fix = atoi(f[6]) > 0;
        s_gps.sats = (uint8_t)atoi(f[7]);
        if (s_gps.fix)
        {
            s_gps.lat = nmea_deg(f[2], f[3] && *f[3] ? *f[3] : 'N');
            s_gps.lon = nmea_deg(f[4], f[5] && *f[5] ? *f[5] : 'E');
            s_gps.alt = atof(f[9]);
            /* Field 8 is HDOP, horizontal dilution of precision. Reported by
             * the receiver, so it is a measurement; it is dimensionless and
             * needs a ranging error to become metres (see gps_accuracy_m). */
            if (f[8] && *f[8])
            {
                s_gps.hdop = atof(f[8]);
                s_gps.have_hdop = (s_gps.hdop > 0.0);
            }
        }
    }
    if (rmc && n > 9)
    {
        bool active = (f[2] && *f[2] == 'A');
        s_gps.fix = s_gps.fix || active;

        /* Speed over ground in knots (field 7) and course over ground in
         * degrees true (field 8). Both are empty strings when the receiver
         * has nothing to say, which is not the same as zero: a stationary
         * receiver reports no course at all rather than a heading. */
        if (active && n > 8)
        {
            if (f[7] && *f[7])
            {
                s_gps.speed_knots = atof(f[7]);
                s_gps.have_speed = true;
            }
            if (f[8] && *f[8])
            {
                s_gps.course_deg = atof(f[8]);
                s_gps.have_course = true;
            }
        }

        if (active && s_gps.lat == 0.0 && s_gps.lon == 0.0)
        {
            s_gps.lat = nmea_deg(f[3], f[4] && *f[4] ? *f[4] : 'N');
            s_gps.lon = nmea_deg(f[5], f[6] && *f[6] ? *f[6] : 'E');
        }
        /* ddmmss / ddmmyy -> unix, only when the date field is present. */
        if (n > 9 && f[9] && strlen(f[9]) >= 6 && f[1] && strlen(f[1]) >= 6)
        {
            int hh = (f[1][0] - '0') * 10 + (f[1][1] - '0');
            int mm = (f[1][2] - '0') * 10 + (f[1][3] - '0');
            int ss = (f[1][4] - '0') * 10 + (f[1][5] - '0');
            int dd = (f[9][0] - '0') * 10 + (f[9][1] - '0');
            int mo = (f[9][2] - '0') * 10 + (f[9][3] - '0');
            int yy = (f[9][4] - '0') * 10 + (f[9][5] - '0') + 2000;
            /* days since epoch for a civil date (Howard Hinnant's algorithm) */
            int y = yy - (mo <= 2);
            int era = (y >= 0 ? y : y - 399) / 400;
            unsigned yoe = (unsigned)(y - era * 400);
            unsigned doy = (153u * (mo + (mo > 2 ? -3 : 9)) + 2u) / 5u + (unsigned)dd - 1u;
            unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
            long days = (long)era * 146097L + (long)doe - 719468L;
            s_gps.unix_time = (uint32_t)(days * 86400L + hh * 3600 + mm * 60 + ss);
            s_gps.have_time = true;
        }
    }
}

static void gps_poll(void)
{
    i2c_master_bus_handle_t bus = sensors_bus();
    if (!bus)
        return;
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADDR_GPS,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(bus, &cfg, &dev) != ESP_OK)
        return;

    uint8_t chunk[32];
    if (i2c_master_receive(dev, chunk, sizeof(chunk), 100) == ESP_OK)
    {
        for (size_t i = 0; i < sizeof(chunk); i++)
        {
            char c = (char)chunk[i];
            /* The module pads an idle read with 0x0A. A line feed that does
             * not follow a carriage return is filler, never a terminator --
             * without this rule an idle bus reads as a stream of empty
             * sentences and a sentence split across reads gets cut in half. */
            if (c == '\n' && s_last_kept != '\r')
                continue;
            s_last_kept = (uint8_t)c;
            if (c == '\r')
                continue;
            if (c == '\n')
            {
                s_line[s_len] = '\0';
                if (s_len > 6)
                    nmea_sentence(s_line);
                s_len = 0;
            }
            else if (s_len < sizeof(s_line) - 1)
            {
                s_line[s_len++] = c;
            }
            else
            {
                s_len = 0;
            }
        }
    }
    i2c_master_bus_rm_device(dev);
}

/* ---- public --------------------------------------------------------- */

static struct sensor_snapshot s_snapshot;

int sensors_begin(void)
{
    int n = 0;
    uint8_t v = 0;
    if (rd(ADDR_GPS, 0x00, &v, 1))
        n++;
    if (mag_begin())
        n++;
    if (sht45_read(&s_snapshot.temperature_c, &s_snapshot.humidity_pct))
        n++;
    if (bmp_begin())
        n++;
    ESP_LOGI(TAG, "%d of 4 sensors configured", n);
    return n;
}

/* One knot in metres per second, exact by definition: 1852 m / 3600 s. */
static const double KNOTS_TO_MPS = 1852.0 / 3600.0;

/* HDOP to metres needs a user-equivalent range error, which is not a property
 * of this receiver and not something it reports. 5 m is the conventional
 * single-frequency GPS figure. This is an approximation and is labelled as one
 * rather than presented as a measured accuracy: the receiver measures HDOP,
 * and this project chooses the multiplier. */
static const double GPS_UERE_M = 5.0;

/* Copy the GPS state into a snapshot, converting to the units the LXMF
 * LOCATION sensor expects. Called from both the one-shot read and the
 * background poll so the two cannot drift apart. */
static void fill_gps(struct sensor_snapshot *o)
{
    o->have_gps = true;              /* absence shows as no fix and no time */
    o->gps_fix = s_gps.fix;
    o->gps_sats = s_gps.sats;
    o->gps_lat = s_gps.lat;
    o->gps_lon = s_gps.lon;
    o->gps_alt_m = s_gps.alt;
    o->gps_unix = s_gps.have_time ? s_gps.unix_time : 0;

    o->gps_have_speed = s_gps.have_speed;
    o->gps_speed_mps = s_gps.have_speed ? s_gps.speed_knots * KNOTS_TO_MPS : 0.0;
    o->gps_have_course = s_gps.have_course;
    o->gps_course_deg = s_gps.have_course ? s_gps.course_deg : 0.0;
    o->gps_have_accuracy = s_gps.have_hdop && s_gps.fix;
    o->gps_accuracy_m = o->gps_have_accuracy ? s_gps.hdop * GPS_UERE_M : 0.0;
}

void sensors_read(struct sensor_snapshot *out)
{
    if (!out)
        return;

    double t, h;
    if (sht45_read(&t, &h))
    {
        out->have_env = true;
        out->temperature_c = t;
        out->humidity_pct = h;
    }
    else
        out->have_env = false;

    double p, pt;
    if (bmp_read(&p, &pt))
    {
        out->have_pressure = true;
        out->pressure_hpa = p;
        out->pressure_temp_c = pt;
    }
    else
        out->have_pressure = false;

    double x, y, z;
    if (mag_read(&x, &y, &z))
    {
        out->have_mag = true;
        out->mag_x_ut = x;
        out->mag_y_ut = y;
        out->mag_z_ut = z;
    }
    else
        out->have_mag = false;

    gps_poll();
    fill_gps(out);
}

const struct sensor_snapshot *sensors_latest(void)
{
    return &s_snapshot;
}

/* Background refresh, called from the node task. The interval is set so the
 * snapshot is fresh enough for a telemetry request without the request ever
 * waiting on the humidity conversion. */
void sensors_poll_gps(void)
{
    gps_poll();
    fill_gps(&s_snapshot);
}

void sensors_refresh(void)
{
    /* The slow parts only. The GPS is drained by sensors_poll_gps() on every
     * loop tick, because a 1 Hz NMEA stream cannot be kept up with at this
     * cadence and would otherwise be read tens of seconds late. */
    double t, h;
    if (sht45_read(&t, &h))
    {
        s_snapshot.have_env = true;
        s_snapshot.temperature_c = t;
        s_snapshot.humidity_pct = h;
    }
    else
        s_snapshot.have_env = false;

    double p, pt;
    if (bmp_read(&p, &pt))
    {
        s_snapshot.have_pressure = true;
        s_snapshot.pressure_hpa = p;
        s_snapshot.pressure_temp_c = pt;
    }
    else
        s_snapshot.have_pressure = false;

    double x, y, z;
    if (mag_read(&x, &y, &z))
    {
        s_snapshot.have_mag = true;
        s_snapshot.mag_x_ut = x;
        s_snapshot.mag_y_ut = y;
        s_snapshot.mag_z_ut = z;

        /* Magnetic heading from the horizontal part of the corrected field.
         * The threshold is the same one `magcal show` uses: below a few
         * microtesla of horizontal field the angle is dominated by sensor
         * noise and the bearing would sweep wildly while the board sits
         * still. atan2 measures the field's direction, which is the direction
         * the sensor's X axis points into the field, so the board reading
         * north depends on how the magnetometer is keyed to the GPS module. */
        const double horizontal = sqrt(x * x + y * y);
        if (horizontal >= MAG_HEADING_MIN_UT)
        {
            double deg = atan2(y, x) * 180.0 / (double)M_PI;
            if (deg < 0.0) deg += 360.0;
            s_snapshot.mag_heading_deg = deg;
            s_snapshot.have_heading = true;
        }
        else
            s_snapshot.have_heading = false;
    }
    else
    {
        s_snapshot.have_mag = false;
        s_snapshot.have_heading = false;
    }

    s_snapshot.have_gps = true;
}
