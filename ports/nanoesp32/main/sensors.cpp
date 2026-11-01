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
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, buf, len, 100);
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

static bool mag_read(double *x, double *y, double *z)
{
    int16_t raw[3];
    if (!mag_raw(raw))
        return false;
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
    ++s_mag_cal_n;
}

void mag_cal_abort(void)
{
    if (!s_mag_cal_active)
        return;
    s_mag_cal_active = false;
    wr(ADDR_MAG, 0x20, MAG_CTRL1_10HZ);
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
    int32_t min_span = span[0], max_span = span[0];
    for (int i = 1; i < 3; ++i)
    {
        if (span[i] < min_span) min_span = span[i];
        if (span[i] > max_span) max_span = span[i];
    }

    struct mag_offsets o;
    o.x = (int16_t)(((int32_t)s_mag_min[0] + s_mag_max[0]) / 2);
    o.y = (int16_t)(((int32_t)s_mag_min[1] + s_mag_max[1]) / 2);
    o.z = (int16_t)(((int32_t)s_mag_min[2] + s_mag_max[2]) / 2);

    /* A tumble that covered the sphere gives roughly equal spans. A small span
     * means that axis was never rotated, so its offset would be meaningless. */
    if (min_span < 1000 || max_span > min_span * 3)
    {
        if (report && n)
            snprintf(report, n,
                     "POOR FIT (n=%lu span %ld/%ld/%ld): rotate through all "
                     "orientations, including upside down",
                     (unsigned long)s_mag_cal_n, (long)span[0], (long)span[1],
                     (long)span[2]);
        return false;
    }

    if (out) *out = o;
    const double ut_per_lsb = 100.0 / 6842.0;
    const double radius_ut =
        (double)((span[0] + span[1] + span[2]) / 6) * ut_per_lsb;
    if (report && n)
        snprintf(report, n,
                 "n=%lu, span %ld/%ld/%ld counts, radius ~%.1f uT",
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
    fprintf(fp, "%d %d %d\n", s_mag_off.x, s_mag_off.y, s_mag_off.z);
    fclose(fp);
    return true;
}

bool mag_cal_load(void)
{
    FILE *fp = fopen(MAG_CAL_FILE, "r");
    if (!fp)
        return false;
    int x = 0, y = 0, z = 0;
    int got = fscanf(fp, "%d %d %d", &x, &y, &z);
    fclose(fp);
    if (got != 3)
        return false;
    /* Bounded so a corrupt file cannot shift the reading into nonsense. */
    if (x < -32768 || x > 32767 || y < -32768 || y > 32767 ||
        z < -32768 || z > 32767)
        return false;
    s_mag_off.x = (int16_t)x;
    s_mag_off.y = (int16_t)y;
    s_mag_off.z = (int16_t)z;
    return true;
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
    }
    else
        s_snapshot.have_mag = false;

    s_snapshot.have_gps = true;
}
