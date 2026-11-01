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

static bool mag_read(double *x, double *y, double *z)
{
    uint8_t d[6];
    if (!rd(ADDR_MAG, 0x28, d, 6))          /* 28h..2Dh, little-endian */
        return false;
    int16_t raw[3];
    raw[0] = (int16_t)((d[1] << 8) | d[0]);
    raw[1] = (int16_t)((d[3] << 8) | d[2]);
    raw[2] = (int16_t)((d[5] << 8) | d[4]);
    const double lsb_per_gauss = 6842.0;    /* +-4 gauss full scale */
    const double ut_per_lsb = 100.0 / lsb_per_gauss;
    *x = raw[0] * ut_per_lsb;
    *y = raw[1] * ut_per_lsb;
    *z = raw[2] * ut_per_lsb;
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
        }
    }
    if (rmc && n > 9)
    {
        bool active = (f[2] && *f[2] == 'A');
        s_gps.fix = s_gps.fix || active;
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
    out->have_gps = true;              /* absence shows as no fix and no time */
    out->gps_fix = s_gps.fix;
    out->gps_sats = s_gps.sats;
    out->gps_lat = s_gps.lat;
    out->gps_lon = s_gps.lon;
    out->gps_alt_m = s_gps.alt;
    out->gps_unix = s_gps.have_time ? s_gps.unix_time : 0;
}

const struct sensor_snapshot *sensors_latest(void)
{
    return &s_snapshot;
}

/* Background refresh, called from the node task. The interval is set so the
 * snapshot is fresh enough for a telemetry request without the request ever
 * waiting on the humidity conversion. */
void sensors_refresh(void)
{
    struct sensor_snapshot next = s_snapshot;
    sensors_read(&next);
    s_snapshot = next;
}
