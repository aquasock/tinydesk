/*
 * rtc.c - the PCF85063A at I2C 0x51.
 *
 * See rtc.h for why this exists. Everything here is from mainline's
 * drivers/rtc/rtc-pcf85063.c; the register map is transcribed rather than
 * inferred, and the BCD conversion matches its masks exactly.
 *
 * THE OSCILLATOR-STOP FLAG IS THE POINT. Bit 7 of the seconds register is set
 * when the chip has lost its backup power, and a time read with that bit set is
 * whatever the registers happened to hold -- usually 2000-01-01 or 1970. A
 * watch that boots to a confidently wrong date is worse than one that says it
 * does not know, so a stopped part is treated as no clock at all, exactly as
 * mainline does (it returns -EINVAL and the kernel falls back to another
 * source).
 *
 * DAYLIGHT SAVING IS NOT HERE. The chip stores UTC and nothing else; the
 * desktop applies the time zone. This board's chip was set to UTC+8 by the
 * factory on the Linux image, and that was corrected there (RTC-001), so it
 * holds true UTC -- but nothing here depends on that being true, because the
 * only thing that reads it is the same clock the desktop displays.
 */
#include "rtc.h"

#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "power.h"

#define TAG "twatch-rtc"

#define RTC_ADDR        0x51
#define RTC_HZ          400000

/* Mainline's map: seconds first, seven bytes, then the alarm and control. */
#define REG_CTRL1       0x00
#define CTRL1_STOP      0x20            /* bit 5: stops the counter */
#define REG_CTRL2       0x01
#define REG_SECONDS     0x04
#define SECONDS_OS      0x80            /* bit 7: oscillator stopped */

#define RTC_BYTES       7

/* Below this the stored time is not plausible for this board: the port did not
 * exist, and a part that answers with 1970 has almost certainly been power
 * cycled without a battery. 2020-01-01 UTC. */
#define MIN_PLAUSIBLE   1577836800LL

static i2c_master_dev_handle_t s_dev;
static bool s_writeback;

static uint8_t bcd2bin(uint8_t v) { return (uint8_t)((v >> 4) * 10 + (v & 0x0F)); }
static uint8_t bin2bcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

static bool present(void)
{
    if (s_dev)
        return true;

    i2c_master_bus_handle_t bus = board_i2c();

    if (!bus)
        return false;

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = RTC_ADDR,
        .scl_speed_hz = RTC_HZ,
    };
    if (i2c_master_bus_add_device(bus, &cfg, &s_dev) != ESP_OK)
    {
        ESP_LOGW(TAG, "no device at 0x%02X", RTC_ADDR);
        s_dev = NULL;
        return false;
    }
    return true;
}

static bool read_regs(uint8_t reg, uint8_t *buf, size_t n)
{
    if (!present())
        return false;
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, n, 100) == ESP_OK;
}

static bool write_regs(uint8_t reg, const uint8_t *buf, size_t n)
{
    uint8_t tmp[RTC_BYTES + 1];

    if (!present() || n > RTC_BYTES)
        return false;
    tmp[0] = reg;
    memcpy(tmp + 1, buf, n);
    return i2c_master_transmit(s_dev, tmp, n + 1, 100) == ESP_OK;
}

/* Seconds in a year, and the day-of-year of a month, for a UTC epoch. */
static int64_t days_from_civil(int64_t y, unsigned m, unsigned d)
{
    y -= m <= 2;
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int64_t)doe - 719468;
}

static void civil_from_days(int64_t z, int64_t *y, unsigned *m, unsigned *d)
{
    z += 719468;
    const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = (unsigned)(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const int64_t yy = (int64_t)yoe + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    *d = doy - (153 * mp + 2) / 5 + 1;
    *m = mp + (mp < 10 ? 3 : -9);
    *y = yy + (*m <= 2);
}

bool rtc_read(int64_t *utc)
{
    uint8_t b[RTC_BYTES];

    if (!read_regs(REG_SECONDS, b, sizeof(b)))
        return false;

    /* Unsourced: mainline refuses a stopped part and so do we. */
    if (b[0] & SECONDS_OS)
    {
        ESP_LOGW(TAG, "oscillator stopped; the stored time is not valid");
        return false;
    }

    int sec = bcd2bin(b[0] & 0x7F);
    int min = bcd2bin(b[1] & 0x7F);
    int hour = bcd2bin(b[2] & 0x3F);        /* already 0-23 */
    int mday = bcd2bin(b[3] & 0x3F);
    int mon = bcd2bin(b[5] & 0x1F);         /* 1-12 */
    int year = bcd2bin(b[6]) + 2000;        /* the chip stores 00-99 */

    if (mon < 1 || mon > 12 || mday < 1 || mday > 31 ||
        hour > 23 || min > 59 || sec > 59)
    {
        ESP_LOGW(TAG, "implausible registers %02x %02x %02x", b[0], b[3], b[5]);
        return false;
    }

    *utc = days_from_civil(year, (unsigned)mon, (unsigned)mday) * 86400 +
           hour * 3600 + min * 60 + sec;
    return true;
}

bool rtc_write(int64_t utc)
{
    int64_t days = utc / 86400;
    int64_t rem = utc % 86400;
    int64_t y;
    unsigned m, d;
    uint8_t b[RTC_BYTES];

    if (rem < 0)                            /* before 1970; not our case */
    {
        days--;
        rem += 86400;
    }
    civil_from_days(days, &y, &m, &d);

    if (y < 2000 || y > 2099)               /* the chip holds two digits */
        return false;

    b[0] = bin2bcd((uint8_t)(rem % 60));
    b[1] = bin2bcd((uint8_t)(rem / 60 % 60));
    b[2] = bin2bcd((uint8_t)(rem / 3600));
    b[3] = bin2bcd((uint8_t)d);
    b[4] = (uint8_t)((days + 4) % 7);       /* 1970-01-01 was a Thursday */
    b[5] = bin2bcd((uint8_t)m);
    b[6] = bin2bcd((uint8_t)(y - 2000));

    if (!write_regs(REG_SECONDS, b, sizeof(b)))
        return false;

    /* Writing clears the oscillator-stop flag as a side effect of the write to
     * the seconds register, so nothing else is needed to mark the part valid. */
    ESP_LOGI(TAG, "wrote %04d-%02u-%02u %02u:%02u:%02u UTC",
             (int)y, m, d, (unsigned)(rem / 3600),
             (unsigned)(rem / 60 % 60), (unsigned)(rem % 60));
    return true;
}

bool rtc_seed_system_clock(void)
{
    int64_t utc;

    if (!rtc_read(&utc))
        return false;

    if (utc < MIN_PLAUSIBLE)
    {
        ESP_LOGW(TAG, "stored time is before 2020; not using it");
        return false;
    }

    struct timeval tv = { .tv_sec = (time_t)utc, .tv_usec = 0 };

    if (settimeofday(&tv, NULL) != 0)
        return false;

    s_writeback = true;
    ESP_LOGI(TAG, "system clock seeded from the RTC, %lld", (long long)utc);
    return true;
}

void rtc_writeback_enable(bool on)
{
    s_writeback = on;
}

/*
 * The shared code's hook (ports/esp_idf/app/main.c, weak by default). Called
 * after the system clock has been set by Date & time or by SNTP, so the chip
 * follows the clock rather than the other way round once a better source has
 * spoken.
 */
void board_time_set_post(int64_t utc);

void board_time_set_post(int64_t utc)
{
    if (s_writeback && utc >= MIN_PLAUSIBLE)
        rtc_write(utc);
}

/*
 * Set the system clock from a source this port owns, and let the chip follow.
 *
 * This is deliberately the same two steps the shared app's time_set() takes for
 * SNTP and for Date & time: settimeofday, then board_time_set_post so the
 * PCF85063A is corrected too. Reusing the sequence rather than calling
 * settimeofday alone is what makes a GPS-set time survive a power cycle -- the
 * whole point of having a battery-backed chip is that the better source writes
 * through it.
 *
 * The plausibility floor is checked before anything is changed, so a bad
 * sentence cannot move the clock even briefly.
 */
bool board_time_apply(int64_t utc)
{
    if (utc < MIN_PLAUSIBLE)
        return false;

    struct timeval tv = { .tv_sec = (time_t)utc, .tv_usec = 0 };

    if (settimeofday(&tv, NULL) != 0)
        return false;

    board_time_set_post(utc);
    return true;
}
