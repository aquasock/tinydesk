/*
 * nmea.c - GGA and RMC, parsed from one line at a time.
 *
 * See nmea.h for why this owns no stream and decodes only two sentence types.
 *
 * Three habits in here are deliberate and each was chosen against a failure
 * that a GPS parser is prone to:
 *
 * FIELDS ARE COUNTED, NOT TRUSTED. NMEA changes shape: GGA has 14 fields in the
 * common form and some receivers emit 12, and RMC grew a mode field that older
 * units do not send. Reading field N by scanning for the Nth comma and stopping
 * when the commas run out means a short sentence yields missing data rather
 * than shifted data.
 *
 * LATITUDE IS DEGREES AND MINUTES, AND MINUTES ARE NOT DECIMAL. `4807.038` is
 * 48 degrees and 07.038 minutes, which is 48.1173 degrees -- not 48.07038. The
 * difference is about 2.8 km, and it looks right on a map until it matters.
 *
 * AN EMPTY FIELD IS NOT A ZERO. A receiver with no fix sends `,,,,,` for the
 * position and an empty HDOP, and treating those as 0 puts a valid-looking
 * point in the Gulf of Guinea off the coast of Africa. Every numeric field
 * here has a `have_` flag beside it for that reason.
 */

#include "nmea.h"

#include <stdlib.h>
#include <string.h>

void nmea_init(nmea_state_t *s)
{
    memset(s, 0, sizeof(*s));
}

uint8_t nmea_checksum(const char *between, size_t len)
{
    uint8_t x = 0;
    for (size_t i = 0; i < len; i++)
        x ^= (uint8_t)between[i];
    return x;
}

/* --- field access -------------------------------------------------------- */

/* Split on commas in place. Returns the number of fields present, capped at
 * `max`. The line is copied first: a caller's buffer is not ours to scribble
 * on, and tests pass string literals. */
static int split(char *buf, char **f, int max)
{
    int n = 0;
    char *p = buf;

    if (max <= 0)
        return 0;

    f[n++] = p;
    while (*p && n < max)
    {
        if (*p == ',')
        {
            *p = '\0';
            f[n++] = p + 1;
        }
        p++;
    }
    return n;
}

static bool field_str(const char *const *f, int n, int i, const char **out)
{
    if (i >= n || f[i][0] == '\0')
        return false;
    *out = f[i];
    return true;
}

/* Signed integer from a field that may be empty. */
static bool field_int(const char *const *f, int n, int i, int *out)
{
    const char *s;
    if (!field_str(f, n, i, &s))
        return false;

    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s)
        return false;
    *out = (int)v;
    return true;
}

/*
 * DEGREES AND MINUTES TO DECIMAL DEGREES.
 *
 * `4807.038` is 48 deg 07.038 min. The integer part of the whole number gives
 * the degrees only after the minutes are divided out: ddmm.mmmm has two degree
 * digits and the rest is minutes, so the split is at the decimal point minus
 * two, not at any fixed character count. Do it as: degrees = value / 100, and
 * minutes = value - degrees * 100, which is exact for the integer part and
 * keeps the fractional minutes intact.
 */
static bool parse_coord(const char *const *f, int n, int idx_val, int idx_hemi,
                        double *out)
{
    const char *s, *h;
    if (!field_str(f, n, idx_val, &s) || !field_str(f, n, idx_hemi, &h))
        return false;

    char *end = NULL;
    double v = strtod(s, &end);
    if (end == s)
        return false;

    double deg = (double)((long)(v / 100.0));
    double min = v - deg * 100.0;
    double dec = deg + min / 60.0;

    if (h[0] == 'S' || h[0] == 'W')
        dec = -dec;

    *out = dec;
    return true;
}

/* --- sentences ----------------------------------------------------------- */

static bool is_talker_gga(const char *s)
{
    /* GNGGA, GPGGA, GLGGA, GAGGA, GBGGA -- any talker, GGA sentence. */
    return strlen(s) >= 5 && strcmp(s + 2, "GGA") == 0;
}

static bool is_talker_rmc(const char *s)
{
    return strlen(s) >= 5 && strcmp(s + 2, "RMC") == 0;
}

/* hhmmss.ss -> hour, minute, second_x100 */
static bool parse_time(const char *s, int *h, int *m, int *sec100)
{
    size_t len = strlen(s);
    if (len < 6)
        return false;
    for (size_t i = 0; i < 6; i++)
        if (s[i] < '0' || s[i] > '9')
            return false;

    *h = (s[0] - '0') * 10 + (s[1] - '0');
    *m = (s[2] - '0') * 10 + (s[3] - '0');
    int sec = (s[4] - '0') * 10 + (s[5] - '0');

    int cs = 0;
    if (len >= 9 && s[6] == '.')
    {
        cs = (s[7] - '0') * 10;
        if (len >= 10)
            cs += s[8] - '0';
    }

    *sec100 = sec * 100 + cs;
    return *h < 24 && *m < 60 && sec < 61;
}

/* ddmmyy -> yyyy, mm, dd */
static bool parse_date(const char *s, int *y, int *mo, int *d)
{
    if (strlen(s) < 6)
        return false;
    for (int i = 0; i < 6; i++)
        if (s[i] < '0' || s[i] > '9')
            return false;

    *d = (s[0] - '0') * 10 + (s[1] - '0');
    *mo = (s[2] - '0') * 10 + (s[3] - '0');
    int yy = (s[4] - '0') * 10 + (s[5] - '0');

    /* The century window u-blox documents: 80..99 is 1900s, 00..79 is 2000s. */
    *y = yy >= 80 ? 1900 + yy : 2000 + yy;

    return *mo >= 1 && *mo <= 12 && *d >= 1 && *d <= 31;
}

/* --- UTC to seconds since the epoch -------------------------------------- */

/*
 * DAYS FROM CIVIL DATE, without a lookup table and without a library.
 *
 * This is Howard Hinnant's algorithm, which works from a shift that starts the
 * year in March so that the leap day falls at the end of it -- that single
 * trick is what removes February from every special case, and it is why this
 * is short enough to be trustworthy. The 719468 constant is the number of days
 * from the civil epoch (0000-03-01) to 1970-01-01.
 *
 * Written out rather than taken from <time.h> because the C library's
 * timegm() honours TZ on some platforms, and this must be UTC unconditionally:
 * a GPS time converted through a local time zone is wrong by exactly the
 * offset, which on a watch set to UTC-7 is seven hours and looks like a
 * hardware fault.
 */
static int64_t days_from_civil(int y, unsigned m, unsigned d)
{
    y -= m <= 2;
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);              /* 0..399 */
    const unsigned doy = (153U * (m + (m > 2 ? -3U : 9U)) + 2U) / 5U + d - 1U;
    const unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
    return era * 146097 + (int64_t)doe - 719468;
}

bool nmea_utc_raw(const nmea_state_t *s, int64_t *out)
{
    if (!out || !s->have_date || !s->have_time)
        return false;

    /* Range checks on the fields themselves, which the parser already applies
     * to a decoded sentence but which also have to hold for a state built by
     * hand. */
    if (s->month < 1 || s->month > 12 || s->day < 1 || s->day > 31)
        return false;
    if (s->hour > 23 || s->minute > 59 || s->second_x100 > 6100)
        return false;

    int64_t days = days_from_civil(s->year, (unsigned)s->month, (unsigned)s->day);

    *out = days * 86400
           + (int64_t)s->hour * 3600
           + (int64_t)s->minute * 60
           + s->second_x100 / 100;
    return true;
}

bool nmea_utc(const nmea_state_t *s, int64_t *out)
{
    /* The policy sits here rather than inside the arithmetic, so the
     * arithmetic can be tested against known epoch values from 1970 onward --
     * a date this function must refuse, but a value the day-count has to get
     * right. Splitting them is what lets both be checked. */
    if (s->year < 2020 || s->year > 2100)
        return false;

    return nmea_utc_raw(s, out);
}

static void do_gga(nmea_state_t *s, char **f, int n)
{
    int v;

    s->have_gga = true;

    if (field_int((const char *const *)f, n, 6, &v))
        s->fix_quality = v;

    if (field_int((const char *const *)f, n, 7, &v))
        s->sats_used = v;

    /* HDOP arrives as a decimal like 0.92. Stored in hundredths because an
     * integer compares exactly and the display does not need better. */
    const char *hd;
    if (field_str((const char *const *)f, n, 8, &hd))
    {
        double d = strtod(hd, NULL);
        s->hdop_x10 = (int)(d * 100.0 + 0.5);
    }

    /* Altitude in metres with one decimal; stored in cm. */
    const char *al;
    if (field_str((const char *const *)f, n, 9, &al))
    {
        double d = strtod(al, NULL);
        s->alt_cm = (int32_t)(d * 100.0 + (d < 0 ? -0.5 : 0.5));
        s->have_alt = true;
    }

    const char *gs;
    if (field_str((const char *const *)f, n, 11, &gs))
    {
        double d = strtod(gs, NULL);
        s->geoid_cm = (int32_t)(d * 100.0 + (d < 0 ? -0.5 : 0.5));
        s->have_geoid = true;
    }

    /* Position is only meaningful with a fix. A receiver with none sends empty
     * position fields, but some send stale ones -- so quality gates it. */
    double lat, lon;
    if (s->fix_quality > 0 &&
        parse_coord((const char *const *)f, n, 2, 3, &lat) &&
        parse_coord((const char *const *)f, n, 4, 5, &lon))
    {
        s->lat = lat;
        s->lon = lon;
        s->have_pos = true;
    }

    int h, m, cs;
    const char *tm;
    if (field_str((const char *const *)f, n, 1, &tm) && parse_time(tm, &h, &m, &cs))
    {
        s->hour = h;
        s->minute = m;
        s->second_x100 = cs;
        s->have_time = true;
    }
}

static void do_rmc(nmea_state_t *s, char **f, int n)
{
    s->have_rmc = true;

    const char *st;
    if (field_str((const char *const *)f, n, 2, &st))
        s->rmc_valid = (st[0] == 'A');

    double lat, lon;
    if (s->rmc_valid &&
        parse_coord((const char *const *)f, n, 3, 4, &lat) &&
        parse_coord((const char *const *)f, n, 5, 6, &lon))
    {
        s->lat = lat;
        s->lon = lon;
        s->have_pos = true;
    }

    /* Speed over ground in knots, converted to mm/s. One knot is 1852 m per
     * hour, so 1852000 mm per 3600 s = 514.444 mm/s. Kept in mm/s because the
     * desktop wants integers and a knot is not a unit anything else uses. */
    const char *sp;
    if (field_str((const char *const *)f, n, 7, &sp))
    {
        double kn = strtod(sp, NULL);
        s->speed_mm_s = (int32_t)(kn * 1852000.0 / 3600.0 + 0.5);
        s->have_speed = true;
    }

    const char *co;
    if (field_str((const char *const *)f, n, 8, &co))
    {
        double d = strtod(co, NULL);
        s->course_mdeg = (int32_t)(d * 1000.0 + 0.5);
        s->have_course = true;
    }

    int h, m, cs;
    const char *tm;
    if (field_str((const char *const *)f, n, 1, &tm) && parse_time(tm, &h, &m, &cs))
    {
        s->hour = h;
        s->minute = m;
        s->second_x100 = cs;
        s->have_time = true;
    }

    const char *dt;
    if (field_str((const char *const *)f, n, 9, &dt))
    {
        int y, mo, d;
        if (parse_date(dt, &y, &mo, &d))
        {
            s->year = y;
            s->month = mo;
            s->day = d;
            s->have_date = true;
        }
    }
}

bool nmea_line(nmea_state_t *s, const char *line)
{
    char buf[128];

    if (!line || line[0] != '$')
        return false;

    /* Length check before the copy: a truncated or overflowing line is a
     * framing fault, and silently parsing the first 127 characters of it would
     * produce a plausible sentence from garbage. */
    size_t len = strcspn(line, "\r\n");
    if (len == 0 || len >= sizeof(buf))
        return false;

    /* The '*' and its two hex digits have to be present for the checksum to
     * mean anything. A receiver always sends them. */
    const char *star = memchr(line, '*', len);
    if (!star || (size_t)(star - line) + 3 > len)
        return false;

    uint8_t want = nmea_checksum(line + 1, (size_t)(star - line) - 1);

    char hex[3] = { star[1], star[2], 0 };
    if (star[1] == '\0' || star[2] == '\0')
        return false;

    uint8_t got = (uint8_t)strtoul(hex, NULL, 16);

    s->sentences++;

    if (got != want)
    {
        s->failed_checksum++;
        return false;
    }

    memcpy(buf, line + 1, (size_t)(star - line) - 1);
    buf[(size_t)(star - line) - 1] = '\0';

    char *f[24];
    int n = split(buf, f, 24);
    if (n < 1)
        return false;

    if (is_talker_gga(f[0]))
    {
        do_gga(s, f, n);
        return true;
    }
    if (is_talker_rmc(f[0]))
    {
        do_rmc(s, f, n);
        return true;
    }

    s->ignored++;
    return false;
}
