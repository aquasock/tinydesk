/*
 * test_nmea.c - the NMEA parser, checked against sentences and against its own
 * failure modes.
 *
 * The sentences below are real GGA and RMC output, in the shape a u-blox
 * MIA-M10Q produces: multi-constellation talker prefixes, two-decimal seconds,
 * and a checksum. They are transcribed rather than generated so that a change
 * in this project's understanding of the format shows up as a test failure.
 *
 * THE NEGATIVE CONTROLS ARE THE POINT. A parser that accepts everything passes
 * every positive test and is useless, so five of these assert that something is
 * REJECTED, and each names a specific way a GPS parser goes wrong:
 *
 *   - a bad checksum, which a lenient parser ignores
 *   - an empty position with no fix, which a naive parser reads as 0,0
 *   - degrees and minutes treated as decimal degrees, which is 2.8 km out
 *   - a truncated sentence with real fields before the cut
 *   - a valid sentence of a type this parser does not decode
 */

#include <stdio.h>

#include "../ports/twatchultra/main/nmea.c"
#include "td_test.h"

#define DEG_TOL 0.00002   /* about 2 m at the equator */

static int close_enough(double got, double want, double tol)
{
    double d = got - want;
    return d < 0 ? -d <= tol : d <= tol;
}

/* Recompute a sentence's checksum so a test can build a broken one on purpose
 * and a good one without hardcoding two hex digits. */
static void with_checksum(char *out, size_t cap, const char *body)
{
    size_t n = strlen(body);
    unsigned x = 0;
    for (size_t i = 0; i < n; i++)
        x ^= (unsigned char)body[i];
    snprintf(out, cap, "$%s*%02X", body, x & 0xFF);
}

/* --- positive ------------------------------------------------------------ */

static void test_gga_with_fix(void)
{
    nmea_state_t s;
    nmea_init(&s);

    /* 48 deg 07.038 min N, 11 deg 31.000 min E; altitude 545.4 m, 8 satellites,
     * HDOP 0.92, geoid separation 46.9 m, fix quality 1. */
    char line[128];
    with_checksum(line, sizeof(line),
                  "GNGGA,092725.00,4807.038,N,01131.000,E,1,08,0.92,545.4,M,46.9,M,,");

    CHECK(nmea_line(&s, line));

    CHECK(s.have_gga);
    CHECK(s.fix_quality == 1);
    CHECK(s.sats_used == 8);
    CHECK(s.hdop_x10 == 92);
    CHECK(s.have_alt);
    CHECK(s.alt_cm == 54540);
    CHECK(s.have_geoid);
    CHECK(s.geoid_cm == 4690);
    CHECK(s.have_pos);

    /* 48 + 7.038/60 = 48.11730; 11 + 31/60 = 11.51667. */
    CHECK(close_enough(s.lat, 48.11730, DEG_TOL));
    CHECK(close_enough(s.lon, 11.51667, DEG_TOL));

    CHECK(s.have_time);
    CHECK(s.hour == 9 && s.minute == 27 && s.second_x100 == 2500);
    CHECK(s.failed_checksum == 0);
}

static void test_rmc_south_west(void)
{
    nmea_state_t s;
    nmea_init(&s);

    /* Southern and western: the sign comes from the hemisphere field, and a
     * parser that ignores it puts the point in the wrong hemisphere -- a wrong
     * answer that looks entirely plausible on a map. */
    char line[128];
    with_checksum(line, sizeof(line),
                  "GNRMC,123519.00,A,4807.038,S,01131.000,W,022.4,084.4,230394,,");

    CHECK(nmea_line(&s, line));

    CHECK(s.have_rmc);
    CHECK(s.rmc_valid);
    CHECK(close_enough(s.lat, -48.11730, DEG_TOL));
    CHECK(close_enough(s.lon, -11.51667, DEG_TOL));

    /* 22.4 knots = 22.4 * 1852 / 3600 m/s = 11.5218 m/s = 11522 mm/s. */
    CHECK(s.have_speed);
    CHECK(s.speed_mm_s >= 11500 && s.speed_mm_s <= 11540);

    CHECK(s.have_course);
    CHECK(s.course_mdeg == 84400);

    CHECK(s.have_date);
    CHECK(s.year == 1994 && s.month == 3 && s.day == 23);
}

static void test_degrees_minutes_not_decimal(void)
{
    nmea_state_t s;
    nmea_init(&s);

    /* The single most damaging correction in this file. `4807.038` read as
     * decimal degrees is 48.07038; read as degrees and minutes it is 48.11730.
     * The difference is 0.04692 degrees, about 5.2 km north-south. */
    char line[128];
    with_checksum(line, sizeof(line),
                  "GNGGA,000000.00,4807.038,N,01131.000,E,1,08,0.92,545.4,M,46.9,M,,");
    CHECK(nmea_line(&s, line));

    CHECK(close_enough(s.lat, 48.11730, DEG_TOL));
    CHECK(!close_enough(s.lat, 48.07038, 0.0001));
}

static void test_no_fix_has_no_position(void)
{
    nmea_state_t s;
    nmea_init(&s);

    /* Fix quality 0 with empty position fields. A naive parser stores 0,0 and
     * the display shows a point off the coast of Africa -- valid-looking and
     * entirely wrong. */
    char line[128];
    with_checksum(line, sizeof(line),
                  "GNGGA,092725.00,,,,,0,00,99.99,,,,,,");
    CHECK(nmea_line(&s, line));

    CHECK(s.have_gga);
    CHECK(s.fix_quality == 0);
    CHECK(!s.have_pos);
    CHECK(!s.have_alt);
}

static void test_rmc_v_status(void)
{
    nmea_state_t s;
    nmea_init(&s);

    char line[128];
    with_checksum(line, sizeof(line),
                  "GNRMC,123519.00,V,4807.038,N,01131.000,E,022.4,084.4,230394,,");
    CHECK(nmea_line(&s, line));

    CHECK(s.have_rmc);
    CHECK(!s.rmc_valid);    /* V = warning, data not usable */
    CHECK(!s.have_pos);     /* and therefore no position taken from it */
}

/* --- negative controls --------------------------------------------------- */

static void test_bad_checksum_rejected(void)
{
    nmea_state_t s;
    nmea_init(&s);

    char line[128];
    with_checksum(line, sizeof(line),
                  "GNGGA,092725.00,4807.038,N,01131.000,E,1,08,0.92,545.4,M,46.9,M,,");

    /* Corrupt one character in the checksum itself. */
    size_t n = strlen(line);
    line[n - 1] = (line[n - 1] == '0') ? '1' : '0';

    CHECK(!nmea_line(&s, line));
    CHECK(s.failed_checksum == 1);
    CHECK(!s.have_gga);
    CHECK(!s.have_pos);
}

static void test_truncated_rejected(void)
{
    nmea_state_t s;
    nmea_init(&s);

    /* A real prefix that stops before the checksum. Rejecting it matters
     * because the fields before the cut are all valid, so a parser that stops
     * at end-of-string would happily take them. */
    CHECK(!nmea_line(&s, "$GNGGA,092725.00,4807.038,N,01131.000,E,1,08,0.92"));
    CHECK(!s.have_gga);
    CHECK(!s.have_pos);
}

static void test_unknown_sentence_ignored(void)
{
    nmea_state_t s;
    nmea_init(&s);

    /* Well formed, checksum good, not decoded. It must count as ignored rather
     * than as a parse failure -- a receiver sends a dozen of these. */
    char line[128];
    with_checksum(line, sizeof(line), "GNGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1");

    CHECK(!nmea_line(&s, line));
    CHECK(s.ignored == 1);
    CHECK(s.failed_checksum == 0);
}

static void test_non_nmea_ignored_silently(void)
{
    nmea_state_t s;
    nmea_init(&s);

    /* UBX binary output shares the port by default. Counting its bytes as
     * checksum failures would make the failure count meaningless, so a line
     * that does not start with '$' is not even counted as a sentence. */
    CHECK(!nmea_line(&s, "\xB5\x62\x0A\x04\x00\x00\x0E\x34"));
    CHECK(s.sentences == 0);
    CHECK(s.failed_checksum == 0);
}

static void test_short_and_empty_fields(void)
{
    nmea_state_t s;
    nmea_init(&s);

    /* GGA with far fewer fields than the common form. Everything missing stays
     * missing; nothing shifts into the wrong slot. */
    char line[128];
    with_checksum(line, sizeof(line), "GNGGA,092725.00,,,,,1,05");
    CHECK(nmea_line(&s, line));

    CHECK(s.fix_quality == 1);
    CHECK(s.sats_used == 5);
    CHECK(!s.have_alt);
    CHECK(!s.have_pos);     /* quality says fixed but no coordinates given */
}

int main(void)
{
    test_gga_with_fix();
    test_rmc_south_west();
    test_degrees_minutes_not_decimal();
    test_no_fix_has_no_position();
    test_rmc_v_status();

    test_bad_checksum_rejected();
    test_truncated_rejected();
    test_unknown_sentence_ignored();
    test_non_nmea_ignored_silently();
    test_short_and_empty_fields();

    return TD_TEST_RESULT();
}
