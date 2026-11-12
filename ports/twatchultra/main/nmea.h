/*
 * nmea.h - the sentences a u-blox MIA-M10Q sends, parsed without a stream.
 *
 * Deliberately NOT a streaming library. The driver hands this whole lines from
 * the UART and asks what they said; nothing here owns a buffer, blocks, or
 * knows what a UART is. That is what makes it testable on a host against
 * recorded sentences, which is the only way to be sure a parser is right when
 * the antenna is on a desk and the sky is not available.
 *
 * Only two sentences are decoded, because they are the two that carry what a
 * watch needs:
 *
 *   GGA  time, latitude, longitude, fix quality, satellites used, HDOP,
 *        altitude, and the geoid separation
 *   RMC  time, date, position, speed over ground, course, and status
 *
 * Everything else is counted and ignored. A receiver sends a dozen sentence
 * types and parsing them all would be work with no consumer; the count of
 * unparsed ones is reported instead, so their absence from the parse is
 * visible rather than silent.
 */
#ifndef TWATCH_NMEA_H
#define TWATCH_NMEA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Fix quality from GGA field 6. */
enum
{
    NMEA_FIX_NONE = 0,
    NMEA_FIX_GPS = 1,
    NMEA_FIX_DGPS = 2,
    NMEA_FIX_PPS = 3,
    NMEA_FIX_RTK = 4,
    NMEA_FIX_FRTK = 5,
    NMEA_FIX_DR = 6,
};

typedef struct
{
    /* --- GGA --- */
    bool have_gga;
    int fix_quality;     /* NMEA_FIX_*, 0 = no fix */
    int sats_used;       /* satellites used in the fix, not visible */
    int hdop_x10;        /* HDOP in hundredths; 0 when the field is empty */
    int32_t alt_cm;      /* altitude above mean sea level, cm */
    bool have_alt;       /* false when the field was empty or malformed */
    int32_t geoid_cm;    /* geoid separation, cm */
    bool have_geoid;

    /* --- RMC --- */
    bool have_rmc;
    bool rmc_valid;      /* the A/V status field: A means the data is usable */

    /* --- position, from whichever sentence last carried a valid one --- */
    bool have_pos;
    double lat;          /* signed degrees, north and east positive */
    double lon;
    int32_t speed_mm_s;  /* speed over ground, mm/s */
    bool have_speed;
    int32_t course_mdeg; /* course over ground, millidegrees */
    bool have_course;

    /* --- time, UTC --- */
    bool have_time;
    int hour, minute;
    int second_x100;     /* hundredths, because GGA carries 2 decimal places */
    bool have_date;
    int year, month, day;

    /* --- what arrived --- */
    unsigned sentences;
    unsigned failed_checksum;
    unsigned ignored;    /* well-formed but not a sentence this parses */
} nmea_state_t;

/* Zero a state. */
void nmea_init(nmea_state_t *s);

/*
 * Consume one line.
 *
 * Accepts a trailing "\r\n", "\n", or nothing at all, so it can be fed either
 * a full line from a reader or a string literal from a test. A line that does
 * not begin with '$' is ignored silently -- a receiver's binary UBX output is
 * on the same port by default, and counting those as errors would make the
 * checksum figure meaningless.
 *
 * Returns true when the line was a sentence this function decoded.
 */
bool nmea_line(nmea_state_t *s, const char *line);

/*
 * The time in this state as seconds since the Unix epoch, UTC.
 *
 * Returns false when the date or the time is missing, or when the date is
 * outside 2020..2100 -- the same plausibility floor the RTC seed uses, because
 * a watch that boots to 1970 is worse than one that admits it does not know.
 * Valid WITHOUT a fix: a receiver knows the time once it has decoded ephemeris,
 * which is exactly the state this watch is in indoors.
 */
bool nmea_utc(const nmea_state_t *s, int64_t *out);

/* The same conversion with no plausibility floor, for testing the day-count
 * against known epochs (1970 is a legitimate value to compute and an
 * illegitimate one to set a watch to). Do not use this to set a clock. */
bool nmea_utc_raw(const nmea_state_t *s, int64_t *out);

/*
 * The checksum NMEA uses: XOR of every byte between '$' and '*'.
 *
 * Exposed because a test wants to build a sentence and check it, and because a
 * driver that relays raw sentences should not have to reimplement it.
 */
uint8_t nmea_checksum(const char *between, size_t len);

#endif
