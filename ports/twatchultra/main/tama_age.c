/*
 * tama_age.c - formatting the pet's up time, and the sidecar's 16 bytes.
 *
 * See tama_age.h for why the unit is guest seconds and why the age lives in its
 * own file beside the checkpoint rather than inside it.
 */
#include "tama_age.h"

#include <stdio.h>
#include <string.h>

/*
 * "12d07h31m".
 *
 * DAYS ARE NOT PADDED; HOURS AND MINUTES ARE. The day count is the number the
 * operator is reading for, so it sits at the left with no leading zero, while the
 * hours and minutes are progress through the current day and hold two digits so
 * the string's width does not jitter as they cross 9.
 *
 * WHY MINUTES ARE THERE AT ALL, given that the request was for days. A counter
 * that shows only days and hours appears FROZEN for the first hour of a pet's
 * life and only moves once an hour after that; on a wrist that reads as a broken
 * label rather than a slow one. Minutes make the number visibly alive, cost one
 * repaint per guest minute -- the same cadence the clock this replaced had -- and
 * are the field that lets the operator tell a running pet from a paused one at a
 * glance. Subject to the operator's direction. */
size_t tama_age_format(uint64_t seconds, char *out, size_t cap)
{
    if (!out || cap == 0)
        return 0;

    const uint64_t days = seconds / TAMA_AGE_SECONDS_PER_DAY;
    const uint64_t rem = seconds % TAMA_AGE_SECONDS_PER_DAY;

    const int n = snprintf(out, cap, "%llud%02lluh%02llum",
                           (unsigned long long)days,
                           (unsigned long long)(rem / 3600),
                           (unsigned long long)(rem % 3600 / 60));

    /* snprintf reports what it WOULD have written. A truncated time is worse
     * than no time -- "999d23h5" clipped reads as a different number -- so the
     * caller is told nothing was written. */
    if (n < 0 || (size_t)n >= cap)
    {
        out[0] = '\0';
        return 0;
    }

    return (size_t)n;
}

/* The sidecar, byte for byte:
 *
 *   0..3   'T' 'M' 'A' 'G'
 *   4      version, 1
 *   5..7   reserved, zero
 *   8..15  guest seconds, little-endian
 *
 * THE MAGIC AND VERSION EXIST SO A FILE FROM somewhere ELSE -- an older build, a
 * renamed file, a truncated write -- is REFUSED rather than believed. A wrong age
 * is not a crash, which is exactly why nothing else would catch it. */
#define TAMA_AGE_VERSION 1

void tama_age_pack(uint64_t seconds, uint8_t out[TAMA_AGE_FILE_BYTES])
{
    memset(out, 0, TAMA_AGE_FILE_BYTES);

    out[0] = 'T';
    out[1] = 'M';
    out[2] = 'A';
    out[3] = 'G';
    out[4] = TAMA_AGE_VERSION;

    for (int i = 0; i < 8; i++)
        out[8 + i] = (uint8_t)(seconds >> (8 * i));
}

bool tama_age_unpack(const uint8_t *in, size_t len, uint64_t *seconds)
{
    if (!in || !seconds || len < TAMA_AGE_FILE_BYTES)
        return false;

    if (in[0] != 'T' || in[1] != 'M' || in[2] != 'A' || in[3] != 'G')
        return false;

    if (in[4] != TAMA_AGE_VERSION)
        return false;

    uint64_t v = 0;

    for (int i = 0; i < 8; i++)
        v |= (uint64_t)in[8 + i] << (8 * i);

    *seconds = v;
    return true;
}
