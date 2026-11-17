/*
 * imu_posture.c - which way the watch is pointing, in words.
 *
 * IN ITS OWN FILE, WITH NO ESP-IDF DEPENDENCIES, so the rule can be walked
 * exhaustively on a host (tests/test_imu_posture.c). It is a pure function of
 * three floats and it decides what a person reads on the glass, which is
 * exactly the kind of thing that should not be verifiable only by holding a
 * watch at an angle and squinting.
 *
 * THE NAMES DESCRIBE WHAT THE WEARER DID, not which edge of the case is low.
 * The first version said EDGE UP and EDGE DOWN -- geometrically true and
 * useless, because nobody turns their wrist and thinks about which edge of a
 * rectangle is lowest. The operator asked for TILT UP / TILT DOWN / TILT LEFT /
 * TILT RIGHT for that reason.
 *
 * THE AXIS ASSIGNMENTS COME FROM MEASUREMENT, not from a datasheet. Tipping the
 * watch forward towards the top strap 90 degrees -- the motion of turning your
 * wrist to look at it -- reads x -1.00, so that direction is TILT UP. The
 * opposite is TILT DOWN. Rotating about y gives left and right, and the sign is
 * chosen so that the low edge of the case is on the side the word names.
 * FACE UP and FACE DOWN are the flat states.
 */
#include <stddef.h>

#include "tinydesk/td_sysinfo.h"
#include "imu.h"

/* The threshold is cos(45 degrees): a name is given only when one axis clearly
 * dominates, so a watch held diagonally reads TILTED rather than being forced
 * into whichever of the six it happens to be nearest. */
#define POSTURE_DOMINANT 0.70f

const char *imu_posture(const td_imu_t *r)
{
    if (r == NULL || !r->accel_valid)
        return "--";

    const float t = POSTURE_DOMINANT;

    if (r->az > t)  return "FACE UP";
    if (r->az < -t) return "FACE DOWN";
    if (r->ax > t)  return "TILT DOWN";
    if (r->ax < -t) return "TILT UP";
    if (r->ay > t)  return "TILT RIGHT";
    if (r->ay < -t) return "TILT LEFT";
    return "TILTED";
}
