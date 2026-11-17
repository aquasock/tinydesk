/*
 * test_imu_posture.c - the posture words, swept on a host.
 *
 * WHY THIS EXISTS. The vocabulary on the IMU window is the only thing a person
 * reads, and it is decided by a pure function of three floats -- so it can be
 * walked exhaustively at a desk instead of by holding a watch at an angle. That
 * matters more than usual here because THE NAMES ARE ALREADY A
 * MISUNDERSTANDING'S WORTH OF WORK: the first set said EDGE UP and EDGE DOWN,
 * which is geometrically true and tells a wearer nothing, and the operator had
 * to correct it.
 *
 * THE AXIS SIGNS ARE PINNED AGAINST A MEASUREMENT, not against the code's own
 * opinion of itself. Tipping the watch forward towards the top strap 90 degrees
 * -- the motion of turning a wrist to look at it -- reads x -1.00 on the real
 * part, and that must say TILT UP. If a future change flips a sign, this fails;
 * a person would only notice by turning their wrist and being confused, which
 * is exactly how the wrong names survived a release.
 */
#include <string.h>

#include "td_test.h"
#include "tinydesk/td_sysinfo.h"
#include "imu.h"

static td_imu_t vec(float x, float y, float z)
{
    td_imu_t r;
    memset(&r, 0, sizeof(r));
    r.accel_valid = true;
    r.ax = x;
    r.ay = y;
    r.az = z;
    return r;
}

static void expect(const char *want, float x, float y, float z)
{
    td_imu_t r = vec(x, y, z);
    const char *got = imu_posture(&r);

    if (strcmp(got, want) != 0)
        printf("  (%+.2f, %+.2f, %+.2f) -> %s, wanted %s\n", (double)x,
               (double)y, (double)z, got, want);
    CHECK(strcmp(got, want) == 0);
}

int main(void)
{
    /* THE TWO FLAT STATES. Gravity straight out of the glass, either way. */
    expect("FACE UP", 0.0f, 0.0f, 1.0f);
    expect("FACE DOWN", 0.0f, 0.0f, -1.0f);

    /* THE FOUR TILTS. The x signs are the ones the hardware measured:
     * tipping forward towards the top strap gives x -1.00 and is TILT UP. */
    expect("TILT UP", -1.0f, 0.0f, 0.0f);
    expect("TILT DOWN", 1.0f, 0.0f, 0.0f);
    expect("TILT RIGHT", 0.0f, 1.0f, 0.0f);
    expect("TILT LEFT", 0.0f, -1.0f, 0.0f);

    /* THE OPERATOR'S OWN MEASUREMENTS, exactly as reported. These are the two
     * data points this project actually has from a hand on the glass. */
    expect("FACE UP", 0.01f, -0.07f, 1.00f);    /* lying flat */
    expect("TILT UP", -1.00f, 0.00f, 0.00f);    /* tipped forward 90 degrees */

    /* BETWEEN THE AXES: no dominant axis, so no claim. A watch held diagonally
     * is TILTED rather than being forced into the nearest name. */
    expect("TILTED", 0.5f, 0.5f, 0.5f);
    expect("TILTED", 0.0f, 0.0f, 0.0f);         /* free fall, or a dead sensor */
    expect("TILTED", 0.69f, 0.0f, 0.0f);        /* just under the threshold */

    /* Just over it, the name appears -- the boundary is where it is documented
     * to be, not where it happened to land. */
    expect("TILT DOWN", 0.71f, 0.0f, 0.0f);

    /* A SENSOR THAT HAS NOTHING TO SAY SAYS SO. A NULL is what a caller with no
     * reading passes, and a silent zero vector must not read as a posture. */
    CHECK(strcmp(imu_posture(NULL), "--") == 0);
    {
        td_imu_t none;
        memset(&none, 0, sizeof(none));
        none.accel_valid = false;
        CHECK(strcmp(imu_posture(&none), "--") == 0);
    }

    return TD_TEST_RESULT();
}
