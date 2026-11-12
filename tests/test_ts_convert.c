/*
 * test_ts_convert.c - the TS thermistor conversion, swept against the datasheet.
 *
 * The conversion is the one part of the battery-thermistor feature that can be
 * wrong in a plausible way: a mis-transcribed table row or an assumed excitation
 * current produces temperatures that look entirely reasonable and are four or
 * five degrees out. So this test does two things a code review cannot.
 *
 * FIRST, it restates the AXP2101 datasheet's Table 6-3 here, independently of
 * the implementation, and checks that every row survives the round trip. The
 * table is given as temperature, resistance and ADC code -- the resistance and
 * code columns are the datasheet's own and are not what the implementation uses,
 * so they are a second opinion on the same physical fact rather than a copy.
 *
 * SECOND, it checks the current scaling by construct: the same thermistor
 * produces a voltage proportional to the excitation current, so a reading taken
 * at 60 uA must decode to the same temperature as 50/60 of that reading taken at
 * 50 uA. A conversion that ignored reg50H would fail that and nothing else.
 */

#include "../ports/twatchultra/main/ts_convert.c"
#include "td_test.h"

/* Half a degree. Used where the test does its own integer scaling of a voltage,
 * which can be a millivolt out before the conversion is asked anything. */
static int near(int got, int want)
{
    return got >= want - 5 && got <= want + 5;
}

/* Table 6-3 of AXP2101_Datasheet_V1.4_en.pdf section 6.7.4.4, transcribed from
 * the vendor PDF rather than from our own implementation. */
static const int ds_t[]  = {-20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60};
static const int ds_r[]  = {63000, 50150, 40260, 32550, 26490, 21680, 17780, 14630,
                            12070, 10000, 8320, 6954, 5839, 4924, 4171, 3549, 3032};
static const int ds_adc[] = {0x189C, 0x1398, 0xFBA, 0xCB8, 0xA5A, 0x878, 0x6F2, 0x5B8,
                             0x4B8, 0x3E8, 0x340, 0x2B8, 0x248, 0x1EC, 0x1A2, 0x162, 0x130};
#define DS_N ((int)(sizeof(ds_t) / sizeof(ds_t[0])))

/* reg50H values for the four currents, with bit 4 clear so the pin is in
 * battery-thermistor mode. */
#define CTRL_20UA 0x04
#define CTRL_40UA 0x05
#define CTRL_50UA 0x06
#define CTRL_60UA 0x07

/* The datasheet's ADC column is the same voltage as its volts column, at 0.5 mV
 * per count. If those two ever disagreed, the table above would be suspect. */
static void test_datasheet_internally_consistent(void)
{
    for (int i = 0; i < DS_N; i++)
    {
        int mv_from_adc = ds_adc[i] / 2;   /* counts * 0.5 mV */

        /* The printed volts column is split across two arrays above only via the
         * ADC column, so compare it against the resistance and the physics:
         * V = R * 50 uA must hold to rounding. */
        int mv_from_r = (int)((long)ds_r[i] * 50 / 1000);

        CHECK(mv_from_adc >= mv_from_r - 4 && mv_from_adc <= mv_from_r + 4);
    }
}

/* Every table row must come back as its own temperature, at the current the
 * datasheet's table assumes. */
static void test_every_row_round_trips(void)
{
    for (int i = 0; i < DS_N; i++)
    {
        int mv = ds_adc[i] / 2;
        int got = ts_tenths_c(mv, CTRL_50UA);

        CHECK_EQ(got, ds_t[i] * 10);
    }
}

/* Midpoints must land between their neighbours, not on one of them: this is
 * what proves the interpolation is actually interpolating. */
static void test_midpoints_are_between(void)
{
    for (int i = 0; i + 1 < DS_N; i++)
    {
        int hi = ds_adc[i] / 2, lo = ds_adc[i + 1] / 2;
        int mid = (hi + lo) / 2;
        int got = ts_tenths_c(mid, CTRL_50UA);
        int expect = (ds_t[i] + ds_t[i + 1]) * 10 / 2;

        /* A straight line between two rows 5 C apart, so the midpoint of the
         * voltages is the midpoint of the temperatures to within rounding. */
        CHECK(got >= expect - 5 && got <= expect + 5);
        CHECK(got >= ds_t[i] * 10 && got <= ds_t[i + 1] * 10);
    }
}

/* THE CURRENT SCALING. The thermistor's resistance does not depend on the
 * current, and voltage across it is proportional to that current -- so a
 * reading at 60 uA is 6/5 of the reading the same temperature would give at
 * 50 uA, and must decode to the same place. */
static void test_current_scaling(void)
{
    for (int i = 0; i < DS_N; i++)
    {
        int mv50 = ds_adc[i] / 2;
        int want = ds_t[i] * 10;

        CHECK_EQ(ts_tenths_c(mv50, CTRL_50UA), want);

        /* Within half a degree: the test scales the voltage with integer
         * arithmetic, so a row can be a millivolt out before the conversion
         * ever sees it. The tolerance is well under the table's 5 C spacing,
         * so it cannot hide a row landing on the wrong neighbour. */
        CHECK(near(ts_tenths_c(mv50 * 6 / 5, CTRL_60UA), want));
        CHECK(near(ts_tenths_c(mv50 * 4 / 5, CTRL_40UA), want));
        CHECK(near(ts_tenths_c(mv50 * 2 / 5, CTRL_20UA), want));
    }

    /* The negative control the whole point depends on: the SAME 60 uA reading
     * interpreted as if it were 50 uA must be wrong, and by a lot. If these two
     * came out equal, the scaling would be doing nothing. */
    {
        int as_60 = ts_tenths_c(600, CTRL_60UA);
        int as_50 = ts_tenths_c(600, CTRL_50UA);

        CHECK(as_60 != as_50);
        /* 600 mV at 60 uA is 500 mV at 50 uA, which is 25.0 C. Read as though it
         * were a 50 uA measurement it is 600 mV, and higher voltage means more
         * resistance means COLDER -- about 20.4 C. So ignoring the current makes
         * the battery look colder, by roughly 4.6 C, which is the error this
         * scaling exists to prevent. */
        CHECK_EQ(as_60, 250);
        CHECK(as_50 < as_60);
        CHECK(as_60 - as_50 > 40);      /* more than 4 C apart */
    }
}

static void test_current_decode(void)
{
    CHECK_EQ(ts_current_ua(CTRL_20UA), 20);
    CHECK_EQ(ts_current_ua(CTRL_40UA), 40);
    CHECK_EQ(ts_current_ua(CTRL_50UA), 50);
    CHECK_EQ(ts_current_ua(CTRL_60UA), 60);
    /* Only bits 1:0 select, so the enable and mode bits must not disturb it. */
    CHECK_EQ(ts_current_ua(0x07), 60);
    CHECK_EQ(ts_current_ua(0x0F), 60);
}

/* Clamping rather than extrapolating: outside the tabulated range the part's
 * accuracy is unspecified, so the ends are reported instead of a guess. */
static void test_clamps_at_the_ends(void)
{
    CHECK_EQ(ts_tenths_c(5000, CTRL_50UA), -200);   /* far above 3150 mV */
    CHECK_EQ(ts_tenths_c(50, CTRL_50UA), 600);      /* far below 152 mV */
}

/* The two sentinels, and the order they are tested in: an absent reading must
 * not be reported as "read, but not a temperature". */
static void test_sentinels(void)
{
    CHECK_EQ(ts_tenths_c(-1, CTRL_50UA), TD_NTC_NONE);
    CHECK_EQ(ts_tenths_c(-1, 0x17), TD_NTC_NONE);        /* absent beats not-temp */
    CHECK_EQ(ts_tenths_c(500, 0x17), TD_NTC_NOT_TEMP);   /* bit 4 set: not the cell */
    CHECK_EQ(ts_tenths_c(500, 0x06), 250);               /* bit 4 clear: 25.0 C */
}

/* Monotonic across the whole range: a hotter battery must never read colder.
 * This is the property a wrong table would break, and it is cheap to state. */
static void test_monotonic(void)
{
    int prev = ts_tenths_c(3200, CTRL_50UA);

    for (int mv = 3150; mv >= 100; mv -= 5)
    {
        int t = ts_tenths_c(mv, CTRL_50UA);

        CHECK(t >= prev);
        prev = t;
    }
}

/* The board's own readings, recorded in PMU-006 and PMU-007, decode to the range
 * those records state. This ties the arithmetic to what the hardware actually
 * produced rather than to the datasheet alone. */
static void test_matches_the_watch(void)
{
    /* 494-527 mV at the 60 uA this board reports, per PMU-007. */
    CHECK_EQ(ts_tenths_c(494, CTRL_60UA), 303);
    CHECK_EQ(ts_tenths_c(527, CTRL_60UA), 286);
}

int main(void)
{
    test_datasheet_internally_consistent();
    test_every_row_round_trips();
    test_midpoints_are_between();
    test_current_scaling();
    test_current_decode();
    test_clamps_at_the_ends();
    test_sentinels();
    test_monotonic();
    test_matches_the_watch();
    return TD_TEST_RESULT();
}
