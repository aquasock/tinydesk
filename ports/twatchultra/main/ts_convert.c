/*
 * ts_convert.c - the TS thermistor's voltage to temperature, with no hardware.
 *
 * See ts_convert.h for why this is separate from charge.c and where the numbers
 * come from. Everything here is a pure function of its arguments.
 */

#include "ts_convert.h"

#include "tinydesk/td_sysinfo.h"

/*
 * Table 6-3 of the AXP2101 datasheet (X-Powers V1.4, section 6.7.4.4): detected
 * voltage at 50 uA against temperature, for the Mitsubishi TH11-3H103F.
 *
 * The datasheet prints this as three columns -- resistance, voltage and ADC
 * code -- and the voltage list here was checked against the ADC column before
 * being trusted: 0x3E8 counts at 0.5 mV per count is 500 mV, which is the
 * table's own figure at 25 C, and all seventeen rows agree to within the
 * datasheet's rounding. A transcription error in a table like this would skew
 * every reading and look entirely plausible, which is why the second column
 * exists to check the first against.
 *
 * Voltage FALLS as temperature rises, the sign of an NTC on a current source,
 * and the sequence is monotonically decreasing throughout.
 */
static const int16_t TS_TEMP_C[] = {
    -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60,
};
static const int16_t TS_MV_AT_50UA[] = {
    3150, 2508, 2013, 1628, 1325, 1084, 889, 732, 604, 500, 416, 348, 292, 246, 209, 177, 152,
};

#define TS_TABLE_N ((int)(sizeof(TS_TEMP_C) / sizeof(TS_TEMP_C[0])))

/* The part's own "sensor fault / pin open" code in the raw 14-bit field. */
#define TS_RAW_FAULT 0x2000

int ts_current_ua(uint8_t ctrl)
{
    /* reg50H[1:0]: 00 = 20 uA, 01 = 40, 10 = 50, 11 = 60. */
    static const int ua[4] = {20, 40, 50, 60};

    return ua[ctrl & 0x03];
}

int ts_tenths_c(int mv, uint8_t ctrl)
{
    int v50, i;

    /* "No reading" is checked FIRST, because it is a fact about the bus and
     * outranks any interpretation: a register that did not answer must not be
     * reported as "read, but not a temperature". */
    if (mv < 0)
        return TD_NTC_NONE;

    /* reg50H[4] set means the pin is a fixed external input and does not gate
     * the charger, so whatever is on it is not being read as the cell's
     * temperature and reporting one would be a fiction. */
    if (ctrl & 0x10)
        return TD_NTC_NOT_TEMP;

    v50 = mv * 50 / ts_current_ua(ctrl);

    /* Clamp at the table's ends rather than extrapolate: outside -20..60 C the
     * part's accuracy is not specified. */
    if (v50 >= TS_MV_AT_50UA[0])
        return TS_TEMP_C[0] * 10;
    if (v50 <= TS_MV_AT_50UA[TS_TABLE_N - 1])
        return TS_TEMP_C[TS_TABLE_N - 1] * 10;

    for (i = 0; i + 1 < TS_TABLE_N; i++)
    {
        int hi = TS_MV_AT_50UA[i], lo = TS_MV_AT_50UA[i + 1];

        if (lo <= v50 && v50 <= hi)
        {
            /* Straight line between adjacent rows, which are 5 C apart. */
            int span = hi - lo;

            return TS_TEMP_C[i] * 10 + (hi - v50) * (TS_TEMP_C[i + 1] - TS_TEMP_C[i]) * 10 / span;
        }
    }
    return TD_NTC_NONE;
}
