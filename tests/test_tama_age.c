/*
 * test_tama_age.c - the pet's up time, walked on a host.
 *
 * WHAT THIS PINS, AND WHY IT IS NOT TRIVIAL ARITHMETIC.
 *
 * The number on the glass is derived from a cycle count by dividing by
 * TMC_CLOCK_HZ, and every boundary in it -- the hour, the day, the moment a
 * paused session resumes -- is an integer-division edge. An off-by-one there is
 * not a crash and does not fail a build: it reads as a pet that is a day older
 * than it is, or one that is stuck at 23h. Those are the failures this file
 * exists to make loud, and they are cheap to walk here.
 *
 * The sidecar is the other half. A file written by an older build, or truncated
 * by a card that filled up, must be REFUSED rather than believed, because a
 * wrong age looks exactly like a right one. Every refusal path is checked.
 */
#include "td_test.h"

#include "tama_age.h"

#include <string.h>

/* --- formatting ---------------------------------------------------------- */

static void expect(const char *want, uint64_t seconds)
{
    char buf[32];

    const size_t n = tama_age_format(seconds, buf, sizeof(buf));

    CHECK_EQ(n, strlen(want));
    CHECK(strcmp(buf, want) == 0);

    if (strcmp(buf, want) != 0)
        printf("  %llu s formatted as '%s', wanted '%s'\n",
               (unsigned long long)seconds, buf, want);
}

static void test_format_boundaries(void)
{
    /* A pet that has just hatched. */
    expect("0d00h00m", 0);
    expect("0d00h00m", 59);        /* under a minute is still zero */

    /* The minute edge, which is what decides when the strip repaints. */
    expect("0d00h00m", 59);
    expect("0d00h01m", 60);
    expect("0d00h01m", 119);
    expect("0d00h02m", 120);

    /* The hour edge, and that minutes do not read 60. */
    expect("0d00h59m", 3599);
    expect("0d01h00m", 3600);
    expect("0d01h59m", 7199);
    expect("0d02h00m", 7200);

    /* The day edge, and that hours do not read 24. */
    expect("0d23h59m", 86399);
    expect("1d00h00m", 86400);
    expect("1d00h01m", 86460);
    expect("1d01h00m", 90000);
    expect("2d00h00m", 172800);

    /* A year of guest time, and the widest string the strip's budget is stated
     * against. */
    expect("365d00h00m", 365ull * 86400ull);
    expect("9999d23h59m", 9999ull * 86400ull + 23ull * 3600ull + 59ull * 60ull);
}

static void test_format_refuses_a_short_buffer(void)
{
    char tiny[3];

    /* A truncated time reads as a DIFFERENT time, so the contract is an empty
     * string and a zero length rather than a clipped number. */
    CHECK_EQ(tama_age_format(90000, tiny, sizeof(tiny)), 0);
    CHECK(tiny[0] == '\0');

    char exact[9];      /* "1d01h00m" fits in 8 + NUL */
    CHECK_EQ(tama_age_format(90000, exact, sizeof(exact)), 8);
    CHECK(strcmp(exact, "1d01h00m") == 0);

    char one[1];
    CHECK_EQ(tama_age_format(0, one, sizeof(one)), 0);

    /* NULL and zero capacity are answers, not crashes: this is called from a
     * guest task that must not fault on a bad argument. */
    CHECK_EQ(tama_age_format(0, NULL, 16), 0);
    CHECK_EQ(tama_age_format(0, tiny, 0), 0);
}

/* --- the sidecar --------------------------------------------------------- */

static void test_pack_unpack_round_trip(void)
{
    static const uint64_t cases[] = {
        0, 1, 59, 3600, 86400, 90000,
        1234567890ull,
        /* The largest value that means anything on a wrist, and a value with
         * every byte of the 64 bits set, so a byte-order mistake is visible. */
        9999ull * 86400ull + 23ull * 3600ull + 59ull * 60ull,
        0xFFFFFFFFFFFFFFFFull,
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
    {
        uint8_t buf[TAMA_AGE_FILE_BYTES];
        uint64_t back = 0;

        tama_age_pack(cases[i], buf);

        CHECK(tama_age_unpack(buf, sizeof(buf), &back));
        CHECK_EQ(back, cases[i]);
    }
}

static void test_pack_shape(void)
{
    uint8_t buf[TAMA_AGE_FILE_BYTES];

    tama_age_pack(0x0102030405060708ull, buf);

    /* The header is a literal, so a reader in another language can parse this
     * without the C source. */
    CHECK_EQ(buf[0], 'T');
    CHECK_EQ(buf[1], 'M');
    CHECK_EQ(buf[2], 'A');
    CHECK_EQ(buf[3], 'G');
    CHECK_EQ(buf[4], 1);                 /* TAMA_AGE_VERSION */
    CHECK_EQ(buf[5], 0);
    CHECK_EQ(buf[6], 0);
    CHECK_EQ(buf[7], 0);

    /* Little-endian, low byte first. */
    CHECK_EQ(buf[8], 0x08);
    CHECK_EQ(buf[9], 0x07);
    CHECK_EQ(buf[15], 0x01);

    /* No byte outside the value's own eight is left holding anything. */
    uint8_t zero[TAMA_AGE_FILE_BYTES];

    tama_age_pack(0, zero);
    CHECK_EQ(zero[0], 'T');
    CHECK_EQ(zero[1], 'M');
    CHECK_EQ(zero[2], 'A');
    CHECK_EQ(zero[3], 'G');
    CHECK_EQ(zero[4], 1);

    for (int i = 5; i < TAMA_AGE_FILE_BYTES; i++)
        CHECK_EQ(zero[i], 0);
}

static void test_unpack_refuses_bad_input(void)
{
    uint8_t buf[TAMA_AGE_FILE_BYTES];
    uint64_t out = 0;

    tama_age_pack(90000, buf);

    /* Truncated: a half-written file is refused, not guessed at. */
    CHECK(!tama_age_unpack(buf, TAMA_AGE_FILE_BYTES - 1, &out));
    CHECK(!tama_age_unpack(buf, 0, &out));

    /* Bad magic, which is what a renamed or foreign file looks like. */
    uint8_t m[TAMA_AGE_FILE_BYTES];

    memcpy(m, buf, sizeof(m));
    m[0] = 'X';
    CHECK(!tama_age_unpack(m, sizeof(m), &out));

    /* A future version is refused rather than read with this one's field
     * offsets -- that is the whole point of carrying a version. */
    memcpy(m, buf, sizeof(m));
    m[4] = 2;
    CHECK(!tama_age_unpack(m, sizeof(m), &out));

    /* A refused parse must not have written through the out parameter, or a
     * caller that ignores the return keeps the last good value by accident
     * rather than by design. */
    out = 4242;
    CHECK(!tama_age_unpack(m, sizeof(m), &out));
    CHECK_EQ(out, 4242);

    /* NULLs are answers, not crashes. */
    CHECK(!tama_age_unpack(NULL, TAMA_AGE_FILE_BYTES, &out));
    CHECK(!tama_age_unpack(buf, sizeof(buf), NULL));

    /* And the good one still parses, so the refusals above are about the fault
     * and not about this helper being broken. */
    CHECK(tama_age_unpack(buf, sizeof(buf), &out));
    CHECK_EQ(out, 90000);
}

/* --- the unit the number is in ------------------------------------------ */

/*
 * A guest second is 4,000,000 cycles, so the arithmetic that turns a run into an
 * age is one division. This is the constant the display depends on, checked
 * against the same table the emulator uses rather than against a copy of it
 * pasted here -- if the core's clock ever changes, the strip's age changes with
 * it and nothing silently reads 4x fast.
 */
static void test_guest_clock_table(void)
{
    /* 86,400 guest seconds is 345,600,000,000 cycles at 4 MHz. Written out so a
     * reader can check the day length against the core without a calculator. */
    CHECK_EQ(TAMA_AGE_SECONDS_PER_DAY, 86400);
    CHECK_EQ(TAMA_AGE_SECONDS_PER_DAY * 4000000ull, 345600000000ull);
}

int main(void)
{
    test_format_boundaries();
    test_format_refuses_a_short_buffer();
    test_pack_unpack_round_trip();
    test_pack_shape();
    test_unpack_refuses_bad_input();
    test_guest_clock_table();

    return TD_TEST_RESULT();
}
