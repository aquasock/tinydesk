/*
 * test_maps_crc.c - the map image's checksum, walked on a host.
 *
 * WHY THIS IS WORTH A FILE. maps.c refuses an image whose CRC does not match the
 * value `tools/maps_tile.py` wrote. If the two implementations disagree, every
 * image is refused -- and the symptom on the glass is "map image checksum", which
 * looks exactly like a corrupt file on a bad card. That is a fault that sends you
 * hunting through the SD path for a bug in your own arithmetic.
 *
 * The vectors below are Python's `zlib.crc32`, not values this project invented:
 * CRC-32/ISO-HDLC is what the packer writes and what every other tool means by
 * "crc32". The 0xCBF43926 case is the standard's own published check value, so at
 * least one of these would fail if the polynomial or the reflection were wrong in
 * a way the others happened to hide.
 */
#include "td_test.h"

#include "maps.h"

#include <string.h>

static void check(const char *what, const void *p, size_t n, uint32_t want)
{
    const uint32_t got = maps_crc32((const uint8_t *)p, n);

    CHECK_EQ(got, want);

    if (got != want)
        printf("  crc('%s') = %08lx, zlib says %08lx\n",
               what, (unsigned long)got, (unsigned long)want);
}

static void test_against_zlib(void)
{
    /* The empty string is the identity case: a wrong final XOR shows up here and
     * almost nowhere else. */
    check("", "", 0, 0x00000000u);
    check("a", "a", 1, 0xE8B7BE43u);
    check("abc", "abc", 3, 0x352441C2u);

    /* THE PUBLISHED CHECK VALUE for CRC-32/ISO-HDLC. */
    check("123456789", "123456789", 9, 0xCBF43926u);

    /* The file's own magic, so a typo in the header agrees with the tooling. */
    check("TMAP", "TMAP", 4, 0x9F6076CAu);

    /* Every byte value once, which exercises the table-less inner loop against a
     * pattern rather than against text. */
    uint8_t all[64];

    for (int i = 0; i < 64; i++)
        all[i] = (uint8_t)i;

    check("all-bytes", all, sizeof(all), 0x100ECE8Cu);
}

static void test_does_not_read_past_end(void)
{
    /* A one-byte range must give the one-byte answer whatever follows it in
     * memory; a stray loop bound would pick up the neighbour. */
    uint8_t buf[8] = { 'a', 'X', 'X', 'X', 'X', 'X', 'X', 'X' };

    CHECK_EQ(maps_crc32(buf, 1), 0xE8B7BE43u);
    CHECK_EQ(maps_crc32(buf, 1), maps_crc32((const uint8_t *)"a", 1));
}

/* --- the header --------------------------------------------------------- */

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
    for (int i = 0; i < 4; i++)
        p[i] = (uint8_t)(v >> (8 * i));
}

static void good_header(uint8_t *h)
{
    h[0] = 'T'; h[1] = 'M'; h[2] = 'A'; h[3] = 'P';
    put16(h + 4, MAPS_VERSION);
    put16(h + 6, MAPS_FORMAT_RGB565);
    put16(h + 8, DISPLAY_W);
    put16(h + 10, DISPLAY_H);
    put32(h + 12, 0xA5A5A5A5u);
}

static void test_header_accepts_the_real_thing(void)
{
    uint8_t h[MAPS_HEADER_BYTES];
    uint16_t w = 0, hh = 0;
    uint32_t crc = 0;

    good_header(h);

    CHECK(maps_header_parse(h, &w, &hh, &crc));
    CHECK_EQ(w, DISPLAY_W);
    CHECK_EQ(hh, DISPLAY_H);
    CHECK_EQ(crc, 0xA5A5A5A5u);
}

static void test_header_refuses_what_it_should(void)
{
    uint8_t h[MAPS_HEADER_BYTES];
    uint16_t w = 0, hh = 0;
    uint32_t crc = 0;

    /* Bad magic. */
    good_header(h);
    h[0] = 'X';
    CHECK(!maps_header_parse(h, &w, &hh, &crc));

    /* A future version. */
    good_header(h);
    put16(h + 4, MAPS_VERSION + 1);
    CHECK(!maps_header_parse(h, &w, &hh, &crc));

    /* A format this app cannot draw. */
    good_header(h);
    put16(h + 6, MAPS_FORMAT_RGB565 + 1);
    CHECK(!maps_header_parse(h, &w, &hh, &crc));

    /* A picture that is not the framebuffer's size, in either direction. These
     * are the cases the packer could plausibly produce and the app must not
     * blit: a narrower image would leave desktop cells showing at the edges. */
    good_header(h);
    put16(h + 8, DISPLAY_W - 1);
    CHECK(!maps_header_parse(h, &w, &hh, &crc));

    good_header(h);
    put16(h + 10, DISPLAY_H + 1);
    CHECK(!maps_header_parse(h, &w, &hh, &crc));
}

int main(void)
{
    test_against_zlib();
    test_does_not_read_past_end();
    test_header_accepts_the_real_thing();
    test_header_refuses_what_it_should();

    return TD_TEST_RESULT();
}
