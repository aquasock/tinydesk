/*
 * test_wav.c - the RIFF/WAVE header parser.
 *
 * The parser exists because a fixed-offset reader plays metadata as samples and
 * does not fail. So the tests here are mostly about CHUNKS THAT ARE NOT WHERE
 * A NAIVE READER WOULD EXPECT THEM: extra metadata before fmt, a fmt longer than
 * 16 bytes, an odd-length chunk with its pad byte, and a data chunk whose
 * declared length exceeds what the file holds.
 *
 * The last case matters most in practice. A WAV written by a tool that was
 * interrupted, or by a streaming writer, can declare more data than it has, and
 * a player that trusts the header reads past the end of the file.
 */

#include "../ports/twatchultra/main/wav.c"
#include "td_test.h"

/* Build a header into `b`, returning its length. Little-endian by hand, since
 * that is what a file holds regardless of host. */
static size_t put32(uint8_t *b, size_t at, uint32_t v)
{
    b[at] = (uint8_t)(v & 0xFF);
    b[at + 1] = (uint8_t)((v >> 8) & 0xFF);
    b[at + 2] = (uint8_t)((v >> 16) & 0xFF);
    b[at + 3] = (uint8_t)((v >> 24) & 0xFF);
    return at + 4;
}

static size_t put16(uint8_t *b, size_t at, uint16_t v)
{
    b[at] = (uint8_t)(v & 0xFF);
    b[at + 1] = (uint8_t)((v >> 8) & 0xFF);
    return at + 2;
}

static size_t puttag(uint8_t *b, size_t at, const char *t)
{
    memcpy(b + at, t, 4);
    return at + 4;
}

static size_t putfmt(uint8_t *b, size_t at, uint16_t fmt, uint16_t ch, uint32_t rate,
                     uint16_t bits)
{
    size_t start = at;

    at = puttag(b, at, "fmt ");
    at = put32(b, at, 16);
    at = put16(b, at, fmt);
    at = put16(b, at, ch);
    at = put32(b, at, rate);
    at = put32(b, at, rate * ch * (bits / 8));       /* byte rate */
    at = put16(b, at, (uint16_t)(ch * (bits / 8)));  /* block align */
    at = put16(b, at, bits);
    CHECK_EQ(at - start, 24);
    return at;
}

/* The common case: a canonical 44-byte header. */
static void test_canonical(void)
{
    uint8_t b[64];
    wav_info_t info;
    size_t at = 0;

    at = puttag(b, at, "RIFF");
    at = put32(b, at, 36);
    at = puttag(b, at, "WAVE");
    at = putfmt(b, at, 1, 2, 44100, 16);
    at = puttag(b, at, "data");
    at = put32(b, at, 1000);

    CHECK_EQ(wav_parse(b, at, &info), WAV_OK);
    CHECK_EQ(info.format, 1);
    CHECK_EQ(info.channels, 2);
    CHECK_EQ(info.rate, 44100);
    CHECK_EQ(info.bits, 16);
    CHECK_EQ(info.data_offset, 44);
    CHECK_EQ(info.data_bytes, 1000);
}

/* A LIST chunk between fmt and data, which is what an editor writes when it
 * stores metadata. A fixed-offset reader would point data at 44 and play the
 * LIST header as samples. */
static void test_list_chunk_between(void)
{
    uint8_t b[128];
    wav_info_t info;
    size_t at = 0;

    at = puttag(b, at, "RIFF");
    at = put32(b, at, 0);
    at = puttag(b, at, "WAVE");
    at = putfmt(b, at, 1, 2, 48000, 16);
    at = puttag(b, at, "LIST");
    at = put32(b, at, 20);
    at += 20;
    at = puttag(b, at, "data");
    at = put32(b, at, 500);

    CHECK_EQ(wav_parse(b, at, &info), WAV_OK);
    CHECK_EQ(info.rate, 48000);
    CHECK_EQ(info.data_offset, at);   /* just past the data chunk's 8-byte header */
}

/* An odd-length chunk is followed by a pad byte that is not counted in its
 * size. Walking without the pad lands one byte early and every later chunk id
 * is shifted by one -- which still parses as *something*. */
static void test_odd_chunk_padding(void)
{
    uint8_t b[160];
    wav_info_t info;
    size_t at = 0;

    at = puttag(b, at, "RIFF");
    at = put32(b, at, 0);
    at = puttag(b, at, "WAVE");
    at = putfmt(b, at, 1, 1, 16000, 16);
    at = puttag(b, at, "junk");
    at = put32(b, at, 5);            /* odd */
    at += 5;
    at += 1;                         /* pad */
    at = puttag(b, at, "data");
    at = put32(b, at, 320);

    CHECK_EQ(wav_parse(b, at, &info), WAV_OK);
    CHECK_EQ(info.channels, 1);
    CHECK_EQ(info.rate, 16000);
    CHECK_EQ(info.data_bytes, 320);
}

/* A fmt chunk longer than 16 bytes, and WAVE_FORMAT_EXTENSIBLE, which is what
 * ffmpeg writes for anything unusual. The real format lives in the first two
 * bytes of the sub-format GUID at offset 24. */
static void test_extensible(void)
{
    uint8_t b[128];
    wav_info_t info;
    size_t at = 0;

    at = puttag(b, at, "RIFF");
    at = put32(b, at, 0);
    at = puttag(b, at, "WAVE");
    at = puttag(b, at, "fmt ");
    at = put32(b, at, 40);
    at = put16(b, at, 0xFFFE);       /* extensible */
    at = put16(b, at, 2);
    at = put32(b, at, 44100);
    at = put32(b, at, 44100 * 4);
    at = put16(b, at, 4);
    at = put16(b, at, 16);
    at = put16(b, at, 22);           /* cbSize */
    at = put16(b, at, 16);           /* valid bits */
    at = put32(b, at, 3);            /* channel mask */
    at = put16(b, at, 1);            /* sub-format GUID begins: 1 = PCM */
    at += 14;                        /* rest of the GUID */
    at = puttag(b, at, "data");
    at = put32(b, at, 800);

    CHECK_EQ(wav_parse(b, at, &info), WAV_OK);
    CHECK_EQ(info.format, 1);        /* resolved from the GUID, not 0xFFFE */
    CHECK_EQ(info.channels, 2);
    CHECK_EQ(info.rate, 44100);
}

/* A header claiming more data than the buffer holds is normal: the caller may
 * have read only the header. It is accepted, and the player clamps against the
 * real file size. */
static void test_data_longer_than_buffer(void)
{
    uint8_t b[64];
    wav_info_t info;
    size_t at = 0;

    at = puttag(b, at, "RIFF");
    at = put32(b, at, 0xFFFFFF);
    at = puttag(b, at, "WAVE");
    at = putfmt(b, at, 1, 2, 44100, 16);
    at = puttag(b, at, "data");
    at = put32(b, at, 50u * 1024 * 1024);

    CHECK_EQ(wav_parse(b, at, &info), WAV_OK);
    CHECK_EQ(info.data_bytes, 50u * 1024 * 1024);
    /* Body starts right after the 8-byte chunk header, which is where `at` is
     * once the length has been written. test_canonical() pins the same fact
     * against the known 44. */
    CHECK_EQ(info.data_offset, at);
}

/* Refusals, each with its own reason so the caller can say something useful. */
static void test_refusals(void)
{
    uint8_t b[64];
    wav_info_t info;
    size_t at;

    /* Not a RIFF at all. */
    memcpy(b, "JUNKjunkJUNKjunk", 16);
    CHECK_EQ(wav_parse(b, 16, &info), WAV_ERR_NOT_RIFF);

    /* RIFF but not WAVE. */
    at = 0;
    at = puttag(b, at, "RIFF");
    at = put32(b, at, 4);
    at = puttag(b, at, "AVI ");
    CHECK_EQ(wav_parse(b, at, &info), WAV_ERR_NOT_RIFF);

    /* WAVE with no fmt. */
    at = 0;
    at = puttag(b, at, "RIFF");
    at = put32(b, at, 0);
    at = puttag(b, at, "WAVE");
    at = puttag(b, at, "data");
    at = put32(b, at, 100);
    CHECK_EQ(wav_parse(b, at, &info), WAV_ERR_NO_FMT);

    /* fmt but no data. */
    at = 0;
    at = puttag(b, at, "RIFF");
    at = put32(b, at, 0);
    at = puttag(b, at, "WAVE");
    at = putfmt(b, at, 1, 2, 44100, 16);
    CHECK_EQ(wav_parse(b, at, &info), WAV_ERR_NO_DATA);

    /* Truncated mid-chunk: a claimed payload past the buffer. */
    at = 0;
    at = puttag(b, at, "RIFF");
    at = put32(b, at, 0);
    at = puttag(b, at, "WAVE");
    at = puttag(b, at, "LIST");
    at = put32(b, at, 4000);          /* far past this 24-byte buffer */
    CHECK(wav_parse(b, at, &info) != WAV_OK);

    /* Too short for even a RIFF header. */
    CHECK_EQ(wav_parse(b, 8, &info), WAV_ERR_TRUNCATED);
}

/* The status strings are what a log line will show, so they must not be empty. */
static void test_status_strings(void)
{
    wav_status_t all[] = {WAV_OK, WAV_ERR_NOT_RIFF, WAV_ERR_NO_FMT, WAV_ERR_NO_DATA,
                          WAV_ERR_TRUNCATED, WAV_ERR_UNSUPPORTED};

    for (unsigned i = 0; i < sizeof(all) / sizeof(all[0]); i++)
    {
        const char *s = wav_status_str(all[i]);

        CHECK(s && s[0]);
        /* Short enough to survive the Log Viewer's 40 columns alongside a tag. */
        CHECK(strlen(s) < 24);
    }
}

int main(void)
{
    test_canonical();
    test_list_chunk_between();
    test_odd_chunk_padding();
    test_extensible();
    test_data_longer_than_buffer();
    test_refusals();
    test_status_strings();
    return TD_TEST_RESULT();
}
