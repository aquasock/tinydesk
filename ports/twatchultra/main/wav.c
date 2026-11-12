/*
 * wav.c - parse a RIFF/WAVE header, with no ESP-IDF in it.
 *
 * See wav.h for why this is separate from the player.
 *
 * THE CHUNK WALK IS THE POINT. A WAV file is a RIFF container: "RIFF", a
 * 32-bit size, "WAVE", then a sequence of chunks each with a four-byte id, a
 * 32-bit length and that many bytes of payload, padded to an even length. The
 * canonical 44-byte header is only the common case -- a file written by ffmpeg,
 * an editor or a phone may carry LIST/INFO metadata, a fact chunk, or a
 * fmt chunk longer than 16 bytes, any of which shifts the data by an amount a
 * fixed-offset reader would not expect. Such a reader does not fail; it plays
 * the metadata bytes as samples, which is sound.
 */

#include "wav.h"

#include <string.h>

/* Little-endian helpers. A WAV is little-endian regardless of the host, so
 * these are written out rather than memcpy'd into a uint32_t. */
static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool tag_is(const uint8_t *p, const char *tag)
{
    return memcmp(p, tag, 4) == 0;
}

const char *wav_status_str(wav_status_t s)
{
    switch (s)
    {
    case WAV_OK:               return "ok";
    case WAV_ERR_NOT_RIFF:     return "not a wav";
    case WAV_ERR_NO_FMT:       return "no fmt chunk";
    case WAV_ERR_NO_DATA:      return "no data chunk";
    case WAV_ERR_TRUNCATED:    return "truncated header";
    case WAV_ERR_UNSUPPORTED:  return "unsupported format";
    default:                   return "?";
    }
}

wav_status_t wav_parse(const uint8_t *buf, size_t len, wav_info_t *out)
{
    size_t pos;
    bool have_fmt = false;

    if (!buf || !out || len < 12)
        return WAV_ERR_TRUNCATED;

    memset(out, 0, sizeof(*out));

    if (!tag_is(buf, "RIFF") || !tag_is(buf + 8, "WAVE"))
        return WAV_ERR_NOT_RIFF;

    /* First chunk starts after "RIFF" <size> "WAVE". */
    pos = 12;

    while (pos + 8 <= len)
    {
        const uint8_t *id = buf + pos;
        uint32_t size = le32(buf + pos + 4);
        size_t body = pos + 8;

        /* A chunk whose payload runs past what we were given cannot be trusted,
         * with one exception: the data chunk itself, which is allowed to be
         * longer than the buffer because the caller may only have read the
         * header. Its own offset and length are still usable, so it is accepted
         * as long as its START is within reach. */
        if (tag_is(id, "data"))
        {
            if (!have_fmt)
                return WAV_ERR_NO_FMT;
            if (body > len)
                return WAV_ERR_TRUNCATED;

            out->data_offset = (uint32_t)body;
            out->data_bytes = size;

            /* A truncated or streaming-written file can claim more data than it
             * has. The caller clamps against the real file size, so a bogus
             * length here is survivable rather than fatal. */
            return WAV_OK;
        }

        if (size > len - body)
            return WAV_ERR_TRUNCATED;

        if (tag_is(id, "fmt "))
        {
            /* 16 bytes is the minimum; the extensible form is longer and the
             * fields this player needs are in the first 16 either way. */
            if (size < 16)
                return WAV_ERR_UNSUPPORTED;

            out->format = le16(buf + body + 0);
            out->channels = le16(buf + body + 2);
            out->rate = le32(buf + body + 4);
            out->bits = le16(buf + body + 14);

            /* 0xFFFE is WAVE_FORMAT_EXTENSIBLE: a 16-byte fmt chunk is
             * followed by a sub-format GUID whose first two bytes are the real
             * format. Files with more than two channels or above 16 bits are
             * written that way by ffmpeg, so refusing it would refuse honest
             * files -- but only PCM underneath is decodable here. */
            if (out->format == 0xFFFE && size >= 26)
                out->format = le16(buf + body + 24);

            have_fmt = true;
        }

        /* Chunks are word-aligned: an odd payload is followed by a pad byte
         * that is not counted in `size`. */
        pos = body + size + (size & 1u);
    }

    return have_fmt ? WAV_ERR_NO_DATA : WAV_ERR_NO_FMT;
}
