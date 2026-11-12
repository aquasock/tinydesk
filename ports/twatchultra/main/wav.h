/*
 * wav.h - parse a RIFF/WAVE header, with no ESP-IDF in it.
 *
 * Split out from the player for the reason ts_convert.c was split from charge.c:
 * a header parser is where wrong-looking-right bugs live. An off-by-four in a
 * chunk walk, a sample rate read from the wrong offset, or a mono file treated
 * as stereo all produce sound -- just the wrong sound, or noise -- and none of
 * them raise an error anywhere. Pure functions over bytes can be swept on a
 * host; the same code behind a file open cannot.
 */
#ifndef TWATCH_WAV_H
#define TWATCH_WAV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* What a caller needs to set up a stream. */
typedef struct
{
    uint16_t format;        /* 1 = PCM, 3 = IEEE float, 0xFFFE = extensible */
    uint16_t channels;      /* 1 or 2 are the ones this project handles */
    uint32_t rate;          /* samples per second */
    uint16_t bits;          /* bits per sample */
    uint32_t data_offset;   /* byte offset of the first sample */
    uint32_t data_bytes;    /* length of the sample data */
} wav_info_t;

/* Why a header was refused, so the caller can say something better than "no". */
typedef enum
{
    WAV_OK = 0,
    WAV_ERR_NOT_RIFF,       /* not a RIFF/WAVE file at all */
    WAV_ERR_NO_FMT,         /* no fmt chunk before the data */
    WAV_ERR_NO_DATA,        /* no data chunk */
    WAV_ERR_TRUNCATED,      /* a chunk header or its payload runs past what we have */
    WAV_ERR_UNSUPPORTED,    /* a format this player does not decode */
} wav_status_t;

/*
 * Parse the header into `out`.
 *
 * `len` is how many bytes of the file are available in `buf`; the parser only
 * reads within that. It walks the chunk list rather than assuming a canonical
 * 44-byte header, because real files carry LIST/INFO, fact and padding chunks
 * between fmt and data and a fixed-offset reader would decode the wrong bytes.
 *
 * `buf` need not contain the sample data -- only the chunks up to and including
 * `data` -- so a caller can give it a small prefix of a large file.
 */
wav_status_t wav_parse(const uint8_t *buf, size_t len, wav_info_t *out);

/* A short name for a status, for a log line. */
const char *wav_status_str(wav_status_t s);

#endif
