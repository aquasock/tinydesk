/*
 * musicdec.h - FLAC decoding for the music player, behind a C interface.
 *
 * WHY THIS FILE EXISTS AT ALL. micro-flac is a C++ component: its decoder is a
 * class with RAII allocation, and every call site in this port is C. Rather
 * than sprinkle `extern "C"` and C++ idioms through the player, the whole of
 * the C++ surface is confined to one .cpp file with a flat C API.
 *
 * WHY micro-flac AND NOT SOMETHING ELSE is argued in
 * components/micro-flac/CMakeLists.txt, where the three candidates were
 * measured. The short version: this board has ~18 KB of internal SRAM free
 * with the desktop running, so rockbox's FLAC decoder -- which reserves 190 KB
 * of it STATICALLY, 7 channels' worth of sample buffer plus a 64 KB input
 * buffer -- cannot run here at all. micro-flac sizes its one working buffer
 * from the file's own STREAMINFO and allocates it in PSRAM, and has Xtensa
 * assembly for the LPC restore loop. Measured on this board it costs 31.7 KB
 * of flash and allocates 32 KB for a 4096-block stereo track.
 *
 * THE API IS A PULL LOOP, because that is what the player's task wants: read
 * compressed bytes, get PCM out, write it to I2S, repeat. The decoder reports
 * how much input it consumed rather than buffering internally, so the file
 * position stays the caller's business and a stalled SD read costs nothing.
 */
#ifndef TWATCH_MUSICDEC_H
#define TWATCH_MUSICDEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct music_dec music_dec_t;

/* Open a decoder. Returns NULL if the decoder object or its working buffer
 * cannot be allocated. The stream is not parsed until the first
 * music_dec_read(). */
music_dec_t *music_dec_open(void);

void music_dec_close(music_dec_t *d);

/*
 * Feed compressed bytes, get PCM out.
 *
 * `in`/`in_len` is a window of the file. On return `*consumed` is how many of
 * those bytes the decoder used, and `*samples` is how many INTERLEAVED STEREO
 * int16 frames were written to `out` (0 if none yet).
 *
 * The caller keeps the file position and feeds the rest of the window on the
 * next call, so a partial read costs nothing and a short SD read is not an
 * error.
 *
 * Returns:
 *   1  a frame was decoded (or the header was parsed -- check music_dec_ready)
 *   0  the decoder needs more input: advance the file, feed more
 *  -1  end of stream
 *  -2  a decode error (the decoder resets its frame state, so a fresh window
 *      may be tried; the caller should count these and stop on repeats)
 */
int music_dec_read(music_dec_t *d, const uint8_t *in, size_t in_len,
                   size_t *consumed, int16_t *out, size_t out_frames,
                   size_t *samples);

/* True once STREAMINFO has been parsed, so the rate and channel count below
 * are meaningful. Allocate the PCM buffer off these values. */
bool music_dec_ready(const music_dec_t *d);

int music_dec_rate(const music_dec_t *d);       /* Hz */
int music_dec_channels(const music_dec_t *d);   /* 1 or 2 */
int music_dec_bits(const music_dec_t *d);       /* 8, 16, 24 or 32 */

/* Total frames (per channel) from STREAMINFO, 0 when the file did not say. */
uint64_t music_dec_total_frames(const music_dec_t *d);

#ifdef __cplusplus
}
#endif

#endif
