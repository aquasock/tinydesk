/*
 * sound.h - a beep, from the MAX98357A I2S amplifier.
 *
 * This exists to answer one question: does the speaker on this board work?
 * Nothing else on the board has ever driven it, it is documented only in
 * LilyGo's pin table and rail table, and the cost of finding out is a tone.
 *
 * It is deliberately not a sound API. TinyDesk has no audio layer at all --
 * td_sysinfo_t has no sound ops and the public header set has nothing -- so
 * anything richer than a beep would be inventing an interface for the desktop
 * to adopt, which is a conversation with its author rather than a port task.
 * What is here is the smallest thing that proves the hardware and gives the
 * terminal's BEL somewhere to go.
 */
#ifndef TWATCH_SOUND_H
#define TWATCH_SOUND_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Raise the amplifier's rail and start an I2S output; false when the rail or
 * the channel would not come up.
 *
 * Creates the channel once; calling it again is harmless.
 */
bool sound_init(void);

/*
 * Play a short beep. Safe before sound_init() and safe when there is no
 * speaker: it returns quietly, because a beep is feedback and never a thing
 * whose failure matters.
 */
void sound_beep(void);

/* sound_beep() as a td_vterm bell callback, which carries a user pointer.
 * Trivial, but it keeps the vterm's signature out of sound.c. */
void sound_bell(void *user);

/* The port's BEL hook: overrides the weak board_bell() in the shared ESP-IDF
 * app, which the desktop calls through td_sysinfo_t when any app's terminal
 * emulator receives a BEL. */
void board_bell(void);

/*
 * Play a 16-bit PCM WAV file from the filesystem.
 *
 * `path` is a real path in the ESP-IDF VFS, so the SD card is "/sd/..." and the
 * shell's own littlefs is "/...". The file is STREAMED, not loaded: a WAV of a
 * few minutes is tens of megabytes and this board has 8 MiB of PSRAM shared with
 * everything else, so the only workable shape is read-a-chunk, write-a-chunk.
 *
 * The I2S rate and slot count are reconfigured from the file's own header and
 * restored afterwards, so a 44.1 kHz stereo file plays at 44.1 kHz and a later
 * beep still comes out at 16 kHz.
 *
 * Returns false when the file cannot be opened, the header is not a WAV this
 * player decodes, or the channel is missing. It runs on the caller's task, so
 * an app that calls it must not be the one drawing the screen: the desktop's
 * UI task would stop repainting for the length of the track.
 */
bool sound_play_wav(const char *path);

/* Stop whatever sound_play_wav() is playing, from another task. */
void sound_stop(void);

/* True while a file is playing. */
bool sound_is_playing(void);

/*
 * A continuous PCM stream, for an app that synthesises audio as it runs.
 *
 * Unlike sound_play_wav() these never block the caller and never touch the
 * filesystem, so they are safe to call from a frame loop with a millisecond of
 * slack. `rate` must be the generator's own output rate or the pitch is wrong.
 *
 * sound_stream_write() takes MONO samples and duplicates them to both slots;
 * it returns how many frames were accepted and silently drops the remainder
 * when the DMA ring is full. Dropping is deliberate: the alternative is
 * blocking the caller for the drain time, which for one video frame of audio is
 * 13.9 ms and would cost more than every other optimisation in this port put
 * together. The frame loop should carry on and let the tick go missing.
 *
 * sound_stream_open() returns false when there is no amplifier, so an app can
 * treat audio as best-effort and keep drawing either way.
 */
bool sound_stream_open(int rate);
size_t sound_stream_write(const int16_t *mono, int frames);
void sound_stream_close(void);

/* Frames dropped since the stream opened, for a status line. */
uint32_t sound_stream_dropped(void);

/* Writes the driver REFUSED, as distinct from frames that did not fit. Zero in
 * a healthy session; non-zero means the drop count is reporting a fault. */
uint32_t sound_stream_errors(void);

/*
 * Why the last sound_stream_open() failed, in words: the exact call and its
 * esp_err_t name. "no amplifier" covered four different faults, and when one
 * started on the fifth track of a session there was no way to tell which.
 * Returns "no amplifier" before any attempt.
 */
const char *sound_stream_error(void);

/*
 * Interleaved stereo, BLOCKING, for a player with its own task.
 *
 * sound_stream_write() takes mono and never waits, because its caller is an
 * emulator frame loop that cannot afford a stall and can tolerate a dropped
 * tick. A music player is the reverse: nothing else to do, and a dropped
 * sample is an audible click. This waits for the I2S DMA instead, which paces
 * the decoder to the output rate for free, and takes stereo so channel
 * separation reaches the amplifier (which mixes (L+R)/2 itself).
 *
 * Returns the frames accepted; a short count means the channel stopped
 * draining, not ordinary backpressure.
 */
size_t sound_stream_write_stereo_wait(const int16_t *stereo, int frames, uint32_t timeout_ms);

#endif
