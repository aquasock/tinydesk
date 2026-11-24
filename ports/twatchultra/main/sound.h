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

/*
 * THE EMULATOR'S VOLUME: FIVE POSITIONS ON ONE BUTTON, STARTING MUTED.
 *
 * One control rather than a slider, because the glass has no room for a slider and
 * a thumb on a 45-pixel column button cannot hit a continuous range anyway. The
 * positions are Mute, 25%, 50%, 75% and Max, and pressing the button steps to the
 * next one and wraps.
 *
 * THE PERCENTAGES ARE LOUDNESS, NOT AMPLITUDE. A quarter of the amplitude is not
 * a quarter as loud; it is about 12 dB quieter, which a person hears as less than
 * half. So the steps are a cubic law -- gain = level cubed -- which is the audio
 * taper a 15A pot gives and which VLC also uses, and it puts half at -18 dB. That
 * is the convention this project already recorded as the answer to "what should a
 * volume control's taper law be" (AUD-001 in core-reference.md), so the labels and
 * the sound agree instead of the label being a number nothing implements.
 *
 * EVERY STREAM STARTS MUTED BY CALLING sound_volume_reset() WHEN IT OPENS, so a
 * watch that boots into an emulator is silent until someone asks for sound. That
 * is the operator's instruction, and it also means the start-up audio burst is
 * not heard unless it is asked for.
 *
 * WHAT THE RESET CANNOT KNOW: whether a finger is already on the glass. The state
 * it clears is sound.c's memory of the last report it saw, and at the moment an
 * emulator starts there has been none -- so a thumb resting on the volume button as
 * the picture comes up reads as a fresh press and steps once. The alternative was
 * worse: starting with the button assumed held would swallow the first real tap
 * entirely, since the driver sends nothing until something is touched. One step is
 * recoverable by pressing again, so it is recorded rather than engineered around.
 *
 * The gain is applied in sound_stream_write(), which is the one funnel every
 * emulator's mono audio passes through (Game Boy, NES, Pokemon Mini and the
 * Tamagotchi). The music player is deliberately NOT affected: it writes through
 * sound_stream_write_stereo_wait() with its own player and its own volume story,
 * and this control belongs to the emulator on the glass.
 */
void sound_volume_reset(void);

/* What to write on the button: "Mute", "25%", "50%", "75%" or "Max". */
const char *sound_volume_label(void);

/* The position, 0 (Mute) to 4 (Max), for a status line. */
int sound_volume_index(void);

/*
 * One press, one step. `held` is whether the glass is under a finger on this
 * frame; the step happens on the transition from not-held to held and nowhere
 * else.
 *
 * THE DEBOUNCE BELONGS HERE RATHER THAN IN EACH EMULATOR. The CST9217 re-reports
 * a held finger about every 66 ms (TCH-012), so acting on every report would spin
 * a held press through all five positions and land on whichever one the finger
 * happened to leave on. The picker already paid for this lesson once, where a
 * single tap moved the highlight five rows.
 *
 * Returns true when this call changed the volume, so the caller knows to relabel.
 * Called from the touch task only.
 */
bool sound_volume_cycle_on_press(bool held);

/*
 * Put the on-glass control in charge of a stream, or take it out again.
 *
 * True from an emulator that DRAWS the volume button (slot B's three); a stream
 * that never calls this is played at unity and is exactly as it was before the
 * control existed. sound_stream_close() clears it, so the control's lifetime is
 * the stream's and a slot B emulator cannot leave the watch muted for an emulator
 * that has no button to undo it.
 */
void sound_volume_apply(bool on);

/* Writes the driver REFUSED, as distinct from frames that did not fit. Zero in
 * a healthy session; non-zero means the drop count is reporting a fault. */
uint32_t sound_stream_errors(void);

/* The ring sound_init() actually got, after the descending-order fallback. A
 * drop rate is only comparable with another from the same ring, so a status line
 * has to report which one is in force. Both are 0 until audio is up. */
uint32_t sound_ring_frames(void);
uint32_t sound_ring_bytes(void);

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
