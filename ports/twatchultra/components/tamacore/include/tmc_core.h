/*
 * tmc_core.h - the Tamagotchi emulator as a firmware component.
 *
 * PHASE 2 OF THE TAMACORE PORT. This replaces tamacore_task.c's shared-memory
 * layer with a plain object: no PSRAM window, no cache maintenance, no
 * core-0/core-1 handshake, no control block. None of that machinery is about
 * emulating a Tamagotchi -- it exists because Linux and ESP-IDF had to share one
 * chip through a fixed physical window (shmproto.h), and inside a single
 * firmware image there is no second core to talk to.
 *
 * WHAT IS KEPT, and it is the part worth keeping: the dirty-row tracking. A V3
 * display is a 32x30 dot matrix plus ten status icons sharing 512 bytes of
 * display RAM, so a changed byte does not say which item moved -- and a naive
 * "everything between the first and last change" span repaints nearly the whole
 * frame to flip one ON/OFF glyph. The tables that map a changed byte to the rows
 * it can affect are real work and move across unchanged.
 *
 * WHAT IS DIFFERENT: the caller supplies where the frame goes. The firmware
 * passes the panel's own framebuffer; the host harness passes a malloc'd
 * buffer. Nothing here knows which.
 *
 * Concurrency: one instance is not reentrant. The emulator runs on one task,
 * and the frame buffer is only written by tmc_frame() on that task.
 */
#ifndef TMC_CORE_H
#define TMC_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Where the LCD sits inside the frame, and how big the frame is. These are the
 * emulator's own geometry, not the panel's: the frame is 300x308 and the caller
 * decides where that lands. */
#define TMC_FRAME_W 300
#define TMC_FRAME_H 308
#define TMC_DOT     5
#define TMC_GAP     1
#define TMC_PAD     4

/* The guest runs at 4 MHz and a frame is 1/16 s of guest time, which is what
 * the artwork's animations are authored against. */
#define TMC_CLOCK_HZ    4000000u
#define TMC_FRAME_HZ    16u
#define TMC_FRAME_CYCLES (TMC_CLOCK_HZ / TMC_FRAME_HZ)

/* The ROM is a 512 KiB image with a four-byte signature at the front; the
 * emulator refuses anything else rather than executing rubbish. */
#define TMC_ROM_BYTES   0x80000u
#define TMC_ROM_SIG0    0xA5
#define TMC_ROM_SIG1    0x5F
#define TMC_ROM_SIG2    0x10
#define TMC_ROM_SIG3    0x49

/*
 * EEPROM save image: the guest's persistent state.
 *
 * 1024, NOT 512, and the difference was a memory-safety bug rather than a
 * wrong number. The serial EEPROM is a 24LC08 -- 8 Kbit, so 1024 bytes -- and
 * `ht24lc08_save()` memcpys HT24LC08_SIZE bytes into the caller's buffer. With
 * this constant at 512 every save wrote 512 bytes past a 512-byte stack array,
 * smashing the caller's frame. On the watch that surfaced as a reboot: the
 * overflow ran up through cmd_tama into tdsh's parser frame and corrupted the
 * FILE* it was about to close, so `tama stop` ended in
 *
 *     assert failed: check_lock_nonzero locks.c:307
 *     (lock != NULL && "Uninitialized lock used")
 *
 * -- a crash in the shell, three calls away from the line that caused it. The
 * load path read 512 bytes past its buffer for the same reason.
 *
 * tmc_core.c asserts this equals HT24LC08_SIZE at compile time, because the
 * only real defence against a size constant drifting from the device it
 * describes is to make the build fail when they disagree.
 */
#define TMC_EEPROM_BYTES 1024

typedef struct tmc tmc;

typedef enum
{
    TMC_OK = 0,
    TMC_ERR_ARG,        /* null pointer or bad length */
    TMC_ERR_SIGNATURE,  /* not a V3 ROM */
    TMC_ERR_MEMORY,     /* allocation failed */
} tmc_err;

/*
 * Create an instance. `rom` must stay valid for the instance's lifetime and be
 * at least TMC_ROM_BYTES; the emulator reads it directly rather than copying,
 * because 512 KiB is too much to duplicate on a board that has one copy anyway.
 *
 * `frame` is where tmc_frame() draws: TMC_FRAME_W * TMC_FRAME_H uint16_t,
 * RGB565. The caller owns it. It is filled with the bezel colour on creation.
 */
tmc_err tmc_new(const uint8_t *rom, size_t rom_len, uint16_t *frame, tmc **out);
void tmc_free(tmc *m);

/*
 * Run up to `cycles` more emulated cycles. Returns the machine's ABSOLUTE cycle
 * count, which is the emulator's own contract (splb32_run_cycles computes
 * target = cycles + argument) and is the shape callers must not get wrong:
 * passing a growing absolute target here makes every step larger than the last.
 *
 * Bounded work per call, so a caller can interleave it with a watchdog feed.
 */
uint64_t tmc_run(tmc *m, uint64_t cycles);

/*
 * Redraw whatever changed since the last call and report which rows moved.
 *
 * Returns the number of rows that changed, 0 when nothing did -- in which case
 * the frame buffer is untouched and the caller has nothing to push. `*y_lo` and
 * `*y_hi` are the half-open row range to blit, valid only when the return is
 * non-zero.
 *
 * This is the whole of what the old core-0/core-1 protocol was carrying: a
 * rectangle changed, here it is. In one image it is a function call.
 */
int tmc_frame(tmc *m, int *y_lo, int *y_hi);

/*
 * Redraw the WHOLE frame from the guest's DPRAM, regardless of what changed.
 *
 * `tmc_frame()` regenerates only the rows it reports, which is right for pushing
 * but leaves the rest of `frame` holding whatever was there when those rows were
 * last drawn. That is fine while every push is preceded by a regeneration of the
 * same range -- and it is exactly what makes a stale-dot fault hard to see: the
 * pushed pixels always agree with a fresh render, so the frame buffer cannot be
 * blamed, and the question of whether the GUEST still holds those dots lit is
 * unanswerable from outside.
 *
 * This regenerates everything, so a caller that pushes the full window afterwards
 * has a frame that is provably a pure function of the current DPRAM. If artefacts
 * survive that, they are in the guest's own dot data; if they vanish, they were in
 * the incremental path.
 */
void tmc_redraw(tmc *m);


/* Press or release a button. `index` 0..2 is the V3's three buttons, matching
 * the order its pin mask uses (0x01, 0x02, 0x04 on port 0). */
void tmc_button(tmc *m, int index, bool down);

/* The save image, for persisting between runs. `out` must hold
 * TMC_EEPROM_BYTES. Loading is rejected unless the length matches, because a
 * truncated image would be silently accepted as a fresh one. */
void tmc_eeprom_save(const tmc *m, uint8_t *out);
bool tmc_eeprom_load(tmc *m, const uint8_t *in, size_t len);

/* Diagnostics: cycles run and the guest's program counter, for a log line that
 * says whether the machine is progressing. */
uint64_t tmc_cycles(const tmc *m);

/*
 * Is the guest making a sound right now?
 *
 * A front end detects the rising edge of this -- silence to not-silence -- and
 * gets one event per sound. That is the only shape that works, and the previous
 * one did not: this used to expose splb32_audio_emits(), a counter that
 * increments on every square-wave toggle, and a caller comparing it against its
 * last reading saw thousands of "events" a second for as long as one tone
 * lasted. On the watch that played a beep back to back, continuously, and it
 * stopped only while the emulator was paused -- because pausing stops stepping
 * the guest and therefore stops the waveform.
 *
 * The waveform is a clock. The envelope is the signal. See
 * splb32_audio_active() for the other half of this argument.
 */
int tmc_audio_active(const tmc *m);

/*
 * ONE AUDIO SAMPLE, mixed from both channels, as a signed 16-bit value.
 *
 * The V3 drives its piezo with a PWM channel and a tone channel, and
 * splb32_sound models both as a square wave whose level flips on driver timer
 * overflows. Those overflows happen INSIDE tmc_run(), at the tone's own rate --
 * `splb32_audio_level()` returns the level left by the last flip -- so a front end
 * that reads it once per frame gets a badly aliased version of the waveform, and
 * one that reads it never gets anything at all.
 *
 * The only faithful way is to step the guest in slices no longer than one sample
 * period and read the level after each. The caller owns that loop; this function
 * is the read, and it exists so the mixing rule lives in one place instead of in
 * every front end.
 *
 * Both channels are summed and clamped. They are independent -- one can hold a
 * tone while the other holds a PWM amplitude -- and the machine has a single
 * speaker, so the sum is what it sounds like.
 */
int tmc_audio_sample(const tmc *m);

/*
 * STEP THE GUEST AND RENDER ITS AUDIO IN THE SAME PASS.
 *
 * The reason this exists rather than a loop in the front end is arithmetic, and it
 * was measured: `splb32_run_cycles()` stops when its target is reached but cannot
 * stop mid-instruction, so EVERY call overshoots by up to one instruction. One call
 * per frame overshoots once and nobody notices. A thousand calls -- one per audio
 * sample, which is what reproducing the waveform needs -- overshoot an extra
 * thousand instructions, and the guest was measured running 352,700 cycles per
 * frame instead of 250,000: 41 percent fast, with the pet ageing at the wrong rate
 * and every sound at the wrong pitch.
 *
 * So the stepping and the sampling happen in ONE loop against ONE target. The
 * residual overshoot is what a single `tmc_run()` would have, which is the floor.
 *
 * Sample i is taken once the guest has passed cycle `cycles * (i+1) / n`, so the
 * quantisation is one instruction rather than one sample period, and `out` always
 * receives exactly `n` values -- the tail is filled with the final level so a
 * caller never has to guess how many were produced.
 */
void tmc_run_audio(tmc *m, uint64_t cycles, int16_t *out, int n);

/* The guest's 512-byte display RAM, for a caller that wants to draw the frame
 * itself rather than through tmc_frame() -- the dirty-row check does exactly
 * that, so it can build a full-repaint reference to compare against. */
const uint8_t *tmc_dpram(const tmc *m);

/* Emulated MHz over the instance's lifetime is the caller's business, but the
 * instruction count is not available at runtime: the per-opcode counters are
 * compiled out (see splb32.c), so cycles are the honest unit here. */

#endif /* TMC_CORE_H */
