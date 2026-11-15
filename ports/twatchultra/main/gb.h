/*
 * gb.h - Game Boy / Game Boy Color on the watch.
 *
 * `components/gnuboy` is the emulator core. This is everything the board
 * supplies: a frame buffer to render into, a double-size row scaler, a 60 Hz
 * pacing loop, and the input mapping.
 *
 * WHY THIS PORT EXISTS AT ALL, AND WHY IT IS NOT A GAMBLE. The NES is parked
 * because 256x240 at 1x is 46 percent of the framebuffer. 160 divides 320
 * exactly, so the Game Boy fills the glass at 2x -- 320x288, 69 percent, no side
 * margins -- and it was measured on this board before a line of this file was
 * written: `gbcbench` reports 8,615 us per frame for Pokemon Crystal, which is
 * 1.94x real time. The scoping document's 28-45 fps projection was wrong because
 * it normalised through the NES interpreter measured at -Og; the firmware has
 * since gained -O2, 120 MHz PSRAM and flash, and the 32 KB instruction cache.
 *
 * So the picture gets much better and the frame rate does not suffer. That is the
 * opposite of the expectation the scoping document recorded, and the measurement
 * is why this port was written rather than argued about.
 *
 * WHAT IS HERE: audio, from the core's own APU. This fork's `sound.c` is a whole
 * four-channel Game Boy sound chip that synthesises into a host buffer, so the
 * port's job is a buffer, a callback and the port's existing `sound_stream_*`
 * sink -- no `minigb_apu`, which an earlier version of this comment wrongly said
 * was required. What matters and is easy to get wrong is the rate: gnuboy does
 * not resample, so the sink has to be clocked at the rate the core actually
 * produces (43690.67 Hz for the round 48-cycle divisor), not at the 44100 that
 * was requested for it. gb.c derives both from one constant.
 */
#ifndef TWATCH_GB_H
#define TWATCH_GB_H

#include <stdbool.h>
#include <stdint.h>

#define GB_ROM_DEFAULT  "/sd/media/gbc/homebrew/Legend of Zelda, The - Link's Awakening DX (USA, Europe) (Rev 2) (SGB Enhanced) (GB Compatible).gbc"
#define GB_ROM_FLAT     "/sd/game.gbc"

/*
 * 2x INTEGER SCALE, and it is the only scale that fits. 160 x 2 = 320, which is
 * the framebuffer's full width, so the picture has no side margins at all -- that
 * is the whole reason to prefer this console over the NES on this glass. 3x needs
 * 480 columns and does not fit.
 *
 * ODD OFFSETS WOULD COST and do not occur here: (416 - 288) / 2 is 64, a multiple
 * of 4, so a row copy stays aligned for the panel's 4-byte SPI framing.
 *
 * TOP-ALIGNED VERTICALLY, like both other emulators, and for the same recorded
 * reason: the eye tracks the top of a small screen, and 64 rows of letterbox
 * above and below reads as badly framed where 128 below does not.
 */
#define GB_SCALE 2
#define GB_OX    ((320 - 160 * GB_SCALE) / 2)   /* 0: full width                */
#define GB_OY    0                              /* top-aligned                  */

/* The Game Boy's own refresh. 4.194304 MHz / 70224 cycles per frame. */
#define GB_FRAME_HZ     59.7275
#define GB_FRAME_US     16742

bool gb_start(const char *rom_path);
void gb_stop(void);
bool gb_running(void);

/* Registers the `gb` shell command. */
bool gb_register_command(void);

#endif /* TWATCH_GB_H */
