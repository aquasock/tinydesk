/*
 * nes.h - the NES emulator on the watch.
 *
 * PHASES 3 AND 4 OF THE NES PORT. `components/nofrendo` is the emulator, vendored
 * unmodified from retro-go and building on the ESP32-S3; this file is everything
 * the board supplies, which is small because nofrendo was designed to be ported.
 * Its whole platform surface is three callbacks -- blit, vsync and input -- plus
 * a frame buffer the caller owns.
 *
 * WHY NOFRENDO. Measured, not assumed: 1.96x faster than InfoNES on three
 * different ROMs with render and palette conversion both live, and it covers
 * 98.2 percent of a 794-ROM collection. The libretro cores (FCEUmm, QuickNES)
 * could not be built at all without a frontend. See NES-001.
 */
#ifndef TWATCH_NES_H
#define TWATCH_NES_H

#include <stdbool.h>
#include <stdint.h>

#define NES_ROM_DEFAULT "/sd/game.nes"

/* Where the picture lands on the panel.
 *
 * 1x, TOP-ALIGNED. The NES is 256x240 in a 320x416 framebuffer, so this is 46
 * percent of the panel with black bars around it -- and it is the only option
 * that is PIXEL-PERFECT, because any scale between 1x and 2x on a 256-wide image
 * either drops or doubles columns unevenly. A sprite that is two pixels wide in
 * one place and three in another reads as a rendering bug to a player, which is
 * why integer scaling is the right default and why upscaling is a separate
 * decision rather than an obvious improvement.
 *
 * THE PICTURE SITS AT ROW 0, and it was centred at row 88 until it was not.
 * The reason is the same one pm.h records for the Pokemon Mini: the eye tracks
 * the top of a small screen, and a 240-row picture floating between two equal
 * bands of dead space reads as badly framed. Centring also splits the space
 * that is left into two unusable halves, where top-aligning gives the whole
 * 176 rows below the picture to the controls that belong there -- the same
 * trade the Pokemon Mini made when its picture moved to row 0 and its d-pad
 * took the room underneath.
 *
 * THIS ONLY CHANGES WHAT IS VISIBLE, NOT WHAT IS ACCEPTABLE. The 45-degree
 * chamfers on the glass cut the corners, so a picture pushed up cannot lose an
 * edge: row 0 of the framebuffer is panel row 45, well inside the chamfer.
 *
 * ODD OFFSETS WOULD COST, and do not here: (320-256)/2 is 32, a multiple of the
 * 4-byte SPI alignment, so a row copy stays aligned. A board with an odd margin
 * would be slower for no visible benefit. */
#define NES_OX ((320 - 256) / 2)
#define NES_OY 0

/*
 * Start the emulator from `rom_path`, or NES_ROM_DEFAULT when NULL.
 *
 * Claims the screen and puts the margins into app mode, exactly as the Tamagotchi
 * does, so the desktop stops painting and the buttons stop sending keys. Returns
 * false and leaves everything as it was when the ROM will not load or PSRAM is
 * short -- a failed start must not leave the screen claimed.
 */
bool nes_start(const char *rom_path);
void nes_stop(void);
bool nes_running(void);

/* Pause the emulation without releasing the screen. Frames stop, the picture
 * stays. Resuming is the same call. */
void nes_pause(bool paused);
bool nes_paused(void);

/* Registers the `nes` shell command. */
bool nes_register_command(void);

#endif /* TWATCH_NES_H */
