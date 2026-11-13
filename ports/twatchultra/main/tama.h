/*
 * tama.h - the Tamagotchi emulator on the watch.
 *
 * PHASE 4 OF THE TAMACORE PORT, and the first one that puts guest pixels on
 * this panel. tmc_core (components/tamacore) is the emulator with no shared
 * memory in it; this file is everything the board has to supply: a ROM from
 * storage, two PSRAM buffers, an owner for the screen, and a task that runs the
 * guest and pushes what changed.
 *
 * WHY THE FRAME IS A SEPARATE BUFFER AND NOT THE PANEL'S OWN. display.c's
 * framebuffer is 320x416 and the emulator's frame is 300x308 with its own bezel
 * and dot pitch. Handing it the panel's buffer would mean the emulator knowing
 * the panel's stride and origin, which is exactly the coupling tmc_core was
 * written to avoid -- the host harness passes a malloc'd buffer and nothing in
 * the emulator can tell the difference. So the emulator owns its frame and this
 * file copies rows out of it, which also makes the dirty-row rectangle the unit
 * of transfer: a changed icon costs a few hundred pixels, not a screen.
 */
#ifndef TAMA_H
#define TAMA_H

#include <stdbool.h>

#include "display.h"
#include "tmc_core.h"

#define TAMA_ROM_DEFAULT "/sd/rom.bin"

/* Where the frame lands on the panel: centred, so the bezel is symmetric. The
 * emulator's geometry comes from tmc_core.h rather than being restated, so a
 * change of artwork moves the offset with it. */
#define TAMA_OX ((DISPLAY_W - TMC_FRAME_W) / 2)
#define TAMA_OY ((DISPLAY_H - TMC_FRAME_H) / 2)

/* Start the guest from `rom_path`, or from TAMA_ROM_DEFAULT when NULL. Claims
 * the screen, so the desktop stops painting until tama_stop(). Returns false
 * and leaves everything as it was when the ROM will not load or memory is
 * short -- a failed start must not leave the screen claimed. */
bool tama_start(const char *rom_path);

/* Release the screen and hand it back to the desktop. Safe when not running. */
void tama_stop(void);

bool tama_running(void);

/* Registers the `tama` shell command with TinyDesk Shell, and offers the margin
 * buttons to the emulator. Both are one call because both are "make this app
 * reachable" and there is no ordering in which one is useful without the
 * other. */
bool tama_register_command(void);

/* How many of the strip's three bars map to the guest, left to right. */
#define TAMA_BTN_N 3

#endif /* TAMA_H */
