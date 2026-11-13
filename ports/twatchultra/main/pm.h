/*
 * pm.h - Pokemon Mini on the watch.
 *
 * `components/pokemini` is the emulator core. This is everything the board
 * supplies, and it is small because PokeMini asks for little: a 96x64 dot
 * buffer to read, a keypad event call, and someone to call EmulateCycles.
 *
 * The panel is 320x416 and the Pokemon Mini is 96x64, so this runs at 3x integer
 * scale -- 288x192, centred -- which is the platform's maximum and fills 41.5
 * percent of the framebuffer. Integer only: 96 x 3.33 would drop and double
 * columns unevenly, and a pixel that is two wide here and three there reads as a
 * rendering bug rather than as scaling.
 */
#ifndef TWATCH_PM_H
#define TWATCH_PM_H

#include <stdbool.h>
#include <stdint.h>

#define PM_ROM_DEFAULT "/sd/pinball.min"

/* A retail BIOS dump, placed on the device by whoever owns one. It is not in
 * this repository and must not be: a retail cartridge needs the retail BIOS,
 * because the GPL free BIOS that ships in the core idles at $0040 and never
 * hands over to a commercial cartridge. */
#define PM_BIOS_DEFAULT "/sd/bios.rom"

/* 3x integer scale, centred in the framebuffer. ODD OFFSETS DO NOT COST HERE:
 * (320-288)/2 is 16 and (416-192)/2 is 112, both multiples of 4, so a row copy
 * stays aligned for the panel's 4-byte SPI framing. */
#define PM_SCALE 3
#define PM_OX    ((320 - 96 * PM_SCALE) / 2)
#define PM_OY    ((416 - 64 * PM_SCALE) / 2)

bool pmini_start(const char *rom_path);
void pmini_stop(void);
bool pmini_running(void);

/* Registers the `pm` shell command. */
bool pmini_register_command(void);

#endif /* TWATCH_PM_H */
