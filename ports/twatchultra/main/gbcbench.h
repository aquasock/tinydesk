/*
 * gbcbench.h - the Game Boy Color core benchmark, on the watch.
 *
 * `components/gnuboy` is the emulator core, vendored verbatim from Escapement's
 * third_party copy of the retro-go fork. This file is the whole of what the board
 * adds: a frame loop, a modelled blit, and one shell command. No display, no
 * input, no audio -- it is a measurement, not a port.
 *
 * It exists to replace the 28-45 fps band that docs/gbc-scoping.md projected by
 * arithmetic with a number taken on this board, before anyone writes a Game Boy
 * Color front end on the strength of the projection.
 *
 * Expected to be removed once the platform is decided, exactly like pmbench.h: if
 * the answer is yes the port brings its own frame loop, and if the answer is no
 * this file and the component both go.
 */
#ifndef TWATCH_GBCBENCH_H
#define TWATCH_GBCBENCH_H

#include <stdbool.h>

/* Registers the `gbcbench` shell command. */
bool gbcbench_register_command(void);

#endif /* TWATCH_GBCBENCH_H */
