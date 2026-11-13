/*
 * pmbench.h - the Pokemon Mini core benchmark, on the watch.
 *
 * `components/pokemini` is the emulator core, vendored from tinytama at a pinned
 * upstream revision. This file is the whole of what the board adds: a synthetic
 * cartridge, a measured loop, and one shell command. There is no display, no
 * input and no audio -- it is a measurement, not a port, and it exists so that
 * the decision to port can be made on this board rather than on arithmetic from
 * a host.
 *
 * It is expected to be removed once the platform is decided. If the answer is
 * yes, the port brings its own front end and this file's measurement loop is
 * replaced by a frame loop; if the answer is no, both this file and the
 * component go.
 */
#ifndef TWATCH_PMBENCH_H
#define TWATCH_PMBENCH_H

#include <stdbool.h>

/* Registers the `pmbench` shell command. */
bool pmbench_register_command(void);

#endif /* TWATCH_PMBENCH_H */
