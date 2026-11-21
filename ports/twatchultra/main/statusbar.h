/*
 * statusbar.h - the always-visible strip above the desktop's taskbar.
 *
 * See statusbar.c for why it exists and what it costs. The interface is one call
 * because there is nothing to configure: the content, the geometry and the rate
 * are all decisions made in the source, and a knob here would only invite a
 * caller to disagree with the arithmetic.
 */
#ifndef TWATCH_STATUSBAR_H
#define TWATCH_STATUSBAR_H

#include <stdbool.h>
#include <stdint.h>

/* Start the 1 Hz updater. False when the task could not be created, which is not
 * fatal -- the watch works without a clock in the bezel. */
bool statusbar_start(void);

/*
 * What the strip has done to the panel: how many pushes, how long the last one
 * took, and how long ago it was. `since_ms` is -1 before the first push.
 *
 * Exposed because the strip and the emulator's blitter share the panel bus, and
 * a blitter that stalls is otherwise a suspect with no alibi. */
void statusbar_push_stats(uint32_t *pushes, uint32_t *last_ms, int64_t *since_ms);

#endif /* TWATCH_STATUSBAR_H */
