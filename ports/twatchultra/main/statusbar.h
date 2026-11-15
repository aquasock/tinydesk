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

/* Start the 1 Hz updater. False when the task could not be created, which is not
 * fatal -- the watch works without a clock in the bezel. */
bool statusbar_start(void);

#endif /* TWATCH_STATUSBAR_H */
