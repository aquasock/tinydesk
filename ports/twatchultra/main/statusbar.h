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
#include <stddef.h>

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

/*
 * Hand the strip to an app, or take it back.
 *
 * WHILE AN APP HOLDS IT THE STRIP SHOWS THAT APP'S TEXT AND NOTHING ELSE -- no
 * date, no clock -- because the operator's direction for the one app that uses
 * this is that the strip should stop reading as the watch's own time and start
 * reading as the pet's. The field is single-slot by construction: there is one
 * strip, so there is one claim, and a second caller replaces the first rather
 * than sharing it.
 *
 * `text` NULL or empty releases the strip, and the date and clock return on the
 * next tick. Callable from any task: the text is copied under a lock, and the
 * strip's own task is what paints. It does NOT paint synchronously, so a caller
 * on a task that must not touch the panel -- an emulator's guest task -- can use
 * it. Returns false when the claim could not be taken (the text is longer than
 * a field), in which case the strip keeps what it had.
 */
bool statusbar_app_text(const char *text);

/* What the strip currently holds as an app claim, for the `statusbar` command.
 * False when no app holds it. */
bool statusbar_app_text_get(char *out, size_t cap);

#endif /* TWATCH_STATUSBAR_H */
