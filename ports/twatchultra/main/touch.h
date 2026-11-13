/*
 * touch.h - the T-Watch Ultra's CST9217 touch panel as the desktop's mouse.
 */
#ifndef TWATCH_TOUCH_H
#define TWATCH_TOUCH_H

#include <stdbool.h>

#include "edges.h"

/* Called with mouse reports (SGR sequences, as a terminal sends them). */
typedef void (*touch_input_fn)(const char *data, int len);

/*
 * What the glass is being touched, reported as STATE rather than as events.
 *
 * THE CHANGE FROM EVENTS TO STATE IS WHAT MAKES A CHORD POSSIBLE. The controller
 * reports both points on every block while two fingers are down (measured --
 * see TCH-012), so "is the arm slot occupied at this instant" is a question
 * with an answer on the same report the action arrives on. An event-shaped
 * callback can only say "this button went down", which cannot distinguish "the
 * arm is held" from "the arm was pressed a moment ago".
 *
 * A frame is rebuilt from scratch on every report, so there is nothing stale to
 * clear: an absent finger is simply absent, and `n` says how many are present.
 * Edge detection belongs to the consumer, because a consumer that cares about
 * chords has to compare the whole set anyway.
 *
 * `p[0]` is the controller's first point and `p[1]` its second, which is
 * ARRIVAL ORDER and not a role -- the driver does not know which finger is the
 * arm. Consumers must treat the frame as a set.
 */
typedef struct
{
    edge_region_t region;   /* EDGE_NONE when this point is not present */
    int index;
} touch_point_t;

#define TOUCH_POINTS 2

typedef struct
{
    touch_point_t p[TOUCH_POINTS];
    int n;
} touch_frame_t;

/*
 * Called on every report while a consumer owns the framebuffer, and once with
 * n == 0 when everything has been released. Never called when nothing owns the
 * screen.
 */
typedef void (*touch_app_touch_fn)(void *user, const touch_frame_t *frame);

/* Start polling the controller; false when it does not answer. */
bool touch_init(touch_input_fn input);

/*
 * Offer margin presses to an application that owns the framebuffer.
 *
 * Set once, at init, rather than per app: the handler must already be in place
 * when a press arrives, and a registration that is torn down with the app has a
 * window where the driver holds a pointer to something that has freed itself.
 * A handler that returns false when it is not running is the same behaviour
 * with none of that risk.
 */
void touch_set_app_touch(touch_app_touch_fn fn, void *user);

/*
 * Capture raw report blocks for `seconds` and write them to /sd/touch.txt.
 *
 * A diagnostic, and it exists because multi-touch is UNMEASURED. The driver
 * decodes one finger from a 15-byte block (bytes 1..3) and has never looked at
 * what a second finger puts in the rest of it -- so anything that depends on
 * two simultaneous touches, like a hold-to-arm modifier, rests on a layout
 * nobody has seen. This dumps every block verbatim; the shape of the second
 * finger is then a reading rather than a guess from a datasheet.
 *
 * Returns the number of blocks captured, or -1 if the card could not be
 * written.
 */
int touch_dump(int seconds);

/* Registers the `touchdump` shell command. */
bool touch_register_command(void);

#endif
