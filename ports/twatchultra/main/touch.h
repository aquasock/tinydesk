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
 * Called when a margin button is pressed or released while something else owns
 * the framebuffer -- the case that used to be "refused, and logged".
 *
 * `region` and `index` are edges.h's: EDGE_BOTTOM 0..2 is the strip, EDGE_LEFT
 * and EDGE_RIGHT 0..4 are the columns. `down` is true on the press and false on
 * the release, and this part has no release event, so the release is inferred
 * from the reports going quiet -- the same rule the desktop's margins follow.
 *
 * Return true to accept the press. False means "this is not mine", and the
 * driver logs it as inert, which is the honest outcome for a button nothing is
 * listening to.
 */
typedef bool (*touch_app_button_fn)(void *user, edge_region_t region, int index, bool down);

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
void touch_set_app_buttons(touch_app_button_fn fn, void *user);

#endif
