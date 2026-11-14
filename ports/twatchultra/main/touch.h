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

    /*
     * WHERE ON THE GLASS, in PANEL coordinates: 0..409 by 0..501, the numbers
     * the touch controller itself reports (TCH-006).
     *
     * These exist because a region and an index describe the MARGINS and nothing
     * else. `edges_hit()` classifies the thirteen margin buttons and returns
     * `EDGE_NONE` with index -1 for anything inside the framebuffer, so an app
     * that draws its own controls -- the Pokemon Mini's d-pad, the NES's -- could
     * not see a press on them at all. The driver was decoding these coordinates
     * all along and then discarding them.
     *
     * PANEL, NOT FRAMEBUFFER, and the distinction has cost this project twice. The
     * touch controller reports panel coordinates; the framebuffer is that window
     * after subtracting `DISPLAY_ORIGIN_X` and `DISPLAY_ORIGIN_Y`, which are both
     * 45. A consumer that forgets is off by 45 px on both axes, which on the d-pad
     * reads as "the controls are dead on their left half" rather than as an
     * obvious error. The conversion is the consumer's job because only the consumer
     * knows whether it drew in framebuffer space or in panel space.
     *
     * Valid only while `region` is not EDGE_NONE -- though for a framebuffer point
     * region IS EDGE_NONE and the coordinates are still what the consumer wants, so
     * the presence test is `index >= 0` for a margin hit and `n` for a point. */
    int x;
    int y;
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
 *
 * RETURN TRUE IF A CONTROL FIRED. The return exists because the driver cannot
 * answer that question itself and was guessing, which cost a real fault: while an
 * app owned the screen the driver buzzed on EVERY press edge, so a finger on blank
 * glass -- or on a margin slot this app had hidden -- produced the same haptic tap
 * as a real button. The operator reported it as phantom buttons and it made the
 * drawn controls impossible to test, because every touch felt like a hit.
 *
 * Only the app knows which pixels are controls: it drew them, and it may have
 * hidden margin slots the driver still reports. So consumption is the app's
 * answer, and the buzz follows it. A consumer that does not care can return false
 * and get no haptics, which is the honest default -- silence rather than a tap
 * that means nothing.
 */
typedef bool (*touch_app_touch_fn)(void *user, const touch_frame_t *frame);

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
 * ADD a consumer rather than replacing the one before it.
 *
 * There are now two apps that draw pixels -- the Tamagotchi and the NES -- and
 * each needs the margins while it owns the screen. Only one can own it at a time,
 * so both receivers are called and each decides for itself whether the frame is
 * meant for it. That is why the two "set" functions above still exist (a single
 * consumer is the simpler contract) and why this one is separate: replacing on
 * registration meant the app that registered LAST silently disabled the other,
 * which is a fault that would only appear once somebody launched the older app
 * second.
 *
 * Returns false when the table is full. The limit is small and deliberate: a
 * handful of screen-owning apps, not a general subscription mechanism.
 */
bool touch_add_app_touch(touch_app_touch_fn fn, void *user);

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
