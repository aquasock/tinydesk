/*
 * gbpick.h - choose a Game Boy cartridge on the glass, before the emulator starts.
 *
 * Slot B boots straight into a cartridge with no shell behind it, so this is the
 * only place a choice can be made. It scans the card, draws what it finds, and
 * returns the tapped path.
 *
 * WHY TOUCH AND NOT THE D-PAD. The on-glass d-pad is gb.c's furniture: it is
 * painted from the emulator's own layout and only while a guest runs. Drawing it
 * here would mean a second control painter, which is the duplication this port
 * keeps paying for. The picker therefore draws rows and takes a tap on them, and
 * the only other input it needs -- "let me out" -- is Pusher A, which is armed
 * before the picker runs.
 */
#ifndef TWATCH_GBPICK_H
#define TWATCH_GBPICK_H

#include <stdbool.h>
#include <stddef.h>

/*
 * Scan, draw, and wait for a tap. Writes the chosen cartridge's full path into
 * `out` and returns true.
 *
 * Returns false when the card holds no cartridge, when `*abort` goes true (the
 * operator pressed Pusher A to leave), or when the display or touch slice it
 * needs is missing. `abort` may be NULL.
 *
 * Requires: display_init() done, pixels claimed, the SD card mounted, and
 * touch_init() running. It registers its own touch consumer and leaves it
 * registered; that consumer answers false once the picker has finished.
 */
bool gbpick_run(char *out, size_t cap, const volatile bool *abort);

/*
 * Put two lines of text on the glass, in red, for a failure with nothing to tap.
 * Slot B's only way to say what went wrong: no console, and a picture kills the
 * USB mirror.
 */
void gbpick_message(const char *line1, const char *line2);

/*
 * A list on the glass, driven by the margin buttons -- the mechanism behind both
 * the cartridge picker and slot B's app menu.
 *
 * Draws `title` and `labels`, moves the highlight on the right column's up and
 * down arrows, chooses on the bottom strip's Enter, and also accepts a tap on a
 * row (once to highlight, a second separate tap to choose).
 *
 * Returns true when a row was chosen and writes its index to `sel_out`; false
 * when `*abort` went true or the list was empty. Requires the same preconditions
 * as gbpick_run().
 */
bool gbpick_list_run(const char *title, const char *const *labels, int n,
                     int *sel_out, const volatile bool *abort);

#endif /* TWATCH_GBPICK_H */
