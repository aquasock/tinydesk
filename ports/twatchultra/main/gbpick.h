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
 * WHAT A CONSOLE LOOKS LIKE TO THE PICKER, AND WHY THIS IS A STRUCT.
 *
 * The navigator is nine hundred lines of directory walking, paging, margin
 * handling and drawing, and NONE of it is about Game Boy. What IS about the
 * console is three things: which file extensions are a cartridge, which root
 * directories to offer, and what to call the thing in a title. So those three
 * live in a table and the navigator takes one.
 *
 * THE ALTERNATIVE WAS TRIED AND REJECTED BEFORE IT WAS WRITTEN. Copying the file
 * for Pokemon Mini would put two ~900-line copies of the same paging and touch
 * logic in the tree, and this port has already paid twice for duplicated drawing
 * code -- once when the picker's console list borrowed the directory list's
 * labels, and once when a control's label and its duty lived in two places. One
 * navigator with a config cannot drift from itself.
 */
#define PK_MAX_ROOTS 4          /* enough for a console list; PM needs one */

typedef struct
{
    const char *label;          /* what the console list row says            */
    const char *path;           /* the directory that holds its cartridges   */
} pk_root_t;

typedef struct
{
    const char *console;        /* used in titles: "Game Boy", "Pokemon Mini" */
    const char *const *exts;    /* NULL-terminated: { ".gbc", ".gb", NULL }   */
    const pk_root_t *roots;
    int n_roots;
} pk_cfg_t;

/* The two consoles this port emulates. */
extern const pk_cfg_t pk_gb;
extern const pk_cfg_t pk_pm;

/*
 * Scan, draw, and wait for a tap. Writes the chosen cartridge's full path into
 * `out` and returns true.
 *
 * `gbpick_run()` is the Game Boy front door and is exactly
 * `gbpick_run_cfg(&pk_gb, ...)`; it is kept because a dozen call sites and every
 * record in the log name it, and renaming them all would have obscured the
 * change rather than described it.
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
bool gbpick_run_cfg(const pk_cfg_t *cfg, char *out, size_t cap,
                    const volatile bool *abort);

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
 * `can_go_up` draws an "Up" bar on the bottom strip, right of Enter (the slot
 * that is Ctrl+Q on the desktop, where it closes a window -- meaningless in slot
 * B, which has neither). Where it is drawn, pressing it ends the list and writes
 * true to `up_out`, for a caller that has a parent directory to climb to. Where
 * there is no parent, pass false and the bar is drawn blank like the other
 * unused slots. `up_out` may be NULL only when `can_go_up` is false.
 *
 * `sel_init` is the row highlighted when the list appears, or -1 for none. It is
 * for a list a wearer comes back to -- the console chooser -- where the choice
 * they made last time is the best first guess, and an unhighlighted list makes
 * the first tap do nothing at all.
 *
 * EVERY MARGIN SLOT IS DRAWN, including the ones this list does not use: pulled
 * chrome left a frame with bites out of it and read as broken rather than
 * complete. A blank box is not a button -- presses are classified by the list
 * itself, and an unused slot neither acts nor buzzes.
 *
 * Returns true when a row was chosen and writes its index to `sel_out`; false
 * when `*abort` went true, when the Up bar was pressed, or when the list was
 * empty. Requires the same preconditions as gbpick_run().
 */
bool gbpick_list_run(const char *title, const char *const *labels, int n,
                     bool can_go_up, int sel_init, int *sel_out, bool *up_out,
                     const volatile bool *abort);

#endif /* TWATCH_GBPICK_H */
