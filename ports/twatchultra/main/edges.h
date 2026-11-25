/*
 * edges.h - the glass around the framebuffer, and which button a point is on.
 *
 * The panel is 410x502 and the desktop draws into a 320x416 rectangle inset 45
 * px from each side, so there is a margin all the way round that no application
 * can reach. This port paints buttons into three of those margins -- a column
 * down the left, a strip along the bottom, and a scroll column on the right --
 * and the touch driver has to agree with the painter about where they are.
 *
 * That agreement is the whole reason this file exists. The geometry drifted
 * between two copies once already, in a way that could not be seen from either
 * file: the strip was divided by the glass width in one place and the desktop
 * width in another, which moved every button and left part of the glass
 * unpainted. One definition, used by both, makes that class of fault
 * impossible rather than merely unlikely.
 *
 * TWO COORDINATE SYSTEMS, AND THEY ARE NOT THE SAME. Getting this wrong cost a
 * build: the left column was dead over its outer half and the bottom strip was
 * shifted 22 px, while the code read as if both were correct.
 *
 * The framebuffer is a window onto the glass, inset by DISPLAY_ORIGIN_X/Y, and
 * the panel's controller RAM is a further PANEL_COL_OFFSET wider than the
 * visible glass -- 432 columns of RAM behind 410 columns of glass, with the
 * glass starting at RAM column 22. Two consequences, and they differ by axis:
 *
 *   CONTROLLER coordinates  what the painter writes. Everything defined below
 *                           is in these, because the painter is where a wrong
 *                           number is visible.
 *   PANEL coordinates       what the touch controller reports: 0..409 across
 *                           the visible glass and nothing else (TCH-006 says it
 *                           reports 410x502, and 410 is the glass, not the RAM).
 *
 * So panel x = controller x - PANEL_COL_OFFSET, and panel y = controller y,
 * because the RAM is wider than the glass and not taller. That is the whole
 * conversion and it lives in edges_hit(), at the one point where the two spaces
 * meet, so the painter never has to think about it and the hit test never sees
 * controller numbers by accident.
 */
#ifndef TWATCH_EDGES_H
#define TWATCH_EDGES_H

#include <stdbool.h>

#include "display.h"

/* The framebuffer, in controller coordinates: x 67..386, y 45..460. */
#define FB_CX0   (DISPLAY_ORIGIN_X + PANEL_COL_OFFSET)
#define FB_CX1   (FB_CX0 + DISPLAY_W - 1)
#define FB_CY0   DISPLAY_ORIGIN_Y
#define FB_CY1   (FB_CY0 + DISPLAY_H - 1)

/* The two columns either side of the framebuffer. They are the same height and
 * carry the same number of buttons, so one set of vertical bounds and one slot
 * formula serve both; only the x bounds differ, and which way each faces. */
#define COL_Y0   FB_CY0
#define COL_Y1   FB_CY1
#define COL_N    5

/* The left column: x 22..66. The desktop is to its right, so the corners that
 * face it -- and the corners cut at each separator -- are on the right edge. */
#define LEFT_X0   PANEL_COL_OFFSET
#define LEFT_X1   (FB_CX0 - 1)

/* The right column: x 387..431, mirrored. The desktop is to its left, so its
 * arcs are on the left edge. */
#define RIGHT_X0   (FB_CX1 + 1)
#define RIGHT_X1   (PANEL_COL_OFFSET + DISPLAY_PANEL_W - 1)

/* The bottom strip: x 67..386, y 461..501, three buttons side by side. */
#define BTN_X0   FB_CX0
#define BTN_X1   FB_CX1
#define BTN_Y0   (FB_CY1 + 1)
#define BTN_Y1   (DISPLAY_PANEL_H - 1)
#define BTN_N    3

/*
 * THE STRIP'S TOUCH AREA IS TWICE ITS DRAWN HEIGHT when an app owns the screen.
 *
 * The drawn strip is 41 rows and each button is 107 wide, so the boxes are wide
 * letters rather than squares and a thumb finds them fussy. Doubling the HEIGHT
 * makes each touch target about square (82 against 107) without touching the
 * artwork at all -- and it can only grow UPWARD, because BTN_Y1 is the last row of
 * the glass.
 *
 * Two geometry facts make that safe. The extended region reaches panel row 420,
 * which is framebuffer row 375, and the Tamagotchi's picture ends at framebuffer
 * row 361 -- so the larger target does not reach into the pet. It does however
 * lie INSIDE the framebuffer's own rows, which on the DESKTOP are the bottom two
 * lines of the console.
 *
 * SO IT IS GATED ON OWNERSHIP, and that gate is the point rather than a detail.
 * An app that claims the pixels loses nothing by a taller strip, because its own
 * content ends well above it. The desktop would lose the bottom of its console to
 * Tab, Enter and Ctrl+Q -- touching two lines of shell output would type into the
 * shell. `display_pixels_claimed()` is already the predicate the driver gates the
 * margins on (TCH-011), so this uses the same one rather than inventing a second
 * idea of who owns the glass.
 */
#define BTN_HIT_Y0   (BTN_Y0 - (BTN_Y1 - BTN_Y0 + 1))

/* The gap between adjacent buttons, and the corner radius, are the same in
 * both groups -- that is what makes the column and the strip read as one frame
 * drawn round the desktop rather than as two unrelated things. */
#define BTN_SEP     2
#define BTN_RADIUS 10

/* What each button is called on the glass.
 *
 * The label names the key the button sends, not the action it performs: "Tab"
 * and "Enter" are what a finger will read, and "Ctrl+Q" is honest about being
 * a key rather than pretending to be a Back button that behaves differently
 * from the key. Kept beside the geometry because a label and a duty that
 * disagree is the same class of fault as a button and a hit test that
 * disagree, and it is avoided the same way -- by keeping them together.
 *
 * A function rather than an array so that a translation unit which includes
 * this header for the geometry alone -- touch.c does -- does not carry an
 * unused table. */
static inline const char *btn_label(int i)
{
    static const char *const labels[BTN_N] = { "Tab", "Enter", "Ctrl+Q" };

    return i >= 0 && i < BTN_N ? labels[i] : "";
}

/* The right column's four arrow buttons. Top to bottom the column is a reserved
 * slot and then Left, Right, Up and Down, labelled with the arrows themselves:
 * U+2190, U+2192, U+2191 and U+2193.
 *
 * Arrows and not words, which is the reverse of the previous cycle's decision.
 * That decision rested on a guess -- that the desktop's 8x16 font might not
 * carry the arrow characters -- and the guess was wrong: all four are present,
 * and looking at them was cheaper than reasoning about them. A glyph also fits
 * where a word only just did: "Right" was 40 px in a 41 px face, one pixel of
 * headroom, and an arrow is 8. The reserved slot stays blank.
 *
 * The strings are UTF-8 because the font is indexed by code point; display.c
 * decodes them once at placement time. */
static inline const char *right_label(int i)
{
    static const char *const labels[COL_N] = {
        "",
        "\xe2\x86\x90",   /* U+2190 leftwards arrow */
        "\xe2\x86\x92",   /* U+2192 rightwards arrow */
        "\xe2\x86\x91",   /* U+2191 upwards arrow   */
        "\xe2\x86\x93",   /* U+2193 downwards arrow */
    };

    return i >= 0 && i < COL_N ? labels[i] : "";
}

/* The left column, top to bottom: the keys it sends, and one blank slot.
 *
 * The labels name the key, not the action, for the same reason the bottom
 * strip's do -- F10 is what a finger will read, and it is honest about being a
 * key. They are stood on end, which is the only way they fit: this column is
 * 41 px across inside its border and "Super+D" is 56 px wide.
 *
 * THE FIFTH SLOT IS BLANK AND INERT FROM NOW ON, and it used to send F11. It was
 * full screen for the focused window, which stopped meaning anything once every
 * app began opening maximised (`TD_OPEN_MAXIMIZED`, set in
 * `project_include.cmake`): a key whose only outcome is that the window grows to
 * the size it already is. The operator removed it rather than leave a word
 * advertising a no-op, which is the label/duty disagreement this file exists to
 * prevent.
 *
 * THE BOX STAYS, AND THAT IS THE SAME SHAPE AS THE RIGHT COLUMN'S RESERVED SLOT.
 * Slot geometry is shared by both columns and by the emulator screens -- 416 rows
 * over COL_N slots, one formula for the painter and the hit test -- so the
 * alternative, re-dividing this column into four taller buttons, would mean the
 * slot height became per-column. That is the sort of change that lets a drawn
 * button and its touch target drift apart, and `display.c`'s `s_col_n` already
 * records the house rule for a column that wants fewer buttons: it is a COUNT,
 * not a height, and the buttons keep the size every other screen gives them.
 *
 * The column has the room the bottom strip does not. Its buttons are 81 to 83
 * rows tall, so a seven-character label needs 56 rows of the 61 between the
 * corner radii, and there are five rows to spare.
 *
 * Turned clockwise, so the labels read top to bottom: the operator reads the
 * watch from the edge opposite the one the counter-clockwise version suited.
 * Each character is still upright with respect to that edge, and the two
 * directions are 180 degrees apart, so this is the same column read from the
 * other side rather than a different arrangement. */
static inline const char *left_label(int i)
{
    static const char *const labels[COL_N] = { "F10", "Esc", "F6", "Super+D", "" };

    return i >= 0 && i < COL_N ? labels[i] : "";
}

/*
 * WHAT THE MARGINS SAY WHILE AN APP OWNS THE SCREEN.
 *
 * The columns go blank and the strip shows a degree circle. This is not
 * decoration and it is not a second set of duties arriving quietly: it is the
 * glass telling the truth about what the buttons do, because while an app owns
 * the screen they do nothing at all (touch.c refuses them -- see
 * display_pixels_claimed() there). A button labelled "Enter" that sends no
 * Enter is exactly the label/duty disagreement this file was built to prevent,
 * so the label has to stop claiming a job it is not doing.
 *
 * The degree circle on the strip is the shape the buttons take WHEN THEY DRIVE
 * THE GUEST, which they have done since the bottom strip was wired: a bar press
 * reaches the pet's own three buttons positionally. It was written as a
 * placeholder -- "the next cycle's work" -- and the cycle happened, so the
 * circle is now the control rather than a promise of one. A drawn shape that
 * does nothing and a drawn shape that does the thing look identical on the
 * glass, which is why this note says which it is.
 *
 * U+00B0 is written as UTF-8 by hand because the label machinery decodes UTF-8
 * into code points and the font has the glyph at 0x00B0. It is one character
 * wide in the 41 px strip, so it fits without the rotation the columns need.
 */
/*
 * The labels an app's own controls carry, in the same index order as the
 * desktop's -- which is what lets display.c swap one table for the other without
 * knowing anything about either.
 *
 * THESE MUST MATCH tama_ui.c's action table. A label here with no action there is
 * a button that does nothing; an action there with no label here is a button
 * nobody can find. tests/test_tama_ui.c walks both and fails when they disagree,
 * because that is the only version of this rule that survives a later edit.
 *
 * The arm slot keeps the desktop's reserved position and gets a word for the
 * first time: it is held, not pressed, so `Hold` says what to do with it.
 */
static inline const char *left_label_app(int i)
{
    static const char *const labels[COL_N] = {
        "Save", "", "Load", "", "",
    };

    return i >= 0 && i < COL_N ? labels[i] : "";
}

/*
 * THE RIGHT COLUMN, TOP TO BOTTOM, IS THE OPERATOR'S ARRANGEMENT: Hold, a blank,
 * the sound level, the rate, and Pause at the bottom.
 *
 * THE ARM STAYS AT THE TOP because that is where a thumb rests without being
 * asked to, and the interlock is only usable if holding it is comfortable.
 * SLOT 2 IS DELIBERATELY EMPTY and is not an oversight: it separates the arm from
 * everything that acts, so a thumb settling on Hold cannot also be on a button,
 * and the column reads as two groups rather than five equal boxes.
 *
 * PAUSE AND SPEED CAME HERE FROM THE LEFT COLUMN. They are MOVES, not copies:
 * two slots driving one action would each want to show the state and could
 * disagree about it, which is the duplication this project has paid for before.
 * The left column keeps Save and Load, which are the pair a player uses
 * together.
 */
static inline const char *right_label_app(int i)
{
    static const char *const labels[COL_N] = {
        "Hold",                 /* the arm: held for a gated button */
        "",                     /* deliberately empty: see above        */
        "Sound",                /* shows the level; steps Mute..Max     */
        "Speed",                /* shows the rate; cycles 1x/2x/4x      */
        "Pause",                /* shows Resume while paused            */
    };

    return i >= 0 && i < COL_N ? labels[i] : "";
}

/* The bottom strip still carries the pet's three buttons, and its label is still
 * a placeholder: there is no short word for the V3's three buttons that fits
 * 41 px. U+00B0 is written as UTF-8 by hand because the label machinery decodes
 * UTF-8 into code points and the font has the glyph there. */
static inline const char *btn_label_app(int i)
{
    (void)i;
    return "\xc2\xb0";       /* U+00B0 DEGREE SIGN */
}

/* The top margin, above the framebuffer. Reserved for status icons, not a touch
 * target. */
#define TOP_Y1   (FB_CY0 - 1)

/* Where the buttons are.
 *
 * Nothing here is stored: both groups divide their run evenly and give the
 * spare pixels to the later slots, so the boundaries are a formula. Keeping
 * them a formula means the painter and the hit test cannot disagree. */

/* First row of button i in either column. 416 rows over five buttons is 83
 * each with a row left over, which the last one takes. */
static inline int col_slot_y(int i)
{
    return COL_Y0 + (i * (COL_Y1 - COL_Y0 + 1)) / COL_N;
}

/* First column of bottom bar i, on the same principle. */
static inline int btn_slot_x(int i)
{
    return BTN_X0 + (i * (BTN_X1 - BTN_X0 + 1)) / BTN_N;
}

/* Which button a row belongs to, in either column.
 *
 * The exact inverse of col_slot_y(), not an approximation of it: with H rows
 * over N buttons the boundaries are at floor(i*H/N), and the inverse of that is
 * floor(((y - Y0 + 1)*N - 1)/H). Dividing by H directly, the obvious thing, is
 * off by one against every boundary that does not divide evenly -- which put a
 * press on the wrong side of the gap at two of the four separators. A gap row
 * therefore counts as the button below it. */
static inline int col_slot_at(int y)
{
    int n = COL_N;
    int h = COL_Y1 - COL_Y0 + 1;
    int i = ((y - COL_Y0 + 1) * n - 1) / h;

    return i < 0 ? 0 : i >= n ? n - 1 : i;
}

/* The same for a column of the bottom strip. */
static inline int btn_slot_at(int x)
{
    int n = BTN_N;
    int w = BTN_X1 - BTN_X0 + 1;
    int i = ((x - BTN_X0 + 1) * n - 1) / w;

    return i < 0 ? 0 : i >= n ? n - 1 : i;
}

/* What a point on the glass is. */
typedef enum
{
    EDGE_NONE = 0,   /* the framebuffer, the top margin, or a black corner */
    EDGE_LEFT,
    EDGE_BOTTOM,
    EDGE_RIGHT,      /* the mirror of EDGE_LEFT: five buttons, arrows */
    EDGE_TOP,        /* classified so the status bar has a region to claim,
                      * but inert: it is not a touch target */
} edge_region_t;

typedef struct
{
    edge_region_t region;
    int index;       /* button within the group, top-to-bottom or left-to-right */
} edge_hit_t;

/* What a point on the glass is, given a point in PANEL coordinates -- the
 * numbers the touch controller reports, 0..409 by 0..501.
 *
 * The constants above are controller coordinates, so the x arrives 22 columns
 * to the left of where they were measured; adding PANEL_COL_OFFSET puts it
 * back. y needs nothing, because the RAM is wider than the glass and not
 * taller. A caller that passed controller coordinates instead would be off by
 * exactly that 22 columns, which reads as "the column is dead on its outer
 * half" rather than as an obvious error. */
static inline void edges_hit(int panel_x, int panel_y, bool tall_strip, edge_hit_t *hit)
{
    int x = panel_x + PANEL_COL_OFFSET;
    int y = panel_y;

    hit->region = EDGE_NONE;
    hit->index = -1;

    /*
     * THE STRIP IS TESTED FIRST, and it has to be. The columns and the strip
     * occupy the same rows once the strip is doubled -- its top reaches panel 420
     * while the columns run to 460 -- and the column test RETURNS for any x in the
     * framebuffer, which would hide the strip's upper half entirely. The two never
     * overlap in x, so testing the strip first is unambiguous rather than a
     * priority: BTN_X0..BTN_X1 is inside the framebuffer's width and the columns
     * sit outside it on both sides.
     *
     * `tall_strip` is the ownership gate described at BTN_HIT_Y0. A caller passes
     * true only when an app owns the glass.
     */
    if (x >= BTN_X0 && x <= BTN_X1 && y >= (tall_strip ? BTN_HIT_Y0 : BTN_Y0) &&
        y <= BTN_Y1)
    {
        hit->region = EDGE_BOTTOM;
        hit->index = btn_slot_at(x);
        return;
    }

    if (y >= COL_Y0 && y <= COL_Y1)
    {
        if (x >= LEFT_X0 && x <= LEFT_X1)
        {
            hit->region = EDGE_LEFT;
            hit->index = col_slot_at(y);
            return;
        }
        if (x >= RIGHT_X0 && x <= RIGHT_X1)
        {
            hit->region = EDGE_RIGHT;
            hit->index = col_slot_at(y);
            return;
        }
        return;      /* including the framebuffer, which is not an edge */
    }

    if (y <= TOP_Y1 && x >= LEFT_X0 && x <= RIGHT_X1)
        hit->region = EDGE_TOP;
}

#endif
