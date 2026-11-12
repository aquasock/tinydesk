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

/* The left column: x 22..66, y 45..460, five buttons stacked. */
#define LEFT_X0   PANEL_COL_OFFSET
#define LEFT_X1   (FB_CX0 - 1)
#define LEFT_Y0   FB_CY0
#define LEFT_Y1   FB_CY1
#define LEFT_N    5

/* The right column, the mirror of the left one: x 387..431. One control, a
 * scroll column, so it has no slot arithmetic. */
#define RIGHT_X0   (FB_CX1 + 1)
#define RIGHT_X1   (PANEL_COL_OFFSET + DISPLAY_PANEL_W - 1)

/* The bottom strip: x 67..386, y 461..501, three buttons side by side. */
#define BTN_X0   FB_CX0
#define BTN_X1   FB_CX1
#define BTN_Y0   (FB_CY1 + 1)
#define BTN_Y1   (DISPLAY_PANEL_H - 1)
#define BTN_N    3

/* The gap between adjacent buttons, and the corner radius, are the same in
 * both groups -- that is what makes the column and the strip read as one frame
 * drawn round the desktop rather than as two unrelated things. */
#define BTN_SEP     2
#define BTN_RADIUS 10

/* The top margin, above the framebuffer. Reserved for status icons, not a touch
 * target. */
#define TOP_Y1   (FB_CY0 - 1)

/* Where the buttons are.
 *
 * Nothing here is stored: both groups divide their run evenly and give the
 * spare pixels to the later slots, so the boundaries are a formula. Keeping
 * them a formula means the painter and the hit test cannot disagree. */

/* First row of left button i. 416 rows over five buttons is 83 each with a row
 * left over, which the last one takes. */
static inline int left_slot_y(int i)
{
    return LEFT_Y0 + (i * (LEFT_Y1 - LEFT_Y0 + 1)) / LEFT_N;
}

/* First column of bottom bar i, on the same principle. */
static inline int btn_slot_x(int i)
{
    return BTN_X0 + (i * (BTN_X1 - BTN_X0 + 1)) / BTN_N;
}

/* Which button a row of the left column belongs to.
 *
 * The exact inverse of left_slot_y(), not an approximation of it: with H rows
 * over N buttons the boundaries are at floor(i*H/N), and the inverse of that is
 * floor(((y - Y0 + 1)*N - 1)/H). Dividing by H directly, the obvious thing, is
 * off by one against every boundary that does not divide evenly -- which put a
 * press on the wrong side of the gap at two of the four separators. A gap row
 * therefore counts as the button below it. */
static inline int left_slot_at(int y)
{
    int n = LEFT_N;
    int h = LEFT_Y1 - LEFT_Y0 + 1;
    int i = ((y - LEFT_Y0 + 1) * n - 1) / h;

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
    EDGE_RIGHT,
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
static inline void edges_hit(int panel_x, int panel_y, edge_hit_t *hit)
{
    int x = panel_x + PANEL_COL_OFFSET;
    int y = panel_y;

    hit->region = EDGE_NONE;
    hit->index = -1;

    if (y >= LEFT_Y0 && y <= LEFT_Y1)
    {
        if (x >= LEFT_X0 && x <= LEFT_X1)
        {
            hit->region = EDGE_LEFT;
            hit->index = left_slot_at(y);
            return;
        }
        if (x >= RIGHT_X0 && x <= RIGHT_X1)
        {
            hit->region = EDGE_RIGHT;
            return;
        }
        return;      /* including the framebuffer, which is not an edge */
    }

    if (y >= BTN_Y0 && y <= BTN_Y1 && x >= BTN_X0 && x <= BTN_X1)
    {
        hit->region = EDGE_BOTTOM;
        hit->index = btn_slot_at(x);
        return;
    }

    if (y <= TOP_Y1 && x >= LEFT_X0 && x <= RIGHT_X1)
        hit->region = EDGE_TOP;
}

#endif
