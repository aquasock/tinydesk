/*
 * gamepad.c - the on-glass d-pad and action buttons.
 *
 * The shapes were designed in pm.c over six cycles of the operator positioning
 * them by eye on the glass; the algorithms are lifted from there unchanged so the
 * two apps draw one object rather than two that resemble each other. See
 * gamepad.h for why the code lives in a module.
 *
 * Everything here is integer and fixed-point. These shapes are rasterised once at
 * an app's start, so the arithmetic cost is irrelevant, and an exactly
 * representable cosine keeps the rendering identical run to run -- a drift of one
 * pixel between boots would read as a different edge on the glass.
 */

#include "gamepad.h"

#include <stddef.h>

#include "display.h"
#include "font8x16.h"

/* The arrows and the letters are both a font glyph doubled to 16x32. Doubling is
 * the only scale that keeps an 8x16 glyph legible without inventing pixels: a
 * non-integer scale drops or doubles stems unevenly, and a stem is one pixel wide
 * at this size. */
#define GP_GLYPH_W 16
#define GP_GLYPH_H 32

/* --- shape space ----------------------------------------------------------
 *
 * `shape` is the d-pad's own frame before rotation: 0..size-1 on both axes, the
 * cross's arms being the OUTER thirds. A framebuffer point is mapped back into it
 * with the inverse rotation, and every test below runs in shape space. That is
 * why the arrows rotate with the arms for free -- they are sampled in shape space
 * too, so there is no rotated bitmap anywhere.
 */
static void gp_to_shape(const gamepad_t *g, int fx, int fy, int *sx, int *sy)
{
    const int dx = fx - g->cx;
    const int dy = fy - g->cy;

    /* Inverse of a clockwise rotation, which is why the signs read backwards: a
     * point in the screen's upper right maps to the TOP arm. At rot_k 65536 this
     * is the identity and the d-pad is upright. */
    const int u = ((dx + dy) * g->rot_k) >> 16;
    const int v = ((dy - dx) * g->rot_k) >> 16;

    *sx = u + g->size / 2;
    *sy = v + g->size / 2;
}

/* Is this point inside the sharp cross, before the tips are rounded? */
static bool gp_cross(const gamepad_t *g, int x, int y)
{
    const int arm = g->size / 3;

    if (x < 0 || y < 0 || x >= g->size || y >= g->size)
        return false;

    return (y >= arm && y < 2 * arm) || (x >= arm && x < 2 * arm);
}

typedef struct { int x, y, dx, dy; } gp_corner_t;

/*
 * A TIP IS ROUNDED BY CUTTING THE CORNER SQUARE, and the table is what makes a
 * wrong direction one entry rather than a wrong triangle in one corner. The
 * direction fields point INTO the material; the arc's centre sits r inside on
 * both axes and everything outside it in the r-by-r square is removed.
 *
 * The eight tips are the four arms' ends. The four INNER corners are deliberately
 * left sharp: rounding them was tried three ways and none read correctly on the
 * glass, and a d-pad's inner corners are conventionally clean right angles.
 */
static void gp_tip(const gamepad_t *g, int i, gp_corner_t *c)
{
    const int arm = g->size / 3;
    const int last = g->size - 1;

    switch (i)
    {
    case 0: *c = (gp_corner_t){ 0,    arm,        +1, +1 }; break;
    case 1: *c = (gp_corner_t){ 0,    2 * arm - 1, +1, -1 }; break;
    case 2: *c = (gp_corner_t){ last, arm,        -1, +1 }; break;
    case 3: *c = (gp_corner_t){ last, 2 * arm - 1, -1, -1 }; break;
    case 4: *c = (gp_corner_t){ arm,      0,      +1, +1 }; break;
    case 5: *c = (gp_corner_t){ 2 * arm - 1, 0,   -1, +1 }; break;
    case 6: *c = (gp_corner_t){ arm,      last,   +1, -1 }; break;
    default: *c = (gp_corner_t){ 2 * arm - 1, last, -1, -1 }; break;
    }
}

static bool gp_dpad_inside(const gamepad_t *g, int x, int y)
{
    bool in = gp_cross(g, x, y);

    if (!in || g->tip_r <= 0)
        return in;

    for (int i = 0; i < 8; i++)
    {
        gp_corner_t c;
        gp_tip(g, i, &c);

        const int lo_x = c.dx > 0 ? c.x : c.x - g->tip_r;
        const int hi_x = c.dx > 0 ? c.x + g->tip_r : c.x;
        const int lo_y = c.dy > 0 ? c.y : c.y - g->tip_r;
        const int hi_y = c.dy > 0 ? c.y + g->tip_r : c.y;

        if (x < lo_x || x > hi_x || y < lo_y || y > hi_y)
            continue;

        const int ox = c.x + c.dx * g->tip_r;
        const int oy = c.y + c.dy * g->tip_r;
        const int ex = x - ox, ey = y - oy;

        if (ex * ex + ey * ey > g->tip_r * g->tip_r)
            in = false;
    }

    return in;
}

/*
 * THE OUTLINE IS A CHEBYSHEV EROSION BY `line`, which is how display.c builds a
 * margin button's border: the shape minus a copy of itself inset by the border
 * width. A one-pixel test repeated twice is not the same thing -- it thickens
 * diagonals unevenly and leaves a notch at each rounded tip, which is exactly
 * where the eye goes.
 */
static bool gp_core(const gamepad_t *g, int x, int y)
{
    for (int dy = -g->line; dy <= g->line; dy++)
        for (int dx = -g->line; dx <= g->line; dx++)
            if (!gp_dpad_inside(g, x + dx, y + dy))
                return false;

    return true;
}

/*
 * The four arrows, taken from the same 8x16 font the desktop's right column
 * uses, so the watch's d-pad and its margin arrows read as one piece of
 * furniture rather than two.
 *
 * ARM CENTRES ARE A/2 AND 5A/2, not A/2 and 3A/2. The arms are the OUTER thirds
 * -- [0, A) and [2A, S) -- so 3A/2 would put the glyph in the middle square.
 * Getting that wrong once drew Down and Right on top of each other in the
 * centre while Up and Left landed correctly.
 */
static bool gp_arrow_ink(const gamepad_t *g, int x, int y)
{
    static const uint16_t cps[4] = { 0x2191, 0x2193, 0x2190, 0x2192 };

    const int arm = g->size / 3;
    const int near = arm / 2;
    const int far = 5 * arm / 2;
    const int centre = (g->size - GP_GLYPH_W) / 2;
    const int half_h = (g->size - GP_GLYPH_H) / 2;

    int gx[4], gy[4];

    gx[0] = centre;              gy[0] = near - GP_GLYPH_H / 2;   /* up    */
    gx[1] = centre;              gy[1] = far  - GP_GLYPH_H / 2;   /* down  */
    gx[2] = near - GP_GLYPH_W/2; gy[2] = half_h;                  /* left  */
    gx[3] = far  - GP_GLYPH_W/2; gy[3] = half_h;                  /* right */

    for (int i = 0; i < 4; i++)
    {
        const int px = x - gx[i], py = y - gy[i];

        if (px < 0 || py < 0 || px >= GP_GLYPH_W || py >= GP_GLYPH_H)
            continue;

        const uint8_t *rows = font8x16_glyph(cps[i]);
        if (!rows)
            continue;

        /* Every source pixel becomes a 2x2 block. */
        if (rows[py / 2] & (0x80 >> (px / 2)))
            return true;
    }

    return false;
}

/* --- the two round buttons ------------------------------------------------ */

/* Inside a circle, measured in half-pixels so the disc is symmetric: an even
 * diameter has no centre pixel and an integer radius test would put the bias on
 * one side. */
static bool gp_pad_inside(const gamepad_t *g, int i, int x, int y)
{
    const int dx2 = 2 * (x - g->pad_cx[i]);
    const int dy2 = 2 * (y - g->pad_cy[i]);
    const int rr = g->pad_d[i] - 1;

    return dx2 * dx2 + dy2 * dy2 <= rr * rr;
}

static bool gp_pad_label_ink(const gamepad_t *g, int i, int x, int y)
{
    const int px = (x - g->pad_cx[i]) + GP_GLYPH_W / 2;
    const int py = (y - g->pad_cy[i]) + GP_GLYPH_H / 2;

    if (px < 0 || py < 0 || px >= GP_GLYPH_W || py >= GP_GLYPH_H)
        return false;

    const uint8_t *rows = font8x16_glyph((uint16_t)g->pad_label[i]);
    if (!rows)
        return false;

    return (rows[py / 2] & (0x80 >> (px / 2))) != 0;
}

/* --- painting ------------------------------------------------------------- */

typedef struct
{
    const gamepad_t *g;
    int x0, y0;          /* the region's top-left in framebuffer coordinates */
    uint16_t face, edge, ink;
} gp_ctx_t;

/*
 * THE CONTEXT IS FILE-SCOPE, NOT A CALLBACK PARAMETER, because
 * `display_panel_rows()` has no user pointer to hand one through -- its signature
 * takes the rectangle and the function and nothing else. pm.c's painter solves it
 * the same way with s_row_x0 and s_row_y0. Painting is sequential on the app's own
 * task and each call rewrites the whole struct before pushing, so there is nothing
 * for a second writer to interleave with; a concurrent paint would already be
 * illegal under DSP-005.
 */
static gp_ctx_t s_ctx;

static void gp_row(void *user, int row, int col, uint16_t *dst, int n)
{
    (void)user;

    const gp_ctx_t *c = &s_ctx;
    const gamepad_t *g = c->g;

    for (int i = 0; i < n; i++)
    {
        /* `col` and `row` are region-local; the shapes are tested in framebuffer
         * coordinates, which is what the two bands' different origins are for. */
        const int fx = (col + i) + c->x0;
        const int fy = row + c->y0;

        uint16_t px = g->bg;

        if (g->grid > 0 && (fx % g->grid == 0 || fy % g->grid == 0))
            px = g->grid_line;

        int sx, sy;
        gp_to_shape(g, fx, fy, &sx, &sy);

        if (gp_dpad_inside(g, sx, sy))
        {
            px = gp_core(g, sx, sy) ? c->face : c->edge;

            if (px == c->face && gp_arrow_ink(g, sx, sy))
                px = c->ink;
        }
        else
        {
            /* Only tested where the d-pad is absent: the two never overlap, and
             * skipping the test leaves the grid where neither shape is. */
            for (int b = 0; b < 2; b++)
            {
                if (!gp_pad_inside(g, b, fx, fy))
                    continue;

                bool core = true;

                for (int ey = -g->line; ey <= g->line && core; ey++)
                    for (int ex = -g->line; ex <= g->line; ex++)
                        if (!gp_pad_inside(g, b, fx + ex, fy + ey))
                        {
                            core = false;
                            break;
                        }

                px = core ? c->face : c->edge;

                if (px == c->face && gp_pad_label_ink(g, b, fx, fy))
                    px = c->ink;

                break;
            }
        }

        dst[i] = px;
    }
}

bool gamepad_paint(const gamepad_t *g, int fb_x0, int fb_y0, int w, int h,
                   uint16_t face, uint16_t edge, uint16_t ink)
{
    if (!g || w <= 0 || h <= 0)
        return false;

    gp_ctx_t c = { g, fb_x0, fb_y0, face, edge, ink };

    s_ctx = c;

    /* Panel coordinates, because the rectangle may extend past the framebuffer
     * into the glass below it. Both offsets are named rather than combined into
     * a magic constant: the framebuffer is inset by DISPLAY_ORIGIN and the
     * controller's RAM is a further PANEL_COL_OFFSET wider than the glass. */
    return display_panel_rows(fb_x0 + DISPLAY_ORIGIN_X + PANEL_COL_OFFSET,
                              fb_y0 + DISPLAY_ORIGIN_Y, w, h, gp_row);
}

/* --- hit testing ---------------------------------------------------------- */

int gamepad_hit(const gamepad_t *g, int fb_x, int fb_y)
{
    if (!g)
        return GP_NONE;

    int sx, sy;
    gp_to_shape(g, fb_x, fb_y, &sx, &sy);

    if (gp_dpad_inside(g, sx, sy))
    {
        const int arm = g->size / 3;

        if (sy < arm)
            return GP_UP;
        if (sy >= 2 * arm)
            return GP_DOWN;
        if (sx < arm)
            return GP_LEFT;
        if (sx >= 2 * arm)
            return GP_RIGHT;

        return GP_NONE;      /* the centre square belongs to no direction */
    }

    for (int b = 0; b < 2; b++)
        if (gp_pad_inside(g, b, fb_x, fb_y))
            return b == 0 ? GP_A : GP_B;

    return GP_NONE;
}
