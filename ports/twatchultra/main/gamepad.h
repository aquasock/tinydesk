/*
 * gamepad.h - an on-glass d-pad and two action buttons, for an app that owns
 *             the framebuffer.
 *
 * WHY THIS IS A MODULE AND NOT A COPY.
 *
 * The Pokemon Mini drew these shapes first, in pm.c, over six cycles of the
 * operator positioning them by eye on the glass. The NES wants the same
 * furniture below its picture. Copying the rasteriser a second time would be the
 * fault this port has already paid for twice -- `scripts/tamacore-sync.sh` exists
 * only because one vendored copy had to be policed against another, and the
 * margin labels have a whole override mechanism because two apps must not edit
 * one shared table.
 *
 * So the shapes live here, parameterised by a layout, and both apps describe
 * where their controls go rather than how to draw one.
 *
 * WHAT IS SHARED AND WHAT IS NOT. The shapes, the outline rule, the arrow and
 * label sampling, and the row painter are shared. Where the controls sit, how
 * big they are, which region of the panel they may touch, and what colours they
 * take are each app's own decision, expressed in `gamepad_t` and the arguments
 * to gamepad_paint(). PM's numbers are unchanged and still its own.
 *
 * The geometry here is the same geometry a hit test needs, which is the point of
 * factoring it out: a painter and a hit test that disagree produce a button that
 * looks pressed and is not, and the only reliable cure is one implementation.
 */
#ifndef TWATCH_GAMEPAD_H
#define TWATCH_GAMEPAD_H

#include <stdbool.h>
#include <stdint.h>

/*
 * A layout, in FRAMEBUFFER coordinates -- the 320x416 window, not the 410x502
 * panel. The paint call converts; nothing else here knows about the panel.
 *
 * `rot_k` is the d-pad's rotation as 16.16 fixed-point cosine: 65536 is upright,
 * 46341 is cos(45). The arms and the arrows rotate together, because the
 * transform is applied to the incoming point rather than to a rotated bitmap.
 */
typedef struct
{
    int cx, cy;          /* d-pad centre */
    int size;            /* d-pad side length; arms are a third of it */
    int rot_k;           /* 16.16 cos of the rotation */
    int line;            /* outline thickness, the margin buttons' own weight */
    int tip_r;           /* arm-tip corner radius, 0 for sharp */

    int pad_cx[2];       /* circle centres, A then B */
    int pad_cy[2];
    int pad_d[2];        /* circle diameters */
    char pad_label[2];   /* the letter drawn in each circle */

    uint16_t bg;         /* background fill, already byte-swapped */
    int grid;            /* grid pitch in px, 0 for none */
    uint16_t grid_line;  /* grid colour, already byte-swapped */
} gamepad_t;

/*
 * Paint the cluster into one rectangle.
 *
 * `fb_x0`, `fb_y0` are the rectangle's top-left in FRAMEBUFFER coordinates and
 * may be negative when the rectangle spans the whole glass rather than just the
 * framebuffer -- which is what an app needs below its side columns, where the
 * buttons stop and the glass does not.
 *
 * `face`, `edge` and `ink` must already be in panel byte order; use
 * display_swap16() to convert. Callers usually read them from
 * display_margin_colours() so the controls cannot drift from the twelve buttons
 * they sit beside.
 *
 * Drawing goes through display_panel_rows(), so it clips at the glass rather than
 * at the framebuffer and refuses unless the pixels are claimed.
 *
 * Returns false when the pixels are not claimed or the push was refused.
 */
bool gamepad_paint(const gamepad_t *g, int fb_x0, int fb_y0, int w, int h,
                   uint16_t face, uint16_t edge, uint16_t ink);

/*
 * The shape tests, exposed for hit testing.
 *
 * A touch point arrives in panel coordinates, so a caller converts to
 * framebuffer coordinates first (subtract DISPLAY_ORIGIN_X/Y) and then asks
 * these. They are the same functions the painter calls, so a hit test written
 * against them cannot disagree with what is drawn.
 *
 * Returns 0 when the point is on neither shape, 1..4 for up/down/left/right on
 * the d-pad, and 10 or 11 for A and B.
 */
#define GP_NONE   0
#define GP_UP     1
#define GP_DOWN   2
#define GP_LEFT   3
#define GP_RIGHT  4
#define GP_A      10
#define GP_B      11

int gamepad_hit(const gamepad_t *g, int fb_x, int fb_y);

#endif
