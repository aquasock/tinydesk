/*
 * render.h - draw the LCD segment map into an RGB565 buffer.
 *
 * Two modes, and the difference matters:
 *
 *   lcd_draw_scaled()   every LCD dot is a DOT x DOT block with no gap. This is
 *                       the true hardware look, and at DOT=4 the pet fills a
 *                       quarter of the screen. It is what "4x, no spacing
 *                       pixels" means.
 *
 *   lcd_draw_dots()     every LCD dot is a DOT x DOT block separated by GAP
 *                       pixels, so the dot matrix is visible as a grid. This is
 *                       the harness's original look: readable across a room
 *                       because the eye is given structure to latch onto, at the
 *                       cost of a larger image for the same information.
 *
 * Both take the dot grid size from the segment map (SEG_W x SEG_H dots), so
 * there is one artwork and one map. Scaling in the renderer rather than
 * rebuilding the map is not just cheaper -- a 4x map was 126 KB of const data
 * and overflowed the 768 KB firmware partition, while the map itself is 32 KB
 * whatever the on-screen size.
 *
 * Do not go back to bounding boxes. The artwork uses H/V path commands and
 * cubic curves, so a naive box over the raw coordinates produces garbage --
 * that mistake showed up as a grid of lines across the display.
 */

#ifndef RENDER_H
#define RENDER_H

#include <stdint.h>
#include "segmap.h"

#define LCD_INK   0x0000
#define LCD_OFF   0xC618

typedef struct {
    uint16_t *px;
    int w, h;
} lcd_fb;

static inline void lcd_put(const lcd_fb *fb, int x, int y, uint16_t c)
{
    if (x >= 0 && x < fb->w && y >= 0 && y < fb->h)
        fb->px[y * fb->w + x] = c;
}

static inline void lcd_rect(const lcd_fb *fb, int x0, int y0, int x1, int y1, uint16_t c)
{
    int y;

    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > fb->w) x1 = fb->w;
    if (y1 > fb->h) y1 = fb->h;
    if (x0 >= x1 || y0 >= y1)
        return;

    for (y = y0; y < y1; y++) {
        uint16_t *row = fb->px + (size_t)y * fb->w;

        for (int x = x0; x < x1; x++)
            row[x] = c;
    }
}

/* Pixels across the dot grid: DOT pixels per dot, GAP between neighbours. */
/*
 * The renderer and the shared-memory contract carry the geometry separately
 * -- shmproto.h cannot include segmap.h, and segmap.h is generated -- so the
 * two are checked here rather than trusted. A mismatch does not fail loudly at
 * runtime: the frame is the size shmproto.h says and the renderer would draw
 * past it or leave a margin, both silently.
 */
/*
 * THE GUARD WAS INVERTED AND THE CHECK HAD NEVER RUN.
 *
 * It read `#ifndef SHMPROTO_H`, which asserts only when shmproto.h has NOT been
 * included -- exactly the case where SHM_DOT and friends are undefined, so the
 * block could not compile and could not check anything. In tamacore_task.c,
 * which includes shmproto.h before this file, the guard was false and the whole
 * cross-check was skipped.
 *
 * Found by compiling this file on a host for the Tamacore port: without
 * shmproto.h the assert fires on undefined macros, and with it nothing runs.
 * The condition is now `#ifdef`, so the comparison happens when the macros
 * exist, which is the only time it means anything. If segmap.h and shmproto.h
 * ever disagree the FIRMWARE build now fails, which is what the comment above
 * always claimed it would do.
 */
#ifdef SHMPROTO_H
_Static_assert(SEG_DOT == SHM_DOT, "segmap.h and shmproto.h disagree on SHM_DOT");
_Static_assert(SEG_GAP == SHM_GAP, "segmap.h and shmproto.h disagree on SHM_GAP");
_Static_assert(SEG_LCD_W == SHM_LCD_W, "segmap.h and shmproto.h disagree on LCD width");
_Static_assert(SEG_LCD_H == SHM_LCD_H, "segmap.h and shmproto.h disagree on LCD height");
#endif

/*
 * Draw the LCD with its top-left corner at (ox, oy), each LCD dot as a DOT x DOT
 * block with GAP pixels between dots.
 *
 * The gap is applied between dots and not around the outside, so an image of W
 * dots is W*dot + (W-1)*gap wide -- the grid ends with a dot, not with a margin.
 * That is what makes the block read as a dot matrix rather than as a waffle.
 */
/*
 * Draw rows [yl, yh) of the LCD only, in framebuffer coordinates. Used by the
 * blitter path, which repaints just the rows whose pixels changed.
 *
 * The background is repainted for those rows first, so a segment that turned
 * OFF is erased -- which is the half that a "draw only what is lit" renderer
 * gets wrong.
 */
/*
 * Draw rows [yl, yh) of the LCD, in framebuffer coordinates.
 *
 * Two passes, because the two things are not the same kind of object:
 *
 *   the matrix   32x30 dots, one bit per dot in the pet's display RAM, each
 *                drawn as a DOT x DOT block on a (DOT+GAP) pitch. It is a dot
 *                matrix and that is how the hardware makes it.
 *
 *   the icons    10 two-state glyphs, stored as full-resolution horizontal
 *                runs in LCD pixels. They are drawn as glyphs, not as dot
 *                blocks: rasterising them onto the dot grid is what made them
 *                look chunky, and at native resolution they are exactly the
 *                shapes the artwork has.
 *
 * This pass OWNS every pixel it covers, including the background between dots
 * and the holes inside a glyph. Drawing only the lit parts leaves the previous
 * frame showing through wherever something turned off, which is the bug an
 * erase-on-OFF pass exists to prevent.
 *
 * A dot cell whose packed value is 0xFF00 is padding -- the grid is 32x30 and
 * fully populated on this artwork, but the sentinel keeps a future incomplete
 * map from reading a random dpram byte.
 */
static inline void lcd_draw_dots_rows(const lcd_fb *fb, const uint8_t *dpram,
                                      int ox, int oy, int dot, int gap,
                                      int yl, int yh)
{
    const int pitch = dot + gap;
    int r, c, i;

    if (yl < oy)
        yl = oy;
    if (yh > oy + SEG_LCD_H)
        yh = oy + SEG_LCD_H;
    if (yl >= yh)
        return;

    /* Background first, for the whole requested range. */
    lcd_rect(fb, ox, yl, ox + SEG_LCD_W, yh, LCD_OFF);

    /* ---- the 32x30 matrix ---- */
    for (r = 0; r < SEG_MATRIX_H; r++) {
        int y0 = oy + SEG_MATRIX_OY + r * pitch;
        int y1 = y0 + dot;

        if (y1 <= yl || y0 >= yh)
            continue;

        for (c = 0; c < SEG_MATRIX_W; c++) {
            uint16_t v = seg_matrix[r * SEG_MATRIX_W + c];
            uint8_t b = (uint8_t)(v >> 8);
            uint8_t n = (uint8_t)(v & 0xFF);
            int x0, yy;

            if (b == 0xFF)
                continue;               /* padding cell */
            if (!((dpram[b] >> n) & 1))
                continue;               /* unlit: background already there */

            x0 = ox + SEG_MATRIX_OX + c * pitch;
            for (yy = y0; yy < y1; yy++) {
                if (yy < yl || yy >= yh)
                    continue;
                lcd_rect(fb, x0, yy, x0 + dot, yy + 1, LCD_INK);
            }
        }
    }

    /* ---- the 10 status icons, as glyphs ---- */
    for (i = 0; i < SEG_ICON_COUNT; i++) {
        const seg_icon *ic = &seg_icons[i];
        const seg_run *u;
        int k;

        if (!((dpram[ic->byte] >> ic->bit) & 1))
            continue;                   /* unlit: background already there */

        u = &seg_icon_runs[ic->run_start];
        for (k = 0; k < ic->run_count; k++, u++) {
            int yy = oy + u->y;

            if (yy < yl || yy >= yh)
                continue;
            lcd_rect(fb, ox + u->x0, yy, ox + u->x1, yy + 1, LCD_INK);
        }
    }
}

static inline void lcd_draw_dots(const lcd_fb *fb, const uint8_t *dpram,
                                 int ox, int oy, int dot, int gap)
{
    lcd_draw_dots_rows(fb, dpram, ox, oy, dot, gap, oy, oy + SEG_LCD_H);
}

/* True-hardware look: dot blocks touching, no grid. */
static inline void lcd_draw_scaled(const lcd_fb *fb, const uint8_t *dpram,
                                   int ox, int oy, int scale)
{
    lcd_draw_dots(fb, dpram, ox, oy, scale, 0);
}

#endif /* RENDER_H */
