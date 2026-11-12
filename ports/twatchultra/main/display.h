/*
 * display.h - the T-Watch Ultra's 410x502 AMOLED as a terminal.
 *
 * The panel is a CO5300 on a four-lane QSPI bus. ESP-IDF 5.3's esp_lcd panel
 * IO has no quad mode, so this port drives the bus with the SPI master driver
 * directly and the CO5300 command set by hand. That is the one part of this
 * port that is not modelled on ports/crowpanel7.
 *
 * The desktop's output goes into a td_vterm, and a task paints its cells onto
 * the panel: 8x16 pixels, 40 across and 26 down in the area the glass offers.
 *
 * WHY 320x400 AND NOT 410x502. The glass is 410x502 with 45-degree chamfers,
 * so the corners are not addressable as a rectangle. 320x400 is the largest
 * rectangle that fits inside the visible area, and it is what the Escapement
 * project measured and used for its own Linux framebuffer. The panel window
 * below is set to the same sub-rectangle rather than to the full glass so the
 * offsets match what the driver writes.
 *
 * This build adds one row below that rectangle, for 40x26: see the note on
 * DISPLAY_H below. DISPLAY_W is still 320.
 */
#ifndef TWATCH_DISPLAY_H
#define TWATCH_DISPLAY_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define DISPLAY_W         320
#define DISPLAY_H         416
#define DISPLAY_MAX_COLS  (DISPLAY_W / 8)    /* 40 */
#define DISPLAY_MAX_ROWS  (DISPLAY_H / 16)   /* 26 */
#define DISPLAY_MAX_CELLS (DISPLAY_MAX_COLS * DISPLAY_MAX_ROWS)

/* Where the framebuffer sits on the 410x502 glass. The panel is
 * addressed in glass coordinates, so anything that converts between the two
 * needs these: the display driver adds them when it writes a pixel, and the
 * touch driver subtracts them to turn a reported touch into a cell. Without
 * the subtraction every touch is 45 px off and scaled as though the desktop
 * were the whole glass.
 *
 * EXPERIMENT: 40 x 26, one row (16 px) taller than the 400 px that was
 * measured as the largest unclipped rectangle, added at the BOTTOM -- the top
 * stays at 45 and the framebuffer now runs to 461 instead of 445.
 *
 * The safe rectangle is 45 inset on every side, so its bottom edge is at
 * 502 - 45 = 457, and the new row's last 4 px are past it. That is the point
 * of the experiment: they may be inside the visible glass anyway, since the
 * chamfers cut the corners rather than the middle of the edges. Set
 * DISPLAY_H back to 400 to undo. */
#define DISPLAY_ORIGIN_X  45        /* (410 - 320) / 2 */
#define DISPLAY_ORIGIN_Y  45        /* top edge, unchanged */

/* The glass itself. The touch controller reports the panel's own resolution,
 * not the framebuffer's -- 410x502, measured (TCH-006) -- so this is what its
 * attribute register is checked against. */
#define DISPLAY_PANEL_W   410
#define DISPLAY_PANEL_H   502

/* The controller's RAM starts 22 columns to the LEFT of the visible glass, so a
 * column written to the panel needs this added (LCD-004: the first column of a
 * window is swallowed). Everything outside the framebuffer -- the button bars
 * and the margins -- works in these controller coordinates, which is also what
 * the touch controller reports. */
#define PANEL_COL_OFFSET  22

/* The cells the panel shows (after display_init()). */
int display_cols(void);
int display_rows(void);

/* Called with the terminal's answers (to the desktop's size and status
 * queries); they go back to the desktop as input. */
typedef void (*display_reply_fn)(const char *data, int len);

/* Panel, terminal and painting task; false on error. */
bool display_init(display_reply_fn reply);

/* What to call when the terminal receives a BEL.
 *
 * The renderer owns the vterm, so it is the only thing that can install this;
 * the port decides what a beep means. After display_init(), because the vterm
 * is initialised there and an earlier call would be overwritten. */
void display_set_bell(void (*fn)(void *user), void *user);

/* Feed terminal output. Bytes are parsed into the td_vterm and the painter is
 * woken; nothing is drawn on this call. */
void display_write(const uint8_t *buf, int len);

/* Sixel and pictures: not implemented, so the desktop is told it has no
 * picture support and falls back to text. The signatures must exist because
 * link.c registers them. */
bool display_picture(void *user, int x, int y, const uint8_t *pixels, int w, int h);
bool display_picture_rgb(void *user, int x, int y, const uint16_t *pixels, int w, int h);
bool display_picture_runs(void *user, int x, int y, const uint16_t *runs, size_t words, int w, int h);
void display_scanout_snapshot(uint32_t out[12]);
void display_scanout_info(uint32_t out[8]);

#endif
