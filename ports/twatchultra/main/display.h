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

/*
 * Claim or release the framebuffer for an app that draws pixels.
 *
 * While claimed the desktop's paint task does not paint, so a blitted image
 * survives cursor blinks and taskbar ticks. On release the painter repaints the
 * whole screen, because by then the glass holds a picture rather than the cells
 * it was tracking.
 *
 * An app that owns the screen MUST claim before its first blit and release when
 * it exits or hands the screen back, or the two writers fight: the painter will
 * repaint cells over the image, which looks like the console bleeding through.
 */
void display_pixels_claim(bool on);
bool display_pixels_claimed(void);

/*
 * Paint the whole framebuffer one colour and push it.
 *
 * The other half of claiming the screen. A blit only writes the rectangle it is
 * given, so whatever the desktop painter left in the framebuffer outside it
 * stays -- and once ownership is claimed, nothing will ever overwrite it. On
 * the glass that is the app sitting on top of the console it was launched
 * from, with the shell's text still legible around the picture.
 *
 * The glass margins are untouched: the button bars and key columns live outside
 * the framebuffer in controller coordinates and are painted once at init, so
 * the watch's own controls keep working while an app owns the screen.
 *
 * Call ONLY after display_pixels_claim(true) -- the painter must not be using
 * s_tx when this flushes (DSP-001, DSP-005).
 */
bool display_fill(uint16_t colour);

/*
 * Compare the framebuffer against the cell grid, and the grid against what the
 * painter believes it has drawn.
 *
 * fb_bad counts cells whose pixels in the framebuffer differ from what the grid
 * says they should be -- a painting or flushing fault in display.c. shown_bad
 * counts cells where the painter's record disagrees with the grid -- a fault
 * outside this file, in whatever failed to write the cells back.
 *
 * Returns -1 in either count when there is no framebuffer.
 */
void display_check_cells(int *fb_bad, int *shown_bad, int *first_col, int *first_row);

/*
 * Repaint the entire screen from the cell grid, in bands.
 *
 * THE DISCRIMINATOR for leftover strips. dcheck() compares the framebuffer with
 * the model and cannot see a fault between the framebuffer and the glass, since
 * the panel has no readback. Run this on a broken screen:
 *
 *   strips gone   -> the flush decisions dropped rectangles; the bug is in
 *                    display.c's dirty-run logic.
 *   strips remain -> the transfer itself is wrong: window registers, chunking,
 *                    or the panel's memory wrap.
 */
bool display_repaint_all(void);

/* Frames the painter drew, frames it skipped while pixels were owned, and the
 * current ownership. Lets a shell command show that suppression is real. */
void display_paint_stats(uint32_t *painted, uint32_t *suppressed, bool *owned);

/* Draw a test pattern, for checking the blit path before trusting it.
 *
 * `what` is 0 for colour bars, 1 for a 1-pixel grid, 2 for a corner-and-edge
 * frame. Each answers a different question: bars catch a stride or byte-order
 * fault, the grid catches an off-by-one in the copy, and the frame catches
 * clipping at the panel's edges. Returns false when there is no framebuffer. */
bool display_test_pattern(int what);
bool display_picture_runs(void *user, int x, int y, const uint16_t *runs, size_t words, int w, int h);
void display_scanout_snapshot(uint32_t out[12]);
void display_scanout_info(uint32_t out[8]);

#endif
