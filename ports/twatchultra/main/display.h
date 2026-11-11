/*
 * display.h - the T-Watch Ultra's 410x502 AMOLED as a terminal.
 *
 * The panel is a CO5300 on a four-lane QSPI bus. ESP-IDF 5.3's esp_lcd panel
 * IO has no quad mode, so this port drives the bus with the SPI master driver
 * directly and the CO5300 command set by hand. That is the one part of this
 * port that is not modelled on ports/crowpanel7.
 *
 * The desktop's output goes into a td_vterm, and a task paints its cells onto
 * the panel: 8x16 pixels, 40x25 of them in the 320x400 area the glass offers.
 *
 * WHY 320x400 AND NOT 410x502. The glass is 410x502 with 45-degree chamfers,
 * so the corners are not addressable as a rectangle. 320x400 is the largest
 * rectangle that fits inside the visible area, and it is what the Escapement
 * project measured and used for its own Linux framebuffer. The panel window
 * below is set to the same 320x400 sub-rectangle rather than to the full glass
 * so the offsets match what the driver writes.
 */
#ifndef TWATCH_DISPLAY_H
#define TWATCH_DISPLAY_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define DISPLAY_W         320
#define DISPLAY_H         400
#define DISPLAY_MAX_COLS  (DISPLAY_W / 8)    /* 40 */
#define DISPLAY_MAX_ROWS  (DISPLAY_H / 16)   /* 25 */
#define DISPLAY_MAX_CELLS (DISPLAY_MAX_COLS * DISPLAY_MAX_ROWS)

/* The cells the panel shows (after display_init()). */
int display_cols(void);
int display_rows(void);

/* Called with the terminal's answers (to the desktop's size and status
 * queries); they go back to the desktop as input. */
typedef void (*display_reply_fn)(const char *data, int len);

/* Panel, terminal and painting task; false on error. */
bool display_init(display_reply_fn reply);

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
