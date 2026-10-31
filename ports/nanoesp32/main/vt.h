/*
 * vt.h - a small terminal emulator that turns TinyDesk's ANSI stream into a
 * cell grid, and draws that grid on the ILI9340.
 *
 * TinyDesk speaks to a terminal: it moves the cursor, sets colours, clears
 * regions and writes text. This layer does the same job the PC terminal was
 * doing, so the desktop core needs no change at all.
 */
#ifndef VT_H
#define VT_H

#include <stdbool.h>
#include <stdint.h>

/* Grid limits. The panel is 320x240 at 8x16, which is 40x15. */
#define VT_MAX_COLS 80
#define VT_MAX_ROWS 30

/* Set up the grid for a given size and clear the panel. */
void vt_init(int cols, int rows);

/* Feed terminal output. Returns true if anything changed on screen. */
bool vt_feed(const uint8_t *buf, int len);

/* Redraw every cell (after a full-screen clear, or a resize). */
void vt_redraw_all(void);

/* Size the emulator was told about. */
int vt_cols(void);
int vt_rows(void);

/* True if the stream contained a cursor-position query (ESC[6n) since the
 * last call, which the link layer must answer with ESC[rows;colsR. */
bool vt_take_size_query(int *rows, int *cols);

#endif
