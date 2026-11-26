/*
 * maps.h - a pannable map on the watch, four tiles at a time.
 *
 * THE DESIGN, AND WHY IT IS FOUR. A window of 2x2 z15 tiles is held in memory and
 * the framebuffer shows a 320x416 view INTO that 512x512 square, so the wearer can
 * pan within it and re-centre on the point where all four tiles meet. An arrow
 * press slides the whole square by one tile, so the view walks across the map
 * while never holding more than four tiles.
 *
 * WHY FOUR AND NOT MORE. Each z15 tile is 256x256 RGB565, which is 128 KB, so four
 * is 512 KB of PSRAM -- affordable, and it is the budget the operator set. Only two
 * of the four are READ on a slide, because the other two are already resident, so
 * an arrow press costs a 256 KB read rather than 512 KB.
 *
 * WHY 1:1 AND NEVER SCALED. A tile is 256 px and the framebuffer is 320x416, whose
 * aspect ratios differ (1.25 against 1.625). Drawing a tile "at framebuffer size"
 * would have to stretch it 30 percent taller than wide, and a map with a 30 percent
 * vertical stretch is not a map -- a round lake renders as an egg. So a tile is
 * drawn at its own 256 px and the screen is a WINDOW, which keeps this project's one
 * rule from the tile work: nothing is resampled, every pixel is the tile's own.
 *
 * WHAT THAT COSTS IN PAN RANGE, stated plainly because it is the surprising part. A
 * 512x512 square seen through 320x416 leaves 192 px of horizontal travel and 96 of
 * vertical; centred, that is 96 px each way across and 48 down, which is 38 and 19
 * percent of a tile. An arrow press moves a whole tile. So the ARROWS do the
 * travelling and the pan is a fine adjustment for framing. If panning is ever
 * wanted as a way to get somewhere, the square has to grow -- and this code takes a
 * bigger one unchanged, because the window is a rectangle into a buffer.
 *
 * THE FILE FORMAT is the TMAP container the single-image phase used, at version 2,
 * with the tile's own grid coordinates in the header. Those coordinates exist so
 * the app can PROVE it opened the tile it meant to: naming files by coordinate and
 * then trusting the name is the same class of silent fault as the 64-byte path that
 * once sent the Tamagotchi looking for the wrong ROM.
 */
#ifndef TWATCH_MAPS_H
#define TWATCH_MAPS_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "display.h"

/* Where the tile set lives. One file per tile, named by grid coordinate, so a
 * missing one names itself in the error message. */
#define MAPS_DIR_DEFAULT "/sd/media/maps/z15"
#define MAPS_DIR_FLAT    "/sd/z15maps"

#define MAPS_TILE        256            /* z15 tile edge, px */
#define MAPS_WIN_W       DISPLAY_W      /* the view into the set */
#define MAPS_WIN_H       DISPLAY_H

/* Centred on the 2x2 intersection: (512-320)/2 and (512-416)/2. */
#define MAPS_PAN_CX      ((2 * MAPS_TILE - MAPS_WIN_W) / 2)   /* 96 */
#define MAPS_PAN_CY      ((2 * MAPS_TILE - MAPS_WIN_H) / 2)   /* 48 */

/* "sentinel: no specific tile wanted", for maps_start() and the header check. */
#define MAPS_ANY         INT32_MIN

/* The header, little-endian, then w*h*2 bytes of pixel data. */
#define MAPS_MAGIC_0 'T'
#define MAPS_MAGIC_1 'M'
#define MAPS_MAGIC_2 'A'
#define MAPS_MAGIC_3 'P'
#define MAPS_VERSION 2
#define MAPS_FORMAT_RGB565 1
#define MAPS_HEADER_BYTES 24

/*
 * CRC-32/ISO-HDLC, the one Python's zlib.crc32 computes.
 *
 * INLINE HERE, NOT IN maps.c, SO IT CAN BE TESTED. A checksum that disagrees with
 * the tool that wrote the file refuses EVERY tile, and the symptom looks like a
 * corrupt card rather than like this project's own arithmetic. Table-less: a slide
 * walks 256 KB, and 1 KB of table is not worth it in this board's scarcest RAM.
 */
static inline uint32_t maps_crc32(const uint8_t *p, size_t n)
{
    uint32_t c = 0xFFFFFFFFu;

    for (size_t i = 0; i < n; i++)
    {
        c ^= p[i];

        for (int k = 0; k < 8; k++)
            c = (c >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(c & 1)));
    }

    return c ^ 0xFFFFFFFFu;
}

/* Parsed header. `tx`/`ty` are the tile's own grid coordinates. */
typedef struct
{
    uint16_t w, h;
    uint32_t crc;
    int32_t tx, ty;
} maps_hdr_t;

/*
 * Parse one header. False unless the magic, version and format match, the
 * dimensions are exactly one tile, and -- when `expect_x` is not MAPS_ANY -- the
 * tile coordinates are the ones asked for.
 *
 * Read field by field rather than by casting: the format is defined little-endian,
 * and a struct would inherit the compiler's alignment and padding.
 */
static inline bool maps_header_parse(const uint8_t raw[MAPS_HEADER_BYTES],
                                     maps_hdr_t *out, int32_t expect_x,
                                     int32_t expect_y)
{
    const uint32_t want = (uint32_t)MAPS_MAGIC_0 | ((uint32_t)MAPS_MAGIC_1 << 8) |
                          ((uint32_t)MAPS_MAGIC_2 << 16) |
                          ((uint32_t)MAPS_MAGIC_3 << 24);
    const uint32_t magic = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) |
                           ((uint32_t)raw[2] << 16) | ((uint32_t)raw[3] << 24);
    const uint16_t ver = (uint16_t)(raw[4] | (raw[5] << 8));
    const uint16_t fmt = (uint16_t)(raw[6] | (raw[7] << 8));
    const uint16_t ww = (uint16_t)(raw[8] | (raw[9] << 8));
    const uint16_t hh = (uint16_t)(raw[10] | (raw[11] << 8));

    if (magic != want || ver != MAPS_VERSION || fmt != MAPS_FORMAT_RGB565)
        return false;

    /* EXACTLY ONE TILE. A 320x416 image is the phase-1 full-screen format and is
     * refused here rather than half-supported, because the window arithmetic
     * assumes tile-sized source rows. */
    if (ww != MAPS_TILE || hh != MAPS_TILE)
        return false;

#define RD32(o) ((uint32_t)raw[o] | ((uint32_t)raw[o + 1] << 8) | \
                 ((uint32_t)raw[o + 2] << 16) | ((uint32_t)raw[o + 3] << 24))

    out->w = ww;
    out->h = hh;
    out->crc = RD32(12);
    out->tx = (int32_t)RD32(16);
    out->ty = (int32_t)RD32(20);

#undef RD32

    if (expect_x != MAPS_ANY && (out->tx != expect_x || out->ty != expect_y))
        return false;

    return true;
}

/* Start the viewer at tile (x, y), or at the default set when x is MAPS_ANY.
 * Claims the screen. False leaves everything as it was. */
bool maps_start(int32_t x, int32_t y);

/* Release the screen. Safe when not running. */
void maps_stop(void);

bool maps_running(void);

/* Why the last start or slide refused, or NULL. The glass has one line. */
const char *maps_last_error(void);

/* Which tile is at the window's top-left, and where the window sits inside the
 * 2x2 square. Exposed for `maps status`, which is how a wrong picture is told
 * apart from a wrong coordinate without rebooting. */
void maps_where(int32_t *tx, int32_t *ty, int *px, int *py);

/* Re-compose and re-blit from the tiles already in memory. */
bool maps_redraw(void);

/* Slide the window one tile in a direction: -1, 0 or 1 on each axis. Exposed so
 * `maps pan <dir>` can drive it from the shell, which is how a slide is tested
 * without a finger on the glass. */
bool maps_step(int dx, int dy);

/*
 * Install the arrow consumer. Separated from the shell command for the reason
 * `gb_install_touch_hook()` is: slot B has no shell, so registering the command
 * there would leave an app nobody can press.
 */
void maps_install_touch_hook(void);

/* Register the `maps` shell command on the desktop. */
bool maps_register_command(void);

#endif /* TWATCH_MAPS_H */
