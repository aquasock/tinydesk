/*
 * maps.h - a static map image on the watch, as a first look at the display.
 *
 * THIS IS PHASE 1 OF A MAP VIEWER AND IT DELIBERATELY DOES ALMOST NOTHING. It
 * reads one file, blits it to the framebuffer, and stops. No panning, no zooming,
 * no tile streaming, no palette. The question it exists to answer is not "can
 * the watch draw a map" -- it can -- but "can a map be READ on a two-inch
 * 320x416 screen", and that is a legibility question that only real pixels at
 * their own scale can settle.
 *
 * WHAT CAME FROM TinyMaps, AND WHAT DID NOT. The TinyMaps project on the Tang is
 * a hard-real-time FPGA renderer: a line engine, its own SDRAM controller, TMDS
 * out at 1280x720. None of that transfers to a watch. What does transfer is one
 * rule, and it decides this file's whole shape -- "nothing is resampled, every
 * pixel kept is the tile's own". A tile is 256x256 and the framebuffer is
 * 320x416, so the screen cannot hold a whole tile at native size; the honest
 * answer is a 320x416 WINDOW of the tile mosaic at 1:1 rather than a tile shrunk
 * to fit. `tools/maps_tile.py` in the Escapement repository cuts that window and
 * never scales.
 *
 * WHY A HEADER AND A CHECKSUM. A 266,240-byte blob with nothing in front of it
 * is indistinguishable from any other blob, and a card read that comes back
 * short is a real fault on this board -- the cartridge picker once reported a
 * truncated read as "did not start" and sent everyone looking at the card. So the
 * file carries a magic, a version, a format, its dimensions and a CRC-32, and
 * this app refuses anything that does not match rather than blitting whatever it
 * got.
 */
#ifndef TWATCH_MAPS_H
#define TWATCH_MAPS_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "display.h"

#define MAPS_PATH_DEFAULT "/sd/media/maps/tile.tmap"
#define MAPS_PATH_FLAT    "/sd/tile.tmap"

/* Header, little-endian, 16 bytes, then w*h*2 bytes of pixel data. */
#define MAPS_MAGIC_0 'T'
#define MAPS_MAGIC_1 'M'
#define MAPS_MAGIC_2 'A'
#define MAPS_MAGIC_3 'P'
#define MAPS_VERSION 1
#define MAPS_FORMAT_RGB565 1
#define MAPS_HEADER_BYTES 16

/*
 * CRC-32/ISO-HDLC, the one Python's zlib.crc32 computes.
 *
 * IT LIVES HERE AS AN INLINE AND NOT IN maps.c BECAUSE IT MUST BE TESTED. A
 * checksum that disagrees with the tool that wrote the file is a fault that
 * looks exactly like a corrupt image, and it would be found by staring at a
 * garbage screen and guessing. `tests/test_maps_crc.c` checks this against the
 * published check value and against vectors from zlib, on a host, where a
 * mismatch costs nothing.
 *
 * Table-less: 266 KB of pixels is walked once per load, so the eight shifts a
 * byte costs are not worth 1 KB of table in the scarcest RAM this board has.
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

/*
 * Read the 16-byte header into fields. Returns false unless the magic, version
 * and format all match and the dimensions are exactly the framebuffer's.
 *
 * INLINE HERE FOR THE SAME REASON AS THE CHECKSUM: it is pure logic over bytes,
 * and a header parser that disagrees with the packer's writer produces a screen
 * full of nothing with one log line. `tests/test_maps_crc.c` walks every field
 * that can make it refuse, on a host.
 *
 * Read field by field rather than by casting to a struct: the format is defined
 * little-endian, and a struct would inherit the compiler's alignment and padding
 * and mean something different on another target.
 */
static inline bool maps_header_parse(const uint8_t raw[MAPS_HEADER_BYTES],
                                     uint16_t *w, uint16_t *h, uint32_t *crc)
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

    /* EXACTLY THE FRAMEBUFFER'S SIZE, not "at most". A smaller picture would
     * leave the desktop's cells showing at the edges under an app that claims the
     * screen, and this phase exists to fill it. */
    if (ww != DISPLAY_W || hh != DISPLAY_H)
        return false;

    *w = ww;
    *h = hh;
    *crc = (uint32_t)raw[12] | ((uint32_t)raw[13] << 8) |
           ((uint32_t)raw[14] << 16) | ((uint32_t)raw[15] << 24);
    return true;
}

/* Start from `path`, or from MAPS_PATH_DEFAULT when NULL. Claims the screen and
 * blits. Returns false and leaves everything as it was when the file will not
 * load or the picture does not validate -- a failed start must not leave the
 * glass claimed. */
bool maps_start(const char *path);

/* Release the screen. Safe when not running. */
void maps_stop(void);

bool maps_running(void);

/* Why the last start refused, or NULL. Named so a start that fails says which
 * of several things went wrong, the way gb_last_error() does -- the glass has
 * one line and no console. */
const char *maps_last_error(void);

/* Re-blit the picture already in memory. For `maps draw` and for recovering the
 * screen after anything that could have repainted the framebuffer. */
bool maps_redraw(void);

/* Register the `maps` shell command on the desktop. Slot B has no shell, so it
 * never calls this -- it calls maps_start() from its app table instead. */
bool maps_register_command(void);

#endif /* TWATCH_MAPS_H */
