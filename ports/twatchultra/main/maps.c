/*
 * maps.c - four z15 tiles, panned and walked by the margin arrows.
 *
 * See maps.h for the design and for why it is four tiles at 1:1 rather than one
 * scaled to fill. This file is the mechanism: four slots, a window into them, and a
 * slide that reads only the two tiles that are new.
 */
#include "maps.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* COL_N, BTN_N and the EDGE_* regions: the margin geometry, so the arrow mapping
 * uses the same indices the painter and the hit test do. */
#include "edges.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "sdpaths.h"
#include "tdsh.h"
#include "touch.h"

static const char *TAG = "maps";

/*
 * FOUR TILE SLOTS AND ONE VIEW BUFFER, ALL IN PSRAM.
 *
 * 4 x 128 KB plus 267 KB is 778 KB, which is affordable here and is NOT affordable
 * in internal DRAM: everything static in this component lands there by construction
 * (see extram.lf), and 4,148 bytes of .bss once starved the SSH server of its
 * 65,536-byte floor (EMU-018). So all five buffers are heap allocations with a PSRAM
 * capability, and a failure to get one is REPORTED rather than papered over.
 *
 * s_tile[r][c] holds the tile at global grid (s_tx + c, s_ty + r), so the array IS
 * the 2x2 square and a slide is a move of pointers rather than a re-index of
 * contents.
 */
static uint16_t *s_tile[2][2];
static uint16_t *s_view;
static char *s_dir;

static int32_t s_tx, s_ty;      /* the tile at the square's top-left */
static int s_px, s_py;          /* the window's offset inside the 512x512 square */
static volatile bool s_running;
static bool s_claimed;
static const char *s_error;
static char s_path[256];

/* Press-edge tracking for the arrows, so a held finger does not slide the map every
 * report. The controller re-reports a held touch about every 66 ms (TCH-012), which
 * would walk four tiles a second. */
static int s_last_dir;          /* -1 when nothing was down last report */

#define FAIL(msg) do { s_error = (msg); ESP_LOGW(TAG, "%s", (msg)); return false; } while (0)

/* ---------------------------------------------------------------- tiles */

static bool tile_path(char *out, size_t cap, int32_t tx, int32_t ty)
{
    const int n = snprintf(out, cap, "%s/t_%ld_%ld.tmap", s_dir,
                           (long)tx, (long)ty);

    return n > 0 && (size_t)n < cap;
}

/*
 * Read one tile into a slot, and PROVE it is the tile that was asked for.
 *
 * The header carries its own grid coordinates and they are compared against the name
 * that produced the path. That is not paranoia: naming files by coordinate and then
 * trusting the name is the same silent-fault class as the 64-byte ROM path, where a
 * truncated string became a well-formed path to the wrong file and the card got the
 * blame. A mis-named or wrongly generated tile is caught here, with an error that
 * says which coordinate disagreed.
 */
static bool load_tile(int r, int c, int32_t tx, int32_t ty)
{
    char path[256];

    if (!tile_path(path, sizeof(path), tx, ty))
        FAIL("tile path too long");

    snprintf(s_path, sizeof(s_path), "%s", path);

    FILE *f = fopen(path, "rb");

    if (!f)
        FAIL("a tile is missing");

    uint8_t hdr[MAPS_HEADER_BYTES];
    maps_hdr_t h;

    if (fread(hdr, 1, sizeof(hdr), f) != sizeof(hdr))
    {
        fclose(f);
        FAIL("a tile header is short");
    }

    if (!maps_header_parse(hdr, &h, tx, ty))
    {
        fclose(f);
        FAIL("a tile is not the one its name says");
    }

    const size_t bytes = (size_t)MAPS_TILE * MAPS_TILE * sizeof(uint16_t);

    if (!s_tile[r][c])
        s_tile[r][c] = (uint16_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);

    if (!s_tile[r][c])
    {
        fclose(f);
        FAIL("no psram for a tile");
    }

    const size_t got = fread(s_tile[r][c], 1, bytes, f);

    fclose(f);

    if (got != bytes)
        FAIL("a tile read short");

    /* CHECKED BEFORE ANYTHING IS COMPOSED. A tile that fails here is garbage in a
     * way a size check cannot see, and drawing it would put noise on the glass with
     * nothing saying where it came from. */
    const uint32_t crc = maps_crc32((const uint8_t *)s_tile[r][c], bytes);

    if (crc != h.crc)
        FAIL("a tile failed its checksum");

    return true;
}

/*
 * Read one tile into a CALLER'S buffer, so a slide can validate before it commits.
 *
 * The same checks as load_tile(): the header's own coordinates must match the one
 * asked for, and the CRC must match the payload. A failure frees the buffer and
 * leaves it NULL, so the caller's cleanup is one loop and cannot double-free.
 */
static bool load_tile_into(uint16_t **slot, int32_t tx, int32_t ty)
{
    char path[256];

    *slot = NULL;

    if (!tile_path(path, sizeof(path), tx, ty))
    {
        s_error = "tile path too long";
        return false;
    }

    snprintf(s_path, sizeof(s_path), "%s", path);

    FILE *f = fopen(path, "rb");

    if (!f)
    {
        s_error = "a tile is missing";
        return false;
    }

    uint8_t hdr[MAPS_HEADER_BYTES];
    maps_hdr_t h;

    if (fread(hdr, 1, sizeof(hdr), f) != sizeof(hdr))
    {
        fclose(f);
        s_error = "a tile header is short";
        return false;
    }

    if (!maps_header_parse(hdr, &h, tx, ty))
    {
        fclose(f);
        s_error = "a tile is not the one its name says";
        return false;
    }

    const size_t bytes = (size_t)MAPS_TILE * MAPS_TILE * sizeof(uint16_t);
    uint16_t *buf = (uint16_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);

    if (!buf)
    {
        fclose(f);
        s_error = "no psram for a tile";
        return false;
    }

    const size_t got = fread(buf, 1, bytes, f);

    fclose(f);

    if (got != bytes)
    {
        heap_caps_free(buf);
        s_error = "a tile read short";
        return false;
    }

    if (maps_crc32((const uint8_t *)buf, bytes) != h.crc)
    {
        heap_caps_free(buf);
        s_error = "a tile failed its checksum";
        return false;
    }

    *slot = buf;
    return true;
}

/* Load the whole 2x2 square from scratch. */
static bool load_square(int32_t tx, int32_t ty)
{
    for (int r = 0; r < 2; r++)
        for (int c = 0; c < 2; c++)
            if (!load_tile(r, c, tx + c, ty + r))
                return false;

    s_tx = tx;
    s_ty = ty;
    return true;
}

/* ---------------------------------------------------------- composition */

/*
 * Compose the window out of the four slots.
 *
 * ONE memcpy PER TILE SPAN PER ROW, so a row that straddles no boundary is a single
 * copy and one that straddles one is two. A pixel loop would walk 133,120 pixels
 * through a function call each time the window moved, which is the kind of cost that
 * turns a pan into a stutter.
 *
 * There is NO SCALING in here, deliberately: a source column of a tile maps to a
 * destination column one for one, which is the rule the whole map line follows.
 */
static void compose(void)
{
    /*
     * REFUSE RATHER THAN FAULT. A NULL slot used to mean a reboot: the slide that
     * created it returned false, the app stayed "running", and the next redraw
     * dereferenced it. The commit-after-load rewrite above means this should not be
     * reachable, and it stays because "should not be reachable" is what the last
     * version said too.
     */
    for (int r = 0; r < 2; r++)
        for (int c = 0; c < 2; c++)
            if (!s_tile[r][c])
                return;

    for (int y = 0; y < MAPS_WIN_H; y++)
    {
        const int my = s_py + y;               /* row inside the 512x512 square */
        const int r = my >> 8;                 /* which tile row (0 or 1)       */
        const int ty = my & (MAPS_TILE - 1);   /* row inside that tile          */

        uint16_t *dst = s_view + (size_t)y * MAPS_WIN_W;

        for (int c = 0; c < 2; c++)
        {
            /* Columns of the square this tile owns, clipped to the window. */
            const int sq0 = c * MAPS_TILE;
            const int sq1 = sq0 + MAPS_TILE;
            const int x0 = s_px > sq0 ? s_px : sq0;
            const int x1 = (s_px + MAPS_WIN_W) < sq1 ? (s_px + MAPS_WIN_W) : sq1;

            if (x1 <= x0)
                continue;

            const uint16_t *src = s_tile[r][c] + (size_t)ty * MAPS_TILE + (x0 - sq0);

            memcpy(dst + (x0 - s_px), src, (size_t)(x1 - x0) * sizeof(uint16_t));
        }
    }
}

bool maps_redraw(void)
{
    if (!s_view || !s_running)
        return false;

    compose();

    /* ONE BLIT, ALREADY IN THE RIGHT BYTE ORDER. display_picture_rgb() copies into
     * the framebuffer and flush_rect() swaps each word on the way out, so s_view
     * holds LOGICAL little-endian RGB565 and the driver swaps it exactly once. That
     * is the opposite contract to display_picture_rows(), whose callback must emit
     * panel order. */
    return display_picture_rgb(NULL, 0, 0, s_view, MAPS_WIN_W, MAPS_WIN_H);
}

/* ---------------------------------------------------------------- slides */

/*
 * Slide the square one tile and centre the window on the new intersection.
 *
 * TWO TILES ARE READ, NOT FOUR. Moving right discards column 0, shifts column 1 into
 * its place and reads a new column 1; the two rows move the same way. Doing it by
 * pointer means the surviving tiles are neither re-read nor copied.
 *
 * THE WINDOW RE-CENTRES, which is the operator's rule and also the only sane one: a
 * window left at an edge of the old square would be off centre in the new one, and
 * the pan range would drift a little on every press.
 */
/*
 * Slide the square one tile and centre the window on the new intersection.
 *
 * TWO TILES ARE READ, NOT FOUR. Moving right discards column 0, shifts column 1 into
 * its place and reads a new column 1; the two rows move the same way.
 *
 * THE NEW TILES ARE LOADED BEFORE THE SQUARE MOVES, AND THAT IS A CORRECTION RATHER
 * THAN A STYLE. The first version shuffled the pointers first and loaded second, so
 * a slide past the edge of the set returned false having already NULLed two slots --
 * and the next redraw dereferenced one and rebooted the watch. The operator's first
 * walk to the block edge would have found it; a chain of shell commands found it
 * instead. Now a failed slide leaves the map EXACTLY as it was: the press does
 * nothing, the picture stays, and the error says which coordinate is missing.
 *
 * The temporaries are freed on the failure path, so a refused slide costs no memory
 * either. This is why the code below loads into `n[2]` and only then commits.
 */
bool maps_step(int dx, int dy)
{
    if (!s_running)
    {
        s_error = "not running";
        return false;
    }

    if (!dx && !dy)
        return true;

    /* Keep the loop to one axis at a time so the reuse stays obvious; a diagonal
     * press is two steps and nothing on the glass offers one. */
    const int32_t ntx = s_tx + dx;
    const int32_t nty = s_ty + dy;

    uint16_t *n[2] = { NULL, NULL };

    if (dx)
    {
        const int32_t col = (dx > 0) ? (ntx + 1) : ntx;

        if (!load_tile_into(&n[0], col, s_ty) ||
            !load_tile_into(&n[1], col, s_ty + 1))
        {
            for (int i = 0; i < 2; i++)
                heap_caps_free(n[i]);
            return false;
        }

        if (dx > 0)
        {
            /* column 1 becomes column 0; the new tiles take column 1 */
            s_tile[0][0] = s_tile[0][1];
            s_tile[1][0] = s_tile[1][1];
            s_tile[0][1] = n[0];
            s_tile[1][1] = n[1];
        }
        else
        {
            /* column 0 becomes column 1; the new tiles take column 0 */
            s_tile[0][1] = s_tile[0][0];
            s_tile[1][1] = s_tile[1][0];
            s_tile[0][0] = n[0];
            s_tile[1][0] = n[1];
        }

        s_tx = ntx;
    }

    if (dy)
    {
        const int32_t row = (dy > 0) ? (nty + 1) : nty;
        const int32_t base = dx ? ntx : s_tx;

        if (!load_tile_into(&n[0], base, row) ||
            !load_tile_into(&n[1], base + 1, row))
        {
            for (int i = 0; i < 2; i++)
                heap_caps_free(n[i]);
            return false;
        }

        if (dy > 0)
        {
            s_tile[0][0] = s_tile[1][0];
            s_tile[0][1] = s_tile[1][1];
            s_tile[1][0] = n[0];
            s_tile[1][1] = n[1];
        }
        else
        {
            s_tile[1][0] = s_tile[0][0];
            s_tile[1][1] = s_tile[0][1];
            s_tile[0][0] = n[0];
            s_tile[0][1] = n[1];
        }

        s_ty = nty;
    }

    s_px = MAPS_PAN_CX;
    s_py = MAPS_PAN_CY;
    s_error = NULL;

    return maps_redraw();
}

/* Move the window inside the square without touching the card. */
static bool pan(int dx, int dy)
{
    int nx = s_px + dx;
    int ny = s_py + dy;

    const int maxx = 2 * MAPS_TILE - MAPS_WIN_W;   /* 192 */
    const int maxy = 2 * MAPS_TILE - MAPS_WIN_H;   /* 96  */

    if (nx < 0) nx = 0;
    if (nx > maxx) nx = maxx;
    if (ny < 0) ny = 0;
    if (ny > maxy) ny = maxy;

    if (nx == s_px && ny == s_py)
        return false;               /* already against that edge */

    s_px = nx;
    s_py = ny;
    return maps_redraw();
}

/* ------------------------------------------------------------ lifecycle */

static void release_buffers(void)
{
    for (int r = 0; r < 2; r++)
        for (int c = 0; c < 2; c++)
        {
            heap_caps_free(s_tile[r][c]);
            s_tile[r][c] = NULL;
        }

    heap_caps_free(s_view);
    s_view = NULL;
    heap_caps_free(s_dir);
    s_dir = NULL;
}

/*
 * The default start, from `start.txt` in the set directory: two integers, the
 * global z/x-like tile coordinates of the set's first tile.
 *
 * A FILE RATHER THAN A COMPILED CONSTANT. The coordinates of a real place are data,
 * and hard-coding one would mean a rebuild to look somewhere else -- and a default
 * of (0,0) is a tile off the coast of Africa, which reads as "no map" rather than as
 * "no start". The generator writes this file next to the tiles it made.
 */
static bool default_start(int32_t *x, int32_t *y)
{
    char path[256];

    if ((size_t)snprintf(path, sizeof(path), "%s/start.txt", s_dir) >= sizeof(path))
        return false;

    FILE *f = fopen(path, "r");

    if (!f)
    {
        snprintf(s_path, sizeof(s_path), "%s", path);
        return false;
    }

    const int n = fscanf(f, "%ld %ld", (long *)x, (long *)y);

    fclose(f);
    return n == 2;
}

bool maps_start(int32_t x, int32_t y)
{
    if (s_running)
        return true;

    s_error = NULL;
    s_last_dir = -1;

    const char *dir = sd_media(MAPS_DIR_DEFAULT, MAPS_DIR_FLAT);

    if (!s_dir)
        s_dir = (char *)heap_caps_malloc(strlen(dir) + 1, MALLOC_CAP_SPIRAM);

    if (!s_dir)
        FAIL("no psram for the path");

    strcpy(s_dir, dir);

    if (!s_view)
        s_view = (uint16_t *)heap_caps_malloc(
            (size_t)MAPS_WIN_W * MAPS_WIN_H * sizeof(uint16_t), MALLOC_CAP_SPIRAM);

    if (!s_view)
        FAIL("no psram for the view");

    if (x == MAPS_ANY || y == MAPS_ANY)
    {
        if (!default_start(&x, &y))
        {
            s_error = "no start tile in the set";
            release_buffers();
            return false;
        }
    }

    if (!load_square(x, y))
    {
        release_buffers();
        return false;
    }

    if (!s_claimed)
    {
        display_pixels_claim(true);
        s_claimed = true;
    }

    display_margins_set(DISPLAY_MARGINS_APP);

    /*
     * THE RIGHT COLUMN BECOMES THE ARROWS, AND THE LEFT GOES BLANK.
     *
     * The arrows are the same glyphs the desktop column draws and they mean the same
     * direction, so a wearer who has used the desktop already knows them. A single
     * multibyte glyph is exempt from the column's rotation (label_is_glyph()), which
     * is why they render upright rather than turning into a different arrow -- the
     * table in display.c records that a transposed U+2191 comes out pointing LEFT.
     *
     * THE LEFT COLUMN IS BLANK BUT THE BOXES STAY: a map has no other controls yet,
     * and an empty box is this port's established way of drawing a slot with nothing
     * in it.
     */
    {
        static const char *const arrows[COL_N] = {
            " ",
            "\xe2\x86\x90",   /* U+2190 leftwards  */
            "\xe2\x86\x92",   /* U+2192 rightwards */
            "\xe2\x86\x91",   /* U+2191 upwards    */
            "\xe2\x86\x93",   /* U+2193 downwards  */
        };
        static const char *const blank[COL_N] = { " ", " ", " ", " ", " " };

        display_margin_labels(0, blank, COL_N - 1);
        display_margin_labels(1, arrows, COL_N - 1);
    }

    for (int i = 0; i < BTN_N; i++)
        display_margin_btn_label(i, " ");

    s_px = MAPS_PAN_CX;
    s_py = MAPS_PAN_CY;
    s_running = true;

    if (!maps_redraw())
    {
        maps_stop();
        FAIL("the panel refused the picture");
    }

    ESP_LOGI(TAG, "showing %s from tile %ld,%ld", s_dir, (long)s_tx, (long)s_ty);
    return true;
}

void maps_stop(void)
{
    if (!s_running && !s_claimed)
        return;

    s_running = false;
    s_last_dir = -1;

    if (s_claimed)
    {
        display_margins_set(DISPLAY_MARGINS_DESKTOP);
        display_pixels_claim(false);
        s_claimed = false;
    }

    release_buffers();
}

bool maps_running(void)
{
    return s_running;
}

const char *maps_last_error(void)
{
    return s_error;
}

void maps_where(int32_t *tx, int32_t *ty, int *px, int *py)
{
    if (tx) *tx = s_tx;
    if (ty) *ty = s_ty;
    if (px) *px = s_px;
    if (py) *py = s_py;
}

/* ------------------------------------------------------------ the arrows */

/*
 * The arrows, as press edges.
 *
 * THE DIRECTION IS A PROPERTY OF THE SLOT, not of a remembered index, so a slot that
 * moves cannot leave a press sending the direction it used to be. Index 0 is the
 * reserved blank and does nothing.
 *
 * ONLY THE EDGE SLIDES. A held finger re-reports about every 66 ms, and following
 * every report would walk the map four tiles a second -- the fault the picker paid
 * for once, where one tap moved the highlight five rows.
 */
static bool maps_touch(void *user, const touch_frame_t *tf)
{
    (void)user;

    if (!s_running || !tf)
        return false;

    int dir = -1;

    for (int i = 0; i < tf->n && i < TOUCH_POINTS; i++)
        if (tf->p[i].region == EDGE_RIGHT && tf->p[i].index >= 1 && tf->p[i].index <= 4)
            dir = tf->p[i].index;

    if (dir < 0)
    {
        s_last_dir = -1;
        return false;
    }

    if (dir == s_last_dir)
        return true;                /* held: already taken */

    s_last_dir = dir;

    switch (dir)
    {
    case 1: (void)maps_step(-1, 0); break;   /* Left  */
    case 2: (void)maps_step(1, 0);  break;   /* Right */
    case 3: (void)maps_step(0, -1); break;   /* Up    */
    case 4: (void)maps_step(0, 1);  break;   /* Down  */
    default: break;
    }

    /* True so the driver buzzes: something happened, and the map moved. */
    return true;
}

/*
 * IDEMPOTENT, because the two callers are `maps_register_command()` on the desktop
 * and `slotb_run()` in slot B, and only one of them runs per boot -- but a second
 * registration would put the same function in the driver's table twice and slide the
 * map TWICE per press. That is a fault that would look like a double-step and be
 * blamed on the file reads.
 */
void maps_install_touch_hook(void)
{
    static bool done;

    if (done)
        return;

    if (!touch_add_app_touch(maps_touch, NULL))
    {
        ESP_LOGW(TAG, "maps touch consumer did not register");
        return;
    }

    done = true;
}

/* ---------------------------------------------------------- shell command */

static int cmd_maps(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    const char *sub = argc >= 2 ? argv[1] : "start";

    if (!strcmp(sub, "stop"))
    {
        if (!maps_running())
        {
            printf("not running\n");
            return 0;
        }
        maps_stop();
        printf("stopped, screen released\n");
        return 0;
    }

    if (!strcmp(sub, "draw"))
    {
        if (!maps_running())
        {
            printf("not running\n");
            return 1;
        }
        printf(maps_redraw() ? "redrawn\n" : "the panel refused it\n");
        return 0;
    }

    if (!strcmp(sub, "status"))
    {
        int32_t tx = 0, ty = 0;
        int px = 0, py = 0;

        maps_where(&tx, &ty, &px, &py);
        printf("running=%d tile=%ld,%ld pan=%d,%d\n",
               maps_running() ? 1 : 0, (long)tx, (long)ty, px, py);
        printf("dir=%s\n", s_dir ? s_dir : "-");
        if (s_error)
            printf("last error: %s (%s)\n", s_error, s_path);
        return 0;
    }

    /* `maps pan <dir>`: slide one tile. The directions are the arrow words, so the
     * shell and the glass cannot disagree about what a press means. */
    if (!strcmp(sub, "pan"))
    {
        const char *d = argc >= 3 ? argv[2] : "";

        int dx = 0, dy = 0;

        if (!strcmp(d, "left"))       dx = -1;
        else if (!strcmp(d, "right")) dx = 1;
        else if (!strcmp(d, "up"))    dy = -1;
        else if (!strcmp(d, "down"))  dy = 1;
        else
        {
            printf("maps pan <left|right|up|down>\n");
            return 1;
        }

        if (!maps_step(dx, dy))
        {
            printf("did not pan: %s (%s)\n", s_error ? s_error : "unknown", s_path);
            return 1;
        }

        printf("tile %ld,%ld\n", (long)s_tx, (long)s_ty);
        return 0;
    }

    /* `maps nudge <dir>`: move the window inside the square, no card read. */
    if (!strcmp(sub, "nudge"))
    {
        const char *d = argc >= 3 ? argv[2] : "";

        int dx = 0, dy = 0;

        if (!strcmp(d, "left"))       dx = -8;
        else if (!strcmp(d, "right")) dx = 8;
        else if (!strcmp(d, "up"))    dy = -8;
        else if (!strcmp(d, "down"))  dy = 8;
        else
        {
            printf("maps nudge <left|right|up|down>\n");
            return 1;
        }

        if (!maps_running())
        {
            printf("not running\n");
            return 1;
        }

        printf("%s\n", pan(dx, dy) ? "moved" : "already at that edge");
        return 0;
    }

    if (!strcmp(sub, "start"))
    {
        /* An optional X Y names the top-left tile explicitly, which is how a set is
         * walked from the shell without a finger on the glass. */
        int32_t x = MAPS_ANY, y = MAPS_ANY;

        if (argc >= 4)
        {
            x = (int32_t)strtol(argv[2], NULL, 10);
            y = (int32_t)strtol(argv[3], NULL, 10);
        }

        if (!maps_start(x, y))
        {
            printf("did not start: %s (%s)\n", s_error ? s_error : "unknown", s_path);
            return 1;
        }
        printf("showing from tile %ld,%ld\n", (long)s_tx, (long)s_ty);
        return 0;
    }

    printf("maps [start [x y] | stop | draw | pan <dir> | nudge <dir> | status]\n");
    return 1;
}

static const tdsh_command_t s_maps_cmd = {
    "maps",
    "maps [start [x y] | stop | draw | pan <dir> | nudge <dir> | status]",
    "Show and walk a map",
    cmd_maps,
    TDSH_CMD_INTERACTIVE,
};

/*
 * Registered for the life of the process, not per run: the handler declines
 * everything while the map is not running, so it behaves exactly like being
 * unregistered without a window in which the driver holds a pointer to something
 * that has been freed.
 *
 * THE TOUCH HOOK IS INSTALLED HERE, AND THAT WAS THE BUG THE OPERATOR FOUND. This
 * function used to register only the shell command, on the reasoning that slot B has
 * no shell and would install the hook itself -- true, but it left the DESKTOP path
 * with no consumer at all, so a map started from a Terminal drew perfectly and
 * ignored every press. Every other app in this port installs its hook here for
 * exactly this reason: `tama_register_command()` and `gb_register_command()` both
 * do. Splitting the two was the mistake, and the split is right in slotb.c, where
 * the hook is called directly because there is no shell to register into.
 */
bool maps_register_command(void)
{
    maps_install_touch_hook();

    return tdsh_register_command(&s_maps_cmd) == 0;
}
