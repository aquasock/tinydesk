/*
 * maps.c - blit one static map image to the framebuffer.
 *
 * See maps.h for what this is for and why it does so little. This file is the
 * board's half: read, validate, claim, blit, release.
 */
#include "maps.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* COL_N and BTN_N: the margin geometry, so blanking the columns uses the same
 * counts the painter and the hit test do instead of a second guess at them. */
#include "edges.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "sdpaths.h"
#include "tdsh.h"

static const char *TAG = "maps";

/* The picture, in PSRAM. 266 KB, which is the same order as the NES's frame and
 * for the same reason: everything static in this component lands in internal
 * DRAM (see extram.lf), and internal DRAM is this board's scarcest pool --
 * 4,148 bytes of .bss once starved the SSH server of its 65,536-byte floor
 * (EMU-018). So the buffer is a heap allocation with a PSRAM capability, and a
 * failure to get one is reported rather than silently falling back. */
static uint16_t *s_px;
static uint16_t s_w, s_h;
static volatile bool s_running;
static bool s_claimed;
static const char *s_error;
static char s_path[256] = MAPS_PATH_DEFAULT;

/* Cheap, readable, and enough to tell the caller which step refused. */
#define FAIL(msg) do { s_error = (msg); ESP_LOGW(TAG, "%s", (msg)); return false; } while (0)

bool maps_start(const char *path)
{
    if (s_running)
        return true;

    s_error = NULL;

    if (path && *path)
    {
        strncpy(s_path, path, sizeof(s_path) - 1);
        s_path[sizeof(s_path) - 1] = '\0';
    }
    else
    {
        snprintf(s_path, sizeof(s_path), "%s",
                 sd_media(MAPS_PATH_DEFAULT, MAPS_PATH_FLAT));
    }

    FILE *f = fopen(s_path, "rb");

    if (!f)
        FAIL("no map file");

    uint8_t hdr[MAPS_HEADER_BYTES];

    if (fread(hdr, 1, sizeof(hdr), f) != sizeof(hdr))
    {
        fclose(f);
        FAIL("map header short");
    }

    uint16_t w = 0, h = 0;
    uint32_t want_crc = 0;

    if (!maps_header_parse(hdr, &w, &h, &want_crc))
    {
        fclose(f);
        FAIL("not a map image for this screen");
    }

    const size_t bytes = (size_t)w * h * sizeof(uint16_t);

    if (!s_px)
        s_px = (uint16_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);

    if (!s_px)
    {
        fclose(f);
        FAIL("no psram for the picture");
    }

    const size_t got = fread(s_px, 1, bytes, f);

    fclose(f);

    if (got != bytes)
        FAIL("map image short read");

    /* THE CHECKSUM IS CHECKED BEFORE ANYTHING IS DRAWN. A picture that fails
     * here is garbage in a way a size check cannot see, and drawing it would
     * put noise on the glass with nothing saying where it came from. */
    const uint32_t crc = maps_crc32((const uint8_t *)s_px, bytes);

    if (crc != want_crc)
    {
        ESP_LOGW(TAG, "crc %08lx != %08lx", (unsigned long)crc,
                 (unsigned long)want_crc);
        FAIL("map image checksum");
    }

    s_w = w;
    s_h = h;

    /* Claim before drawing, or the painter repaints the cells it still believes
     * are on the glass and the shell bleeds through the picture -- the fault
     * tama.c records. */
    if (!s_claimed)
    {
        display_pixels_claim(true);
        s_claimed = true;
    }

    /*
     * THE MARGINS GO EMPTY, AND THE BOXES STAY.
     *
     * A map has no margin controls yet, so every slot is blank -- but an empty
     * BOX is the established appearance for a slot with nothing in it, and this
     * port has twice removed words that advertised a duty nobody had written.
     * Leaving the picker's PgUp/PgDn/arrows on the glass beside a map would be
     * exactly that fault, since presses there do nothing once the pixels are
     * claimed (touch.c refuses them).
     *
     * TWO PAINTS, NOT TEN. display_margin_label() repaints the whole column per
     * call, so blanking five slots that way would push 46x417 pixels five times
     * and show as a cascade. display_margin_labels() sets every override under
     * one lock and paints once.
     */
    display_margins_set(DISPLAY_MARGINS_APP);

    {
        static const char *const blank[COL_N] = { " ", " ", " ", " ", " " };

        display_margin_labels(0, blank, COL_N - 1);
        display_margin_labels(1, blank, COL_N - 1);
    }

    for (int i = 0; i < BTN_N; i++)
        display_margin_btn_label(i, " ");

    s_running = true;

    if (!maps_redraw())
    {
        maps_stop();
        FAIL("the panel refused the picture");
    }

    ESP_LOGI(TAG, "showing %s (%dx%d)", s_path, w, h);
    return true;
}

bool maps_redraw(void)
{
    if (!s_px || !s_running)
        return false;

    /* ONE BLIT, ALREADY IN THE RIGHT BYTE ORDER. display_picture_rgb() copies
     * into the framebuffer and flush_rect() swaps each word on the way out, so
     * the buffer holds LOGICAL little-endian RGB565 and the driver swaps it
     * exactly once. This is the opposite contract to display_picture_rows(),
     * whose callback must emit panel order. */
    return display_picture_rgb(NULL, 0, 0, s_px, s_w, s_h);
}

void maps_stop(void)
{
    if (!s_running && !s_claimed)
        return;

    s_running = false;

    if (s_claimed)
    {
        /* Hand the glass back while ownership still makes this the only writer,
         * so the desktop's labels are repainted by a task entitled to draw. */
        display_margins_set(DISPLAY_MARGINS_DESKTOP);
        display_pixels_claim(false);
        s_claimed = false;
    }

    /* The picture is kept in PSRAM between runs on purpose: `maps draw` can put
     * it back without touching the card, which is how a blit fault is told apart
     * from a read fault. maps_free() is what actually releases it. */
}

bool maps_running(void)
{
    return s_running;
}

const char *maps_last_error(void)
{
    return s_error;
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
        printf("running=%d w=%u h=%u path=%s\n",
               maps_running() ? 1 : 0, (unsigned)s_w, (unsigned)s_h, s_path);
        if (s_error)
            printf("last error: %s\n", s_error);
        return 0;
    }

    if (!strcmp(sub, "start"))
    {
        const char *p = argc >= 3 ? argv[2] : NULL;

        if (!maps_start(p))
        {
            printf("did not start: %s\n", s_error ? s_error : "unknown");
            return 1;
        }
        printf("showing %s\n", s_path);
        return 0;
    }

    printf("maps [start [path] | stop | draw | status]\n");
    return 1;
}

static const tdsh_command_t s_maps_cmd = {
    "maps",
    "maps [start [path] | stop | draw | status]",
    "Show a map image on the screen",
    cmd_maps,
    TDSH_CMD_INTERACTIVE,
};

bool maps_register_command(void)
{
    return tdsh_register_command(&s_maps_cmd) == 0;
}
