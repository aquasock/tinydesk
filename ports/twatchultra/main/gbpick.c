/*
 * gbpick.c - the cartridge picker, drawn on the glass.
 *
 * The header says why it exists. This file is the scan, the drawing and the
 * touch handling.
 *
 * IT IS ALSO THE FAILURE REPORT NOW. Slot B has no console (CONFIG_ESP_CONSOLE_NONE)
 * and a picture kills the USB mirror (DSP-003), so "no cartridge found" and "the
 * card did not mount" used to be undiagnosable without a flash marker. Text on
 * the glass says the same thing to the person holding the watch, which is where
 * a failure needs to be legible.
 */

#include <dirent.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display.h"
#include "edges.h"
#include "font8x16.h"
#include "gbpick.h"
#include "touch.h"

/* The glass, in framebuffer terms: 320x416 at 8x16. */
#define PK_W      DISPLAY_W
#define PK_ROW_H  16
#define PK_TITLE_Y 8
#define PK_LIST_Y  40
#define PK_ROWS    ((DISPLAY_H - PK_LIST_Y - PK_ROW_H - 8) / PK_ROW_H)  /* 22 */

#define PK_MAX_ROM  64
#define PK_NAME_MAX 96

static char s_roms[PK_MAX_ROM][PK_NAME_MAX];
static int s_rom_n;
static int s_sel = -1;        /* index of the tapped row, -1 for none */

/*
 * EDGE STATE, BECAUSE THE CONTROLLER RE-REPORTS A HELD FINGER.
 *
 * The CST9217 re-sends its report about every 66 ms while a finger stays down --
 * the same fact that makes the haptic fire on the press transition only, and
 * recorded with it. The picker acted on every report, so one tap on the down
 * arrow moved the highlight five to eight rows: the operator's own count, and
 * the first thing they said about it.
 *
 * The same fault was also in the tap-to-commit path and would have been worse
 * there: "tap a row, tap it again" is two presses, and a single held tap looks
 * like two reports 66 ms apart, so ONE tap could have selected and started a
 * cartridge. Nobody had hit it yet only because the arrows were unusable.
 *
 * So each control is acted on when it goes DOWN, never while it stays down, and
 * a row is acted on only by a finger that was not on that row in the previous
 * report. A release arrives as a frame with no points, which clears all of it.
 */
static bool s_prev_up, s_prev_down, s_prev_enter;
static int s_prev_row = -1;
static int s_page;
static volatile bool s_done;
static char *s_out;
static size_t s_out_cap;

/* One row's worth of pixels, PSRAM because 10 KB doubled per draw is not a
 * budget to spend from internal DRAM on this board. */
static uint16_t *s_rowbuf;

/* ---------------------------------------------------------------- scanning */

static bool is_rom(const char *name)
{
    const char *dot = strrchr(name, '.');

    if (!dot || name[0] == '.')
        return false;
    return !strcasecmp(dot, ".gbc") || !strcasecmp(dot, ".gb");
}

static void add_rom(const char *path)
{
    if (s_rom_n >= PK_MAX_ROM || strlen(path) >= PK_NAME_MAX)
        return;

    /* Skip duplicates: the fallback scan of /sd can reach a directory the first
     * pass already walked. */
    for (int i = 0; i < s_rom_n; i++)
        if (!strcmp(s_roms[i], path))
            return;

    snprintf(s_roms[s_rom_n], PK_NAME_MAX, "%s", path);
    s_rom_n++;
}

/* Read one directory for cartridges, and recurse into subdirectories to `depth`.
 * Depth 0 reads only the named directory. */
static void scan_dir(const char *dir, int depth)
{
    DIR *d = opendir(dir);

    if (!d)
        return;

    struct dirent *e;
    while ((e = readdir(d)) != NULL)
    {
        if (e->d_name[0] == '.')
            continue;

        char path[PK_NAME_MAX];
        int n = snprintf(path, sizeof(path), "%s/%s", dir, e->d_name);

        if (n < 0 || (size_t)n >= sizeof(path))
            continue;

        if (is_rom(e->d_name))
        {
            add_rom(path);
            continue;
        }

        /* d_type is not reliable on every VFS, so a directory is confirmed by
         * opening it rather than trusted from the entry. */
        if (depth > 0)
        {
            DIR *probe = opendir(path);

            if (probe)
            {
                closedir(probe);
                scan_dir(path, depth - 1);
            }
        }
    }

    closedir(d);
}

static int by_name(const void *a, const void *b)
{
    return strcasecmp((const char *)a, (const char *)b);
}

static int scan_roms(void)
{
    s_rom_n = 0;

    /*
     * THE ORDER MATTERS. /sd/media/gbc is where this project's cards keep
     * cartridges (GB_ROM_DEFAULT lives under it), so it is walked two levels
     * deep -- the cartridge sits in a `homebrew` subdirectory, which is one.
     * Only if that turns up nothing is the card's root walked, which catches a
     * card organised by hand.
     */
    scan_dir("/sd/media/gbc", 2);

    if (s_rom_n == 0)
        scan_dir("/sd", 2);

    if (s_rom_n > 1)
        qsort(s_roms, (size_t)s_rom_n, sizeof(s_roms[0]), by_name);

    return s_rom_n;
}

/* ----------------------------------------------------------------- drawing */

static void row_pixels(uint16_t *dst, const char *s, bool sel)
{
    const uint16_t bg = sel ? 0x0320 : 0x0000;
    const uint16_t fg = sel ? 0xFFFF : 0xC618;

    for (int i = 0; i < PK_W * PK_ROW_H; i++)
        dst[i] = bg;

    int x = 8;

    for (const char *p = s; *p && x + 8 <= PK_W - 8; p++)
    {
        const uint8_t *g = font8x16_glyph((uint16_t)(unsigned char)*p);

        if (g)
        {
            for (int r = 0; r < PK_ROW_H; r++)
            {
                const uint8_t bits = g[r];

                for (int c = 0; c < 8; c++)
                    if (bits & (0x80 >> c))
                        dst[r * PK_W + x + c] = fg;
            }
        }

        x += 8;
    }
}

static void draw_row(int row, const char *s, bool sel)
{
    row_pixels(s_rowbuf, s, sel);
    display_picture_rgb(NULL, 0, PK_LIST_Y + row * PK_ROW_H, s_rowbuf, PK_W, PK_ROW_H);
}

static const char *base_name(const char *path)
{
    const char *slash = strrchr(path, '/');

    return slash ? slash + 1 : path;
}

static void draw_screen(void)
{
    display_fill(0x0000);

    row_pixels(s_rowbuf, "Game Boy - select a cartridge", false);
    /* Title in green, centred by the same 8-px cell the list uses. */
    for (int i = 0; i < PK_W * PK_ROW_H; i++)
        if (s_rowbuf[i])
            s_rowbuf[i] = 0x07E0;
    display_picture_rgb(NULL, 0, PK_TITLE_Y, s_rowbuf, PK_W, PK_ROW_H);

    const int first = s_page * PK_ROWS;

    for (int i = 0; i < PK_ROWS; i++)
    {
        const int idx = first + i;

        if (idx >= s_rom_n)
            break;

        draw_row(i, base_name(s_roms[idx]), idx == s_sel);
    }

    /* Footer. It names the BUTTONS, because that is what a person can find
     * without being told: the arrows are drawn on the glass beside the list and
     * Enter is in the bottom strip. The row count is here so a long list does not
     * look truncated. */
    char foot[64];

    snprintf(foot, sizeof(foot), "arrows: move  enter: start  %d/%d",
             s_sel >= 0 ? s_sel + 1 : 0, s_rom_n);

    row_pixels(s_rowbuf, foot, false);
    for (int i = 0; i < PK_W * PK_ROW_H; i++)
        if (s_rowbuf[i])
            s_rowbuf[i] = 0x7BEF;
    display_picture_rgb(NULL, 0, PK_LIST_Y + PK_ROWS * PK_ROW_H, s_rowbuf, PK_W, PK_ROW_H);
}

/*
 * A message covering the whole picture, for the case where there is nothing to
 * tap. Kept here because the caller has no other way to speak.
 */
void gbpick_message(const char *line1, const char *line2)
{
    if (!s_rowbuf)
        return;

    display_fill(0x0000);

    row_pixels(s_rowbuf, line1 ? line1 : "", false);
    for (int i = 0; i < PK_W * PK_ROW_H; i++)
        if (s_rowbuf[i])
            s_rowbuf[i] = 0xF800;      /* red: something went wrong */
    display_picture_rgb(NULL, 0, DISPLAY_H / 2 - PK_ROW_H, s_rowbuf, PK_W, PK_ROW_H);

    if (line2 && line2[0])
    {
        row_pixels(s_rowbuf, line2, false);
        display_picture_rgb(NULL, 0, DISPLAY_H / 2 + PK_ROW_H, s_rowbuf, PK_W, PK_ROW_H);
    }
}

/* ------------------------------------------------------------------- touch */

/*
 * THE MARGIN COLUMNS ARE THE WAY A LIST IS DRIVEN, and this is the operator's
 * own correction after trying to use the picker: a finger cannot reliably hit a
 * 16-row band, and a list you can only tap is a list you scroll by accident.
 *
 * The right column already carries U+2191 and U+2193 at indices 3 and 4, and the
 * bottom strip already carries Enter at index 1, so the furniture for this
 * existed and was merely unused. setup_margins() makes the glass say so and
 * blanks the other ten buttons, which do nothing here -- an F10 button that
 * sends nothing is the label/duty disagreement edges.h was written to prevent.
 */
static void setup_margins(void)
{
    /* The left column is the app's five verbs on the desktop; none of them apply
     * to a list, so all five go. */
    for (int i = 0; i < COL_N; i++)
    {
        display_margin_chrome(0, i, false);
        display_margin_label(0, i, " ");
    }

    /* Right column: the reserved slot, the left arrow and the right arrow have
     * no duty; the up and down arrows do. */
    for (int i = 0; i <= 2; i++)
    {
        display_margin_chrome(1, i, false);
        display_margin_label(1, i, " ");
    }

    display_margin_chrome(1, 3, true);
    display_margin_label(1, 3, "\xe2\x86\x91");   /* U+2191 up   */
    display_margin_chrome(1, 4, true);
    display_margin_label(1, 4, "\xe2\x86\x93");   /* U+2193 down */

    /* Bottom strip: Enter in the middle, which is where it already is. */
    display_margin_chrome(2, 0, false);
    display_margin_btn_label(0, " ");
    display_margin_chrome(2, 1, true);
    display_margin_btn_label(1, "Enter");
    display_margin_chrome(2, 2, false);
    display_margin_btn_label(2, " ");

    /* The overrides only take effect in APP mode; in desktop mode display.c
     * deliberately ignores them so an app cannot paint over the desktop's own
     * column. */
    display_margins_set(DISPLAY_MARGINS_APP);
}

static void move_sel(int delta)
{
    if (s_rom_n == 0)
        return;

    if (s_sel < 0)
        s_sel = delta > 0 ? 0 : s_rom_n - 1;
    else
        s_sel += delta;

    if (s_sel < 0)
        s_sel = 0;
    if (s_sel >= s_rom_n)
        s_sel = s_rom_n - 1;

    /* The page follows the selection, so the arrows can walk the whole list
     * without a separate paging step. */
    s_page = s_sel / PK_ROWS;
    draw_screen();
}

static void commit_sel(void)
{
    if (s_sel < 0 || s_sel >= s_rom_n)
        return;                     /* nothing highlighted: Enter does nothing */

    snprintf(s_out, s_out_cap, "%s", s_roms[s_sel]);
    s_done = true;
}

static bool picker_touch(void *user, const touch_frame_t *f)
{
    (void)user;

    if (s_done)
        return false;

    /* ONE PASS TO CLASSIFY, THEN ONE PASS TO ACT.
     *
     * Mixing the two would make the result depend on the order the controller
     * happened to list the points in, and a frame can carry two fingers. */
    bool up = false, down = false, enter = false;
    int row = -1;

    if (f && f->n > 0)
    {
        for (int i = 0; i < f->n && i < TOUCH_POINTS; i++)
        {
            if (f->p[i].region == EDGE_RIGHT)
            {
                if (f->p[i].index == 3)
                    up = true;
                else if (f->p[i].index == 4)
                    down = true;
                continue;
            }

            if (f->p[i].region == EDGE_BOTTOM)
            {
                if (f->p[i].index == 1)
                    enter = true;
                continue;
            }

            if (f->p[i].region != EDGE_NONE)
                continue;           /* the left column and the top margin are inert */

            /* Panel coordinates to framebuffer coordinates: the touch controller
             * reports the glass, and the framebuffer is that window offset by the
             * origin. gb.c's own hit test does the same subtraction. */
            const int fx = f->p[i].x - DISPLAY_ORIGIN_X;
            const int fy = f->p[i].y - DISPLAY_ORIGIN_Y;

            if (fx < 0 || fx >= PK_W || fy < PK_LIST_Y)
                continue;

            const int r = (fy - PK_LIST_Y) / PK_ROW_H;
            const int idx = s_page * PK_ROWS + r;

            if (r < PK_ROWS && idx < s_rom_n)
                row = idx;
        }
    }

    /* RISING EDGES ONLY. A held finger changes nothing after its first report. */
    const bool fire_up = up && !s_prev_up;
    const bool fire_down = down && !s_prev_down;
    const bool fire_enter = enter && !s_prev_enter;
    const bool fire_row = row >= 0 && row != s_prev_row;

    s_prev_up = up;
    s_prev_down = down;
    s_prev_enter = enter;
    s_prev_row = row;

    if (fire_up)
    {
        move_sel(-1);
        return true;
    }
    if (fire_down)
    {
        move_sel(+1);
        return true;
    }
    if (fire_enter)
    {
        commit_sel();
        return true;
    }
    if (fire_row)
    {
        /* A finger that lands on a row highlights it; a NEW press on the row
         * that is already highlighted starts it. Because repeats are filtered
         * out above, one physical tap can no longer do both. */
        if (row == s_sel)
            commit_sel();
        else
        {
            s_sel = row;
            draw_screen();
        }
        return true;
    }

    /* A press that was consumed as a repeat still counts as "used" so the touch
     * driver does not buzz for it. */
    return up || down || enter || row >= 0;
}

/* -------------------------------------------------------------------- entry */

bool gbpick_run(char *out, size_t cap, const volatile bool *abort)
{
    if (!out || cap == 0)
        return false;

    s_out = out;
    s_out_cap = cap;
    s_done = false;
    s_sel = -1;
    s_page = 0;
    s_prev_up = s_prev_down = s_prev_enter = false;
    s_prev_row = -1;
    out[0] = '\0';

    if (!s_rowbuf)
    {
        s_rowbuf = heap_caps_malloc((size_t)PK_W * PK_ROW_H * sizeof(uint16_t),
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_rowbuf)
            return false;
    }

    if (scan_roms() == 0)
    {
        gbpick_message("No cartridge found",
                       "looked in /sd/media/gbc");
        return false;
    }

    if (!touch_add_app_touch(picker_touch, NULL))
    {
        gbpick_message("Touch did not register", "");
        return false;
    }

    setup_margins();
    draw_screen();

    while (!s_done)
    {
        if (abort && *abort)
            return false;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return out[0] != '\0';
}
