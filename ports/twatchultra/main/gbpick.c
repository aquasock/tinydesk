/*
 * gbpick.c - a list on the glass, driven by the margin buttons.
 *
 * THE LIST IS THE MECHANISM; THE CARTRIDGE PICKER IS ITS FIRST USER. Slot B
 * needs two lists -- which cartridge, and which app -- and a third copy of the
 * touch handling would be the third chance to reintroduce the fault that made
 * one tap move the highlight five rows (see the touch section below). So the
 * drawing, the margin setup and the edge detection live here once, and
 * gbpick_list_run() takes a title and an array of labels.
 *
 * Touch only ever reached this list through a finger hitting a 16-row band,
 * which the operator could not do reliably; the buttons are the real interface
 * and the tap is the fallback.
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

/*
 * LONG ENOUGH FOR THE REAL NAMES. The default cartridge's filename alone is 110
 * characters -- "Legend of Zelda, The - Link's Awakening DX (USA, Europe) (Rev 2)
 * (SGB Enhanced) (GB Compatible).gbc" -- so a 96-byte field truncated it, and a
 * truncated name builds a path that opens nothing. 160 leaves room for the
 * longest name on the card plus a margin.
 */
#define PK_MAX_ROM  64
#define PK_NAME_MAX 160

/* ------------------------------------------------------------- list state */

static const char *const *s_items;   /* labels to draw                       */
static int s_item_n;
static char s_title[48];
static int s_sel = -1;
static int s_page;
static volatile bool s_done;
static int *s_sel_out;
static bool s_registered;

/*
 * EDGE STATE, BECAUSE THE CONTROLLER RE-REPORTS A HELD FINGER.
 *
 * The CST9217 re-sends its report about every 66 ms while a finger stays down --
 * the same fact that makes the haptic fire on the press transition only, and
 * recorded with it (TCH, HAP-001). Acting on every report moved the highlight
 * five to eight rows per tap, and in the tap-to-commit path ONE held tap looked
 * like two presses, so a single tap could have selected and started a cartridge.
 *
 * So each control is acted on when it goes DOWN, never while it stays down, and
 * a row is acted on only by a finger that was not on that row in the previous
 * report. A release arrives as a frame with no points, which clears all of it.
 */
static bool s_prev_up, s_prev_down, s_prev_enter;
static int s_prev_row = -1;

/* One row's worth of pixels, PSRAM: 10 KB doubled per draw is not a budget to
 * spend from this board's internal DRAM. */
static uint16_t *s_rowbuf;

/* Tagged "GBC name" / "GB  name" labels, in PSRAM -- see the note where they are
 * built. Internal DRAM is this board's scarce resource. */
static char *s_tagged;

/* ---------------------------------------------------------------- scanning */

static bool is_rom(const char *name)
{
    const char *dot = strrchr(name, '.');

    if (!dot || name[0] == '.')
        return false;
    return !strcasecmp(dot, ".gbc") || !strcasecmp(dot, ".gb");
}

/*
 * THE PICKER NAVIGATES THE TREE INSTEAD OF FLATTENING IT, AND IT HAS TO.
 *
 * The card carries over a thousand cartridges, organised into alphabetical
 * buckets of at most 61 because that is what fits one screen of this list. The
 * previous version scanned everything into a single array capped at 64 entries
 * and sorted it, which was fine for a handful of cartridges and would now show
 * 64 of 1,143 and silently hide the rest -- a picker that lies about what is on
 * the card is worse than no picker.
 *
 * So the list is one directory at a time: subdirectories are drawn first with a
 * trailing slash, then cartridges, and Enter descends into a folder or launches
 * a cartridge. `..` goes up. That gives the whole library in two or three taps
 * and needs no more memory than one screen already used.
 */
#define PK_MAX_ENT  64

typedef struct
{
    char name[PK_NAME_MAX];
    bool is_dir;
} pk_ent_t;

/* Both PSRAM: 64 entries of 96 bytes is 6 KB, and internal DRAM is this board's
 * scarcest resource -- the same reason the row buffer lives there. */
static pk_ent_t *s_ent;
#define PK_PATH_MAX  (PK_NAME_MAX * 2)   /* dir + bucket + name, with room */
#define PK_LABEL_MAX (PK_NAME_MAX + 8)   /* "GBC " prefix or a trailing slash */
static char *s_cwd;
static int s_ent_n;
static bool s_truncated;
static const char *s_labels[PK_MAX_ENT];

static void ent_init(void)
{
    if (!s_ent)
        s_ent = heap_caps_calloc(PK_MAX_ENT, sizeof(pk_ent_t),
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_cwd)
        s_cwd = heap_caps_malloc(PK_PATH_MAX,
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

static int by_name(const void *a, const void *b)
{
    const pk_ent_t *x = a, *y = b;

    /* Directories first, then by name: the buckets are the map, and a folder
     * buried between cartridges would be hard to find. */
    if (x->is_dir != y->is_dir)
        return x->is_dir ? -1 : 1;

    return strcasecmp(x->name, y->name);
}

static const char *base_name(const char *path)
{
    const char *slash = strrchr(path, '/');

    return slash ? slash + 1 : path;
}

/*
 * Fill s_ent from one directory. `parent` adds a `..` row at the top, and is
 * false at the root where there is nowhere to go up to.
 */
static int list_dir(const char *dir, bool parent)
{
    s_ent_n = 0;
    s_truncated = false;

    if (parent && s_ent_n < PK_MAX_ENT)
    {
        snprintf(s_ent[s_ent_n].name, PK_NAME_MAX, "..");
        s_ent[s_ent_n].is_dir = true;
        s_ent_n++;
    }

    DIR *d = opendir(dir);

    if (!d)
        return s_ent_n;

    /* `..` is collected with the rest and then sorted to the top by hand, so
     * that it does not compete alphabetically with a real folder. */
    int first_real = s_ent_n;
    struct dirent *e;

    while ((e = readdir(d)) != NULL)
    {
        if (e->d_name[0] == '.')
            continue;                       /* ".", ".." and dotfiles */

        const bool rom = is_rom(e->d_name);
        char probe[PK_PATH_MAX];
        bool dirp = false;

        if (!rom)
        {
            snprintf(probe, sizeof(probe), "%s/%s", dir, e->d_name);
            DIR *sub = opendir(probe);

            if (sub)
            {
                closedir(sub);
                dirp = true;
            }
        }

        if (!rom && !dirp)
            continue;                       /* neither a cartridge nor a folder */

        if (s_ent_n >= PK_MAX_ENT)
        {
            s_truncated = true;
            break;
        }

        /* A name that does not fit is SKIPPED, not truncated: a truncated name
         * builds a path that opens nothing, which reads as a broken cartridge. */
        if (strlen(e->d_name) >= PK_NAME_MAX)
            continue;

        snprintf(s_ent[s_ent_n].name, PK_NAME_MAX, "%s", e->d_name);
        s_ent[s_ent_n].is_dir = dirp;
        s_ent_n++;
    }

    closedir(d);

    const int real = s_ent_n - first_real;

    if (real > 1)
        qsort(s_ent + first_real, (size_t)real, sizeof(s_ent[0]), by_name);

    for (int i = 0; i < s_ent_n; i++)
    {
        if (s_ent[i].is_dir && strcmp(s_ent[i].name, "..") == 0)
        {
            s_labels[i] = "..  (back)";
        }
        else if (s_ent[i].is_dir)
        {
            /* A folder is shown with a trailing slash so the distinction from a
             * cartridge is visible rather than inferred. */
            if (s_tagged)
            {
                char *slot = s_tagged + (size_t)i * PK_LABEL_MAX;
                snprintf(slot, PK_LABEL_MAX, "%s/", s_ent[i].name);
                s_labels[i] = slot;
            }
            else
            {
                s_labels[i] = s_ent[i].name;
            }
        }
        else
        {
            if (s_tagged)
            {
                const bool dmg = strstr(dir, "/media/gb") != NULL &&
                                 strstr(dir, "/media/gbc") == NULL;
                char *slot = s_tagged + (size_t)i * PK_LABEL_MAX;
                snprintf(slot, PK_LABEL_MAX, "%s %s", dmg ? "GB  " : "GBC ",
                         s_ent[i].name);
                s_labels[i] = slot;
            }
            else
            {
                s_labels[i] = s_ent[i].name;
            }
        }
    }

    return s_ent_n;
}

/* Strip the last path component, for `..`. */
static void path_up(char *path)
{
    char *slash = strrchr(path, '/');

    if (!slash)
        return;
    if (slash == path)
    {
        path[1] = '\0';
        return;
    }
    *slash = '\0';
}

/* The console directories the picker offers at its root, filtered to those that
 * exist so a card with only one console does not present an empty branch. */
static const struct
{
    const char *label;
    const char *path;
} k_roots[] = {
    { "Game Boy Color", "/sd/media/gbc" },
    { "Game Boy",       "/sd/media/gb"  },
};

#define PK_ROOT_N ((int)(sizeof(k_roots) / sizeof(k_roots[0])))

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

static void draw_screen(void)
{
    display_fill(0x0000);

    row_pixels(s_rowbuf, s_title, false);
    for (int i = 0; i < PK_W * PK_ROW_H; i++)      /* title in green */
        if (s_rowbuf[i])
            s_rowbuf[i] = 0x07E0;
    display_picture_rgb(NULL, 0, PK_TITLE_Y, s_rowbuf, PK_W, PK_ROW_H);

    const int first = s_page * PK_ROWS;

    for (int i = 0; i < PK_ROWS; i++)
    {
        const int idx = first + i;

        if (idx >= s_item_n)
            break;

        draw_row(i, s_items[idx], idx == s_sel);
    }

    /* Footer names the BUTTONS, because that is what a person can find: the
     * arrows are drawn on the glass beside the list and Enter is in the strip. */
    char foot[64];

    snprintf(foot, sizeof(foot), "arrows: move  enter: start  %d/%d",
             s_sel >= 0 ? s_sel + 1 : 0, s_item_n);

    row_pixels(s_rowbuf, foot, false);
    for (int i = 0; i < PK_W * PK_ROW_H; i++)
        if (s_rowbuf[i])
            s_rowbuf[i] = 0x7BEF;
    display_picture_rgb(NULL, 0, PK_LIST_Y + PK_ROWS * PK_ROW_H, s_rowbuf, PK_W, PK_ROW_H);
}

/*
 * A message covering the whole picture, for the case where there is nothing to
 * tap. The caller has no other way to speak: no console, and a picture kills
 * the USB mirror.
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
 * existed and was merely unused. The other ten buttons are blanked and their
 * chrome removed, because they do nothing here -- an F10 button that sends
 * nothing is the label/duty disagreement edges.h was written to prevent.
 */
static void setup_margins(void)
{
    for (int i = 0; i < COL_N; i++)
    {
        display_margin_chrome(0, i, false);
        display_margin_label(0, i, " ");
    }

    for (int i = 0; i <= 2; i++)
    {
        display_margin_chrome(1, i, false);
        display_margin_label(1, i, " ");
    }

    display_margin_chrome(1, 3, true);
    display_margin_label(1, 3, "\xe2\x86\x91");   /* U+2191 up   */
    display_margin_chrome(1, 4, true);
    display_margin_label(1, 4, "\xe2\x86\x93");   /* U+2193 down */

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
    if (s_item_n == 0)
        return;

    if (s_sel < 0)
        s_sel = delta > 0 ? 0 : s_item_n - 1;
    else
        s_sel += delta;

    if (s_sel < 0)
        s_sel = 0;
    if (s_sel >= s_item_n)
        s_sel = s_item_n - 1;

    /* The page follows the selection, so the arrows walk the whole list without
     * a separate paging step. */
    s_page = s_sel / PK_ROWS;
    draw_screen();
}

static void commit_sel(void)
{
    if (s_sel < 0 || s_sel >= s_item_n)
        return;                     /* nothing highlighted: Enter does nothing */

    if (s_sel_out)
        *s_sel_out = s_sel;
    s_done = true;
}

static bool list_touch(void *user, const touch_frame_t *f)
{
    (void)user;

    if (s_done)
        return false;

    /* ONE PASS TO CLASSIFY, THEN ONE PASS TO ACT: mixing them would make the
     * result depend on the order the controller listed the points in, and a
     * frame can carry two fingers. */
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

            /* Panel coordinates to framebuffer coordinates: the controller
             * reports the glass, and the framebuffer is that window offset by
             * the origin. */
            const int fx = f->p[i].x - DISPLAY_ORIGIN_X;
            const int fy = f->p[i].y - DISPLAY_ORIGIN_Y;

            if (fx < 0 || fx >= PK_W || fy < PK_LIST_Y)
                continue;

            const int r = (fy - PK_LIST_Y) / PK_ROW_H;
            const int idx = s_page * PK_ROWS + r;

            if (r < PK_ROWS && idx < s_item_n)
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
         * that is already highlighted chooses it. Because repeats are filtered
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

    return up || down || enter || row >= 0;
}

/* -------------------------------------------------------------------- entry */

bool gbpick_list_run(const char *title, const char *const *labels, int n,
                     int *sel_out, const volatile bool *abort)
{
    if (!labels || n <= 0)
        return false;

    if (!s_rowbuf)
    {
        s_rowbuf = heap_caps_malloc((size_t)PK_W * PK_ROW_H * sizeof(uint16_t),
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_rowbuf)
            return false;
    }

    s_items = labels;
    s_item_n = n;
    s_sel_out = sel_out;
    s_done = false;
    s_sel = -1;
    s_page = 0;
    s_prev_up = s_prev_down = s_prev_enter = false;
    s_prev_row = -1;
    snprintf(s_title, sizeof(s_title), "%s", title ? title : "");

    if (sel_out)
        *sel_out = -1;

    /* REGISTERED ONCE, NOT PER CALL. Slot B shows this list again every time an
     * app exits, and `touch_add_app_touch()` APPENDS -- so registering on each
     * launch would fill the consumer table and leave later lists deaf. The
     * consumer itself is stateless between lists, so one registration serves
     * every one. */
    if (!s_registered)
    {
        if (!touch_add_app_touch(list_touch, NULL))
        {
            gbpick_message("Touch did not register", "");
            return false;
        }
        s_registered = true;
    }

    setup_margins();
    draw_screen();

    while (!s_done)
    {
        if (abort && *abort)
            return false;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return sel_out && *sel_out >= 0;
}

bool gbpick_run(char *out, size_t cap, const volatile bool *abort)
{
    if (!out || cap == 0)
        return false;

    out[0] = '\0';

    if (!s_rowbuf)
    {
        s_rowbuf = heap_caps_malloc((size_t)PK_W * PK_ROW_H * sizeof(uint16_t),
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }

    ent_init();

    if (!s_tagged)
        s_tagged = heap_caps_malloc((size_t)PK_MAX_ENT * PK_LABEL_MAX,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!s_rowbuf || !s_ent || !s_cwd || !s_tagged)
    {
        gbpick_message("Out of memory", "cannot build the list");
        return false;
    }

    /* THE ROOT LIST IS THE CONSOLES THAT ARE ACTUALLY THERE, so a card with only
     * Game Boy does not offer a Colour branch that opens on nothing. */
    int root_n = 0;
    char root_labels[PK_ROOT_N][24];

    for (int i = 0; i < PK_ROOT_N; i++)
    {
        DIR *d = opendir(k_roots[i].path);

        if (!d)
            continue;

        closedir(d);
        snprintf(root_labels[root_n], sizeof(root_labels[0]), "%s",
                 k_roots[i].label);
        s_labels[root_n] = root_labels[root_n];
        root_n++;
    }

    if (root_n == 0)
    {
        gbpick_message("No cartridges found", "looked in /sd/media");
        return false;
    }

    char root_base[PK_PATH_MAX];

    /* One console only: do not make the wearer choose from a list of one. */
    if (root_n == 1)
    {
        snprintf(s_cwd, PK_PATH_MAX, "%s", k_roots[0].path);
    }
    else
    {
        int sel = -1;

        if (!gbpick_list_run("Game Boy - select a console", s_labels, root_n,
                             &sel, abort))
            return false;

        snprintf(s_cwd, PK_PATH_MAX, "%s", k_roots[sel].path);
    }

    snprintf(root_base, sizeof(root_base), "%s", s_cwd);

    /* THE NAVIGATION LOOP. Each pass lists the current directory and acts on one
     * choice: a folder descends, `..` climbs, a cartridge ends the picker. */
    for (;;)
    {
        /* The console directory is as far up as this picker goes, so its list
         * has no `..` -- there is nowhere useful to climb to. */
        const bool at_root = (strcmp(s_cwd, root_base) == 0);

        if (list_dir(s_cwd, !at_root) == 0)
        {
            gbpick_message("No cartridges here", base_name(s_cwd));
            return false;
        }

        /* The title names where we are, so a bucket list is not mistaken for the
         * whole card. */
        const char *where = at_root ? base_name(s_cwd) : base_name(s_cwd);
        int sel = -1;

        if (!gbpick_list_run(where, s_labels, s_ent_n, &sel, abort))
            return false;                   /* Pusher A: out of the picker */

        if (sel < 0 || sel >= s_ent_n)
            return false;

        pk_ent_t *chosen = &s_ent[sel];

        if (strcmp(chosen->name, "..") == 0)
        {
            path_up(s_cwd);
            continue;
        }

        if (chosen->is_dir)
        {
            char next[PK_PATH_MAX];
            snprintf(next, sizeof(next), "%s/%s", s_cwd, chosen->name);
            snprintf(s_cwd, PK_PATH_MAX, "%s", next);
            continue;
        }

        const int need = snprintf(NULL, 0, "%s/%s", s_cwd, chosen->name);

        if (need < 0 || (size_t)need >= cap)
        {
            gbpick_message("Path too long", chosen->name);
            return false;
        }

        snprintf(out, cap, "%s/%s", s_cwd, chosen->name);
        return out[0] != '\0';
    }
}
