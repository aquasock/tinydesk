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
#include "slotb.h"
#include "touch.h"

/* The glass, in framebuffer terms: 320x416 at 8x16. */
#define PK_W      DISPLAY_W
#define PK_ROW_H  16
#define PK_TITLE_Y 8
#define PK_LIST_Y  40
#define PK_ROWS    ((DISPLAY_H - PK_LIST_Y - PK_ROW_H - 8) / PK_ROW_H)  /* 22 */

/*
 * THE SCREEN IS SIX BANDS, AND THEY MUST TILE IT EXACTLY.
 *
 * The picker used to open every draw with display_fill(0x0000), which writes all
 * 320x416 of the framebuffer and PUSHES THE WHOLE PANEL -- about 100 ms of black
 * on the glass at the port's own measured cost -- and then painted the title, the
 * 22 rows and the footer back on top of it. Every arrow press therefore cost a
 * black frame followed by the list returning, which is the flash the operator
 * reported.
 *
 * So the black is no longer a separate thing that happens first: it is what the
 * gap bands and the unused row slots contain. Every band carries its final
 * content, no frame exists in which the screen is empty, and a move repaints
 * only the bands a move changes.
 */
#define PK_FOOT_Y (PK_LIST_Y + PK_ROWS * PK_ROW_H)               /* 392 */
#define PK_GAP_TOP  PK_TITLE_Y                                  /*  8 */
#define PK_GAP_MID  (PK_LIST_Y - (PK_TITLE_Y + PK_ROW_H))        /* 16 */
#define PK_GAP_BOT  (DISPLAY_H - (PK_FOOT_Y + PK_ROW_H))        /*  8 */

/* A gap that nothing paints is a row of whatever the last screen left in the
 * framebuffer, and with no fill left to cover for it that would be a permanent
 * stripe rather than a flicker. */
_Static_assert(PK_GAP_TOP + PK_ROW_H + PK_GAP_MID + PK_ROWS * PK_ROW_H +
               PK_ROW_H + PK_GAP_BOT == DISPLAY_H,
               "picker bands must tile the framebuffer");

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
static bool s_prev_up, s_prev_down, s_prev_enter, s_prev_back;
static bool s_prev_pgup, s_prev_pgdn;
static int s_prev_row = -1;

/*
 * THE RIGHT COLUMN, BY NAME: two page buttons above the two arrows.
 *
 * PgUp and PgDn move the highlight by a whole screen; the arrows move it by one row.
 * They sit directly above the arrows because that is a pair above a pair, so the
 * column reads page-up, page-down, up, down from the top.
 *
 * WHY THEY SAY "PgUp" AND "PgDn" RATHER THAN "Pg" AND AN ARROW GLYPH, which is the
 * obvious way to write it: THIS COLUMN ROTATES ITS LABELS. `label_is_glyph()` exempts
 * a label from the rotation only when it is EXACTLY ONE multibyte character, so a
 * mixed string like "Pg" plus U+2191 is rotated with everything else -- and rotating
 * an arrow does not stand it on end, it turns it into a different arrow. The table in
 * display.c measured this: a transposed U+2191 comes out pointing LEFT. So a page
 * button written that way would have shown a left arrow, which is worse than a word.
 *
 * The words are drawn rotated like every other column label, four characters standing
 * on end, which is the same treatment "Select" and "Start" get on the Game Boy side.
 */
#define SLOT_BLANK 0            /* top: drawn, inert                       */
#define SLOT_PGUP  1
#define SLOT_PGDN  2
#define SLOT_UP    3
#define SLOT_DOWN  4

/* Whether this run draws the bottom strip's "Up" bar, and where to say so when
 * it is pressed. Only the cartridge navigator has a parent to climb to; the app
 * menu and the console chooser are single screens. */
static bool s_can_up;
static bool *s_up_out;

/* One row's worth of pixels, PSRAM: 10 KB doubled per draw is not a budget to
 * spend from this board's internal DRAM. */
static uint16_t *s_rowbuf;

/* Tagged "GBC name" / "GB  name" labels, in PSRAM -- see the note where they are
 * built. Internal DRAM is this board's scarce resource. */
static char *s_tagged;


/*
 * EVERY FAILURE IN HERE IS RECORDED, NOT ONLY SHOWN.
 *
 * The picker can refuse for five reasons and only one of them has a name the
 * wearer can act on; the rest reach the glass as "did not start / no ROM?", which
 * is a guess. slotb_record() puts the reason and the path in flash where a host
 * can read them back after a reset, which is the only way to find out why a
 * cartridge that is visibly on the card would not open.
 */
static void pick_fail(const char *why, const char *path)
{
    slotb_record(why, path);
}

/* ---------------------------------------------------------------- scanning */

static bool is_rom(const char *name, const pk_cfg_t *cfg)
{
    const char *dot = strrchr(name, '.');

    if (!dot || name[0] == '.')
        return false;

    for (int i = 0; cfg->exts[i]; i++)
        if (!strcasecmp(dot, cfg->exts[i]))
            return true;

    return false;
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
 * a cartridge. The bottom strip's "Up" bar climbs out of a folder. That gives
 * the whole library in two or three taps and needs no more memory than one
 * screen already used.
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
 * Fill s_ent from one directory: its folders, then its cartridges.
 *
 * THERE IS NO `..` ROW ANY MORE. Climbing used to be a row at the top of the
 * list, spending one of the 22 lines on screen to say something the bottom
 * strip says for nothing: the third bar is "Up" whenever there is a parent to
 * climb to, and blank where there is not. A row that exists only to be left is
 * one more line to read past on every screen of a 1,143-cartridge library.
 */
static int list_dir(const char *dir, const pk_cfg_t *cfg)
{
    s_ent_n = 0;
    s_truncated = false;

    DIR *d = opendir(dir);

    if (!d)
        return 0;

    struct dirent *e;

    while ((e = readdir(d)) != NULL)
    {
        if (e->d_name[0] == '.')
            continue;                       /* ".", ".." and dotfiles */

        const bool rom = is_rom(e->d_name, cfg);
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

    if (s_ent_n > 1)
        qsort(s_ent, (size_t)s_ent_n, sizeof(s_ent[0]), by_name);

    for (int i = 0; i < s_ent_n; i++)
    {
        if (s_ent[i].is_dir)
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
            /*
             * NO CONSOLE PREFIX ON A CARTRIDGE ROW. The operator's correction,
             * and the prefix had stopped earning its four characters: the list
             * is entered through the console it belongs to, so "GBC " on every
             * row repeated what the navigation had already said. Four characters
             * of a 38-character row is 10.5 per cent of the line handed back to
             * the filename, which is the only part of these rows that varies.
             *
             * What identifies the console now: the branch that was chosen to get
             * here, and the extension at the end of the name. The one case the
             * prefix would still have flagged -- a .gb cartridge sitting inside
             * the colour tree -- is a cartridge the firmware runs as GB_HW_CGB
             * because it lives there, so the row was describing the directory
             * rather than the file and was not adding information anyway.
             */
            s_labels[i] = s_ent[i].name;
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
/* Game Boy offers BOTH trees, because a card can carry either or both and the
 * firmware decides the machine type from where the cartridge lives. Pokemon Mini
 * has one tree; with a single root the console list is skipped entirely, which
 * the navigator already does. */
static const pk_root_t k_gb_roots[] = {
    { "Game Boy Color", "/sd/media/gbc" },
    { "Game Boy",       "/sd/media/gb"  },
};

static const pk_root_t k_pm_roots[] = {
    { "Pokemon Mini",   "/sd/media/pokemini" },
};

static const char *const k_gb_exts[] = { ".gbc", ".gb", NULL };
static const char *const k_pm_exts[] = { ".min", NULL };

const pk_cfg_t pk_gb = {
    .console = "Game Boy",
    .exts    = k_gb_exts,
    .roots   = k_gb_roots,
    .n_roots = (int)(sizeof(k_gb_roots) / sizeof(k_gb_roots[0])),
};

const pk_cfg_t pk_pm = {
    .console = "Pokemon Mini",
    .exts    = k_pm_exts,
    .roots   = k_pm_roots,
    .n_roots = (int)(sizeof(k_pm_roots) / sizeof(k_pm_roots[0])),
};

/*
 * THE CONSOLE LIST HAS ITS OWN LABEL STORAGE, AND IT HAS TO.
 *
 * `s_labels` belongs to the directory listing: list_dir() overwrites entries
 * 0..n-1 on every pass. The console list was borrowing that array, so on the
 * second trip -- after Up at a console's root -- it drew two CARTRIDGE OR FOLDER
 * names left over from the tree that had just been left, while Enter went to
 * whichever console the row really named. The operator hit it directly: back out
 * of Game Boy and the screen shows names that are not consoles, and Enter
 * appears to open the wrong one.
 *
 * Two arrays, because they are two different lists and sharing one storage was
 * the whole fault. Declared here because PK_MAX_ROOTS is what sizes them and
 * the console tables are defined above.
 */
static char s_root_lbl[PK_MAX_ROOTS][24];
static const char *s_root_ptr[PK_MAX_ROOTS];

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

/* A band of background. Shorter than a row for the two gaps, which is why the
 * height is a parameter rather than PK_ROW_H. */
static void paint_black(int y, int h)
{
    if (h <= 0)
        return;

    if (h > PK_ROW_H)
        h = PK_ROW_H;

    for (int i = 0; i < PK_W * h; i++)
        s_rowbuf[i] = 0x0000;

    display_picture_rgb(NULL, 0, y, s_rowbuf, PK_W, h);
}

static void draw_title(void)
{
    row_pixels(s_rowbuf, s_title, false);
    for (int i = 0; i < PK_W * PK_ROW_H; i++)      /* title in green */
        if (s_rowbuf[i])
            s_rowbuf[i] = 0x07E0;
    display_picture_rgb(NULL, 0, PK_TITLE_Y, s_rowbuf, PK_W, PK_ROW_H);
}

/* Footer names the BUTTONS, because that is what a person can find: the arrows
 * are drawn on the glass beside the list, Enter and Up are in the strip. Up is
 * named only where it exists -- a footer that advertises a blanked bar is the
 * label/duty disagreement edges.h was written against. */
static void draw_footer(void)
{
    char foot[64];

    snprintf(foot, sizeof(foot), "arrows:move enter:start %s%d/%d",
             s_can_up ? "up:back " : "",
             s_sel >= 0 ? s_sel + 1 : 0, s_item_n);

    row_pixels(s_rowbuf, foot, false);
    for (int i = 0; i < PK_W * PK_ROW_H; i++)
        if (s_rowbuf[i])
            s_rowbuf[i] = 0x7BEF;
    display_picture_rgb(NULL, 0, PK_FOOT_Y, s_rowbuf, PK_W, PK_ROW_H);
}

/* One screen row of the list, by SLOT (0..PK_ROWS-1) rather than by index: a
 * slot past the end of the list is painted empty, which is what the fill used to
 * do for the whole screen. */
static void draw_row_slot(int slot)
{
    const int idx = s_page * PK_ROWS + slot;

    if (idx >= 0 && idx < s_item_n)
        draw_row(slot, s_items[idx], idx == s_sel);
    else
        paint_black(PK_LIST_Y + slot * PK_ROW_H, PK_ROW_H);
}

static void draw_body(void)
{
    for (int i = 0; i < PK_ROWS; i++)
        draw_row_slot(i);
}

/*
 * A FULL PAINT, AND IT NO LONGER STARTS WITH A BLACK FRAME.
 *
 * The bands go down top to bottom with their real content, so the glass never
 * shows an empty screen between two states. Used when the list appears and when
 * the page changes; a move inside a page uses redraw_sel().
 */
static void draw_screen(void)
{
    paint_black(0, PK_GAP_TOP);
    draw_title();
    paint_black(PK_TITLE_Y + PK_ROW_H, PK_GAP_MID);
    draw_body();
    draw_footer();
    paint_black(PK_FOOT_Y + PK_ROW_H, PK_GAP_BOT);
}

/*
 * WHAT A MOVE ACTUALLY CHANGES: two rows and the counter.
 *
 * The row that lost the highlight, the row that gained it, and the footer --
 * whose n/m changes on every step. Three bands is about 11 ms of panel time
 * against the ~190 ms a fill-plus-full-repaint cost, and none of it is blank.
 * A page change really does change every row, so that case falls back to the
 * body and the footer.
 */
static void redraw_sel(int old_sel, int old_page)
{
    if (s_page != old_page)
    {
        draw_body();
        draw_footer();
        return;
    }

    if (old_sel >= 0 && old_sel != s_sel)
        draw_row_slot(old_sel - s_page * PK_ROWS);

    if (s_sel >= 0)
        draw_row_slot(s_sel - s_page * PK_ROWS);

    draw_footer();
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
 * The right column carries U+2191 and U+2193 at indices 3 and 4, and the bottom
 * strip carries Enter at index 1 and Ctrl+Q at index 2, so the furniture for this
 * existed and was merely unused. Ctrl+Q is the window manager's close key and
 * means nothing in slot B, which has no shell and no window to close; it is
 * relabelled "Up" and made the way out of a folder.
 *
 * EVERY OTHER SLOT IS DRAWN AND LEFT BLANK, which is the operator's correction
 * after using it: pulled chrome left a frame with bites out of it, so the list
 * looked broken rather than complete. A blank box is not a promise to act --
 * this list owns the glass, so its presses are classified in list_touch() below,
 * where an unhandled slot falls through and neither acts nor buzzes. The
 * key-sending path in touch.c never runs while the pixels are claimed.
 */
static void setup_margins(bool can_up)
{
    for (int i = 0; i < COL_N; i++)
    {
        display_margin_chrome(0, i, true);
        display_margin_label(0, i, " ");
    }

    display_margin_chrome(1, SLOT_PGUP, true);
    display_margin_label(1, SLOT_PGUP, "PgUp");
    display_margin_chrome(1, SLOT_PGDN, true);
    display_margin_label(1, SLOT_PGDN, "PgDn");

    display_margin_chrome(1, SLOT_UP, true);
    display_margin_label(1, SLOT_UP, "\xe2\x86\x91");   /* U+2191 up   */
    display_margin_chrome(1, SLOT_DOWN, true);
    display_margin_label(1, SLOT_DOWN, "\xe2\x86\x93");   /* U+2193 down */

    display_margin_chrome(1, SLOT_BLANK, true);
    display_margin_label(1, SLOT_BLANK, " ");

    display_margin_chrome(2, 0, true);
    display_margin_btn_label(0, " ");
    display_margin_chrome(2, 1, true);
    display_margin_btn_label(1, "Enter");

    /* THE THIRD BAR IS UP WHERE THERE IS SOMEWHERE TO CLIMB TO, AND BLANK WHERE
     * THERE IS NOT -- a drawn box with no word on it, which is what the other
     * blank slots are. */
    display_margin_chrome(2, 2, true);
    display_margin_btn_label(2, can_up ? "Up" : " ");

    /* The overrides only take effect in APP mode; in desktop mode display.c
     * deliberately ignores them so an app cannot paint over the desktop's own
     * column. */
    display_margins_set(DISPLAY_MARGINS_APP);
}

static void move_sel(int delta)
{
    if (s_item_n == 0)
        return;

    const int old_sel = s_sel;
    const int old_page = s_page;

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

    redraw_sel(old_sel, old_page);
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
    bool up = false, down = false, enter = false, back = false;
    bool pgup = false, pgdn = false;
    int row = -1;

    if (f && f->n > 0)
    {
        for (int i = 0; i < f->n && i < TOUCH_POINTS; i++)
        {
            if (f->p[i].region == EDGE_RIGHT)
            {
                /* BY NAME, so a slot that moves cannot leave a press on the old
                 * index sending the arrow it used to be. */
                if (f->p[i].index == SLOT_PGUP)
                    pgup = true;
                else if (f->p[i].index == SLOT_PGDN)
                    pgdn = true;
                else if (f->p[i].index == SLOT_UP)
                    up = true;
                else if (f->p[i].index == SLOT_DOWN)
                    down = true;
                continue;
            }

            if (f->p[i].region == EDGE_BOTTOM)
            {
                if (f->p[i].index == 1)
                    enter = true;
                else if (f->p[i].index == 2 && s_can_up)
                    back = true;
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
    const bool fire_pgup = pgup && !s_prev_pgup;
    const bool fire_pgdn = pgdn && !s_prev_pgdn;
    const bool fire_enter = enter && !s_prev_enter;
    const bool fire_back = back && !s_prev_back;
    const bool fire_row = row >= 0 && row != s_prev_row;

    s_prev_up = up;
    s_prev_down = down;
    s_prev_pgup = pgup;
    s_prev_pgdn = pgdn;
    s_prev_enter = enter;
    s_prev_back = back;
    s_prev_row = row;

    /* PAGE BEFORE ROW: if a frame somehow carries both, the coarser move wins and the
     * single row of an arrow is not silently lost -- a page press is the deliberate one,
     * and an arrow press that arrives with it is a finger resting on the column. */
    if (fire_pgup)
    {
        move_sel(-PK_ROWS);
        return true;
    }
    if (fire_pgdn)
    {
        move_sel(+PK_ROWS);
        return true;
    }
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
    if (fire_back)
    {
        /* Not a choice, so the list ends without a selection: the caller reads
         * this as "climb one level" and re-lists the parent. */
        *s_up_out = true;
        s_done = true;
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
            /* A tap is on a row that is already on screen, so the page cannot
             * have changed and this is the same two-band repaint a move gets. */
            const int old_sel = s_sel;

            s_sel = row;
            redraw_sel(old_sel, s_page);
        }
        return true;
    }

    return up || down || enter || back || row >= 0;
}

/* -------------------------------------------------------------------- entry */

bool gbpick_list_run(const char *title, const char *const *labels, int n,
                     bool can_go_up, int sel_init, int *sel_out, bool *up_out,
                     const volatile bool *abort)
{
    if (!labels || n <= 0)
        return false;

    /* A programming error, not a runtime condition: asking for an Up bar with
     * nowhere to report the press would draw a button that does nothing. */
    if (can_go_up && !up_out)
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
    s_up_out = up_out;
    s_can_up = can_go_up;
    s_done = false;
    s_sel = (sel_init >= 0 && sel_init < n) ? sel_init : -1;
    s_page = s_sel >= 0 ? s_sel / PK_ROWS : 0;
    s_prev_up = s_prev_down = s_prev_enter = s_prev_back = false;
    s_prev_pgup = s_prev_pgdn = false;
    s_prev_row = -1;
    snprintf(s_title, sizeof(s_title), "%s", title ? title : "");

    if (sel_out)
        *sel_out = -1;
    if (up_out)
        *up_out = false;

    /* REGISTERED ONCE, NOT PER CALL. Slot B shows this list again every time an
     * app exits, and `touch_add_app_touch()` APPENDS -- so registering on each
     * launch would fill the consumer table and leave later lists deaf. The
     * consumer itself is stateless between lists, so one registration serves
     * every one. */
    if (!s_registered)
    {
        if (!touch_add_app_touch(list_touch, NULL))
        {
            pick_fail("touch did not register", title);
            gbpick_message("Touch did not register", "");
            return false;
        }
        s_registered = true;
    }

    setup_margins(s_can_up);
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
    return gbpick_run_cfg(&pk_gb, out, cap, abort);
}

bool gbpick_run_cfg(const pk_cfg_t *cfg, char *out, size_t cap,
                    const volatile bool *abort)
{
    if (!cfg || !cfg->exts || !cfg->roots || cfg->n_roots <= 0)
        return false;

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
        pick_fail("out of memory", NULL);
        gbpick_message("Out of memory", "cannot build the list");
        return false;
    }

    /* THE ROOT LIST IS THE CONSOLES THAT ARE ACTUALLY THERE, so a card with only
     * Game Boy does not offer a Colour branch that opens on nothing. `root_ix`
     * maps a row of this list back to cfg->roots: the rows are compacted, so the
     * row number is NOT the console index when the first directory is missing --
     * the previous version used the row number for the table and would have opened
     * `/sd/media/gbc` for a card that only had Game Boy. */
    int root_n = 0;
    int root_ix[PK_MAX_ROOTS];

    for (int i = 0; i < cfg->n_roots && i < PK_MAX_ROOTS; i++)
    {
        DIR *d = opendir(cfg->roots[i].path);

        if (!d)
            continue;

        closedir(d);
        root_ix[root_n] = i;
        snprintf(s_root_lbl[root_n], sizeof(s_root_lbl[0]), "%s",
                 cfg->roots[i].label);
        s_root_ptr[root_n] = s_root_lbl[root_n];
        root_n++;
    }

    if (root_n == 0)
    {
        pick_fail("no console directories", cfg->roots[0].path);
        gbpick_message("No cartridges found", "check the card is seated");
        return false;
    }

    char root_base[PK_PATH_MAX];
    char console[24];
    int cur = -1;                        /* which console, as a root row */

    /*
     * TWO LEVELS, AND THE CONSOLE LIST IS ABOVE THE CARTDRIDGE TREE.
     *
     * It began as a screen shown once, on the way in, with the console's own
     * directory as the top of the tree and no way back to the other console. The
     * operator asked for Up to work there too, which makes it the top of the
     * navigation rather than a preamble: Up at the console's root now returns to
     * the console list, with the console you were in already highlighted, so
     * switching between Game Boy and Game Boy Color is a press and not a relaunch.
     *
     * With ONE console on the card there is nothing above the root, the console
     * list is skipped, and Up stays blank there -- the same rule as before.
     */
    for (;;)
    {
        /* One console only: do not make the wearer choose from a list of one. */
        if (root_n == 1)
        {
            cur = 0;
        }
        else
        {
            int sel = -1;

            char pick_title[48];

            snprintf(pick_title, sizeof(pick_title), "%s - select a console",
                     cfg->console);

            if (!gbpick_list_run(pick_title, s_root_ptr,
                                 root_n, false, cur, &sel, NULL, abort))
                return false;

            if (sel < 0 || sel >= root_n)
                return false;

            cur = sel;
        }

        snprintf(s_cwd, PK_PATH_MAX, "%s", cfg->roots[root_ix[cur]].path);
        /* `s_root_lbl` and `console` are both 24 bytes, so the copy cannot
         * truncate -- but the compiler only knows the SOURCE is at most 24 and
         * the destination exactly 24, so the width is stated. */
        snprintf(console, sizeof(console), "%.*s", (int)sizeof(console) - 1,
                 s_root_lbl[cur]);
        snprintf(root_base, sizeof(root_base), "%s", s_cwd);

        /* THE NAVIGATION LOOP. Each pass lists the current directory and acts on
         * one choice: a folder descends, Up climbs, a cartridge ends the picker. */
        for (;;)
        {
            /*
             * THE LIST IS ONLY EVER BUILT UNDER root_base, AND THAT IS A GUARD
             * RATHER THAN A COMMENT.
             *
             * Up climbs with path_up(), and path_up() will happily walk out of
             * the console's directory into /sd, / and beyond if anything ever
             * makes at_root() answer false when s_cwd is already at the root --
             * and a picker browsing /sd and / is a picker that lists folders the
             * wearer has no business in and can build paths nothing can open.
             * The comparison here makes that unreachable instead of unlikely: a
             * path outside the tree is snapped back to the tree.
             */
            const size_t rb = strlen(root_base);
            if (strncmp(s_cwd, root_base, rb) != 0 ||
                (s_cwd[rb] != '\0' && s_cwd[rb] != '/'))
                snprintf(s_cwd, PK_PATH_MAX, "%s", root_base);

            const bool at_root = (strcmp(s_cwd, root_base) == 0);

            if (list_dir(s_cwd, cfg) == 0)
            {
                pick_fail("empty directory", s_cwd);
                gbpick_message("No cartridges here", base_name(s_cwd));
                return false;
            }

            /*
             * THE TITLE CARRIES THE CONSOLE, ONCE. The cartridge rows used to
             * repeat "GBC " or "GB  " four characters deep on every line, which
             * was the operator's complaint: the console is already known from the
             * branch that was chosen, and repeating it 22 times per screen spends
             * filename space on something the wearer selected a moment ago.
             * Stated once in the title it keeps the context and costs the rows
             * nothing.
             *
             * The title shows where in the tree the list is: the console alone at
             * its root, then "Game Boy Color - retail", then "Game Boy Color - A".
             */
            char where[48];

            if (at_root)
                snprintf(where, sizeof(where), "%s", console);
            else
                snprintf(where, sizeof(where), "%s - %s", console,
                         base_name(s_cwd));

            /* Up is drawn wherever it leads somewhere: out of a folder, or --
             * at the console's root -- back to the console list, when there is
             * more than one console to return to. */
            const bool can_up = !at_root || root_n > 1;

            int sel = -1;
            bool climb = false;

            if (!gbpick_list_run(where, s_labels, s_ent_n, can_up, -1, &sel,
                                 &climb, abort))
            {
                if (!climb)
                    return false;           /* Pusher A: out of the picker */

                if (!at_root)
                {
                    path_up(s_cwd);
                    continue;
                }

                break;                      /* Up at the root: console list */
            }

            if (sel < 0 || sel >= s_ent_n)
                return false;

            pk_ent_t *chosen = &s_ent[sel];

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
                pick_fail("path too long", chosen->name);
                gbpick_message("Path too long", chosen->name);
                return false;
            }

            snprintf(out, cap, "%s/%s", s_cwd, chosen->name);
            return out[0] != '\0';
        }
    }
}
