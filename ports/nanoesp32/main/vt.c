/*
 * vt.c - terminal emulator and cell renderer for the ILI9340 panel.
 *
 * Only what TinyDesk actually emits is implemented: cursor addressing,
 * erase-in-display/line, SGR colours (16 and 256), printable text including
 * UTF-8 box drawing, and the cursor-position query the desktop uses to learn
 * the terminal size. Modes (alternate screen, mouse reporting, auto-wrap)
 * are recognised and ignored -- none of them changes what should be drawn.
 *
 * Drawing is diff-based per text row: a row is recomposed into a 320-pixel
 * line buffer one pixel row at a time and blitted, so an idle desktop writes
 * nothing and a small update touches one or two rows.
 */
#include "vt.h"

#include <string.h>

#include "esp_log.h"
#include "font8x16.h"
#include "ili9340.h"

static const char *TAG = "vt";

/* ---- 256-colour palette, computed to RGB565 -----------------------
 *
 * TinyDesk's cell colours are 256-colour indices and anything above 15 is
 * emitted as "38;5;N" / "48;5;N". Mapping those onto the 16 base colours
 * (an earlier attempt) turned every dark grey into a light one and made the
 * whole desktop washed out, so the real palette is built here instead:
 *  0-15   the system colours
 *  16-231 a 6x6x6 cube, levels 0/95/135/175/215/255
 *  232-255 a 24-step grey ramp
 */

static uint16_t s_pal[256];
static bool s_pal_ready;

static uint16_t rgb565(int r, int g, int b)
{
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

static void build_palette(void)
{
    static const int sys[16][3] = {
        {0, 0, 0},       {170, 0, 0},     {0, 170, 0},     {170, 85, 0},
        {0, 0, 170},     {170, 0, 170},   {0, 170, 170},   {170, 170, 170},
        {85, 85, 85},    {255, 85, 85},   {85, 255, 85},   {255, 255, 85},
        {85, 85, 255},   {255, 85, 255},  {85, 255, 255},  {255, 255, 255},
    };
    for (int i = 0; i < 16; i++)
        s_pal[i] = rgb565(sys[i][0], sys[i][1], sys[i][2]);

    static const int lv[6] = {0, 95, 135, 175, 215, 255};
    for (int i = 0; i < 216; i++)
        s_pal[16 + i] = rgb565(lv[i / 36], lv[(i / 6) % 6], lv[i % 6]);

    for (int i = 0; i < 24; i++)
    {
        int v = 8 + i * 10;
        s_pal[232 + i] = rgb565(v, v, v);
    }
    s_pal_ready = true;
}

typedef struct
{
    uint16_t cp;   /* codepoint; 0 is a blank cell */
    uint16_t fg;   /* 256-colour index */
    uint16_t bg;   /* 256-colour index */
} cell_t;

static cell_t s_grid[VT_MAX_ROWS][VT_MAX_COLS];
static cell_t s_shown[VT_MAX_ROWS][VT_MAX_COLS];
static bool s_row_dirty[VT_MAX_ROWS];

static int s_cols = 40, s_rows = 15;
static int s_cx, s_cy;
static uint16_t s_fg = 7, s_bg = 0;   /* 256-colour indices */

/* parser state */
enum { P_GROUND, P_ESC, P_CSI };
static int s_state = P_GROUND;
static int s_params[8];
static int s_nparams;
static int s_cur_param;
static bool s_have_param;
static int s_utf8_left;
static uint32_t s_utf8_acc;

static bool s_size_query;
static int s_query_rows, s_query_cols;

/* ---- glyph lookup ------------------------------------------------- */

static const unsigned char *glyph_for(uint16_t cp)
{
    if (cp >= FONT_FIRST && cp < FONT_FIRST + FONT_COUNT)
        return font8x16[cp - FONT_FIRST];
    for (int i = 0; i < FONT_EXTRA_COUNT; i++)
        if (font_extra[i].cp == cp)
            return font_extra[i].px;
    return NULL;
}

static bool cp_is_blank(uint16_t cp)
{
    return cp == 0 || cp == ' ';
}

/* ---- grid helpers ------------------------------------------------- */

static void mark_dirty(int y)
{
    if (y >= 0 && y < s_rows)
        s_row_dirty[y] = true;
}

static void clear_row(int y, uint8_t bg)
{
    for (int x = 0; x < s_cols; x++)
    {
        s_grid[y][x].cp = 0;
        s_grid[y][x].fg = s_fg;
        s_grid[y][x].bg = bg;
    }
    mark_dirty(y);
}

static void put_cp(uint16_t cp)
{
    if (s_cy < 0 || s_cy >= s_rows)
        return;
    if (s_cx < 0 || s_cx >= s_cols)
        return;
    cell_t *c = &s_grid[s_cy][s_cx];
    if (c->cp != cp || c->fg != s_fg || c->bg != s_bg)
    {
        c->cp = cp;
        c->fg = s_fg;
        c->bg = s_bg;
        mark_dirty(s_cy);
    }
    s_cx++;
}

/* ---- CSI handling ------------------------------------------------- */

static int param(int i, int def)
{
    return (i < s_nparams && s_params[i] > 0) ? s_params[i] : def;
}

static void do_sgr(void)
{
    for (int i = 0; i < s_nparams; i++)
    {
        int p = s_params[i];
        if (p == 0)
        {
            s_fg = 7;
            s_bg = 0;
        }
        else if (p == 1)
        {
            if (s_fg < 8)
                s_fg = (uint16_t)(s_fg + 8);
        }
        else if (p == 22)
        {
            if (s_fg >= 8 && s_fg < 16)
                s_fg = (uint16_t)(s_fg - 8);
        }
        else if (p == 7)
        {
            uint16_t t = s_fg;
            s_fg = s_bg;
            s_bg = t;
        }
        else if (p >= 30 && p <= 37)
            s_fg = (uint16_t)(p - 30);
        else if (p >= 40 && p <= 47)
            s_bg = (uint16_t)(p - 40);
        else if (p >= 90 && p <= 97)
            s_fg = (uint16_t)(p - 90 + 8);
        else if (p >= 100 && p <= 107)
            s_bg = (uint16_t)(p - 100 + 8);
        else if (p == 38 || p == 48)
        {
            if (i + 2 < s_nparams && s_params[i + 1] == 5)
            {
                /* Palette index, used as-is: the full 256 entries exist. */
                uint16_t c = (uint16_t)(s_params[i + 2] & 0xFF);
                if (p == 38)
                    s_fg = c;
                else
                    s_bg = c;
                i += 2;
            }
            else if (i + 4 < s_nparams && s_params[i + 1] == 2)
            {
                /* 24-bit colour: quantise to a palette entry by brightness so
                 * the desktop still reads correctly if it ever uses it. */
                int r = s_params[i + 2], g = s_params[i + 3], b = s_params[i + 4];
                int v = (r * 3 + g * 6 + b) / 10;
                uint16_t c = (uint16_t)(232 + (v * 23) / 255);
                if (p == 38)
                    s_fg = c;
                else
                    s_bg = c;
                i += 4;
            }
        }
    }
}

static void do_csi(char final)
{
    switch (final)
    {
    case 'H':
    case 'f':
        s_cy = param(0, 1) - 1;
        s_cx = param(1, 1) - 1;
        if (s_cy >= s_rows)
            s_cy = s_rows - 1;
        if (s_cx >= s_cols)
            s_cx = s_cols - 1;
        break;
    case 'A': s_cy -= param(0, 1); if (s_cy < 0) s_cy = 0; break;
    case 'B': s_cy += param(0, 1); if (s_cy >= s_rows) s_cy = s_rows - 1; break;
    case 'C': s_cx += param(0, 1); if (s_cx >= s_cols) s_cx = s_cols - 1; break;
    case 'D': s_cx -= param(0, 1); if (s_cx < 0) s_cx = 0; break;
    case 'G': s_cx = param(0, 1) - 1; if (s_cx < 0) s_cx = 0; if (s_cx >= s_cols) s_cx = s_cols - 1; break;
    case 'd': s_cy = param(0, 1) - 1; if (s_cy < 0) s_cy = 0; if (s_cy >= s_rows) s_cy = s_rows - 1; break;
    case 'J':
    {
        int mode = param(0, 0);
        if (mode == 2 || mode == 3)
        {
            for (int y = 0; y < s_rows; y++)
                clear_row(y, s_bg);
            s_cx = s_cy = 0;
        }
        else if (mode == 0)
        {
            for (int x = s_cx; x < s_cols; x++)
            {
                s_grid[s_cy][x].cp = 0;
                s_grid[s_cy][x].bg = s_bg;
            }
            for (int y = s_cy + 1; y < s_rows; y++)
                clear_row(y, s_bg);
            mark_dirty(s_cy);
        }
        break;
    }
    case 'K':
    {
        int mode = param(0, 0);
        int from = 0, to = s_cols - 1;
        if (mode == 0)
            from = s_cx;
        else if (mode == 1)
            to = s_cx;
        for (int x = from; x <= to && x < s_cols; x++)
        {
            s_grid[s_cy][x].cp = 0;
            s_grid[s_cy][x].bg = s_bg;
        }
        mark_dirty(s_cy);
        break;
    }
    case 'm':
        do_sgr();
        break;
    case 'n':
        if (param(0, 0) == 6)
        {
            s_size_query = true;
            s_query_rows = s_rows;
            s_query_cols = s_cols;
        }
        break;
    default:
        /* h, l, r, t, q, s, u, X, @, P... do not change drawn output here */
        break;
    }
}

/* ---- parser ------------------------------------------------------- */

static void feed_byte(uint8_t b)
{
    switch (s_state)
    {
    case P_GROUND:
        if (b == 0x1B)
        {
            s_state = P_ESC;
            return;
        }
        if (s_utf8_left)
        {
            s_utf8_acc = (s_utf8_acc << 6) | (b & 0x3F);
            if (--s_utf8_left == 0)
                put_cp((uint16_t)s_utf8_acc);
            return;
        }
        if (b < 0x80)
        {
            if (b == '\r')
                s_cx = 0;
            else if (b == '\n')
            {
                if (s_cy + 1 < s_rows)
                    s_cy++;
            }
            else if (b == 0x08)
            {
                if (s_cx > 0)
                    s_cx--;
            }
            else if (b == '\t')
                s_cx = (s_cx + 8) & ~7;
            else if (b >= 0x20 && b != 0x7F)
                put_cp(b);
            return;
        }
        /* UTF-8 lead byte */
        if ((b & 0xE0) == 0xC0)
        {
            s_utf8_acc = b & 0x1F;
            s_utf8_left = 1;
        }
        else if ((b & 0xF0) == 0xE0)
        {
            s_utf8_acc = b & 0x0F;
            s_utf8_left = 2;
        }
        else if ((b & 0xF8) == 0xF0)
        {
            s_utf8_acc = b & 0x07;
            s_utf8_left = 3;
        }
        else
        {
            put_cp('?');
        }
        return;

    case P_ESC:
        if (b == '[')
        {
            s_state = P_CSI;
            s_nparams = 0;
            s_cur_param = 0;
            s_have_param = false;
            return;
        }
        if (b == ']')
        {
            /* OSC: not used by the desktop; drop until the next ESC */
            s_state = P_GROUND;
            return;
        }
        if (b == '(' || b == ')' || b == '#')
        {
            s_state = P_GROUND;   /* charset selection: ignore the next byte */
            return;
        }
        s_state = P_GROUND;
        return;

    case P_CSI:
        if (b >= '0' && b <= '9')
        {
            s_cur_param = s_cur_param * 10 + (b - '0');
            s_have_param = true;
            return;
        }
        if (b == ';' || b == ':')
        {
            if (s_nparams < 8)
                s_params[s_nparams++] = s_have_param ? s_cur_param : 0;
            s_cur_param = 0;
            s_have_param = false;
            return;
        }
        if (b == '?')
            return;                     /* private modes: params still parsed */
        if (b >= 0x40 && b <= 0x7E)
        {
            if (s_nparams < 8)
                s_params[s_nparams++] = s_have_param ? s_cur_param : 0;
            do_csi((char)b);
            s_state = P_GROUND;
            return;
        }
        s_state = P_GROUND;
        return;
    }
}

/* ---- drawing ------------------------------------------------------ */

/* Compose one pixel row of a text row into line[], then blit it. */
static void draw_row_band(int y, int band, uint16_t *line)
{
    for (int x = 0; x < s_cols; x++)
    {
        const cell_t *c = &s_grid[y][x];
        uint16_t fg = s_pal[c->fg];
        uint16_t bg = s_pal[c->bg];
        const unsigned char *g = cp_is_blank(c->cp) ? NULL : glyph_for(c->cp);
        uint8_t bits = g ? g[band] : 0;
        for (int b = 0; b < FONT_W; b++)
            line[x * FONT_W + b] = (bits & (0x80 >> b)) ? fg : bg;
    }
    ili9340_blit(0, y * FONT_H + band, s_cols * FONT_W, 1, line);
}

static void draw_row(int y)
{
    static uint16_t line[VT_MAX_COLS * FONT_W];
    for (int band = 0; band < FONT_H; band++)
        draw_row_band(y, band, line);
    memcpy(s_shown[y], s_grid[y], sizeof(s_shown[y]));
    s_row_dirty[y] = false;
}

void vt_redraw_all(void)
{
    for (int y = 0; y < s_rows; y++)
        s_row_dirty[y] = true;
}

static void flush_dirty(void)
{
    for (int y = 0; y < s_rows; y++)
    {
        if (!s_row_dirty[y])
            continue;
        if (memcmp(s_grid[y], s_shown[y], sizeof(s_shown[y])) == 0)
        {
            s_row_dirty[y] = false;
            continue;
        }
        draw_row(y);
    }
}

/* ---- public ------------------------------------------------------- */

void vt_init(int cols, int rows)
{
    if (cols > VT_MAX_COLS)
        cols = VT_MAX_COLS;
    if (rows > VT_MAX_ROWS)
        rows = VT_MAX_ROWS;
    if (!s_pal_ready)
        build_palette();
    s_cols = cols;
    s_rows = rows;
    s_cx = s_cy = 0;
    s_fg = 7;
    s_bg = 0;

    memset(s_grid, 0, sizeof(s_grid));
    memset(s_shown, 0xFF, sizeof(s_shown));   /* force a full first paint */
    for (int y = 0; y < s_rows; y++)
    {
        for (int x = 0; x < s_cols; x++)
            s_grid[y][x].fg = 7;
        s_row_dirty[y] = true;
    }
    ESP_LOGI(TAG, "grid %dx%d", s_cols, s_rows);
}

bool vt_feed(const uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++)
        feed_byte(buf[i]);
    flush_dirty();
    return true;
}

int vt_cols(void) { return s_cols; }
int vt_rows(void) { return s_rows; }

bool vt_take_size_query(int *rows, int *cols)
{
    if (!s_size_query)
        return false;
    s_size_query = false;
    if (rows)
        *rows = s_query_rows;
    if (cols)
        *cols = s_query_cols;
    return true;
}
