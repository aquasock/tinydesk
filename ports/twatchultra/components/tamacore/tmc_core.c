/*
 * tmc_core.c - the Tamagotchi emulator, with no shared memory in it.
 *
 * See tmc_core.h for what this is and why it exists. The short version: this is
 * tamacore_task.c minus the two-core protocol, because that protocol was about
 * Linux and ESP-IDF sharing one chip and there is no second core here.
 *
 * THE ONE PIECE MOVED ACROSS UNCHANGED is the dirty-row tracking, and it is
 * worth understanding before changing it. A V3 display is a 32x30 dot matrix
 * plus ten status icons, all addressed through 512 bytes of display RAM. Several
 * items share a byte: bytes 0, 1, 5 and 6 each carry one icon from the top row
 * and one from the bottom, so "something in this byte changed" does not say
 * which of them moved. A span from the first change to the last therefore
 * repaints nearly the whole frame to flip a single ON/OFF glyph.
 *
 * So each byte carries a list of the ITEMS that read it, and each item carries
 * its own row span. A changed byte is XORed with its shadow, and only the items
 * whose bit actually changed contribute rows. That is why an icon flip costs one
 * row band instead of a frame.
 */

#include "tmc_core.h"

#include <stdlib.h>
#include <string.h>

#include "splb32.h"
#include "segmap.h"
#include "render.h"

/* The renderer's LCD box must match this file's frame geometry. render.h
 * already asserts the same pairing against shmproto.h when that header is
 * present; this is the check for a build that includes neither. */
_Static_assert(SEG_LCD_W + 2 * TMC_PAD == TMC_FRAME_W, "frame width does not fit the artwork");
_Static_assert(SEG_LCD_H + 2 * TMC_PAD == TMC_FRAME_H, "frame height does not fit the artwork");
_Static_assert(SEG_DOT == TMC_DOT && SEG_GAP == TMC_GAP, "dot pitch disagrees with segmap.h");

/* The bezel is black, matching the desktop behind a full-screen app, so only
 * the LCD itself is visible. It is deliberately NOT the LCD's off colour:
 * filling the surround with that made a blank screen and a screen full of
 * pixels look alike. */
#define BEZEL 0x0000

/* --- dirty-row tables ---------------------------------------------------- */

/*
 * An entry encodes (kind, index, bit):
 *   bit 15     0 = matrix cell, 1 = icon
 *   bits 14:8  matrix ROW (0..29) or icon number (0..9)
 *   bits  7:0  the dpram bit that lights it
 *
 * Matrix entries are keyed by ROW rather than by cell because a row's span is
 * the same for every cell in it, so duplicates within a row are harmless -- the
 * min/max is idempotent. Icons are keyed by number because each has its own
 * span.
 */
#define ITEM_ICON_FLAG 0x8000u
#define ITEM_ENC(is_icon, idx, bit) \
    ((uint16_t)(((is_icon) ? ITEM_ICON_FLAG : 0u) | (((idx) & 0x7F) << 8) | ((bit) & 0xFF)))
#define ITEM_IS_ICON(e) (((e) & ITEM_ICON_FLAG) != 0)
#define ITEM_IDX(e)     (((e) >> 8) & 0x7F)
#define ITEM_BIT(e)     ((e) & 0xFF)

#define ITEM_MAX (SEG_MATRIX_W * SEG_MATRIX_H + SEG_ICON_COUNT)
#define DPRAM_BYTES 0x200

struct tmc
{
    splb32 *cpu;
    struct tmc_tables *tables;        /* shared, refcounted; see above */
    uint16_t *frame;                  /* the caller's buffer, TMC_FRAME_W x TMC_FRAME_H */
    uint8_t dpram_shadow[DPRAM_BYTES];
    bool shadow_valid;
};

/* --- row tables: built once, and there is only one geometry ---------------
 *
 * They were file-static in tamacore_task.c, keyed to SHM_ constants. They are
 * still shared rather than per-instance, because the geometry is a compile-time
 * property of the artwork and there is exactly one of it -- two instances would
 * describe the same picture twice.
 *
 * BUT THEY ARE ON THE HEAP NOW, AND THAT WAS NOT A TIDINESS CHANGE. As .bss
 * they were 4,148 bytes of internal DRAM, which reading the number alone does
 * not reveal the cost of: on the watch this board has 62,443 bytes of internal
 * RAM free after boot and tdsh's SSH server refuses to start below 65,536, so
 * the emulator's tables turned a working `ssh start` into
 *
 *     ssh: insufficient internal RAM (62443 bytes free; need at least 65536)
 *
 * a refusal that has nothing to do with the emulator and no visible connection
 * to it. A heap allocation of this size lands in PSRAM (the port sets
 * CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL to 4096) and is released when the last
 * instance goes away, so the memory is held only while the guest runs -- and
 * internal DRAM, which is the scarcest thing on the board, is not spent on a
 * picture that is only sometimes on screen. The refcount is what makes a shared
 * table safe once it is a heap allocation.
 *
 * The scratch array was a 1 KiB local in build_tables(), which is a large slice
 * of a small task's stack; it is dead after the first build, so it lives here.
 */
struct tmc_tables
{
    uint16_t byte_first[DPRAM_BYTES];
    uint16_t byte_nitem[DPRAM_BYTES];
    uint16_t byte_items[ITEM_MAX];
    uint16_t item_row_lo[SEG_MATRIX_H + SEG_ICON_COUNT];
    uint16_t item_row_hi[SEG_MATRIX_H + SEG_ICON_COUNT];
    uint16_t fill[DPRAM_BYTES];
};

static struct tmc_tables *s_tables;
static int s_tables_refs;

static uint16_t item_first_row(uint16_t e)
{
    return ITEM_IS_ICON(e) ? (uint16_t)(SEG_MATRIX_H + ITEM_IDX(e))
                           : (uint16_t)ITEM_IDX(e);
}

static void build_tables(struct tmc_tables *t)
{
    const int pitch = TMC_DOT + TMC_GAP;
    int i, r, c;
    uint16_t acc;

    /* Matrix rows, in frame coordinates: the LCD begins at PAD, and the matrix
     * begins at SEG_MATRIX_OY inside the LCD. */
    for (r = 0; r < SEG_MATRIX_H; r++)
    {
        int y0 = TMC_PAD + SEG_MATRIX_OY + r * pitch;

        t->item_row_lo[r] = (uint16_t)y0;
        t->item_row_hi[r] = (uint16_t)(y0 + TMC_DOT);
    }

    /* Icon rows come from the artwork's own run list rather than a guess. */
    for (i = 0; i < SEG_ICON_COUNT; i++)
    {
        const seg_icon *ic = &seg_icons[i];
        uint16_t lo = TMC_FRAME_H, hi = 0;
        int k;

        for (k = 0; k < ic->run_count; k++)
        {
            uint16_t yy = seg_icon_runs[ic->run_start + k].y;

            if (yy < lo)
                lo = yy;
            if ((uint16_t)(yy + 1) > hi)
                hi = (uint16_t)(yy + 1);
        }
        t->item_row_lo[SEG_MATRIX_H + i] = (uint16_t)(TMC_PAD + lo);
        t->item_row_hi[SEG_MATRIX_H + i] = (uint16_t)(TMC_PAD + hi);
    }

    for (i = 0; i < DPRAM_BYTES; i++)
        t->byte_nitem[i] = 0;

    /* Count, then place: two passes so byte_items is exactly sized and
     * byte_first can be the prefix sum. */
    for (r = 0; r < SEG_MATRIX_H; r++)
        for (c = 0; c < SEG_MATRIX_W; c++)
        {
            uint8_t b = (uint8_t)(seg_matrix[r * SEG_MATRIX_W + c] >> 8);

            if (b != 0xFF)
                t->byte_nitem[b]++;
        }
    for (i = 0; i < SEG_ICON_COUNT; i++)
        t->byte_nitem[seg_icons[i].byte]++;

    acc = 0;
    for (i = 0; i < DPRAM_BYTES; i++)
    {
        t->byte_first[i] = acc;
        acc = (uint16_t)(acc + t->byte_nitem[i]);
    }

    {
        uint16_t *fill = t->fill;

        for (i = 0; i < DPRAM_BYTES; i++)
            fill[i] = t->byte_first[i];

        for (r = 0; r < SEG_MATRIX_H; r++)
            for (c = 0; c < SEG_MATRIX_W; c++)
            {
                uint16_t v = seg_matrix[r * SEG_MATRIX_W + c];
                uint8_t b = (uint8_t)(v >> 8);

                if (b == 0xFF)
                    continue;         /* padding cell */
                t->byte_items[fill[b]++] = ITEM_ENC(0, r, v & 0xFF);
            }
        for (i = 0; i < SEG_ICON_COUNT; i++)
            t->byte_items[fill[seg_icons[i].byte]++] =
                ITEM_ENC(1, i, seg_icons[i].bit);
    }

}

/* --- drawing ------------------------------------------------------------- */

static void fill_bezel(uint16_t *px)
{
    for (int i = 0; i < TMC_FRAME_W * TMC_FRAME_H; i++)
        px[i] = BEZEL;
}

/*
 * Draw rows [yl, yh). `full` paints the bezel too, which is only needed on the
 * first frame or after the buffer has been handed to something else.
 *
 * The partial path repaints only the horizontal margins of the rows it touches:
 * the renderer repaints the LCD area it is asked for, and the bezel beside it is
 * along for the ride.
 */
static void draw_rows(uint16_t *px, const uint8_t *dpram, int yl, int yh, bool full)
{
    lcd_fb fb = { px, TMC_FRAME_W, TMC_FRAME_H };

    if (full)
    {
        fill_bezel(px);
    }
    else
    {
        for (int y = yl; y < yh; y++)
        {
            uint16_t *row = px + (size_t)y * TMC_FRAME_W;

            for (int x = 0; x < TMC_PAD; x++)
            {
                row[x] = BEZEL;
                row[TMC_FRAME_W - 1 - x] = BEZEL;
            }
        }
    }

    lcd_draw_dots_rows(&fb, dpram, TMC_PAD, TMC_PAD, TMC_DOT, TMC_GAP, yl, yh);
}

/*
 * Which rows changed since the shadow was taken, as a half-open range.
 *
 * Returns 0 when nothing did. The two-band trick from tamacore_task.c is
 * deliberately NOT reproduced: it published two bands to core 1 because the top
 * icon row and the matrix are far apart, and here the caller gets one range and
 * decides for itself. On this panel a full-frame push is about 4 ms measured, so
 * a good-enough range is good enough -- and one number is much harder to get
 * wrong than two.
 */
static int dirty_rows(tmc *m, int *y_lo, int *y_hi)
{
    const uint8_t *dp = splb32_dpram(m->cpu);
    int lo = TMC_FRAME_H, hi = 0;

    if (!m->shadow_valid)
    {
        /* First frame: everything is new. */
        memcpy(m->dpram_shadow, dp, DPRAM_BYTES);
        m->shadow_valid = true;
        *y_lo = 0;
        *y_hi = TMC_FRAME_H;
        return TMC_FRAME_H;
    }

    for (int b = 0; b < DPRAM_BYTES; b++)
    {
        uint8_t diff = (uint8_t)(dp[b] ^ m->dpram_shadow[b]);

        if (!diff)
            continue;

        m->dpram_shadow[b] = dp[b];

        for (int k = 0; k < m->tables->byte_nitem[b]; k++)
        {
            uint16_t e = m->tables->byte_items[m->tables->byte_first[b] + k];
            unsigned bit = ITEM_BIT(e);

            /* Only an item whose own bit actually changed contributes rows. */
            if (!((diff >> bit) & 1))
                continue;

            uint16_t ir = item_first_row(e);

            if (m->tables->item_row_lo[ir] < lo)
                lo = m->tables->item_row_lo[ir];
            if (m->tables->item_row_hi[ir] > hi)
                hi = m->tables->item_row_hi[ir];
        }
    }

    if (hi <= lo)
        return 0;

    if (lo < 0)
        lo = 0;
    if (hi > TMC_FRAME_H)
        hi = TMC_FRAME_H;

    *y_lo = lo;
    *y_hi = hi;
    return hi - lo;
}

/* --- lifecycle ----------------------------------------------------------- */

/*
 * The shared table set, built on first use and freed when the last instance
 * goes. Returns NULL when the allocation fails, which tmc_new turns into
 * TMC_ERR_MEMORY -- the tables are not optional, and a guest that runs without
 * them would draw nothing and report nothing changed.
 */
static struct tmc_tables *tables_acquire(void)
{
    if (!s_tables)
    {
        s_tables = (struct tmc_tables *)calloc(1, sizeof(*s_tables));

        if (!s_tables)
            return NULL;

        build_tables(s_tables);
    }

    s_tables_refs++;
    return s_tables;
}

static void tables_release(void)
{
    if (s_tables_refs > 0)
        s_tables_refs--;

    if (s_tables_refs == 0)
    {
        free(s_tables);
        s_tables = NULL;
    }
}

tmc_err tmc_new(const uint8_t *rom, size_t rom_len, uint16_t *frame, tmc **out)
{
    if (!rom || !frame || !out)
        return TMC_ERR_ARG;

    *out = NULL;

    /* The signature check is here rather than in the caller because it is the
     * only thing standing between a wrong file and executing garbage. */
    if (rom_len < TMC_ROM_BYTES ||
        rom[0] != TMC_ROM_SIG0 || rom[1] != TMC_ROM_SIG1 ||
        rom[2] != TMC_ROM_SIG2 || rom[3] != TMC_ROM_SIG3)
        return TMC_ERR_SIGNATURE;

    struct tmc_tables *tables = tables_acquire();

    if (!tables)
        return TMC_ERR_MEMORY;

    tmc *m = (tmc *)calloc(1, sizeof(*m));
    if (!m)
    {
        tables_release();
        return TMC_ERR_MEMORY;
    }

    m->tables = tables;

    m->cpu = splb32_new(rom, rom_len);
    if (!m->cpu)
    {
        free(m);
        return TMC_ERR_MEMORY;
    }

    m->frame = frame;
    m->shadow_valid = false;
    fill_bezel(frame);

    /*
     * THE MACHINE'S WIRING, in this order and with NO splb32_reset() after it.
     *
     * splb32_reset() memsets pullup_ext, so a reset placed after set_pullup()
     * silently removes the EEPROM's SDA pull-up and the ROM then hangs in the
     * I2C read -- running at full speed and never drawing. splb32_new() has
     * already reset the core, so the wiring goes on afterwards. This cost a
     * debugging session on the host harness; it is the single most expensive
     * trap in bringing this emulator up.
     *
     * The addresses are the vendor front end's: EEPROM at 0xA0 (the 8-bit form)
     * with SDA on port D bit 1 and SCL on bit 0, and the IR pair both on port C
     * bit 2.
     */
    splb32_set_pullup(m->cpu, 3, 0x02);
    splb32_attach_eeprom(m->cpu, 0xA0, 0x02, 0x01);
    splb32_attach_ir(m->cpu, 0x04, 0x04);

    *out = m;
    return TMC_OK;
}

void tmc_free(tmc *m)
{
    if (!m)
        return;
    if (m->cpu)
        splb32_free(m->cpu);
    if (m->tables)
        tables_release();
    free(m);
}

uint64_t tmc_run(tmc *m, uint64_t cycles)
{
    if (!m)
        return 0;
    return splb32_run_cycles(m->cpu, cycles);
}

int tmc_frame(tmc *m, int *y_lo, int *y_hi)
{
    int lo = 0, hi = 0;

    if (!m || !y_lo || !y_hi)
        return 0;

    int rows = dirty_rows(m, &lo, &hi);

    if (rows <= 0)
        return 0;

    draw_rows(m->frame, splb32_dpram(m->cpu), lo, hi, false);

    *y_lo = lo;
    *y_hi = hi;
    return rows;
}

void tmc_button(tmc *m, int index, bool down)
{
    static const uint8_t bit[3] = { 0x01, 0x02, 0x04 };

    if (!m || index < 0 || index > 2)
        return;

    /* splb32_pin() takes 1 for "driven" and -1 for "released to the pull-up".
     * The line is pulled up, so a press drives it low -- the semantics are the
     * opposite of the caller's, and passing 0 here (which is neither) leaves
     * every button inert. */
    splb32_pin(m->cpu, 0, bit[index], down ? 1 : -1);
}

void tmc_eeprom_save(const tmc *m, uint8_t *out)
{
    if (!m || !out)
        return;
    if (splb32_eeprom_save(m->cpu, out) != 0)
        memset(out, 0, TMC_EEPROM_BYTES);
}

bool tmc_eeprom_load(tmc *m, const uint8_t *in, size_t len)
{
    if (!m || !in || len != TMC_EEPROM_BYTES)
        return false;
    splb32_eeprom_load(m->cpu, in);
    return true;
}

uint64_t tmc_cycles(const tmc *m)
{
    return m ? splb32_state_cycles(m->cpu) : 0;
}

const uint8_t *tmc_dpram(const tmc *m)
{
    return m ? splb32_dpram(m->cpu) : NULL;
}
