/*
 * display.c - the T-Watch Ultra's CO5300 AMOLED as a terminal.
 *
 * The desktop's output is parsed into a td_vterm and a task paints the cells
 * that changed. This file is the only one in the port that is NOT modelled on
 * ports/crowpanel7, because the two panels have nothing in common:
 *
 *   CrowPanel   800x480 RGB parallel, driven by esp_lcd's RGB panel driver,
 *               which scans a framebuffer continuously and needs no writes.
 *   T-Watch     410x502 AMOLED, CO5300 controller on a four-lane QSPI bus,
 *               write-only, with no scanout of its own.
 *
 * ESP-IDF 5.3's esp_lcd panel IO has no quad mode, so the bus is driven with
 * the SPI master driver directly and the CO5300 command set by hand. The
 * sequences below come from the Escapement project's working Linux driver
 * (patches/kernel/display/final/co5300.c), which in turn took them from the
 * watch's factory firmware at file offset 0x130dd4 -- they are the ones that
 * actually bring this panel up, not guesses from a datasheet.
 *
 * PIXELS ARE SWAPPED. The CO5300 takes the low byte of an RGB565 word first,
 * so every pixel written into the line buffer is byte-swapped. Black is
 * identical either way, which is exactly why a missing swap looks like a
 * working driver until something coloured is drawn.
 */

#include "display.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "font8x16.h"
#include "edges.h"
#include "tinydesk/td_vterm.h"

#define TAG "twatch-display"

/* ---- pins, from board.example.conf ------------------------------------- */
#define LCD_SCK   40
#define LCD_D0    38
#define LCD_D1    39
#define LCD_D2    42
#define LCD_D3    45
#define LCD_CS    41
#define LCD_RST   37
#define LCD_SPI_HZ (80 * 1000 * 1000)   /* LilyGo runs this panel at 80 MHz */

/* The panel's own coordinate space. The glass is 410x502; DISPLAY_W/H in
 * display.h is the 320x400 rectangle inside it that the desktop uses. The
 * offset centres one in the other. */
#define PANEL_W   410
#define PANEL_H   502

#define CELL_W    8
#define CELL_H    16

/* ---- low level ---------------------------------------------------------
 *
 * TRANSCRIBED FROM LILYGO'S OWN FIRMWARE, which is the thing that actually
 * works on this panel. Two earlier attempts in this file were wrong, and both
 * failures were silent -- no error, no busy bit, a black screen:
 *
 *   attempt 1  command and address on four lanes. Wrong.
 *   attempt 2  command and address on ONE lane, on the theory that only the
 *              pixel payload is quad. Also wrong.
 *
 * LilyGo's LilyGoDispQSPI::writeCommand() is the authority:
 *
 *     t.flags = (SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR);
 *     t.cmd   = 0x02;
 *     t.addr  = reg << 8;
 *
 * with the DEVICE configured command_bits = 8, address_bits = 24,
 * spics_io_num = -1 (chip select driven by hand) and SPI_DEVICE_HALFDUPLEX.
 * So a register write is: multiline command byte 0x02, the register number in
 * bits 15:8 of a 24-bit address, then the payload.
 *
 * Pixels are a different shape again (pushColorsDMA): SPI_TRANS_MODE_QIO with
 * cmd 0x32 and addr 0x002C00 to open the stream, then continuations with
 * VARIABLE_CMD|VARIABLE_ADDR and zero-length command/address so CS can be
 * held across the whole transfer. There is no separate 0x2C command -- the
 * 0x002C00 address IS the memory-write marker.
 *
 * Chip select is driven by hand because the controller treats a CS edge as
 * end-of-burst and discards whatever follows.
 */

#define CMD_WRITE   0x02
#define CMD_PIXELS  0x32
#define ADDR_RAMWR  0x002C00
/*
 * WHERE THE FRAMEBUFFER SITS ON THE GLASS.
 *
 * The panel is 410x502 with 45-degree chamfered corners, so the largest
 * rectangle guaranteed to render in full is inset 45 px on every side, which
 * is 320x400. DISPLAY_ORIGIN_X/Y in display.h hold where it actually starts;
 * they are both 45, and this build is one row TALLER than 400 because it is
 * trying an extra row at the bottom (see the note there).
 *
 * On top of that the controller's own RAM starts 22 columns to the left of the
 * visible glass, so a column written to the panel needs PANEL_COL_OFFSET added
 * to its x. Both offsets are required: without them the picture appears
 * shifted up and to the left, which is exactly what the first working flash of
 * this port showed.
 *
 * These are the Escapement project's measured constants, from
 * patches/kernel/display/final/co5300.c where the Linux driver uses the same
 * geometry.
 */
#define PANEL_SAFE_INSET DISPLAY_ORIGIN_X
/* Shared with touch.c through display.h, which is the only place that has to
 * agree about where the framebuffer sits on the glass. PANEL_COL_OFFSET moved
 * there for the same reason: the margins and the touch driver are both written
 * in controller coordinates. */
#define FB_X             DISPLAY_ORIGIN_X
#define FB_Y             DISPLAY_ORIGIN_Y
#define PIX_CHUNK   16384        /* pixels per SPI transaction, as LilyGo */

static spi_device_handle_t s_spi;
static SemaphoreHandle_t s_lock;
static td_vterm_t *s_vt;
static td_vcell_t *s_shown;
static uint16_t *s_fb;
static int s_cols, s_rows;
/* Where the cursor was last drawn. paint_cell inverts a cell when the cursor is
 * on it, but the cell's own colours are unchanged, so the comparison below
 * cannot see the cursor leaving: the cell would stay inverted on the glass for
 * ever. One cursor exists at a time, so remembering where it was is enough. */
static int s_cur_col = -1, s_cur_row = -1;
static TaskHandle_t s_paint_task;
static display_reply_fn s_reply;

#define BAND_ROWS 16

/*
 * The SPI DMA source: one full-width band, byte-swapped and pushed.
 *
 * STATIC, in internal DRAM, and deliberately not allocated. It has to be
 * internal -- DMA out of PSRAM goes through the cache and is markedly slower,
 * and this buffer is read by the peripheral on every flush -- but asking the
 * heap for MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA after the Bluetooth controller
 * has taken its share is a request that can simply fail, and it did: the port
 * booted to a dark screen with no output, because with CONFIG_ESP_CONSOLE_NONE
 * the failure has nowhere to be reported until the desktop link is up. A
 * static array in this component is internal DRAM by construction (only
 * libtinydesk, libtdsh and libmicropython are mapped to PSRAM by extram.lf),
 * so it costs nothing to allocate and cannot fail. 10 KB of .bss.
 */
static uint16_t s_tx[DISPLAY_W * BAND_ROWS];

/*
 * The full xterm 256-colour palette, converted to RGB565 once at init.
 *
 * td_vterm stores COLOUR INDICES, not colours: 0-15 are the ANSI palette,
 * 16-231 a 6x6x6 cube, 232-255 a grey ramp. The first version of this file
 * masked them with 0x0F on the theory that only 16 colours mattered, which
 * folded every index into the wrong entry -- 75 became 11, 236 became 12, and
 * the desktop came out in colours nobody chose while the same bytes looked
 * correct in a PC terminal, because the PC terminal expands the indices
 * itself.
 */
static uint16_t s_palette[256];

static void palette_init(void)
{
    static const uint8_t ansi[16][3] = {
        { 0, 0, 0 },       { 205, 0, 0 },     { 0, 205, 0 },     { 205, 205, 0 },
        { 0, 0, 238 },     { 205, 0, 205 },   { 0, 205, 205 },   { 229, 229, 229 },
        { 127, 127, 127 }, { 255, 0, 0 },     { 0, 255, 0 },     { 255, 255, 0 },
        { 92, 92, 255 },   { 255, 0, 255 },   { 0, 255, 255 },   { 255, 255, 255 },
    };
    static const uint8_t lv[6] = { 0, 95, 135, 175, 215, 255 };
    int i;

    for (i = 0; i < 16; i++) {
        int r = ansi[i][0] >> 3, g = ansi[i][1] >> 2, b = ansi[i][2] >> 3;
        s_palette[i] = (uint16_t)((r << 11) | (g << 5) | b);
    }
    for (i = 0; i < 216; i++) {
        int r = lv[i / 36] >> 3, g = lv[(i / 6) % 6] >> 2, b = lv[i % 6] >> 3;
        s_palette[16 + i] = (uint16_t)((r << 11) | (g << 5) | b);
    }
    for (i = 0; i < 24; i++) {
        int v = 8 + 10 * i;
        int r = v >> 3, g = v >> 2, b = v >> 3;
        s_palette[232 + i] = (uint16_t)((r << 11) | (g << 5) | b);
    }
}

static inline uint16_t swap16(uint16_t v)
{
    return (uint16_t)((v >> 8) | (v << 8));
}

/* A register write, exactly as LilyGoDispQSPI::writeCommand(). */
static void reg_write(uint8_t reg, const uint8_t *data, size_t len)
{
    spi_transaction_t t = { 0 };

    t.flags = SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
    t.cmd = CMD_WRITE;
    t.addr = (uint64_t)reg << 8;
    t.tx_buffer = len ? data : NULL;
    t.length = 8 * len;

    spi_device_acquire_bus(s_spi, portMAX_DELAY);
    gpio_set_level(LCD_CS, 0);
    esp_err_t err = spi_device_polling_transmit(s_spi, &t);
    gpio_set_level(LCD_CS, 1);
    spi_device_release_bus(s_spi);

    if (err != ESP_OK)
        ESP_LOGE(TAG, "reg 0x%02X failed: %s", reg, esp_err_to_name(err));
}

static void set_window(int x, int y, int w, int h)
{
    uint8_t cas[4] = { (uint8_t)((x >> 8) & 0xFF), (uint8_t)(x & 0xFF),
                       (uint8_t)(((x + w - 1) >> 8) & 0xFF), (uint8_t)((x + w - 1) & 0xFF) };
    uint8_t ras[4] = { (uint8_t)((y >> 8) & 0xFF), (uint8_t)(y & 0xFF),
                       (uint8_t)(((y + h - 1) >> 8) & 0xFF), (uint8_t)((y + h - 1) & 0xFF) };

    reg_write(0x2A, cas, 4);
    reg_write(0x2B, ras, 4);
}

/* Push w*h pixels from s_tx at RAW controller coordinates, byte-swapped and
 * already in the buffer. The framebuffer converters below add their own
 * offsets; this one is for the areas outside the framebuffer, where those
 * offsets do not apply. */
static void push_pixels_raw(int cx, int cy, int w, int h);

/* Push w*h byte-swapped pixels from s_tx, CS held across the whole stream. */
static void push_pixels(int x, int y, int w, int h)
{
    push_pixels_raw(x + FB_X + PANEL_COL_OFFSET, y + FB_Y, w, h);
}

static void push_pixels_raw(int cx, int cy, int w, int h)
{
    size_t npix = (size_t)w * h;
    uint16_t *p = s_tx;
    size_t remaining = npix;
    bool first = true;
    esp_err_t err = ESP_OK;

    set_window(cx, cy, w, h);

    spi_device_acquire_bus(s_spi, portMAX_DELAY);
    gpio_set_level(LCD_CS, 0);

    while (remaining)
    {
        size_t chunk = remaining > PIX_CHUNK ? PIX_CHUNK : remaining;
        spi_transaction_ext_t t = { 0 };

        if (first)
        {
            t.base.flags = SPI_TRANS_MODE_QIO;
            t.base.cmd = CMD_PIXELS;
            t.base.addr = ADDR_RAMWR;
            first = false;
        }
        else
        {
            /* Continuation: no command, no address, CS still low. */
            t.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD |
                           SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
            t.command_bits = 0;
            t.address_bits = 0;
            t.dummy_bits = 0;
        }
        t.base.tx_buffer = p;
        t.base.length = chunk * 16;

        err = spi_device_polling_transmit(s_spi, (spi_transaction_t *)&t);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "pixel stream failed: %s", esp_err_to_name(err));
            break;
        }
        p += chunk;
        remaining -= chunk;
    }

    gpio_set_level(LCD_CS, 1);
    spi_device_release_bus(s_spi);
}

static const struct {
    uint8_t op;
    uint8_t data[4];
    uint8_t len;
} s_init[] = {
    { 0xFE, { 0x00 }, 0x01 },
    { 0xC4, { 0x80 }, 0x01 },
    { 0x3A, { 0x55 }, 0x01 },         /* RGB565 */
    { 0x35, { 0x00 }, 0x01 },
    { 0x53, { 0x20 }, 0x01 },
    { 0x63, { 0xFF }, 0x01 },
    { 0x2A, { 0x00, 0x16, 0x01, 0xAF }, 0x04 },
    { 0x2B, { 0x00, 0x00, 0x01, 0xF5 }, 0x04 },
    { 0x11, { 0x00 }, 0x80 },         /* sleep out */
    { 0x29, { 0x00 }, 0x80 },         /* display on */
    { 0x51, { 0x00 }, 0x01 },         /* brightness; 0xFF follows */
};

static bool panel_init(void)
{
    uint8_t bright = 0xFF;
    size_t i;

    /* CS is driven by hand, so it must be a plain output. */
    gpio_config_t cs = {
        .pin_bit_mask = 1ULL << LCD_CS,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config_t rst = {
        .pin_bit_mask = 1ULL << LCD_RST,
        .mode = GPIO_MODE_OUTPUT,
    };

    gpio_config(&cs);
    gpio_set_level(LCD_CS, 1);

    /*
     * The reset pulse, timed as LilyGo's own init times it: HIGH 200 ms,
     * LOW 300 ms, HIGH 200 ms. An earlier version here used 20/120 ms, which
     * is far too short for the controller's power-on reset to complete -- it
     * holds the panel in reset and every later command is discarded.
     */
    gpio_config(&rst);
    gpio_set_level(LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    gpio_set_level(LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(200));

    for (i = 0; i < sizeof(s_init) / sizeof(s_init[0]); i++) {
        reg_write(s_init[i].op, s_init[i].data, s_init[i].len & 0x1F);
        if (s_init[i].len & 0x80)
            vTaskDelay(pdMS_TO_TICKS(120));
    }
    reg_write(0x51, &bright, 1);
    return true;
}

/* ---- painting ---------------------------------------------------------- */

static void paint_cell(int col, int row, td_vcell_t c, bool cursor)
{
    const uint8_t *glyph = font8x16_glyph(c.ch);
    uint16_t fg = s_palette[c.fg];
    uint16_t bg = s_palette[c.bg];
    int x0 = col * CELL_W;
    int y0 = row * CELL_H;
    int y, bit;

    if (cursor)
    {
        uint16_t t = fg;
        fg = bg;
        bg = t;
    }

    for (y = 0; y < CELL_H; y++)
    {
        uint8_t bits = (fg == bg || !glyph) ? 0 : glyph[y];
        uint16_t *dst = s_fb + (size_t)(y0 + y) * DISPLAY_W + x0;

        for (bit = 0; bit < CELL_W; bit++)
            dst[bit] = (bits & (0x80 >> bit)) ? fg : bg;
    }
}

/* Push one rectangle of the framebuffer to the panel.
 *
 * x0/x1 are pixel columns (x1 exclusive), y0/y1 pixel rows, all in framebuffer
 * space. Pushing only the columns that changed is what makes a partial update
 * cheap: a change confined to one 8-pixel icon column used to be pushed as a
 * full 320-pixel band, forty times the bytes for nothing. A single mouse move
 * repaints the whole icon column on this desktop, so that waste was 96% of the
 * framebuffer per move -- which is the lag. */
static void flush_rect(int x0, int y0, int x1, int y1)
{
    int w, y, band;

    if (x0 < 0)
        x0 = 0;
    if (x1 > DISPLAY_W)
        x1 = DISPLAY_W;
    if (y1 > DISPLAY_H)
        y1 = DISPLAY_H;
    if (x0 >= x1)
        return;

    w = x1 - x0;

    /* How many rows one transaction can carry: as many as fit in s_tx, which
     * is a fixed size. A full-width band is 16 rows; an 8-pixel column is 640,
     * so a single changed column goes out as ONE transaction instead of 24.
     * That matters because each transaction re-programs the window registers
     * (two QSPI writes) and takes the bus, so at narrow widths the per-call
     * overhead dominated the pixel data. */
    band = (int)(sizeof(s_tx) / sizeof(s_tx[0])) / w;
    if (band < 1)
        band = 1;

    for (y = y0; y < y1; y += band)
    {
        int rows = (y1 - y) > band ? band : (y1 - y);
        uint16_t *dst = s_tx;
        int r;

        /* Only the columns in this rectangle are copied out, so the transfer
         * is w wide rather than DISPLAY_W. */
        for (r = 0; r < rows; r++)
        {
            const uint16_t *src = s_fb + (size_t)(y + r) * DISPLAY_W + x0;

            for (int i = 0; i < w; i++)
                *dst++ = swap16(src[i]);
        }
        push_pixels(x0, y, w, rows);
    }
}


/* Fill the entire panel with one colour, in strips narrow enough for s_tx.
 *
 * The framebuffer is 320x416 inside a 410x502 glass, inset 45 columns and 45
 * rows to clear the chamfers, so a margin of the panel is never written by the
 * paint loop. The controller's RAM is not cleared at reset, so whatever the
 * previous firmware left there survives -- and that firmware was a Linux
 * framebuffer with a different geometry. Whether anything visible was left in
 * those margins is unproven: this wipe was added while chasing leftover pixels
 * inside the window and did NOT fix them, so the margins may well have been
 * clean all along. It stays because one pass over the panel at start-up costs
 * nothing and makes the panel's state independent of whatever ran before.
 */
static void panel_fill(uint16_t colour)
{
    const int strip = (int)(sizeof(s_tx) / sizeof(s_tx[0])) / 16;   /* columns */

    for (int x = 0; x < PANEL_W; x += strip)
    {
        int w = PANEL_W - x > strip ? strip : PANEL_W - x;

        for (int r = 0; r < 16; r++)
            for (int i = 0; i < w; i++)
                s_tx[r * w + i] = colour;

        for (int y = 0; y < PANEL_H; y += 16)
        {
            int h = PANEL_H - y > 16 ? 16 : PANEL_H - y;
            size_t remaining = (size_t)w * h;
            uint16_t *pp = s_tx;
            bool first = true;

            set_window(x + PANEL_COL_OFFSET, y, w, h);
            spi_device_acquire_bus(s_spi, portMAX_DELAY);
            gpio_set_level(LCD_CS, 0);

            while (remaining)
            {
                size_t chunk = remaining > PIX_CHUNK ? PIX_CHUNK : remaining;
                spi_transaction_ext_t t = { 0 };

                if (first)
                {
                    t.base.flags = SPI_TRANS_MODE_QIO;
                    t.base.cmd = CMD_PIXELS;
                    t.base.addr = ADDR_RAMWR;
                    first = false;
                }
                else
                {
                    t.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD |
                                   SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
                    t.command_bits = 0;
                    t.address_bits = 0;
                    t.dummy_bits = 0;
                }
                t.base.tx_buffer = pp;
                t.base.length = chunk * 16;
                (void)spi_device_polling_transmit(s_spi, (spi_transaction_t *)&t);
                pp += chunk;
                remaining -= chunk;
            }
            gpio_set_level(LCD_CS, 1);
            spi_device_release_bus(s_spi);
        }
    }
}


/*
 * THE THREE BUTTONS BELOW THE FRAMEBUFFER.
 *
 * The glass has a strip below the desktop's 320x416 rectangle that nothing
 * draws into, and on the Linux build it carried three coloured bars -- red,
 * green, blue -- which are the board's three capacitive buttons. This paints
 * them again, as buttons rather than images now: the touch driver reads a press
 * here and sends the key the label names.
 *
 * Three bars, labelled Tab, Enter and Ctrl+Q. They were full-primary green,
 * which named the three capacitive buttons of the Linux build and nothing else;
 * they are grey now, because a lit-up frame around the desktop competes with
 * the desktop. The two black separators are kept: without them the strip is one
 * band with nothing to show where a button ends and the next begins.
 *
 * The strip has to be read as the bottom run of a frame drawn round the
 * desktop, because that is what it is: the left column runs the framebuffer's
 * full height and this runs its full width, and the end buttons of each are
 * capped rather than square so that neither one runs into the glass.
 *
 * Geometry, in the controller's coordinates (the visible glass is 22..431):
 *
 *   x  67 .. 172  Tab
 *     173 .. 174  black separator
 *     175 .. 279  Enter
 *     280 .. 281  black separator
 *     282 .. 386  Ctrl+Q
 *
 * 67 and 386 are the framebuffer's own left and right edges, so the strip
 * finishes where the desktop above it finishes. The leftmost and rightmost
 * bars round the corners at their outer end -- both the one facing the desktop
 * and the one facing the glass, which is what makes it a cap. The corners at
 * the separators round on the desk side only, exactly as the left column's do.
 * Between the caps the bottom edge is flush with the glass, like the left
 * column's left edge.
 *
 * The colours are grey rather than the Linux driver's full-primary green: a dark
 * grey face with a lighter grey border, so the margins read as controls rather
 * than as a signal. The green was the brightest thing on the glass, which was
 * the point while the buttons were images; now that they are labelled and
 * pressable, a lit-up green frame competes with the desktop it frames.
 */
#define COL_FILL  0x2104         /* RGB565(4,8,4):    dark grey, ~13% */
#define COL_EDGE  0x8410         /* RGB565(16,32,16):  lighter grey, ~51% */
#define BTN_BORDER 2

/* Which corners of a button are rounded.
 *
 * A button is round only where it faces something: the edge that faces the
 * desktop, and the outer end of a group. The edge that runs along the glass
 * between buttons stays square, which is what makes a column read as one
 * continuous run and the strip as one continuous run. */
typedef struct
{
    bool tl, tr, bl, br;
} corners_t;

/* Is (x, y) inside the button at x0..x1 by y0..y1, rounded as `c` says?
 *
 * The rounded corners are circles of radius r whose centres sit r in from the
 * corner, and a point in a corner's quadrant is inside when it is within r of
 * that centre. Away from the corners there is no distance test at all -- only a
 * bounds check -- which is what keeps the straight edges straight; applying the
 * distance test everywhere would eat the whole row.
 *
 * Testing the only-rounded-corners shape rather than a true rounded rectangle
 * is deliberate: the design rounds four specific corners, not all of them, and
 * a general form would have to be told the radii per corner anyway. */
static bool button_has(int x, int y, int x0, int y0, int x1, int y1, int r, corners_t c)
{
    int cx = x, cy = y;
    int dx, dy;

    if (x < x0 || x > x1 || y < y0 || y > y1)
        return false;
    if (r <= 0)
        return true;

    if (y < y0 + r)
    {
        if (c.tl && x < x0 + r)
        {
            cx = x0 + r;
            cy = y0 + r;
        }
        else if (c.tr && x > x1 - r)
        {
            cx = x1 - r;
            cy = y0 + r;
        }
        else
            return true;
    }
    else if (y > y1 - r)
    {
        if (c.bl && x < x0 + r)
        {
            cx = x0 + r;
            cy = y1 - r;
        }
        else if (c.br && x > x1 - r)
        {
            cx = x1 - r;
            cy = y1 - r;
        }
        else
            return true;
    }
    else
        return true;

    dx = x - cx;
    dy = y - cy;
    return dx * dx + dy * dy <= r * r;
}

/* The colour of one pixel of a button: the border, the fill, or black outside.
 *
 * The border is the shape minus a copy of itself inset by BTN_BORDER. That is
 * exact rather than approximate: inset every edge by t and reduce the radius by
 * t, and the result is the set of points at distance t from the original, which
 * is the offset curve for a convex shape. So the border is the same width on
 * the straight edges as around a corner, with no thickening at the arcs. */
static uint16_t button_pixel(int x, int y, int x0, int y0, int x1, int y1, corners_t c)
{
    int b = BTN_BORDER;
    int r = BTN_RADIUS - b;

    if (!button_has(x, y, x0, y0, x1, y1, BTN_RADIUS, c))
        return 0x0000;
    if (x1 - x0 + 1 <= 2 * b || y1 - y0 + 1 <= 2 * b)
        return COL_FILL;                 /* too small to border; no button is */
    if (!button_has(x, y, x0 + b, y0 + b, x1 - b, y1 - b, r < 0 ? 0 : r, c))
        return COL_EDGE;
    return COL_FILL;
}

/* Stream the band just filled in s_tx, continuing the window the caller opened.
 *
 * CS is held across bands so the panel sees one window -- the first row of a
 * new window is partly swallowed, and four windows swallowed four rows. The
 * first transfer carries RAMWR and the rest are continuations with no command
 * and no address. */
static void push_band(bool *first, size_t npix)
{
    spi_transaction_ext_t t = { 0 };

    if (*first)
    {
        t.base.flags = SPI_TRANS_MODE_QIO;
        t.base.cmd = CMD_PIXELS;
        t.base.addr = ADDR_RAMWR;
        *first = false;
    }
    else
    {
        t.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD |
                       SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
        t.command_bits = 0;
        t.address_bits = 0;
        t.dummy_bits = 0;
    }
    t.base.tx_buffer = s_tx;
    t.base.length = npix * 16;
    (void)spi_device_polling_transmit(s_spi, (spi_transaction_t *)&t);
}

/* ---- labels on the margins ---------------------------------------------
 *
 * The margins are painted with the desktop's own 8x16 font, at its own size,
 * so a button's label is the same text as everything else on the glass rather
 * than a second typeface invented for the margins. The font is the one the
 * terminal uses (font8x16.c, CELL_W by CELL_H), and the labels are centred in
 * whatever rectangle they are given, which is what keeps them inside a rounded
 * button without the caller doing arithmetic twice.
 *
 * Yellow, because it reads on the dark grey face without competing with the grey
 * border around it -- and because it was chosen while the faces were green, is
 * still legible now, and changing a colour that works is not an improvement.
 */
#define LABEL_FG 0xFFE0          /* RGB565(31,63,0): yellow */

/* Where a label's glyph box starts, so the per-pixel test below is two
 * subtractions and a table lookup. The length is kept here rather than measured
 * per pixel: the test runs for every green pixel of every button, and strlen on
 * a short string is cheap but not free enough to call 40,000 times at boot. */
typedef struct
{
    const char *text;
    int x, y;
    int len;
} label_pos_t;

static int label_width(const char *s)
{
    return (int)strlen(s) * CELL_W;
}

/* Centre the text in the rectangle, but never let it start before the
 * rectangle: a label wider than its button would otherwise spill into the
 * separator, and truncation on the left reads as a blank button. */
static void label_place(const char *s, int x0, int x1, int y0, int y1, label_pos_t *out)
{
    int w = label_width(s), h = CELL_H;
    int cx = x0 + (x1 - x0 + 1 - w) / 2;
    int cy = y0 + (y1 - y0 + 1 - h) / 2;

    out->text = s;
    out->x = cx < x0 ? x0 : cx;
    out->y = cy < y0 ? y0 : cy;
    out->len = (int)strlen(s);
}

/* Is this pixel part of the label? Bit 7 of each glyph row is its leftmost
 * pixel, which is the same convention paint_cell() uses for the terminal. */
static bool label_bit(const label_pos_t *p, int x, int y)
{
    int dx = x - p->x, dy = y - p->y;
    int idx = dx / CELL_W;
    const uint8_t *glyph;

    if (dx < 0 || dy < 0 || dy >= CELL_H || idx >= p->len)
        return false;
    glyph = font8x16_glyph((uint8_t)p->text[idx]);
    return glyph && (glyph[dy] & (0x80 >> (dx % CELL_W))) != 0;
}

/* The corners of bottom bar s.
 *
 * The top edge faces the desktop, so both top corners are round on every bar --
 * that is what opens the gap between neighbours. The bottom edge is the glass,
 * where only the two outer ends of the strip are capped. */
static corners_t bar_corners(int s)
{
    corners_t c;

    c.tl = true;
    c.tr = true;
    c.bl = s == 0;
    c.br = s == BTN_N - 1;
    return c;
}

static void paint_buttons(void)
{
    /* One sacrificial column: exactly one pixel is lost at the top-left of a
     * window (docs/panel-geometry.md, "The first row and column are
     * swallowed"), so the window starts one column before the first bar and
     * the dropped pixel is a black one. Column 66 is in the margin below the
     * left column, which stops at row 460, so it costs nothing. A row cannot be
     * spent the same way -- that would be the framebuffer's last row. */
    const int wx = BTN_X0 - 1;                       /* 66 */
    const int width = BTN_X1 - wx + 1;               /* 321 */
    const int band = (int)(sizeof(s_tx) / sizeof(s_tx[0])) / width;
    label_pos_t label[BTN_N];
    int y = BTN_Y0;
    bool first = true;

    if (band < 1)
        return;

    /* One label per bar, centred inside the corner radius rather than in the
     * bar's full rectangle, so a label cannot sit in a rounded corner and be
     * clipped by the border. */
    for (int s = 0; s < BTN_N; s++)
    {
        int g0 = s == 0 ? BTN_X0 : btn_slot_x(s) + BTN_SEP;
        int g1 = btn_slot_x(s + 1) - 1;

        label_place(btn_label(s), g0 + BTN_RADIUS, g1 - BTN_RADIUS,
                    BTN_Y0 + BTN_RADIUS, BTN_Y1 - BTN_RADIUS, &label[s]);
    }

    /* The window is the whole strip; every transfer below continues it, so CS
     * stays low and the panel sees one window rather than one per band. */
    set_window(wx, BTN_Y0, width, BTN_Y1 - BTN_Y0 + 1);
    spi_device_acquire_bus(s_spi, portMAX_DELAY);
    gpio_set_level(LCD_CS, 0);

    while (y <= BTN_Y1)
    {
        int rows = BTN_Y1 - y + 1 > band ? band : BTN_Y1 - y + 1;

        for (int r = 0; r < rows; r++)
        {
            int yy = y + r;

            for (int i = 0; i < width; i++)
            {
                int x = wx + i;
                uint16_t c = 0x0000;

                if (x >= BTN_X0 && x <= BTN_X1)
                {
                    /* Which bar's columns this is, if any: a separator column
                     * belongs to no bar, which is what leaves the gap. Green
                     * starts two columns into every bar but the first, and ends
                     * one column before the next bar begins. */
                    for (int s = 0; s < BTN_N; s++)
                    {
                        int g0 = s == 0 ? BTN_X0 : btn_slot_x(s) + BTN_SEP;
                        int g1 = btn_slot_x(s + 1) - 1;

                        if (x < g0 || x > g1)
                            continue;

                        c = button_pixel(x, yy, g0, BTN_Y0, g1, BTN_Y1,
                                         bar_corners(s));
                        /* Gated on the fill, so a glyph pixel can never eat
                         * into the border: the label is centred well inside the
                         * inset shape, but the gate is what makes that a rule
                         * rather than a coincidence of the font's size. */
                        if (c == COL_FILL && label_bit(&label[s], x, yy))
                            c = LABEL_FG;
                        break;
                    }
                }

                s_tx[r * width + i] = swap16(c);
            }
        }

        push_band(&first, (size_t)(rows * width));

        y += rows;
    }

    gpio_set_level(LCD_CS, 1);
    spi_device_release_bus(s_spi);
}

/*
 * THE BUTTONS DOWN EACH EDGE.
 *
 * The same treatment as the strip below the framebuffer, turned through a right
 * angle: slots stacked down the margins the framebuffer does not reach,
 * with a 10 px radius on the corners that face a neighbour. Both margins get
 * it, mirrored, so the column on each side of the desktop is the same object
 * seen from a different side.
 *
 * Which corners are which follows the strip rather than being chosen again. The
 * strip sits on the bottom glass edge, so its corners at the separators are cut
 * on the top edge, facing the desktop. A column sits on a side glass edge, so
 * its corners at the separators are cut on the edge that faces the desktop --
 * the right edge of the left column, the left edge of the right one.
 *
 * The two end buttons are round on all four corners. That is what stops them
 * running into the glass: with the corner cut, the green recedes from both the
 * desk-facing edge and the glass edge at the top of the first and the bottom of
 * the last, so the column ends in a finished cap rather than a square one. The
 * three in between keep square glass-side corners, because they are a
 * continuous run down the glass and there is nothing there to turn.
 *
 * Geometry: the visible glass is controller 22..431, the framebuffer occupies
 * 67..386, so each margin is 45 columns running the framebuffer's own height,
 * rows 45..460. The buttons therefore finish exactly at the framebuffer's top
 * and bottom edges, one at each, which is where the desktop starts and stops
 * beside them. The strip below starts at 461, so a column and the strip do not
 * overlap. The margin above the framebuffer is left alone: the operator intends
 * it for status icons. Those bounds are edges.h's, along with the slot
 * arithmetic, because the touch driver needs the same numbers: a button the
 * painter and the hit test disagree about is a button that does something other
 * than what it looks like.
 */
 
/* The corners of column button s, given which side the desktop is on.
 *
 * The desk-facing edge is the right one for the left column and the left one
 * for the right column, and both of its corners are round on every button --
 * that is what opens the gap between neighbours. The glass-facing edge is a
 * continuous run, where only the two ends of the column are capped. */
static corners_t column_corners(int s, bool desk_right)
{
    corners_t c;

    if (desk_right)
    {
        c.tl = s == 0;
        c.tr = true;
        c.bl = s == COL_N - 1;
        c.br = true;
    }
    else
    {
        c.tl = true;
        c.tr = s == 0;
        c.bl = true;
        c.br = s == COL_N - 1;
    }
    return c;
}

/* One column, at controller x0..x1. desk_right says which side the desktop is
 * on, which is the only thing that differs between the two. */
static void paint_column(int x0, int x1, bool desk_right)
{
    /* One sacrificial row and column: exactly one pixel is lost at the top-left
     * of a window (docs/panel-geometry.md, "The first row and column are
     * swallowed"). The row is 44, black margin, and the column is one to the
     * left of the green -- 21 on the left column, which is off the glass, and
     * 386 on the right, which is the framebuffer's last column and is repainted
     * by the desktop's first frame a moment later. The bottom strip already
     * pays its column the same way at 66, and the operator has seen no artefact
     * at that edge, so the sacrifice is known to work rather than assumed. */
    const int wx = x0 - 1;
    const int wy = COL_Y0 - 1;
    const int width = x1 - wx + 1;                  /* 46 */
    const int height = COL_Y1 - wy + 1;             /* 417 */
    const int band = (int)(sizeof(s_tx) / sizeof(s_tx[0])) / width;
    int y = wy;
    bool first = true;

    if (band < 1)
        return;

    /* The window is the whole column; every transfer below continues it. */
    set_window(wx, wy, width, height);
    spi_device_acquire_bus(s_spi, portMAX_DELAY);
    gpio_set_level(LCD_CS, 0);

    while (y < wy + height)
    {
        int rows = wy + height - y > band ? band : wy + height - y;

        for (int r = 0; r < rows; r++)
        {
            int yy = y + r;

            for (int i = 0; i < width; i++)
            {
                int x = wx + i;
                uint16_t c = 0x0000;

                if (x >= x0 && x <= x1)
                {
                    /* Which button's rows this is, if any: a separator row
                     * belongs to no button, which is what leaves the gap. The
                     * fill starts two rows into every slot but the first, and
                     * ends one row before the next slot begins. */
                    for (int s = 0; s < COL_N; s++)
                    {
                        int g0 = s == 0 ? COL_Y0 : col_slot_y(s) + BTN_SEP;
                        int g1 = col_slot_y(s + 1) - 1;

                        if (yy < g0 || yy > g1)
                            continue;

                        c = button_pixel(x, yy, x0, g0, x1, g1,
                                         column_corners(s, desk_right));
                        break;
                    }
                }

                s_tx[r * width + i] = swap16(c);
            }
        }

        push_band(&first, (size_t)(rows * width));

        y += rows;
    }

    gpio_set_level(LCD_CS, 1);
    spi_device_release_bus(s_spi);
}

static void paint_task(void *arg)
{
    (void)arg;

    for (;;)
    {
        /* The rows that changed, flushed as contiguous runs.
         *
         * The previous version kept one span from the topmost changed row to
         * the bottommost and pushed that. It repaints far more than it needs
         * to when two changes are far apart -- one cell at the top of the
         * screen and one at the bottom made the span the whole panel, which is
         * what the once-a-second terminal-size query did: it parks the cursor
         * in the bottom-right corner to measure the terminal, so the cursor
         * cell near the top and the cell it touched at the bottom stretched
         * the span over all 400 rows. A run per contiguous block of rows means
         * a screen that changed in two places costs two bands, not a frame.
         *
         * The column extent of each run is tracked too. Rows alone are not
         * enough: this desktop repaints a whole column of icons on a single
         * pointer move, which is one contiguous run of 23 rows but only 8
         * pixels wide -- pushed as a full-width band that is 96 per cent of
         * the framebuffer for a change that touches one column. */
        int run_lo = -1, run_hi = -1;   /* pixel rows of the open run */
        int run_x0 = 0, run_x1 = 0;     /* pixel columns of the open run */
        /* Where the cursor was drawn last frame; the cell it leaves has to be
         * painted back, and this is what makes that happen exactly once. */
        int old_cur_col = s_cur_col, old_cur_row = s_cur_row;

        s_cur_col = s_cur_row = -1;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        xSemaphoreTake(s_lock, portMAX_DELAY);
        for (int r = 0; r < s_rows; r++)
        {
            int row_x0 = DISPLAY_W, row_x1 = 0;

            for (int c = 0; c < s_cols; c++)
            {
                td_vcell_t want;
                td_vcell_t *have = &s_shown[r * TD_VT_MAX_COLS + c];
                bool cur = s_vt->cursor_visible &&
                           s_vt->cx == c && s_vt->cy == r;

                want = s_vt->cells[r * TD_VT_MAX_COLS + c];
                if (want.ch == have->ch && want.fg == have->fg &&
                    want.bg == have->bg && !cur &&
                    !(old_cur_col == c && old_cur_row == r))
                    continue;               /* and nothing to un-invert */
                paint_cell(c, r, want, cur);
                *have = want;
                if (cur)
                {
                    s_cur_col = c;
                    s_cur_row = r;
                }
                if (c * CELL_W < row_x0)
                    row_x0 = c * CELL_W;
                if ((c + 1) * CELL_W > row_x1)
                    row_x1 = (c + 1) * CELL_W;
            }

            if (row_x1 > row_x0)
            {
                if (run_lo >= 0 && r * CELL_H == run_hi)
                {
                    /* Its first row follows the open run: extend. */
                    if (row_x0 < run_x0)
                        run_x0 = row_x0;
                    if (row_x1 > run_x1)
                        run_x1 = row_x1;
                    run_hi = (r + 1) * CELL_H;
                }
                else
                {
                    if (run_lo >= 0)
                        flush_rect(run_x0, run_lo, run_x1, run_hi);
                    run_lo = r * CELL_H;
                    run_hi = (r + 1) * CELL_H;
                    run_x0 = row_x0;
                    run_x1 = row_x1;
                }
            }
            else if (run_lo >= 0)
            {
                flush_rect(run_x0, run_lo, run_x1, run_hi);
                run_lo = -1;
            }
        }
        if (run_lo >= 0)
            flush_rect(run_x0, run_lo, run_x1, run_hi);
        s_vt->dirty = false;
        xSemaphoreGive(s_lock);
    }
}

/* ---- td_vterm plumbing ------------------------------------------------- */

static void vt_reply(void *user, const char *data, int len)
{
    (void)user;
    if (s_reply)
        s_reply(data, len);
}

int display_cols(void) { return s_cols; }
int display_rows(void) { return s_rows; }

bool display_init(display_reply_fn reply)
{
    spi_bus_config_t bus = {
        .mosi_io_num = LCD_D0,
        .miso_io_num = LCD_D1,
        .sclk_io_num = LCD_SCK,
        .quadwp_io_num = LCD_D2,
        .quadhd_io_num = LCD_D3,
        .max_transfer_sz = 0x40000 + 8,
        /* GPIO_PINS routes the quad lines through the GPIO matrix rather
         * than IO_MUX, which is what LilyGo's init asks for. */
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
    };
    /* As LilyGo: command_bits and address_bits are fixed on the DEVICE, chip
     * select is -1 because it is driven by hand, and the device is
     * half-duplex so the command/address phases are output-only. */
    spi_device_interface_config_t dev = {
        .command_bits = 8,
        .address_bits = 24,
        .mode = 0,
        .clock_speed_hz = LCD_SPI_HZ,
        .spics_io_num = -1,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 8,
    };

    if (spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI2 bus did not start");
        return false;
    }
    if (spi_bus_add_device(SPI2_HOST, &dev, &s_spi) != ESP_OK)
    {
        ESP_LOGE(TAG, "the panel device did not register");
        return false;
    }

    s_lock = xSemaphoreCreateMutex();
    s_reply = reply;
    palette_init();
    if (!panel_init())
        return false;

    s_vt = heap_caps_calloc(1, sizeof(*s_vt),
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_shown = heap_caps_calloc(TD_VT_MAX_ROWS * TD_VT_MAX_COLS,
                               sizeof(*s_shown),
                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_fb = heap_caps_malloc((size_t)DISPLAY_W * DISPLAY_H * 2,
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_vt || !s_shown || !s_fb)
    {
        ESP_LOGE(TAG, "no PSRAM for the terminal (%u B vt, %u B fb)",
                 (unsigned)sizeof(*s_vt),
                 (unsigned)(DISPLAY_W * DISPLAY_H * 2));
        return false;
    }

    /*
     * A test pattern before anything else. This is the only way to tell a
     * dark panel apart from a panel that is lit but showing a desktop which
     * never painted: three bands of solid colour that survive until the
     * first real paint. If these do not appear, the problem is power or the
     * QSPI framing, and the desktop is irrelevant.
     */
    for (int y = 0; y < DISPLAY_H; y++)
    {
        uint16_t c = (y < DISPLAY_H / 3) ? 0xF800        /* red   */
                   : (y < 2 * DISPLAY_H / 3) ? 0x07E0    /* green */
                   : 0x001F;                             /* blue  */
        for (int x = 0; x < DISPLAY_W; x++)
            s_fb[(size_t)y * DISPLAY_W + x] = c;
    }
    flush_rect(0, 0, DISPLAY_W, DISPLAY_H);
    ESP_LOGI(TAG, "test pattern written: %dx%d", DISPLAY_W, DISPLAY_H);

    /* Then wipe the margins this port never writes, which still hold whatever
     * the previous firmware left in the controller's RAM. */
    panel_fill(0);

    /* The three button bars below the framebuffer, images only for now. */
    paint_buttons();

    /* Five down each margin, same treatment, mirrored. */
    paint_column(LEFT_X0, LEFT_X1, true);
    paint_column(RIGHT_X0, RIGHT_X1, false);

    s_cols = DISPLAY_MAX_COLS;
    s_rows = DISPLAY_MAX_ROWS;
    td_vterm_init(s_vt, s_cols, s_rows);
    /* The reply hook is a field set after init, not an init argument: the
     * terminal answers ESC [ 6 n and friends through it, and those answers
     * have to reach the desktop as input. */
    s_vt->reply = vt_reply;
    s_vt->reply_user = NULL;
    s_vt->cell_w = CELL_W;
    s_vt->cell_h = CELL_H;

    /* A cell that has never been drawn must not match anything, so the shown
     * grid starts as a value no real cell can hold. */
    memset(s_shown, 0xFF, TD_VT_MAX_ROWS * TD_VT_MAX_COLS * sizeof(*s_shown));

    if (xTaskCreatePinnedToCore(paint_task, "tdpaint", 4096, NULL, 4,
                                &s_paint_task, 1) != pdPASS)
    {
        ESP_LOGE(TAG, "the painter did not start");
        return false;
    }
    return true;
}

void display_write(const uint8_t *buf, int len)
{
    if (!s_vt)
        return;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    td_vterm_write(s_vt, buf, len);
    xSemaphoreGive(s_lock);

    xTaskNotifyGive(s_paint_task);
}

/* ---- pictures: not supported ------------------------------------------- */

bool display_picture(void *user, int x, int y, const uint8_t *pixels, int w, int h)
{
    (void)user; (void)x; (void)y; (void)pixels; (void)w; (void)h;
    return false;
}
bool display_picture_rgb(void *user, int x, int y, const uint16_t *pixels, int w, int h)
{
    (void)user; (void)x; (void)y; (void)pixels; (void)w; (void)h;
    return false;
}
bool display_picture_runs(void *user, int x, int y, const uint16_t *runs, size_t words, int w, int h)
{
    (void)user; (void)x; (void)y; (void)runs; (void)words; (void)w; (void)h;
    return false;
}
void display_scanout_snapshot(uint32_t out[12]) { memset(out, 0, 12 * sizeof(uint32_t)); }
void display_scanout_info(uint32_t out[8]) { memset(out, 0, 8 * sizeof(uint32_t)); }
