/*
 * ili9340.h - minimal driver for the ILI9340 (Adafruit 2.2" PiTFT HAT,
 * 320x240). The controller has its own pixel-addressable GRAM, so there is
 * no framebuffer in MCU RAM: a rectangle is written straight to the panel.
 *
 * This board is dedicated to TinyDesk, so the wiring is chosen around the
 * screen and the ESP32-S3's hardware SPI pins are used directly (IO_MUX, not
 * the GPIO matrix), which allows the panel's full 32 MHz.
 *
 *   signal   Nano pin   GPIO   PiTFT HAT (CONN1)
 *   ------   --------   ----   --------------------------------------
 *   SCK      D13         48    23  SCLK
 *   MOSI     D11         38    19  MOSI
 *   MISO     D12         47    21  MISO   (display-ID read-back)
 *   CS       D10         21    24  CE0
 *   DC       D9          18    18  GPIO25 / D/C
 *   3V3      3V3          -     1   (panel logic; 1 and 17 are joined)
 *   5V       VBUS         -     2   (backlight anode; 2 and 4 are joined)
 *   GND      GND          -     9, 14, 20, 25
 *
 * Two things the HAT does for itself, both read from its schematic
 * (Adafruit-PiTFT-2.2-Inch-HAT-PCB, rev B):
 *
 *  - RESET is generated on the board by the reset supervisor IC1, whose
 *    output drives U2 RESET with R6 as pull-up. No host GPIO is involved on
 *    a Pi either, so LCD_PIN_RST is -1.
 *  - The backlight is an NPN (Q1) low-side switch on the LED cathodes. Its
 *    base is pulled to 3.3 V through R10, so it is ON unless a host pulls it
 *    down, and the Pi's GPIO18 reaches it only through the solder jumper
 *    SJ2. With that left alone the backlight is always on, so LCD_PIN_BL is
 *    -1 unless software dimming is wanted.
 *
 * POWER. The HAT has no regulator: 3.3 V goes to the display logic and the
 * 5 V pin feeds the backlight LED anode, so both rails are required. On the
 * Nano ESP32 the 5 V source is VBUS (pin 12), not VIN: VIN is a 6-21 V
 * input to the MP2322GQH buck, and the datasheet states plainly that there
 * is no 5 V pin and that 5 V "can only be provided via the VBUS".
 *
 * GPIO 26..37 are the octal flash/PSRAM lines and GPIO 19/20 are USB, so
 * neither is touched here.
 */
#ifndef ILI9340_H
#define ILI9340_H

#include <stdbool.h>
#include <stdint.h>

#define LCD_PIN_SCK   48   /* D13 */
#define LCD_PIN_MOSI  38   /* D11 */
#define LCD_PIN_MISO  47   /* D12 */
#define LCD_PIN_CS    21   /* D10 */
#define LCD_PIN_DC    18   /* D9  */

/* Handled on the HAT itself; see the notes above. Set to a GPIO only if a
 * future carrier needs the host to drive them. */
#define LCD_PIN_RST   (-1)
#define LCD_PIN_BL    (-1)

/* Adafruit's overlay runs this panel at 32 MHz; the IO_MUX pins allow it. */
#define LCD_SPI_HZ    32000000

#define LCD_W 320
#define LCD_H 240

/* Initialise SPI and the panel. Returns false if SPI setup failed. */
bool ili9340_init(void);

/* Read the display ID (0x04). Returns the 32-bit value, 0 if nothing
 * answered on MISO -- which is what an absent or miswired panel looks like. */
uint32_t ili9340_read_id(void);

/* Fill the whole panel with one RGB565 colour. */
void ili9340_fill(uint16_t colour);

/* Write an RGB565 rectangle. */
void ili9340_blit(int x, int y, int w, int h, const uint16_t *px);

/* Set the drawing window and leave the panel expecting pixel data. */
void ili9340_window(int x, int y, int w, int h);

/* Push one pixel after ili9340_window(). */
void ili9340_pixel(uint16_t c);

#endif
