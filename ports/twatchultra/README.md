# LilyGo T-Watch Ultra

An ESP32-S3 with 16 MB flash and 8 MB PSRAM, wearing a 410x502 AMOLED with a
CO5300 controller on a four-lane QSPI bus, a CST9217 capacitive touch panel,
an AXP2101 PMU and a DRV2605 haptic driver. This port puts the desktop on the
watch's own screen and touch panel, with the keyboard on the ESP32-S3's
built-in USB Serial/JTAG port.

```
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash
```

No baud argument is needed or used: the link is USB, and line speed is
ignored.

The desktop is **40x25 cells** of 8x16. The glass is bigger than that; see the
second trap below for why the desktop does not use all of it.

## Three things that are not like the other ports

**1. The panel is QSPI, and ESP-IDF 5.3's `esp_lcd` has no quad mode.**
Every other ESP-IDF port here drives its panel through `esp_lcd` — the
CrowPanel's RGB panel driver, or a `esp_lcd_panel_io_spi` 4-wire device. That
is not available for this controller: `esp_lcd_panel_io_spi_config_t` has
`spi_mode` for the traditional 0-3 modes and nothing for QIO, and the CO5300's
framing needs four data lanes to push a frame at a usable rate. So
`main/display.c` drives the SPI master driver directly and writes the CO5300
command set by hand. It is the only file in the port with no `esp_lcd` in it,
and the init sequence in it came from this watch's factory firmware rather
than from a datasheet.

**2. The desktop does not use the whole glass.** The panel is 410x502 with
45-degree chamfers, so the corners are not addressable as a rectangle. The
largest unclipped rectangle inside the visible area is **320x400**, which is
what `DISPLAY_W`/`DISPLAY_H` are set to and what the panel window is
programmed to. Setting a bigger window draws into the chamfers and produces a
band of nonsense along two edges.

**3. PSRAM is QUAD, and getting this wrong looks like a dead board.** The
trap is the mirror image of the CrowPanel's: this S3 has *quad* PSRAM, and
`CONFIG_SPIRAM_MODE_OCT=y` — which is what you get by copying a CrowPanel or
`ports/esp32s3` config — stops the board at boot with

```
E octal_psram: PSRAM ID read error: 0x00000000
```

and nothing else, because the abort happens before anything can report it.
`sdkconfig.defaults` here sets `CONFIG_SPIRAM_MODE_QUAD=y` for that reason.

## The partition table replaces the stock layout

This watch normally runs something else entirely. `partitions.csv` here is the
`ports/nanoesp32` shape — two 3 MB OTA slots and a 9.9 MB `/fs` — and flashing
it overwrites whatever occupied those offsets. Read the flash out first if it
holds anything that cannot be regenerated:

```
esptool --port /dev/ttyACM0 --before usb_reset read_flash 0 0x1000000 backup.bin
```

## Wiring

`board.example.conf` has every pin, with the reason for each. All of this
board's peripherals share one I2C bus (PMU 0x34, expander 0x20, RTC 0x51,
haptic 0x5A, touch 0x1A), which is why the touch address is spelled out rather
than probed.

## What is not here

No picture support: the desktop is told the panel has no Sixel, and the
`display_picture*` hooks return false. No sound, no haptics, no PMU. Those are
all present on the board and none of them are needed to show a desktop.

## Status

**Builds clean** with ESP-IDF v5.3.1 on 2026-11-11: `tinydesk.bin` is
2,327,472 bytes, 26% free in the 3 MB app slot, with no warnings from the
port's own sources under `-Werror`.

**Not yet flashed to a board.** The parts most likely to need adjustment on
first contact, in order:

1. **The QSPI framing in `display.c`.** No other ESP-IDF port here drives a
   panel by hand, so the command/address/lane structure is derived rather than
   copied from working code. `co5300_cmd()` sends the opcode with an 8-bit
   command phase and no address phase, all on four lanes. If the panel stays
   dark, that framing is the first thing to question -- and the second is
   whether the CO5300 wants a 24-bit address carrying the parameter count, as
   some QSPI controllers do.
2. **`lcd.spi_mhz`.** 40 MHz is what the board's Linux driver settled on, but
   the panel's own maximum is worth confirming before trusting it.
3. **The touch unlock.** The handshake follows the hardware's measured
   behaviour, so a failure here should be visible as a log line rather than a
   dead panel: `bring_up()` logs the checkcode it read and the value that
   failed the unlock.
