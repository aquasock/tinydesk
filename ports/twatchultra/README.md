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

**Works on hardware, 2026-11-11.** The desktop renders on the AMOLED: the
sign-in screen with its header, account list and buttons, and the touch panel
drives the mouse. Getting there took four attempts and every one of them
failed silently, so the failure modes are written down here.

### What was wrong, in the order it was found

1. **No panel power.** The AMOLED's rail is switched by XL9555 **P07** and
   nothing else; on a cold boot the expander has every pin an input. The panel
   was unpowered while a perfectly correct init sequence went into it.
   `main/power.c` raises ALDO2 on the AXP2101 and then closes P07.

2. **The QSPI framing.** Command and address go out with
   `SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR` on a device configured
   `command_bits = 8, address_bits = 24, spics_io_num = -1,
   SPI_DEVICE_HALFDUPLEX`. Two earlier attempts -- everything on four lanes,
   then command and address on one lane -- were both wrong and both silent.
   Read `LilyGoDispQSPI` in LilyGoLib before guessing at this protocol.

3. **The reset pulse.** 200 ms HIGH, 300 ms LOW, 200 ms HIGH. A shorter pulse
   leaves the controller held in reset and every later command is discarded.

4. **The framebuffer origin.** The 320x400 rectangle sits at (45,45) on the
   glass, and the controller's RAM starts a further 22 columns left, so a
   column is written at `x + 45 + 22` and a row at `y + 45`. Without both the
   picture is shifted up and to the left.

### Everything here came from LilyGo or from the Escapement project

The CO5300 framing and init table are transcribed from LilyGo's own firmware
(LilyGoLib, `src/display/LilyGoDispInterface.cpp`). The panel geometry, the
expander register map and the CST9217 handshake come from the Escapement
project's measured reference (`.ai/core-reference.md`, records BRD-003,
BRD-004, LCD-003 and TCH-001..010). None of it is from a datasheet.

### Subsystem status

A port is the board, not the apps, so this is what the BOARD needs to make
TinyDesk whole:

| System | Status |
| --- | --- |
| Display (CO5300 QSPI) | **Works.** Verified on the glass: the sign-in screen renders, correctly positioned. |
| Touch (CST9217) | Bring-up rewritten to LilyGo's own `getAttribute()` sequence and the report/acknowledge protocol measured on this board. **Not verified on the glass yet.** |
| Keyboard (USB Serial/JTAG) | Works; it is the link, and the desktop is driven through it. |
| `/fs` (littlefs, 9.9 MB) | Should work: `partitions.csv` defines it and the littlefs component is linked. Unverified. |
| WiFi | ESP-IDF native on this SoC, so no port code. Unverified; needs credentials via `wificonnect`. |
| OTA | `TD_UPDATE_BOARD="twatchultra"` so this image will not be replaced by a UART desktop's. No update feed exists for this board yet. |
| SD card | Hardware **unproven on this board** (see `board.example.conf`). Opt-in and commented out. |
| Battery / PMU | No TinyDesk UI consumes it. The AXP2101 is used only to raise ALDO2. |
| RTC | TinyDesk has no RTC hook -- it takes time from SNTP. The PCF85063A is unused. |
| Haptics | No TinyDesk integration. The DRV2605 is unused. |

### Known gaps

No picture/Sixel support, no sound, no haptics, no PMU battery reporting.

**The desktop's terminal-size poll is turned off for this port**
(`TD_SIZE_POLL_MS=0`). It exists upstream because a serial terminal cannot
report a resize; this panel's grid is fixed at build time, so there is nothing
to discover. It was not free here: the query parks the cursor in the
bottom-right corner to measure the terminal, and the cells it touched marked
rows at the top and the bottom of the screen changed, so the frame's dirty
region reached from row 0 to row 24 and repainted the whole panel once a
second. Turning it off needs a definition in the `tinydesk` component, which is
not this port's to edit, so it is set from `main/project_include.cmake` — the
one point ESP-IDF evaluates before components are registered. Setting it at
project scope in the top-level `CMakeLists.txt` does not work and the port
carried that mistake for a while: the definition reached 19 translation units
and not `src/td.c`, the one that reads it.

`TD_ACK_TIMEOUT_MS=0` is set the same way. Frame pacing exists so a slow remote
terminal never has two frames queued; here the terminal is this board's own
`td_vterm`, which answers `ESC[5n` and whose reply is fed back as desktop
input, so pacing was answered every time and never switched itself off — one
query, one flush and one reply per frame, controlling nothing.
