# Arduino Nano ESP32

The Nano ESP32 is an ESP32-S3 (NORA-W106) with 16 MB flash and 8 MB Octal
PSRAM, so the chip matches `ports/esp32s3`. What differs is the console: the
board has **no USB-UART bridge chip**. Its USB-C connector is wired to the
ESP32-S3's own USB Serial/JTAG peripheral (GPIO19/20), so this port serves the
desktop from `main/link_usj.c` the way `ports/esp32c6` does, not from a UART the
way `ports/esp32s3` does.

```
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash
```

No baud argument is needed or used: the link is USB, and line speed is ignored.

## Two things that cost time once already

**The partition table replaces the Arduino one, and data does not come back.**
The board ships with `app3M_spiffs9M_fact512k_16MB.csv`, whose 9 MB spiffs
volume starts at `0x610000` — the offset this project's `partitions.csv` gives
`otadata`. Flashing this firmware over an Arduino or PlatformIO sketch erases
that volume, and flashing the sketch back does **not** restore it, because the
partition table is not rebuilt from the sketch alone. Read the flash out first
if it holds anything that cannot be regenerated:

```
esptool --port /dev/ttyACM0 --before no_reset read-flash 0 0x1000000 backup.bin
```

**A reset over USB-JTAG is not a UART reset.** `esptool --after hard_reset`
toggles RTS, which does nothing on this port, and `--before no_reset` leaves the
chip parked in the stub loader with no USB response at all. If `idf.py flash`
seems to have produced a silent board, reset it into the application with:

```
esptool --port /dev/ttyACM0 --before usb_reset --after watchdog_reset read-mac
```

or unplug and replug. Plain `idf.py flash` is correct and needs neither.

## The shell's root is `/fs`

The desktop's Terminal starts in `/root`, which is `/fs/root` — the LittleFS
volume mounted from the `storage` partition. A shell path of `/Desktop/x` means
`/fs/Desktop/x`; the Editor's "Save as" dialog writes to the Desktop, so a file
saved as `untitled.txt` is at `/fs/root/Desktop/untitled.txt`.

## Not built here

The Shell edition is not part of this port. On this board the USB-C connector
belongs to the desktop, so a standalone shell would have to come up on UART0
(D0/D1) through an external adapter, which is a different bring-up.

## Software Update

This port defines `TD_UPDATE_BOARD="nanoesp32"`, which has no feed on the
official update site. Without it the port would inherit `esp32s3` as its board
name and Software Update would offer the UART build, replacing a working
firmware with one this board cannot display.

## Drawing on an SPI panel

`-DTD_LINK=lcd` swaps the desktop link for `link_lcd.c`, which renders the
desktop on an ILI9340 panel (Adafruit 2.2" PiTFT HAT, 320x240) instead of
printing it to the terminal. Input still arrives over USB, so the desktop is
driven from a PC terminal while it draws on the panel:

```
idf.py -DTD_LINK=lcd build
idf.py -p PORT flash
```

Nothing in the desktop core changes: `vt.c` is a small terminal emulator that
turns TinyDesk's ANSI stream into a cell grid, and `ili9340.c` writes changed
rows to the panel's GRAM. The default `-DTD_LINK=usb` keeps `link_usj.c`.

| signal | Nano pin | GPIO | PiTFT HAT (2x20) |
| --- | --- | --- | --- |
| SCK | D13 | 48 | 23 |
| MOSI | D11 | 38 | 19 |
| MISO | D12 | 47 | 21 |
| CS | D10 | 21 | 24 |
| D/C | **D9** | 18 | **22** (GPIO25) |
| 3V3 | 3V3 | — | 1 |
| 5V | **VBUS** | — | 2 |
| GND | GND | — | 6, 9, 14, 20, 25 |

Three things about this wiring cost time once already:

- **D/C is pin 22, not 18.** Pin 18 on a Pi header is GPIO24; this HAT routes
  D/C to GPIO25. With D/C on the wrong pin the panel lights but stays blank,
  because every byte looks like a command.
- **5 V comes from VBUS, not VIN.** The Nano's VIN is a 6-21 V input to the
  buck converter, and the datasheet states there is no 5 V pin. The HAT has no
  regulator of its own: 3.3 V feeds the display logic and 5 V feeds only the
  backlight anode, so both rails are required.
- **No reset or backlight wires.** The HAT generates RESET from its own
  supervisor (IC1), and its backlight NPN is pulled up so it is on whenever
  5 V is present. Both are set to `-1` in `ili9340.h`.

The panel gives 40x15 cells at 8x16. That is the floor of what TinyDesk
supports (`TD_MIN_COLS 40`, `TD_MIN_ROWS 12`), so windows are cramped; a
smaller font in `tools`-generated `font8x16.h` would trade legibility for room.

The display-ID register (0x04) reads back as zero on this module even though
the panel draws correctly, so the probe result is not a reliable wiring check
here; visible pixels are.
