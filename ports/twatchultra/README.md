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

The desktop is **40x26 cells** of 8x16. The glass is bigger than that; see the
second trap below for why the desktop does not use all of it.

**Apps open maximised** (`TD_OPEN_MAXIMIZED=1`, set in
`main/project_include.cmake`). On a grid this small a window that asks for a
desktop-sized size has most of itself below row 26, off the glass — this port's
Bluetooth window asks for 72x34. Maximised keeps the frame and title bar, so an
app's own layout is untouched and it is still movable and closable, while giving
it the whole desk area. Dialogs, tools and windows that ask to keep their own
size (`TD_WIN_OWN_SIZE`) are unaffected. Switching between open apps is **F6**
(or Alt+Tab): the taskbar is already full at 40 columns with `[Start]`, the user,
the network tray and the clock, so it has no room for per-window buttons.

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
what `DISPLAY_W`/`DISPLAY_H` were originally set to and what the panel window
is programmed to. Setting a bigger window draws into the chamfers and produces
a band of nonsense along two edges.

This build runs one row **taller** than that, 320x416 for 40x26. The measured
rectangle is inset 45 px on every side, so its bottom edge is at row 457 and
the extra row reaches 461 — four pixels past it. They are visible on the glass:
the chamfers cut the corners rather than the middle of the edges, so the lower
test row survives. Treat the width as the hard limit and the height as having a
few pixels of slack; `DISPLAY_H` back to 400 returns to the measured rectangle.

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
`display_picture*` hooks return false. No sound, no PMU. Those are
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
| Touch (CST9217) | **Works, and interrupt-driven.** `CST9220 410x502`, then `irq 12 up`: the checkcode, chip type, resolution and firmware reads all pass, and the controller's INT line (GPIO 12, active low, pull-up enabled as the part requires) now wakes the driver instead of a fixed 20 ms poll. Polling is kept as a fallback at two rates -- 20 ms while a finger is down, 250 ms when idle -- so a dead INT degrades touch to sluggish rather than losing it; `touch.int = -1` forces the polling path for an A/B. Measured on the board: 300 reads in 75 s at rest (4/s) against the ~50/s the old fixed poll produced, and `irq0` at idle, so the line is not spuriously asserting. The ISR touches no I2C -- it only notifies the task, which is what LilyGo's own handler does with an event-group bit. On the desktop the panel is **not** a pointer: a press on one of the three margins is a button, and a press on the framebuffer does nothing. An application that owns the whole screen -- kiosk mode or a full-screen window -- gets the panel back as an absolute pointer, `td_wm_app_owns_screen()` being the test. |
| Keyboard (USB Serial/JTAG) | Works; it is the link, and the desktop is driven through it. |
| Bluetooth (NimBLE, `bt.enable = 1`) | **Works, verified on the glass.** A combo keyboard-and-mouse paired from a scan into the Combo slot and drives the desktop; the pairing survives in NVS across reboots. The Mouse and Keyboard slots (one device each) are the same code path but have not been tried. |
| `/fs` (littlefs, 9.9 MB) | **Works.** Verified on the watch: `ls /` in the Terminal lists `etc home root sd shared tmp`, so the shell's own root is the littlefs partition, and a file written to `/tmp` read back byte-exact. |
| WiFi | **Works, verified from the host.** The watch associates and answers on 10.10.0.15: 12/12 pings at 5.2/34.3/64.4 ms min/avg/max, telnet on 23 presenting `tinydesk - log in with your TinyDesk account` with a login prompt, and the `tdsh-idf SSH/SFTP` server on 22 (wolfSSH 1.5.0, per-device ECDSA P-256) with `ssh status` reporting `running`, 0/1 clients and a 64 KiB private heap in PSRAM. No port code: this is ESP-IDF's own radio. **It does not associate at boot** -- `s_wifi_autoconnect` defaults to false in the shell's network policy, so the taskbar reads `Offline` after every reset until `network autowifi on` is set. Saved credentials live in `/fs/etc` and persist; the link coming up does not. |
| OTA | `TD_UPDATE_BOARD="twatchultra"` so this image will not be replaced by a UART desktop's. No update feed exists for this board yet. |
| SD card | **Works**, opt-in via the `sd.*` keys in `board.example.conf` (commented there by default). A 29 GB and a 128 GB FAT32 card each mounted at `/sd`, wrote a file that read back byte-exact, and appeared in Files. FAT32 only: FatFs in ESP-IDF 5.3 is built with `FF_FS_EXFAT` off, so an exFAT card cannot mount at any capacity. Hot-plug untested. |
| Battery / PMU | **Works.** `main/power.c` raises three rails -- ALDO1 for the SD card, ALDO2 for the display and BLDO2 for the speaker -- and reads the AXP2101's fuel gauge for System Monitor, which shows `Battery: 100%  4.17 V full`. A gauge reading above 100 is reported as `--%` rather than as a percentage, because a freshly powered AXP2101 has not settled and a wrong number on a battery is worse than no number. **Charging works, and `main/charge.c` checks it at every boot.** The port never writes a charge register, because the AXP2101's power-on defaults turned out to be right for the fitted cell and were measured rather than assumed: CV 4200 mV, fast charge 500 mA, precharge 125 mA, termination 125 mA, cell charger and NTC sensing both enabled. Against LilyGo's 1100 mAh 3.7 V pack that is 0.45 C, under both their "below 500 mA" guidance and the 0.5 C rule, with a 4.2 V full-charge voltage correct for a standard Li-ion cell. The boot log prints `cv4200 ic500` and warns only if a value drifts, so a future change that breaks charging cannot pass unnoticed. |
| Buttons (the three margins) | **Painted and live.** `main/edges.h` holds the geometry, `main/display.c` paints it and `main/touch.c` hit-tests it, so a button and its target cannot drift apart. **Left column**, top to bottom: `F10` Start menu, `Esc` cancel, `F6` window switcher, `Super+D` show desktop, `F11` full screen, labelled with the key name stood on end, reading top to bottom so it is upright when the watch is read from the right-hand edge. Labels are the terminal's ANSI green. **Bottom strip**, left to right: `Tab`, `Enter`, `Ctrl+Q` back -- the same path as a window's `[x]`, so a window that has something to lose can refuse. **Right column**, top to bottom: reserved, then Left, Right, Up, Down arrows. Not F1 to F5: those are app keys (Files renames on F2, Editor finds on F3) and do nothing on the desktop, where these are the actions that otherwise need a mouse. Green, 10 px radius where a button faces a neighbour, and the end buttons of both columns and of the strip are capped on all four corners so they stop at the framebuffer's edges rather than running into the glass. |
| Pictures (RGB565) | **Blits an RGB565 bitmap.** `display_picture_rgb()` copies it into the framebuffer and flushes the rectangle; it is registered as the desktop's RGB sink, so anything with a frame rather than a terminal can draw. 8-bit and run-length pictures are still unsupported. `pattern [0\|1\|2]` in the shell draws generated colour bars, a 1-pixel grid, or an edge frame for checking the copy path on its own -- bars catch a stride or byte-order fault, the grid an off-by-one, the frame clipping at the panel's edges. **A full-panel RGB565 frame cannot be a static array:** 256 KB overflowed `dram0_0_seg` by 88,896 bytes, because everything static in this component is internal DRAM by construction (`extram.lf` maps only libtinydesk, libtdsh and libmicropython to PSRAM) and internal DRAM has about 47 KB free. Frame buffers go in the heap, in PSRAM. |
| RTC (PCF85063A) | **Works.** `main/rtc.c` reads the chip at boot and seeds the system clock, so the desktop has a time with no network; it writes back when the time is set by Date & time or SNTP. Verified on the glass: the taskbar showed the correct UTC with Wi-Fi offline. Reading uses mainline's register map and refuses a stopped oscillator. |
| Haptics | **Works.** `main/haptic.c` drives the DRV2605 at 0x5A: library 1 (ERM, the chip's FEEDBACK register reads 0x36 so the motor is ERM and not LRA), internal trigger, Strong Click on each margin-button press only. The sequence follows LilyGo's own `initDrv()`/`vibrator()` split -- library and mode once at init, waveform and terminator on every tap. `board_haptic_power_on()` raises XL9555 P06 first, because the chip answers I2C with its supply off and that failure is silent. |
| NFC (ST25R3916) | **Reads cards.** `main/nfc.c` owns the rail and the bus; `main/nfc_scan.cpp` drives the part through ST's own stack, vendored unmodified under `components/st25r3916`. Verified on the glass with the tag that ships on a LilyGo board: `card A uid 1dd30d9e1c1080`, `sak 00 T2T ndef 19 bytes`, and the message decoded as the text record `T Hello LilyGo`. It is **not a PN532 and not on I2C** -- an ST25R3916 on the second SPI bus (CS 4, INT 5, MOSI/MISO/SCK shared with the SD card and the LoRa radio), fed by the AXP2101's **DLDO1** (register `0x99`, enable bit 7 of `0x90`), which nothing else in this port raised. `nfc` prints the identity (`id 2a` is rev 2 of the base silicon, not the 3916B, so the two use different analog tables), `nfc scan [seconds]` runs the field and prints whatever answers, and `nfc reg <hex>` reads one register. **The field is only on while a scan is running** -- a background poller would keep the antenna cycling for ever on a battery. Discovery polls A/B/F/V. The `ndef none` path -- a tag with a capability container but no message -- is written and **not yet exercised**, because the only tag to hand turned out to carry factory content; a tag with no NDEF at all is likewise untested. |
| GNSS (u-blox MIA-M10Q) | **Tracks satellites.** `main/gps.c` owns the rail and the UART, `main/nmea.c` parses; the module is on UART1 at 38400 8N1, TX 43 / RX 44, PPS 13, fed by the AXP2101's **BLDO1** (voltage `0x96`, enable bit 4 of `0x90` -- one register below the speaker's BLDO2 in both, so writing the wrong one powers the amplifier and leaves the receiver dark with nothing reporting a fault). Verified on the glass: `rail up`, `port 38400 8N1`, 624-900 bytes heard per second, and `gps raw 12` counting `GGA 12 RMC 12 GSA 48 GSV 54 GLL 12 VTG 12` with **5 satellites visible**. Over a 60 s monitor the receiver then reported `20:28:41` UTC with `s03` and `hd2.38` -- acquiring UTC means it decoded real ephemeris off the satellites, which is stronger evidence than a fix would be about the antenna. **No fix was obtained indoors**, and that is the expected limit rather than a fault: three satellites is one short of the four a 3D solution needs. `gps` probes and prints the model, `gps monitor [s]` streams UTC, fix quality, satellites used, HDOP, position and altitude once a second, and `gps raw [s]` counts sentence types and reports satellites visible -- which is what separates "no antenna" from "no fix". **`gps time [s]` sets the watch clock from the receiver**, and that works indoors because a time needs ephemeris and not a fix: three consecutive corrections thirty seconds apart each reported the clock `was off +0 s`/`+1 s`/`+0 s`, the write reaches the PCF85063A through the same `board_time_set_post()` path SNTP and Date & time use (log: `twatch-rtc: wrote 2026-11-12 20:34:54 UTC`), and the time survived a hard reset -- after reboot the shell read 13:46:16 MST against a host at 13:46:28, the gap being this agent's command-injection latency rather than clock error. A watch with no network can therefore be given an accurate time by holding it up. The UBX-MON-VER probe does **not** get a reply on this unit even though NMEA flows, so the model is not confirmed by the module itself; that is recorded rather than smoothed over. |

### What a partial update costs, and why it matters here

The panel is write-only and has no scanout, so every frame the desktop draws
has to be pushed over the bus. `main/display.c` keeps a full 320x400
framebuffer, compares the terminal's cells against what is on the glass, and
pushes only what changed -- as a **rectangle**, not a band.

Getting that wrong is expensive and does not look like a rendering bug. The
first version flushed contiguous runs of *rows* at full width. That is fine
until something changes one column from top to bottom, which this desktop does
on every pointer move: hovering repaints the whole icon column. 23 changed
rows is a contiguous run, so the run was pushed as 320 pixels wide for a
column that is 8 pixels wide -- **122,880 pixels, 96% of the framebuffer, per
mouse move**. Measured on the board: 641 us average per flush in steady state,
and a full-width run of that height cost about 16 ms, which is what "the touch
lags" feels like.

Tracking the column extent of each run turns that same move into 3,072 pixels,
a 40x reduction, and the band height is now sized to the rectangle so the whole
column goes out as one transaction rather than 24.

Two related things are worth knowing before changing this file:

- **The DMA source buffer is static and internal on purpose.** It started as a
  PSRAM allocation, which makes every transfer read through the cache, and
  moving it to a `MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA` allocation made the port
  boot to a dark screen: with the Bluetooth controller already holding its
  share, that request can fail, and with `CONFIG_ESP_CONSOLE_NONE` the failure
  has nowhere to be reported before the desktop link is up. It is a 10 KB
  static array now, which cannot fail.

- **Diagnostics are short deliberately.** The Log Viewer is 40 columns and
truncates silently. A whole fault stayed invisible for several builds because
the line reporting it was too long to read.

### Known gaps

No picture/Sixel support, no sound, no PMU battery reporting.

**Bluetooth is enabled for input.** The watch's input is a Bluetooth LE keyboard
and mouse (`bt.enable = 1`, `CONFIG_BT_ENABLED=y` with NimBLE as a central):
320 pixels across of glass is too small to type on, so there is no on-screen keyboard in
the intended flow. Until this was turned on the image was built without
Bluetooth at all, `bt_hid_start()` returned the empty stub, and the Mouse,
Keyboard and Bluetooth apps were absent from the start menu — that absence is
the quickest way to check the build.

**Touch has two write shapes, and they are not interchangeable.** A command-mode
entry is a two-byte write (`0xD1 0x01`); a register write carrying a value is
three (`hi lo value`). Sending the two-byte command as three addresses register
`0x00D1` instead, which the part accepts and ignores, so command mode is never
entered and the `0xD1FC` checkcode read returns normal-mode content. That was
the fault that kept touch from ever initialising.

**The Log Viewer is 40 columns and truncates.** A diagnostic line longer than
that is silently cut, which hid the checkcode fault across several builds. The
touch driver's messages are deliberately short for that reason.

**The touch controller NACKs at idle, by design.** Every read taken while it has
nothing to hand over fails, and ESP-IDF's `i2c.master` logs each one at ERROR —
three lines per poll, several hundred a second. That is normal for this part
(TCH-003), not a fault, so the driver sets that tag's log level to none.

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
