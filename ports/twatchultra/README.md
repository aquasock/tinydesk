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

This watch normally runs something else entirely. `partitions.csv` here splits
the 16 MB into four equal 4 MiB regions — two app slots, a 4 MB `/fs`, and 4 MB
for executable codec blobs — and flashing it overwrites whatever occupied those
offsets. Read the flash out first if it holds anything that cannot be
regenerated:

```
esptool --port /dev/ttyACM0 --before usb_reset read_flash 0 0x1000000 backup.bin
```

```
region 1  0x000000 - 0x400000
  nvs        0x0009000    64K    settings, accounts, WiFi, Bluetooth bonds
  phy_init   0x0019000     4K    RF calibration, regenerated when erased
  ota_0      0x0020000  3968K    the running app (1005 KB free)
region 2  0x400000 - 0x800000
  ota_1      0x0400000  3968K    the update target -- the real size budget
  coredump   0x07e0000    64K    panic backtraces; no console, so this is it
  otadata    0x07f0000     8K
region 3  0x800000 - 0xC00000
  storage    0x0800000     4M    littlefs, the shell's /
region 4  0xC00000 - 0x1000000
  modules    0x0C00000     2M    codec blobs, mapped as INSTRUCTIONS
  moddata    0x0E00000     2M    their .rodata, mapped as DATA
```

**Four equal 4 MiB regions**, so where something lives is answerable at a glance.

**The slots are 3968 KB, not 4 MiB, and that is the one place an even split
cannot be literal.** The bootloader, partition table, NVS and phy_init have to
sit inside region 1, so it has 176 KB less usable space than the other three --
and both slots must be the same size, because an update writes whichever one is
not running. 4+4+4+4 would need 176 KB more flash than the chip has. The 56 KB
left in region 2 after coredump and otadata is unused.

**NVS is 64 KB because 24 KB ran out of headroom.** Measured on the hardware in
November 2026: the old 24 KB region had five of its six pages full or active,
420 of 756 entry slots used and page sequence numbers of 66/100/101/102, so
garbage collection never stopped. What happens at the end of that is silent --
`nvs_flash_init()` returns `ESP_ERR_NVS_NO_FREE_PAGES` and `app_main()` answers
with `nvs_flash_erase()`, which deletes every account, saved network and
Bluetooth bond without saying anything. The TinyDesk author flagged this on
issue 20; every other ESP32 port here already gave NVS 64 KB.

**storage halved on 2026-11-17 and lost nothing**: it held 11 directories, 29
files and 23,047 bytes. The migration extracted the littlefs, rebuilt it at the
new size and wrote every file back, verified by SHA-256 of all 29. Shrinking a
littlefs partition by truncating it is how data is lost; see entry 25.

**The two slots are equal because a smaller one is the real limit.** An update
writes whichever slot is not running, so an image has to fit the smaller of the
two; 3 MB against 3008 KB meant the budget was 3008 KB while the table looked
like it promised 3 MB.

**Nothing above 0xC00000 moves**, so the four regions are independent: an app
slot can be re-flashed, or `storage` resized, without touching `modules` and
`moddata` or relinking a codec blob. Within region 1 the bootloader, the
partition table, NVS and `phy_init` all sit below 0x20000, which is why `ota_0`
starts there rather than at 0x10000.

**The 24 KB at 0x1A000-0x20000 IS NOT UNUSED, and `partitions.csv` still says it
is.** The table calls it deliberate padding that puts `ota_0` on a 64 KB
boundary, and it is that -- but slot B writes its stage trail at 0x1A000 and its
launch record at 0x1B000, both in the same range. The conflict is real and known:
both writes are instruments kept for diagnosing a personality with no console,
both are on the decision log's removal list, and a future cleanup has to decide
whether the padding is a boundary or a scratch area rather than claiming both.

## Slot B — a second personality from the same image

**The same app image is flashed into BOTH OTA slots.** Which slot the bootloader
picked decides which personality runs, and nothing else does:

```
ota_0   the desktop. board_startup_override() returns false and app_main()
        carries on exactly as it always has.
ota_1   slot B. A bare emulator personality -- the three emulators and the
        display, touch, audio and card drivers they need. No desktop, no shell,
        no WiFi, no SSH. The littlefs at `/` is never mounted.
```

**One image and two boot paths, not a second firmware.** A separate build forked
from this one would carry its own copy of `display.c`, and that file is where this
port's hardest-won panel defects live -- so the two copies would have to be kept in
step by hand, which is the exact drift `scripts/tamacore-sync.sh` exists to police
for another file. It would also need a second `sdkconfig`, and TOL-008 records
what a regenerated `sdkconfig` silently does to this board. A second boot path in
one binary costs flash and nothing else: the image is 3.0 MB against a 3968 KB
slot, so ~900 KB is free either way. What slot B saves is **runtime** -- no
`esp_wifi_init()` (41,500 bytes of internal DMA-capable RAM, SOC-005), no wolfSSH
heap floor, no littlefs mount, no desktop painter and no shell stacks competing
with the emulator for a core.

### How it is entered

Three paths, all ending in the same place:

- **The Start menu.** `●■ GBC`, `●♦ PokéMini` and `●☼ Tama V3` are native apps
  registered in `board_apps_register()` that call `slotb_launch(key)`. That
  writes the key to NVS, sets `ota_1` as the boot partition, claims the pixels,
  blacks the panel and restarts -- so the wearer sees black and then a game, not
  a reboot.
- **`slotb play <key>`** from a shell, which is the same switch, and
  **`slotb start`**, which clears the key and gives slot B's own menu.
- **`ota rollback` works too**, but it is not what the port uses: "rollback"
  means "the other slot", which is true only by accident, and it has a branch
  that marks the running image invalid when it is pending verification -- not
  what someone who wants to play a game is asking for.

**A shortcut boots straight into its app and never shows the menu.** The key is
read once and erased, so it is a one-shot instruction; leaving it set would make
every later entry start that app. If the cartridge then fails to load, the path
goes back to the desktop rather than falling through to the menu -- a wearer who
tapped "Game Boy Color" and picked a cartridge should not be shown a choice they
have already made.

### What is in it

The menu is drawn by `gbpick_list_run()` -- the same list renderer the cartridge
picker uses, so it gets the same paging, touch and margin behaviour for free.
Three apps, in this order:

```
Game Boy Color   pk_gb     the navigator wrapper, then gb_start()
Pokemon Mini     pk_pm     .min under /sd/media/pokemini
Tamagotchi V3    pk_tama   .bin under /sd/media/tama
```

**Each app is a row in `s_apps[]` and nothing else.** A row is what the menu
draws AND what `slotb play <key>` resolves through, so an app there cannot be
launchable-but-invisible or visible-but-unlaunchable -- and removing a row
changes two menus at once, deliberately. Each app's touch consumer is installed
once at start-up (`gb_install_touch_hook()` and its three siblings) and declines
unless it is the one running, which is what lets the menu hand the glass between
apps without touching the drivers again.

**Every path buffer is 256 bytes** (`s_gb_rom`, `s_pm_rom`, `s_tama_rom`). A
bucket directory plus a long cartridge name runs past 130, and a shorter buffer
fails *silently*: `strncpy` caps and NUL-terminates, producing a well-formed path
to the wrong file, which reads as "no ROM" and gets blamed on the card.

**Each start records the path the picker handed over** (`slotb_record("pick", ...)`)
*before* the emulator can refuse it, so a later "did not start" can be told apart
from a picker that built the wrong path.

### Peripheral bring-up, and the order that matters

`slotb_run()` re-raises everything `app_main()` would have raised after the
override returned, because slot B never returns to it:

```
board_power_rails_preinit()   SD rail (ALDO1), IMU rail, NFC/LoRa/SD CS parked high
tdsh_board_load()             board.conf, because the shell that loads it is absent
display_boot_pattern()        from board.conf, default off
display_init()                SPI2, the CO5300 init table, panel_fill(), paint task
display_pixels_claim(true)    take the glass from the painter display_init() began
statusbar_clock_present(false); statusbar_start()      the top strip
slotb_peripherals():          board_display_power_on()   PMU rail, then XL9555 P07
                              board_touch_reset_release()  P08, in reset until now
                              touch_init()                 the poll task
                              board_haptic_power_on(); haptic_init()
                              charge_watch_enable()
tdsh_sdcard_mount()           the card
gb/pm/tama_install_touch_hook()  one per app; each declines if not running
pwrkey_init(); pwrkey_set_app_key()   Pusher A
```

**THE ORDER ABOVE IS WHAT THE CODE DOES, AND IT DISAGREES WITH ITS OWN COMMENT.**
`slotb_peripherals()` documents its list as "the panel's rail and its expander
switch before `display_init()` touches the bus, or the driver drives a dark
panel" -- but `display_init()` is called in `slotb_run()` before
`slotb_peripherals()` runs at all, so the rail is raised *after* the panel driver
has started. The same comment admits why it works anyway: the XL9555's CONFIG and
LATCH registers survive an MCU reset, so a warm reboot leaves P07 driving the
panel on. On a cold boot every expander pin is an input, and the comment says so
in as many words -- the panel coming up was "luck as well as the rail being set".
This is written down here rather than silently reordered because it is a latent
cold-boot fault, not a documentation preference, and moving the call is a hardware
change that wants a test on a genuinely cold board.

**The first run of this module had dead touch, dead haptics and a dead Pusher A
while picture, audio and emulation were perfect**, and the cause was one mistake
repeated: it registered hooks into drivers whose tasks had never been started.
`touch_add_app_touch()` and `pwrkey_set_app_key()` install a consumer into an
existing poll loop; the loop is created by `touch_init()` and `pwrkey_init()`,
which only the desktop's `link_init()` was calling. Audio escaped because
`sound_stream_open()` calls `sound_init()` itself. **Registering a hook is not
starting a task.**

### How it is left

**Pusher A.** The press only raises a flag; the boot task then calls the app's
`stop()`, which writes SRAM and waits for its own tasks to leave, and only then
returns to the desktop. Doing the stop on the key task instead would reboot on
top of the save it had just started. That is why slot B installs its own key hook
rather than `gb.c`'s.

**Where "back" goes depends on where the wearer came from**, and that is a
parameter rather than a remembered local -- the previous version kept it in a
local that one path lost, which is how tapping "Game Boy Color" and picking a
cartridge ended up at the app menu. From a shortcut, every way out is the
desktop. From the menu, backing out of a game returns to the menu, so changing
games costs a tap and not a boot.

**Leaving erases `otadata` rather than pointing it at `ota_0`,** and the reason
is rollback protection. With `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` on, writing
a fresh `ota_0` entry makes it PENDING_VERIFY on the next boot; reset inside the
30-second trial and the bootloader sees an unconfirmed image, marks it ABORTED
and falls back to the other valid one -- which after a game session is slot B.
Landing in the emulator when you meant to be at the desktop is correct firmware
behaviour and bad user experience, and a wearer putting the watch down after a
game is exactly the person who power cycles within 30 seconds. With `otadata`
erased there is no second valid image for a rollback to choose, measured: a reset
at t=12 s still came up on the desktop. The cost is that `ota status` reports no
sequence number afterwards -- a report, not a behaviour.

**A broken slot B cannot brick the watch** for the same reason. `slotb_confirm()`
cancels the rollback and is called from exactly one place: inside `slotb_app_go()`
once an app has actually started. So a slot B that dies before that -- on the way
up, or sitting on its own menu -- is rolled back to the desktop by the bootloader
on the next reset.

### What it does not have

**No console and no log.** `CONFIG_ESP_CONSOLE_NONE` is set for the whole image,
and a picture owning the panel kills the USB mirror anyway (DSP-003). A hard hang
writes no coredump -- measured on the first run of this module, where the
coredump partition read back empty while the desktop returned.

**Two instruments exist for exactly that**, both read from a host with a
four-second `esptool read_flash`, and both on the decision log's removal list:

```
0x1A000  stage trail   magic "SLBS", one 4 KB sector, rewritten at each stage.
                       A value with no later stage after it is the only thing
                       that tells "stopped here" from "never started".
0x1B000  launch record magic "SBRC", {uptime_ms, why[48], path[256]}. Written
                       once per launch and once per failure -- for the one error
                       the glass cannot carry, because "cannot open the
                       cartridge" is 26 characters and the path that decides the
                       answer is 40 to 130 against a 38-character row.
```

**It does start the status strip**, which it did not until recently. The strip's
task lives in `statusbar_start()` inside `link_init()`, and `slotb_run()` runs
before `link_init()` is ever reached -- so for many cycles the top 44 rows were
simply dark in the personality that runs the emulators. It is started here, after
`display_init()` (whose `panel_fill()` would wipe it), with
`statusbar_clock_present(false)` because slot B never reaches `fill_sysinfo()`
and has no clock: with nothing claiming it the strip stays blank rather than
painting `--/--` placeholders, and the Tamagotchi claims it for its up-time
counter.

**No WiFi, no SSH, no shell, and no `board.conf` from the card.** Slot B takes the
built-in `board.conf` text that `app_main()` passes down, because the device copy
at `/etc/board.conf` lives on the littlefs the shell would have mounted. The one
setting that still works from there is `lcd.boot_pattern`, which is why the boot
bands are diagnosed identically on both personalities.

## Wiring

`board.example.conf` has every pin, with the reason for each. All of this
board's peripherals share one I2C bus (PMU 0x34, expander 0x20, RTC 0x51,
haptic 0x5A, touch 0x1A), which is why the touch address is spelled out rather
than probed.

## What is not here

No Sixel, and no 8-bit or run-length pictures — the `display_picture*` hooks
handle RGB565 only, which is what the emulators and the console need.

This section used to say "no picture support, no sound, no PMU". That was true
when the port could only draw a desktop; all three have since been added and the
subsystem table below records them. Left corrected here rather than deleted,
because a stale "not here" is how a working feature gets re-implemented.

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
| Sound (MAX98357A) | **Plays FLAC from the SD card, out of the speaker, by default.** `music <file.flac>` streams the file, decodes it, and writes PCM to an I2S channel (BCLK 9, WS 10, DOUT 11, `I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG` -- MSB framing latches every sample one bit early on this amplifier, which is a real bug that was found by listening to it). The rail is BLDO2, raised by `main/power.c`; with it down the part accepts every sample and outputs nothing, so an unpowered amplifier and a wrong I2S configuration look identical from the driver. **The decoder is not in the app image.** micro-flac v0.3.0 lives in the `modules` partition and executes from flash by XIP, costing no internal SRAM to load; its `.rodata` is in `moddata` because the instruction window cannot serve a data load. Measured on a 5:58 track: 15,792,504 frames, 0 errors, 0 short writes, internal SRAM 20,707 bytes free during playback and 33,727 after. The stream rate comes from the file's own STREAMINFO, so 48 kHz plays at 48 kHz. `modflac` is an alias of `music`, kept because the logs use the name. Deleting the inline decoder that used to be here recovered 32 KB of app image. See the modules/Makefile notes and MOD-001/MOD-002 in the Escapement reference. |
| Buttons (the three margins) | **Painted and live.** `main/edges.h` holds the geometry, `main/display.c` paints it and `main/touch.c` hit-tests it, so a button and its target cannot drift apart. **Left column**, top to bottom: `F10` Start menu, `Esc` cancel, `F6` window switcher, `Super+D` show desktop, `F11` full screen, labelled with the key name stood on end, reading top to bottom so it is upright when the watch is read from the right-hand edge. Labels are the terminal's ANSI green. **Bottom strip**, left to right: `Tab`, `Enter`, `Ctrl+Q` back -- the same path as a window's `[x]`, so a window that has something to lose can refuse. **Right column**, top to bottom: reserved, then Left, Right, Up, Down arrows. Not F1 to F5: those are app keys (Files renames on F2, Editor finds on F3) and do nothing on the desktop, where these are the actions that otherwise need a mouse. Green, 10 px radius where a button faces a neighbour, and the end buttons of both columns and of the strip are capped on all four corners so they stop at the framebuffer's edges rather than running into the glass. |
| Pictures (RGB565) | **Blits an RGB565 bitmap.** `display_picture_rgb()` copies it into the framebuffer and flushes the rectangle; it is registered as the desktop's RGB sink, so anything with a frame rather than a terminal can draw. 8-bit and run-length pictures are still unsupported. `pattern [0\|1\|2]` in the shell draws generated colour bars, a 1-pixel grid, or an edge frame for checking the copy path on its own -- bars catch a stride or byte-order fault, the grid an off-by-one, the frame clipping at the panel's edges. **A full-panel RGB565 frame cannot be a static array:** 256 KB overflowed `dram0_0_seg` by 88,896 bytes, because everything static in this component is internal DRAM by construction (`extram.lf` maps only libtinydesk, libtdsh and libmicropython to PSRAM) and internal DRAM has about 47 KB free. Frame buffers go in the heap, in PSRAM. |
| RTC (PCF85063A) | **Works.** `main/rtc.c` reads the chip at boot and seeds the system clock, so the desktop has a time with no network; it writes back when the time is set by Date & time or SNTP. Verified on the glass: the taskbar showed the correct UTC with Wi-Fi offline. Reading uses mainline's register map and refuses a stopped oscillator. |
| Haptics | **Works.** `main/haptic.c` drives the DRV2605 at 0x5A: library 1 (ERM, the chip's FEEDBACK register reads 0x36 so the motor is ERM and not LRA), internal trigger, Strong Click on each margin-button press only. The sequence follows LilyGo's own `initDrv()`/`vibrator()` split -- library and mode once at init, waveform and terminator on every tap. `board_haptic_power_on()` raises XL9555 P06 first, because the chip answers I2C with its supply off and that failure is silent. |
| NFC (ST25R3916) | **Reads cards.** `main/nfc.c` owns the rail and the bus; `main/nfc_scan.cpp` drives the part through ST's own stack, vendored unmodified under `components/st25r3916`. Verified on the glass with the tag that ships on a LilyGo board: `card A uid 1dd30d9e1c1080`, `sak 00 T2T ndef 19 bytes`, and the message decoded as the text record `T Hello LilyGo`. It is **not a PN532 and not on I2C** -- an ST25R3916 on the second SPI bus (CS 4, INT 5, MOSI/MISO/SCK shared with the SD card and the LoRa radio), fed by the AXP2101's **DLDO1** (register `0x99`, enable bit 7 of `0x90`), which nothing else in this port raised. `nfc` prints the identity (`id 2a` is rev 2 of the base silicon, not the 3916B, so the two use different analog tables), `nfc scan [seconds]` runs the field and prints whatever answers, and `nfc reg <hex>` reads one register. **The field is only on while a scan is running** -- a background poller would keep the antenna cycling for ever on a battery. Discovery polls A/B/F/V. The `ndef none` path -- a tag with a capability container but no message -- is written and **not yet exercised**, because the only tag to hand turned out to carry factory content; a tag with no NDEF at all is likewise untested. |
| GNSS (u-blox MIA-M10Q) | **Tracks satellites.** `main/gps.c` owns the rail and the UART, `main/nmea.c` parses; the module is on UART1 at 38400 8N1, TX 43 / RX 44, PPS 13, fed by the AXP2101's **BLDO1** (voltage `0x96`, enable bit 4 of `0x90` -- one register below the speaker's BLDO2 in both, so writing the wrong one powers the amplifier and leaves the receiver dark with nothing reporting a fault). Verified on the glass: `rail up`, `port 38400 8N1`, 624-900 bytes heard per second, and `gps raw 12` counting `GGA 12 RMC 12 GSA 48 GSV 54 GLL 12 VTG 12` with **5 satellites visible**. Over a 60 s monitor the receiver then reported `20:28:41` UTC with `s03` and `hd2.38` -- acquiring UTC means it decoded real ephemeris off the satellites, which is stronger evidence than a fix would be about the antenna. **No fix was obtained indoors**, and that is the expected limit rather than a fault: three satellites is one short of the four a 3D solution needs. `gps` probes and prints the model, `gps monitor [s]` streams UTC, fix quality, satellites used, HDOP, position and altitude once a second, and `gps raw [s]` counts sentence types and reports satellites visible -- which is what separates "no antenna" from "no fix". **`gps time [s]` sets the watch clock from the receiver**, and that works indoors because a time needs ephemeris and not a fix: three consecutive corrections thirty seconds apart each reported the clock `was off +0 s`/`+1 s`/`+0 s`, the write reaches the PCF85063A through the same `board_time_set_post()` path SNTP and Date & time use (log: `twatch-rtc: wrote 2026-11-12 20:34:54 UTC`), and the time survived a hard reset -- after reboot the shell read 13:46:16 MST against a host at 13:46:28, the gap being this agent's command-injection latency rather than clock error. A watch with no network can therefore be given an accurate time by holding it up. The UBX-MON-VER probe does **not** get a reply on this unit even though NMEA flows, so the model is not confirmed by the module itself; that is recorded rather than smoothed over. |
| NES (nofrendo) | **BACKBURNERED: runs and is playable, but is offered nowhere, because 320x240 is not legible on this glass.** Both menu entries were retired at `0cc6c8f` (Escapement decision-log entry 31) -- the `s_app_nes` registration in `link_twatch.c` and the NES row in `slotb.c`'s `s_apps[]` -- so a wearer cannot reach it. **NOTHING WAS DELETED**: `main/nes.c`, `components/nofrendo`, its touch hook and this row's measurements below are all intact, and `nes` still runs from a Terminal in the desktop, which is the path to use while testing it. Reviving it is those two registrations and nothing else. `main/nes.c` is the board's half and `components/nofrendo` is the emulator, vendored unmodified from retro-go at `4ced120`. `nes` loads a ROM from `/sd/game.nes`, converts the core's palette-indexed 256x240 output through a 256-entry RGB565 table, and blits it 1x letterboxed and centred -- pixel-perfect, 46 percent of the panel. The operator launched a ROM, started a game and flew around with the margin buttons, so display, input and emulation are all correct. **Throughput never was.** Measured on the board: 21 fps, with the frame split between the 6502 interpreter (8,160 us) and the blit (6,166 us) against a 16,667 us budget. The blit figure is the surprise -- moving 61 KB should be about 100 us, and it makes four passes over PSRAM because `display_picture_rgb()` copies into `s_fb` and `flush_rect()` then byte-swaps into `s_tx`. A direct path that converts straight into the flush buffer is the obvious next change, and it is not scheduled, because speed is not what stopped this shipping. **No core work was needed to make it run**: `nes/utils.h` has an `#ifdef RETRO_GO` branch, so with the macro undefined logging is `printf` and `CRC32` is `(0)`, and the only edit to upstream is guarding that branch's `IRAM_ATTR` define against ESP-IDF's own. Why nofrendo and not InfoNES or the libretro cores: measured 1.96x faster than InfoNES with render and palette conversion live, 98.2 percent of a 794-ROM collection supported, and FCEUmm and QuickNES cannot be built at all without `libretro.h` and a frontend. **SELECT is unmapped**, which a few games need to pick a mode. **A real controller drives it.** The operator's Xbox Wireless Controller connects over **BLE HID** (not Bluetooth Classic -- an earlier reading of this project asserted otherwise and was wrong; the S3 has no Classic radio at all) and every button works: face buttons, D-pad, MENU and VIEW. `main/hid_gamepad.c` discriminates on the **top-level collection usage** -- Game Pad `0x05` / Joystick `0x04` against Mouse `0x02` / Keyboard `0x06` -- and `bt_hid.c` asks it **before** the mouse parser, because a gamepad declares Generic Desktop X and Y for its left stick and that is all `hid_mouse_parse()` looks for, so the controller was being claimed as a mouse and moving the cursor. The layout was **measured**: a diagnostic captured the Report Map (283 bytes, report ID 1, hat at bit 96, fifteen buttons at 104) and the raw reports to the card, and `tests/test_gamepad.c` runs the parser on a host against that real data -- 27 checks, proved to fail three ways. `nes pad` samples the live controller to `/sd/nespad.txt` for 30 s. **Games run at 18-19 fps**, and the split says the blit is 37 percent of the frame, so the full-frame display path is the fix if this is ever revived. **SRAM is not persisted** -- nofrendo keeps battery saves in RAM only, so a Zelda file does not survive a reset. **THE LEGIBILITY QUESTION IS THE ONE THAT DECIDES ITS FUTURE, and it is a display question rather than an emulator one.** A 2x nearest-neighbour scale fills the 320-pixel width exactly and crops the height to 240 of 416 rows, which is the shape a revival would have to argue for; making the core faster does not help a wearer read it. |
| Tamacore (Tamagotchi V3 emulator) | **Runs, and the pet is on the glass.** `main/tama.c` is the board's half and `components/tamacore` is the emulator, copied from the Escapement project's `patches/harness` where the host harness compiles the same files (`scripts/tamacore-sync.sh check` there fails when the two copies differ). The Tamagotchi picks its cartridge from the same navigator the other two consoles use (`pk_tama`: root `/sd/media/tama`, filter `.bin`), and `tama` with no argument falls back to `TAMA_ROM_DEFAULT` (`/sd/media/tama/rom.bin`). It loads a 512 KiB V3 ROM into PSRAM, runs the guest's own 4 MHz clock at 16 frames a second, and blits only the rows that changed -- measured on the watch over two runs, **549 frames pushed 69 times** and 358 pushed 45, which is the host harness's 88% skip rate arriving on the board. `tama status` prints frames, blits, cycles and guest seconds; `tama press\|release 0..2` works the guest's three buttons from the shell; `tama save` and `tama stop` write the 512-byte EEPROM image to `/sd/saves/tama/tama.sav` and it is loaded on the next start; the up time rides beside it in `tama.age`. **Both the ROM and the frame are PSRAM heap allocations**, because a full-panel RGB565 buffer cannot be static here (see Pictures) and the ROM is 512 KiB. Moving the emulator's dirty-row tables off `.bss` was not tidiness: as 4,148 bytes of internal DRAM they pushed `ssh start` under its 65,536-byte floor (`insufficient internal RAM (62443 bytes free...)`), a refusal with no visible connection to the emulator; as a refcounted heap allocation they land in PSRAM and internal free went 62,443 -> 66,599. **It takes the whole framebuffer, not just a rectangle.** `display_fill()` clears the 320x416 framebuffer to black on start, because claiming the screen is not the same as owning what is on it: a blit writes only the emulator's rectangle, so without the clear the pet appeared on top of the console it was launched from with the shell's text still legible around it. The glass margins are deliberately untouched by the clear -- the button bars and key columns live outside the framebuffer -- but they are not idle. **While the guest has the screen the margins stop sending desktop keys**, because `touch.c` gates them on `display_pixels_claimed()` rather than on `td_wm_app_owns_screen()`: the latter is false for an app that draws pixels without being a window manager window, so pressing Enter mid-game was typing into the hidden shell. A refused press logs `e<n> i<n> off` and does not buzz. **The labels change with the mode** (`display_margins_set()`): the desktop's key names are replaced, and the bottom strip loses `Enter`/`Ctrl+Q` for a degree circle, because a button labelled Enter that sends no Enter is exactly the label/duty disagreement `edges.h` exists to prevent. WHICH WORDS LAND IN THE TWO COLUMNS DEPENDS ON WHO OWNS THE GLASS: the cartridge picker paints its own furniture, and once the pet is running `paint_column_state()` writes both columns in full from current state, so the columns are NOT blank during play -- see the layout below. `tama stop` restores the desktop labels before releasing the claim, so the restore paints while ownership still makes it the only writer of `s_tx`. Checked on the host by `test_twatch_edges`: 1,591 assertions including a second UTF-8 decoder that confirms the degree sign really is U+00B0 and that the font table draws it. **The bottom strip now drives the pet.** A press on a bar reaches the guest's own buttons positionally -- left is button 0, middle 1, right 2 -- through `tmc_button()`. Verified on the glass across one run: **31 presses, 0 drops**, over 9,107 frames and 569 s of guest time. THE PRESS IS QUEUED RATHER THAN APPLIED. `touch.c` runs its poll task pinned to core 0 at priority 3 and the guest task is unpinned at the same priority, while `tmc_core.h` states that one instance is not reentrant and `tmc_button()` writes the emulated CPU's pin state that `tmc_run()` is reading -- so an accepted press goes into an eight-deep queue and the guest task drains it between frames, keeping the emulator single-threaded instead of growing a lock. The symptom this avoids would not have been a crash but a pet that occasionally acted on a button it never received. `tama press N` had the same race from Phase 4 and now takes the same road. BOTH COLUMNS ARE WIRED NOW, and this is the layout as shipped. The LEFT column, top to bottom, is `Save`, blank, `Load`, blank, blank, and the `Save` and `Load` words appear ONLY while the arm is held -- with the arm up the whole column is blank, so the pair cannot be pressed by accident and the arm is the way to REACH them rather than a condition on using them. The RIGHT column is `Hold`, a deliberate blank, the current sound level, the current rate, and `Pause`, which relabels itself `Resume` while paused. The blank at right 1 is not an oversight: it separates the arm from everything that acts, so a thumb settling on the arm cannot also be on a button. **`SHOT`, `RESET` AND `EXIT` HAVE NO MARGIN SLOT.** `Shot` and `Reset` still run from the shell (`tama do shot`, `tama do reset`). Leaving the emulator is `tama stop` from a shell, or Pusher A when slot B is the personality running -- and neither of those goes through the Tama's own `Exit` action, which is worth stating plainly because it is easy to assume the opposite. `TAMA_ACT_EXIT` is gated, named and handled in `apply_action()`, and **NOTHING CAN TRIGGER IT**: it is not in `tama_ui.c`'s map, and `tama do exit` is refused because `cmd_tama`'s loop stops at `TAMA_ACT_RESET` (its usage line reads `pause save load speed sound shot reset`). It is dead code kept for the shape of the enum rather than a live path. `main/tama_ui.c` holds the button map and the rules with no ESP dependencies, so the whole rule is swept on a host; `tests/test_tama_ui.c` does exactly that, 182 checks including the order a person never tries (arm down, arm lifted, then the action -- refused). The arm is the reserved slot the desktop never used. `tama_ui_is_gated()` gates FOUR actions -- `Save`, `Load`, `Reset` and `Exit` -- and the glass can reach only the first two, since `Reset` and `Exit` lost their slots; the rule stays on all four because a mechanism half-removed is worse than either state, and a slot could come back. The check is against the SAME report the action arrives on rather than the previous one, since a gate on "was armed a moment ago" would fire when a thumb rolled off the arm. Order does not matter: the rule is a property of the set, because the controller numbers points by arrival order. The toggles relabel themselves FROM STATE, so the glass cannot claim a duty the code is not doing -- the label/duty lie this port has removed twice.

**The Sound button is the shared one, and this emulator boots muted.** It used to be a private stepper here -- 100/75/50/25/Mute, starting at full volume -- which was a second implementation of a control `sound.c` already had and ran in the opposite direction. It now calls `sound_volume_step()`: Mute, 25, 50, 75, Max, wrapping back to Mute, the same control the other three emulators use. `tama_start()` calls `sound_volume_reset()` and `sound_volume_apply(true)`, so the pet is silent until someone asks for sound. **The gain is applied once, in `sound_stream_write()`** -- applying it here as well would square it, leaving Max correct and every step below it quieter than its label. What stays Tamagotchi-specific is the decision to skip audio rendering entirely while muted, because muting must not change how fast the pet runs; it asks `sound_volume_muted()` rather than holding a gain of its own.

**The pet's up time is in the top strip.** `main/tama_age.c` -- host-tested by `tests/test_tama_age.c`, 98 checks -- formats guest seconds as `0d00h00m`, and the count is `base + cycles / TMC_CLOCK_HZ`: derived from the emulator's OWN cycle count, not a wall clock, which is what makes 4x speed, pause and a Reset all behave correctly without three separate rules. A Load resumes it from a 16-byte sidecar, `/sd/saves/tama/tama.age`, written only where the checkpoint is written, so an age can never describe a pet other than the one on the glass. The strip is claimed through `statusbar_app_text()` for the life of the run and shows the age INSTEAD of the date and clock. **THIS WORKS ONLY BECAUSE SLOT B NOW STARTS THE STRIP**: `statusbar_start()` is called from `link_init()`, `slotb_run()` runs before `link_init()` ever does, and so until `257be10` the top 44 rows of the glass were simply dark in the personality that runs the emulators.

**Multi-touch is measured, not assumed** (`touchdump` captures raw report blocks to `/sd/touch.txt`): a 15-byte block carries two points, six bytes each, and an idle read is fifteen 0xFF bytes -- not the 0x00 or 0xAB the driver's comments claimed. **Lifting the first finger of a chord promotes the survivor into the primary slot with event 0x16, whose low nibble is 0x06**, so the press test is an exact `b[0] == 0x06` rather than a nibble mask; without that, releasing the arm would fire whatever the other finger was resting on, with the interlock already satisfied and gone. **Sound watches the envelope, not the toggle counter.** `tmc_audio_emits()` counts square-wave toggles -- thousands a second for one held tone -- so gating a beep on it played continuously and stopped only while paused, because pausing stops stepping the guest and therefore stops the waveform; `tmc_audio_active()` answers "is it making a sound" and the firmware fires on its rising edge. `./scripts/tamacore-host.sh audio` in the Escapement project is the check, and it reports one 1000-toggle tone as one sound. A Reset deliberately does NOT overwrite the checkpoint on exit: quitting after a Reset would otherwise destroy the save the interlock exists to protect. |

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

No 8-bit or run-length picture support (RGB565 works, see Pictures); no Sixel; no exFAT on the
SD card. The Tamagotchi emulator has no touch input yet, and its LoRa, microphone and
BHI260AP siblings are still undriven.

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
