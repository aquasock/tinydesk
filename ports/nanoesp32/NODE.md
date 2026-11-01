# Running Reticulum inside the desktop

`node.cpp` brings microReticulum up alongside the desktop: WiFi, a Reticulum
instance with a persisted identity, and a UDP interface, all on the board that
draws the 320x240 panel. It replaces the standalone TinyNode firmware.

## What it took to build microReticulum on ESP-IDF

microReticulum is an Arduino library. The desktop firmware is bare ESP-IDF.
Getting it to link took five things, all in `components/rns/`:

**1. Do not define `ARDUINO`.** This is the important one. Without it:

- `Utilities/OS.h` takes its POSIX branch (`gettimeofday`, `nanosleep`) instead
  of `millis()`/`delay()`.
- ArduinoJson compiles against `std::string` rather than its Arduino mode.
  Defining `ARDUINO` flips ArduinoJson into String/Stream/PROGMEM/
  FlashStringHelper mode and the error count goes from a handful to hundreds.

**2. A shim for what is left** (`shim/arduino_shim.h`): `ESP.restart()`,
`ESP.getPsramSize()`, `delay()`, and a `StringSumHelper` for the two `Log`
overloads that take one. It is force-included into the C++ sources only — it
declares a class, so plain C (`nanosleep.c`) must not see it.

**3. `nanosleep`** (`components/rns/nanosleep.c`). microReticulum's POSIX sleep
path calls it; newlib under ESP-IDF does not provide it. It forwards to
`vTaskDelay` so the UI task keeps drawing.

**4. `-fexceptions`.** microReticulum uses try/catch, and ESP-IDF compiles C++
with exceptions off by default.

**5. `millis()`/`micros()` must not be shimmed** — Crypto's `RNG.cpp` already
defines them for non-Arduino targets, and defining them twice is a
redefinition error.

`attermann/Crypto` needed nothing: it has no Arduino includes and its RNG
already calls `esp_random()`.

## The filesystem is the load-bearing part

Reticulum works in **relative paths**: `storagepath()` is `"."`, the cache is
`"./cache"`, the path store is `"./path_store/"`. The desktop's filesystem is
mounted at `/fs`.

**`Reticulum::storagepath()` does not work.** The constructor assigns `"."` and
`"./cache"` to the static `char` arrays, and it runs after any setter call, so
the value is overwritten before `start()` reads it:

```cpp
// Reticulum.cpp, in the constructor
strncpy(_storagepath, ".", FILEPATH_MAXSIZE);
strncpy(_cachepath, "./cache", FILEPATH_MAXSIZE);
```

The mechanism is a `FileSystemImpl` whose `open`/`exists`/`mkdir`/… prefix
every path with the mount point. `node.cpp` carries a small stdio
implementation (`MountedFileSystem`) rather than microStore's shipped adapter,
because that one is written for Arduino: its header includes `LittleFS.h` under
`#if defined(ESP32)` and brace-initialises `struct stat` in a way a `-Werror`
build rejects.

It must be registered **before** `Reticulum::start()`, which is what opens the
stores:

```cpp
s_filesystem = new MountedFileSystem("/fs");
s_filesystem.init();
RNS::Utilities::OS::register_filesystem(s_filesystem);
/* ... then construct and start */
```

Leaving it unregistered left `start()` faulting in a reboot loop — 293 boots in
the log before it was found.

## WiFi

`UDPInterface::start()` connects WiFi itself through the Arduino `WiFi`
library, inside `#ifdef ARDUINO`. On this build that block is absent, so the
interface falls through to its POSIX socket path (lwIP serves it) and **WiFi is
brought up in `node.cpp` first**.

One detail that cost time: `esp_wifi_set_config()` on a station normally starts
the association, but it did not here — `esp_wifi_connect()` has to be called
explicitly. Without it `esp_wifi_start()` returns `ESP_OK` and the board simply
never associates, with no disconnect event to explain why.

## Configuration

```
/fs/etc/wifi.conf     ssid = ...      password = ...
/fs/reticulum/        identity and Reticulum's stores
/fs/node.log          one line per state change
```

`node.log` exists because a headless board has no console: the state can be
recovered by dumping flash at the `storage` partition and reading the strings
out. A build-time fallback exists for a bench image with no way to write a file
yet — `-DTD_WIFI_SSID=... -DTD_WIFI_PASSWORD=...`.

## State: where it stops

Working on hardware, each verified by reading `/fs/node.log` out of flash:

- WiFi associates: `state=wifi_up ip=10.10.0.35 rssi=-43`
- The storage adapter registers: `rns: storage /fs init=1`
- `Reticulum::start()` **completes**: `rns: started`
- An identity is generated and hashes: `identity=3872c2a7…`

Then it stops, before the UDP interface is created. Two findings from that:

**`Identity::from_file()` on a missing file returns a truthy but
uninitialised object**, and hashing it faults. The file's existence is now
checked with `stat()` first. This is worth knowing because the obvious
`if (!identity) generate()` guard is not enough.

**The identity changes on every boot** and the log always reads "no identity
file", so `to_file()` is not persisting. `node.log` *does* survive across
boots, so the mount is writable — the difference is that Reticulum writes
through `microStore`, and ours is the `MountedFileSystem` in `node.cpp`. That
adapter, or microStore's `File` buffering around it, is the next thing to
check: a write that does not close is invisible to the next boot.

Until that is fixed the node cannot keep an address, which is the one property
a node has to have.

## Diagnosing without a console

`/fs/node.log` is append-only and read by dumping the `storage` partition from
flash:

```
esptool --port /dev/ttyACM0 --before usb_reset --after no_reset \
    read-flash 0x620000 0x200000 /tmp/fs.bin
strings /tmp/fs.bin | grep -E 'rns:|state=|alive'
```

Two things to know before trusting that output:

- **Flashing does not erase `storage`.** The merged image ends near 0x210000,
  so old log lines survive and a first read looks like a crash loop that is
  really stale history. Erase `0x620000 0x9E0000` and reset before reading.
- **Your own esptool calls count as boots.** Every `write-flash`, `erase-region`
  and `usb_reset` resets the chip, so several `== boot` lines after a flash
  sequence are the harness, not the firmware. Watch the USB device number with
  `lsusb` instead: a stable number over a minute means the board is not
  resetting.
