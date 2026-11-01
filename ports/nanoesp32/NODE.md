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

## Telemetry

A client's `TELEMETRY_REQUEST` is answered with `FIELD_TELEMETRY`, the numbered
sensor map Sideband's own telemetry panel reads. Verified end to end on
2026-11-01 by sending a real request from a host client and decoding the reply
with Sideband's `sbapp/sideband/sense.py`:

```
== DELIVERY ==  from 89f609774be9dc1b2640bc5f23d81714
   fields: ['0x2']
   FIELD_TELEMETRY: 33 bytes
   sensors parsed: ['humidity', 'pressure', 'temperature', 'time']
   RENDER Timestamp        -> {"UTC": 45}
   RENDER Ambient Pressure -> {"mbar": 968.56}
   RENDER Temperature      -> {"c": 26.73}
   RENDER Relative Humidity-> {"percent": 42.41}
```

`node telemetry` in the Terminal packs the map and prints its size without
sending, which is the cheap way to check the encoder after a change.

### The field value has to be msgpack-encoded *bytes*

microLXMF stores message fields as `RNS::Bytes` and re-emits them with
`packRawBytes()`, so whatever is put in a field lands in the payload verbatim.
Writing the map itself into the field therefore makes the *outer* msgpack parse
it into a dictionary on the far end, and the client rejects it:

```
An error occurred while unpacking telemetry.
The contained exception was: packed data must be type 'bytes' or 'bytearray'
```

It looks like a decode bug in the client and is not: the field value must be a
msgpack **bin** containing the packed map. The Python reference does the same
thing implicitly, because it packs the field value with `umsgpack.packb()` and
the outer packer then emits that as a bin. `lxmf_send_telemetry()` wraps the map
with `mp_bin()` for this reason.

Inbound the rule is the mirror image, which is why it did not show up there:
`FIELD_COMMANDS` arrives as a raw msgpack array, so `parse_commands()` reads it
as msgpack directly.

### Why the reply is the compact subset

The full map is about 495 bytes, which cannot travel in one encrypted packet
(`LORA_ENCRYPTED_PACKET_MDU` is 159), so the router switches to link delivery --
and link delivery is currently broken against a Python peer:

```
Could not assemble LXMF message from received data     (607 bytes, method 2)
```

A reply nobody can read is worse than a smaller one that arrives, so what goes
back is the subset that fits a packet: `SID_TIME` plus pressure, temperature and
humidity. `SID_LOCATION` (7 big-endian blobs, ~43 bytes) and
`SID_MAGNETIC_FIELD` (3 floats) are the two that push it over.

The full map is still what `telemetry_pack(..., compact=false)` builds and what
`node telemetry` reports, so once link delivery works the change in
`lxmf_send_telemetry()` is dropping the compact flag.

### Identity recall, and the RAM store

Announces validated and entered the path table, and every reply still died on
`identity for <peer> not known`. The cause is the file-backed known-destination
store on this port: it does not retain, so `Identity::recall()` never returns a
key it was just given. `components/rns/CMakeLists.txt` sets
`RNS_PERSIST_KNOWN_DESTINATIONS=0`, which selects the library's heap store
instead; recall works and nothing is written to flash per announce. Path
persistence is unaffected.

Two further notes from chasing it:

- **An announce handler runs inside the path-table insert**, before the record
  is flushed. Reading the store back from inside that callback takes the
  Reticulum task down. `node.cpp` keeps its own small peer-key table
  (`remember_peer`) so a reply has a key even though the callback cannot read.
- **`Transport::path_table()` is not the live store.** `has_path()` and the
  insert path use `new_path_table()`; `size()` on the old table reads nothing
  useful, and calling `size()` on the path *store* printed bytes of a path
  string (`paths=1634742062`). Use `Transport::paths_added()` for that number.

## State: working

The node is complete for its purpose. Verified on hardware, live over USB:

```
state=wifi_up ip=10.10.0.35 rssi=-38
rns: identity from file (ok)          14f4ab99..., same across reboots
rns: interface start=1
lxmf: starting router
lxmf: announced, delivery address 89f609774be9dc1b2640bc5f23d81714
state=ready ip=10.10.0.35 free=8238664
lxmf: message from 1a60e414..., 19 bytes
```

and from the host, on the same LAN:

- the shared instance learns the path and it **refreshes on each announce**
- an LXMF message is DELIVERED (DIRECT in 1 s, OPPORTUNISTIC in 29 s)
- the node's delivery callback fires with the message content
- **Sideband connects to it** (operator-confirmed, 2026-11-01) -- the client the
  old TinyNode existed for, so the replacement is real rather than
  protocol-compatible-in-principle

### Three bugs in the announce scheduler

All three meant the node announced once at boot and then never again:

1. The re-announce branch reset its counters and **never called `announce()`**.
2. `elapsed` counts 50 ms ticks but `next_announce` is in seconds, so the
   interval was 20x too short -- invisible, because of (1).
3. `LXMRouter::_announce_interval` defaults to `0`, which disables the
   library's own auto-announce, so nothing covered the gap.

A one-shot announce is enough to appear in a path table but not enough to be
seen: Sideband lists announces as they arrive, and a node that announced hours
ago is invisible even though its path entry is still valid for seven days.

`node announce` in the Terminal forces one on demand.

### A sampling error worth recording

`rx` read 0 through three successful sends, which looked like a broken
callback. It was not: every heartbeat inspected came from a boot that finished
before the message was sent. Capture and send in the same run, and the callback
fires immediately.

## Historical: where it used to stop

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
