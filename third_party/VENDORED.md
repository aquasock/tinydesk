# Vendored dependencies

This repository is self-contained so it builds without fetching anything first.
The Reticulum trees are vendored under `third_party/` rather than used as submodules, because each
carries a local modification needed to compile for this board.

| Directory | Upstream | Commit | License | Local change |
|---|---|---|---|---|
| `third_party/microReticulum/` | https://github.com/attermann/microReticulum | `40fa628` (v0.5.0) | Apache-2.0 | WiFi credentials read from `wifi_config.h` / `-DRNS_WIFI_SSID` instead of being hardcoded in `examples/common/udp_interface/UDPInterface.cpp` |
| `third_party/microLXMF/` | https://github.com/attermann/microLXMF | `9876dff` (v0.1.0) | **GPL-3.0-only** | none |
| `third_party/MsgPack-vendored/` | https://github.com/hideakitai/MsgPack | 0.4.2 | MIT | `Packer::packRawBytes` (both overloads) and `Unpacker::indices` made public — microLXMF uses them, and 0.4.2 keeps them private |

## Why the MsgPack patch

`microLXMF/src/LXMF/LXMessage.cpp` calls `packRawBytes()` to append pre-formed
bytes and reads `Unpacker::indices` for field offsets. Both are private in MsgPack
0.4.2 (and in the forks of it). microLXMF's CI only builds that source for native
targets, so the Arduino/ESP32 path is untested upstream and fails against a stock
MsgPack. Making exactly those members public in a vendored copy is the smallest
change that keeps microLXMF itself unmodified.

## Licensing note

microReticulum is Apache-2.0 and MsgPack is MIT, but **microLXMF is GPL-3.0-only**.
Linking it into the firmware makes the resulting binary GPL-3.0. This is fine for
personal or research use; it matters if the firmware image is ever distributed.

Each vendored tree keeps its own `LICENSE` file.

## The Arduino dependencies

`third_party/arduino/` holds the six libraries microReticulum needs that are not
part of ESP-IDF: `ArduinoJson`, `ArxContainer`, `ArxTypeTraits`, `Crypto`,
`DebugLog` and `microStore`. PlatformIO used to resolve these into
`nano-node/.pio/libdeps/nano-esp32` in the superseded P1 tree, which made this
port's build depend on a directory outside this repository; they are copied here
verbatim so `idf.py build` needs nothing but this checkout.
