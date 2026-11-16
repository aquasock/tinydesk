# micro-flac, vendored

Source: https://github.com/esphome-libs/micro-flac at tag `v0.3.0`
(`idf_component.yml` version 0.3.0), plus its `micro-ogg-demuxer` submodule
(v1.2.1) which the optional Ogg-FLAC container path needs.

Licences: micro-flac is Apache-2.0 (`LICENSE`); micro-ogg-demuxer is MIT
(`lib/micro-ogg-demuxer/LICENSE`).

## What was changed

Nothing. The sources are as published. The only edits to the tree are
housekeeping: `.git`, `.github`, `.clang-format`, `.clangd`, `.pre-commit-config.yaml`,
`.markdownlint.yaml`, `CLAUDE.md`, `tests/`, `script/` and the README files of the
submodule were deleted so the vendored copy carries no VCS metadata or tooling.

`CMakeLists.txt` and `Kconfig` are unmodified, so `idf.py menuconfig` shows the
component's own options and the defaults apply: `MICRO_FLAC_PREFER_PSRAM`,
`MICRO_FLAC_ENABLE_XTENSA_ASM` (on for ESP32-S3) and `MICRO_FLAC_ENABLE_OGG`.

## Why this decoder

Argued with measurements in `CMakeLists.txt`'s neighbour, `musicdec.h`, and in
core-log entry 24. Short version: Rockbox's FLAC decoder reserves ~190 KB of
INTERNAL SRAM statically (7 x 4608 x 4 channel buffers plus a 64 KB input
buffer) and this board has ~18 KB free, so it cannot run here at all. micro-flac
sizes one working buffer from the file's STREAMINFO and allocates it in PSRAM.

## Updating

Copy the new tag's `include/`, `src/`, `cmake/`, `Kconfig` and (if the Ogg path
is still wanted) `lib/micro-ogg-demuxer/`, then rebuild. The integration surface
this project uses is only `musicdec.cpp`, which calls the documented streaming
API (`decode()`, `get_stream_info()`, `get_output_buffer_size_samples()`).

## Measured on this board

Flash cost 31.7 KB (built the real decode path for xtensa-esp32s3 and diffed
against an empty app; the component's own `.a` is 76 KB because the encoder is
statically linked but dropped by link-time GC). Decodes a 5:58 44.1 kHz stereo
FLAC in real time with `0 errors, 0 short writes`.
