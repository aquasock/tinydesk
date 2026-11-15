# gnuboy

Vendored verbatim from Escapement's `third_party/gnuboy`, which is the
[retro-go](https://github.com/ducalex/retro-go) fork of gnuboy 2.0.1 at the same
pin as the nofrendo tree this fork already carries.

- **Licence**: GPL-2.0. Full text in `COPYING`.
- **Origin**: upstream gnuboy by laguna (GPL-2.0). retro-go maintains the fork.
- **Files carried**: `cpu.c`, `gnuboy.c`, `hw.c`, `lcd.c`, `sound.c` and their
  headers, plus `tables.h`. These five compiled files are the whole core.

## Why not binjgb

`docs/gbc-scoping.md` in Escapement measured both over four commercial
cartridges, best of three: gnuboy is **1.54-1.79x faster**, and it emits
`GB_PIXEL_565_BE` -- 16-bit colour, high byte first -- which is exactly the
watch panel's byte order, so a port needs no per-pixel conversion at all.
binjgb produces RGBA only. binjgb is the accuracy reference and is kept in
Escapement for when a game misbehaves, not for the firmware.

## Why not Peanut-GB

It is the fastest portable Game Boy core and it is **DMG only**. Its CGB support
is preliminary work in an issue, not in the tree. It is the obvious first choice
by reputation and the wrong choice for a Game Boy *Color* port.

## What is not here

The front end. gnuboy renders into a caller-supplied buffer and expects the
caller to pace frames, read input and push audio. `main/gbcbench.c` does exactly
that for the on-target benchmark; a playable port would add display, input and
sound on the same six calls:

```
gnuboy_init(samplerate, audio_fmt, video_fmt, video_cb, NULL)
gnuboy_set_hwtype(GB_HW_CGB)
gnuboy_set_framebuffer(buf)
gnuboy_load_rom_file(path)
gnuboy_reset(true)      /* NOT optional: sets PC, see gbcbench.c */
gnuboy_run(true)        /* one frame */
```

`gnuboy_reset()` is the line that looks removable and is not: with no boot ROM
loaded, `PC = GB.bios ? 0 : 0x100` is decided inside the reset, so skipping it
leaves PC at 0 and the first fetch dereferences a NULL BIOS pointer.
