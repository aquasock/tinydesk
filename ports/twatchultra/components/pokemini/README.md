# PokeMini core (vendored into TinyDesk)

The Pokémon Mini emulation core, as a firmware component, so this board can be
asked the one question a host measurement cannot answer: **how many S1C88
instructions per second does the ESP32-S3 retire while emulating a Pokemon
Mini.**

- Upstream: <https://github.com/pokemon-mini/PokeMini>
- Revision: `8a64225e21307ee89b18feca0a429a8d2d43e6d1` (2026-10-31), v0.61
- Licence: **GPL-3.0** — full text in `LICENSE-PokeMini`
- Taken from: `~/GIT/tinytama/core/pokemini` (2026-11-09)

## Provenance

Every `.c`/`.h` here is **verbatim** from tinytama's vendored tree, which is
itself upstream unmodified apart from one 2-line patch. `diff -r` this directory
against `~/GIT/tinytama/core/pokemini` and the only difference is this README.

The one modification is an instruction counter, needed by any harness that wants
to report instructions per second:

```
patches/0001-exec-counter.patch
  +  unsigned long g_exec_count = 0;      (before MinxCPU_Exec)
  +  g_exec_count++;                      (first statement of MinxCPU_Exec)
```

Apply it to a clean checkout of the revision above to reproduce this tree.

**Do not edit these sources here.** Change upstream or in tinytama and re-copy,
so that the audit stays a single diff against a pinned revision.

## What tinytama dropped, and why

- `sourcex/InstructionProc.c`, `sourcex/InstructionInfo.c` — disassembler
  tables, debugger only. Verified by link test that execution never references
  them.
- every `platform/` directory (SDL, NDS, PSP, Wii, Dreamcast, Dingoo, Windows,
  GTK) — this project has its own front ends.
- minizip and zlib. `minizip_stubs.c` satisfies the link for the `.min`-is-a-ZIP
  path; each stub fails the way an unreadable archive would, and no cartridge
  image needs it.

## Where the emulated state lives, because it decides the measurement

This is the part that matters for interpreting `pmbench`:

- `PM_RAM[8192]` and `PM_BIOS[4096]` are **global arrays**, so they land in
  `.bss` and therefore in internal SRAM. That is the per-instruction hot state
  and it is as fast as this board gets.
- The cartridge image is **not copied**. `PokeMini_SetMINMem()` stores the
  caller's pointer (`PM_ROM = mem`), so whoever calls it decides where the ROM
  lives — internal SRAM, PSRAM, or flash. Since S1C88 instruction fetch comes
  out of the cartridge, that choice dominates the result, which is why
  `pmbench` measures both.
- `MinxLCD`, `MinxColorPRC`, `EEPROM` and the audio FIFO are `malloc`ed, about
  51 KB in total at `PokeMini_Create(0, 0)`. They are touched once per frame
  rather than per instruction.

## The free BIOS

`freebios.c` embeds a 4 KB GPL BIOS as an array, so the core can be reset and
run with no cartridge at all. It idles at `$0040`, so it is useful for checking
the memory map and useless for measuring speed. `pmbench` uses a synthetic
cartridge instead, and no commercial ROM is present in this repository.
