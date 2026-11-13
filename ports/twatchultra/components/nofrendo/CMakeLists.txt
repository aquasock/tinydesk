# nofrendo -- the NES emulator core, as a firmware component.
#
# WHY NOFRENDO, AND WHY HERE. Measured on the host against InfoNES and unable to
# build the libretro cores at all (they need libretro.h and a whole frontend):
# nofrendo runs 1.96x InfoNES on the same ROM, and 394 Mcyc/s emulated against
# the NES's own 1.79 MHz is ~206x realtime on x86. See Escapement's
# `scripts/nesbench.sh` and `~/twatch-backup/nesbench/RESULTS.md`.
#
# THESE SOURCES ARE VENDORED, from retro-go at commit 4ced120, and NOT MODIFIED.
# The one thing that makes that possible is `nes/utils.h`, which has an
# `#ifdef RETRO_GO` branch: with the macro undefined, logging becomes `printf`,
# `IRAM_ATTR` becomes empty and `CRC32` becomes `(0)`. That is the entire
# retro-go dependency of the core -- grep it and check. So this component
# deliberately does NOT define RETRO_GO, and the static assert in `port.c`
# fails the build if anything ever starts relying on it.
#
# WHAT IS NOT VENDORED: `nofrendo.c`, retro-go's own front end. It calls
# rg_system functions and this port supplies its own system layer instead.
#
# database.h IS 400 KB OF SOURCE that compiles to about 49 KB of .rodata: a
# Mesen/Nestopia game database used to auto-detect mapper quirks in known ROMs.
# rom.c uses it when present and prints "Game not found in database" otherwise.
# It is kept because it is part of upstream's behaviour and 49 KB is affordable
# inside a 3 MiB slot, but dropping it is the single largest flash saving
# available if that ever matters.
#
# NO -Werror. This is upstream code with its own warning set; the port's own
# sources keep the strict flag and these keep compiling, which is the useful
# result.
idf_component_register(
    SRCS
        "port.c"
        "nes/nes.c"
        "nes/cpu.c"
        "nes/ppu.c"
        "nes/apu.c"
        "nes/mem.c"
        "nes/mmc.c"
        "nes/rom.c"
        "nes/state.c"
        "nes/input.c"
        "nes/dis.c"
    INCLUDE_DIRS "." "nes"
)

# VENDORED CODE DOES NOT INHERIT THE PORT'S STRICTNESS, and it has to be said
# explicitly because ESP-IDF applies `-Werror=all` from the SDK defaults to every
# component. nofrendo is 1990s C that prints `uint32` (a `long` here) with `%u`,
# which is harmless on this ABI and fails the build under -Wformat. Silencing the
# error rather than patching 68 vendored files keeps upstream diffable, which is
# the point of vendoring rather than forking.
#
# -Wno-format rather than only -Wno-error=format: the diagnostic fires inside
# macros that expand to MESSAGE_INFO, so the "warning" points at the caller and
# there is no single place to cast the argument.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-format -Wno-unused-but-set-variable)

# The mappers are 58 translation units that exist only as a lookup table. They
# are added as a glob because listing them would be a maintenance burden with no
# benefit -- and because a mapper that is present but not referenced still has to
# compile, which is a cheap correctness check on the set.
file(GLOB NOFRENDO_MAPPERS "${CMAKE_CURRENT_LIST_DIR}/mappers/map*.c")

target_sources(${COMPONENT_LIB} PRIVATE ${NOFRENDO_MAPPERS})
