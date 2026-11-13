# pokemini -- the Pokemon Mini emulation core, as a firmware component.
#
# WHY THIS EXISTS. Sixteen cycles of emulator work on this board produced one
# question it could not answer from a host measurement: how many S1C88
# instructions per second can the ESP32-S3's LX7 retire while emulating a
# Pokemon Mini. The requirement is 0.22-0.36 MIPS (twelve retail cartridges
# measured at 3,091-5,053 instructions per frame against a 13,889 budget at
# 72 Hz), and the only honest way to compare that against this board is to run
# the core on this board. See docs/pokemini-scoping.md in Escapement.
#
# THESE SOURCES ARE VERBATIM from Escapement's sibling project tinytama
# (~/GIT/tinytama/core/pokemini, 2026-11-09), which vendors upstream PokeMini at
# revision 8a64225e21307ee89b18feca0a429a8d2d43e6d1 (v0.61) under GPL-3.0 with
# exactly one 2-line modification: an instruction counter in MinxCPU_XX.c,
# recorded in patches/0001-exec-counter.patch and reproducible by applying that
# patch to a clean upstream checkout. `diff -r` against tinytama's tree reports
# only this README, which is the provenance note you are reading. Do not edit the
# sources here; change them upstream or in tinytama and re-copy, so the audit
# stays a single diff against a pinned revision.
#
# LICENCE. GPL-3.0, not MIT. This is consistent with the fork's stated position
# (README.md, "Licence"): the source tree stays MIT, while firmware images that
# also contain wolfSSH/wolfSSL (GPLv3) are distributed under the GPLv3 as a
# whole. Adding a second GPLv3 component does not change that classification.
# Full text in LICENSE-PokeMini; upstream attribution in README.md here.
#
# PERFORMANCE IS NOT OPTIONAL. It inlines the PRC's memory access instead of
# going through a call pointer per access: measured 77.8 -> 96.7 M emulated
# instructions/sec on the host, a 24% gain for about 1.2 KB of text. A
# measurement taken without it would understate the core and could refuse a
# platform that actually ships.
#
# NO -Werror, for the same reason nofrendo and tamacore have none: this is
# upstream code with its own warning set, compiled by a compiler it was not
# written for. The port's own sources keep the strict flag.
#
# It has to be said explicitly: ESP-IDF applies `-Werror=all` from the SDK
# defaults to every component. And `-Wno-error` on its own is not enough here,
# because GCC gives a specific -Werror=FLAG priority over the general -Wno-error
# regardless of order on the command line -- `-Wno-error=address` is what
# actually demotes the one that fires. That diagnostic is a false positive on
# this code: CommandLine.c tests `if (custom[i].name)` against a `char name[32]`
# member, which is a truth test on an array, not a null check. Silencing it keeps
# the vendored tree diffable against upstream, which is the point of vendoring.
#
# format-overflow is NOT a false positive, and it is demoted rather than fixed
# because fixing it means editing vendored source. GCC 14 finds three genuine
# latent overflows in the cartridge/E-EEPROM filename path -- for example
# `sprintf(CommandLine.eeprom_file, "%s.eep", CommandLine.min_file)` into a
# 256-byte destination that a 256-byte input would overrun. None of them is
# reachable from pmbench, which never opens a file, but a real port that loads a
# .min from /sd by filename WOULD reach them, so this is recorded in README.md
# here as a defect to fix upstream rather than forgotten.
#
# PERFORMANCE IS NOT OPTIONAL. It inlines the PRC's memory access instead of
# going through a call pointer per access: measured 77.8 -> 96.7 M emulated
# instructions/sec on the host, a 24% gain for about 1.2 KB of text. A
# measurement taken without it would understate the core and could refuse a
# platform that actually ships.
idf_component_register(
    SRCS "CommandLine.c" "Hardware.c" "MinxAudio.c" "MinxCPU.c" "MinxCPU_CE.c"
         "MinxCPU_CF.c" "MinxCPU_SP.c" "MinxCPU_XX.c" "MinxColorPRC.c" "MinxIO.c"
         "MinxIRQ.c" "MinxLCD.c" "MinxPRC.c" "MinxTimers.c" "Multicart.c"
         "PMCommon.c" "PokeMini.c" "PokeMini_ColorPal.c" "Video.c" "Video_x1.c"
         "freebios.c" "minizip_stubs.c"
    INCLUDE_DIRS "."
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error
    -Wno-error=address -Wno-address
    -Wno-error=implicit-fallthrough -Wno-implicit-fallthrough
    -Wno-error=format-overflow -Wno-format-overflow
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE PERFORMANCE)
