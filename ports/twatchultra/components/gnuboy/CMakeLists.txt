# gnuboy -- the Game Boy / Game Boy Color core, as a firmware component.
#
# WHY THIS EXISTS. Same reason as `pokemini` and `nofrendo`: whether this board
# can emulate a Game Boy Color at full speed is a question no host measurement
# answers. docs/gbc-scoping.md in Escapement measured gnuboy at 1.54-1.79x
# binjgb on a desktop and projected 28-45 fps onto the watch by normalising
# through the NES interpreter -- and that projection was made before three
# optimisations landed, so the band is stale in a direction nobody has checked.
# The only way to replace it with a number is to run the core here.
#
# THESE SOURCES ARE VERBATIM from Escapement's third_party/gnuboy, which is the
# retro-go fork at the same pin as the NES core's tree. Nothing here is edited:
# `cmp` against that copy reports every file identical, and the provenance note
# in THIRD_PARTY-style form is at the top of README.md in this directory's
# Escapement original. GPL-2.0; full text in COPYING.
#
# WHAT IS NOT HERE. The front end. gnuboy renders into a buffer the caller
# supplies and expects the caller to pace frames, read input and push audio --
# which is what a port would write, and what gbcbench.c does for the benchmark.
# No retro-go `rg_system.h` is needed because gnuboy.h falls back to stdio and a
# no-op IRAM_ATTR when RETRO_GO is undefined.
idf_component_register(
    SRCS "cpu.c" "gnuboy.c" "hw.c" "lcd.c" "sound.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_timer
)

# THE INTERPRETER IS COMPILED WITH JUMP TABLES, WHICH ESP-IDF DISABLES GLOBALLY.
#
# `gb_cpu_emulate()` is a 256-case switch over the opcode. ESP-IDF passes
# `-fno-jump-tables -fno-tree-switch-conversion` on every compile line, so GCC
# builds a decision tree for that switch instead of one indirect jump -- and for
# 256 dense cases that is several comparisons and branches per DISPATCH, which
# happens once per emulated instruction.
#
# THIS IS APPLIED TO cpu.c ONLY and changes no source, so the component stays
# verbatim against Escapement's third_party/gnuboy. Per-file compile options are
# appended after the global ones, so these win.
set_source_files_properties(cpu.c PROPERTIES COMPILE_OPTIONS
    "-fjump-tables;-ftree-switch-conversion")
