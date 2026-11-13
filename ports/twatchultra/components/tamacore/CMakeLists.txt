# tamacore -- the Tamagotchi emulator, as a firmware component.
#
# THESE SOURCES ARE COPIES, and the copy is deliberate but must not drift.
# The emulator's portable half lives in the Escapement project at
# patches/harness/{splb32,segmap-1x,core0}, where the host harness
# (scripts/tamacore-host.sh) compiles it straight out of the tree. ESP-IDF
# cannot reach outside its own project directory, so the same files are
# collected here and `scripts/tamacore-sync.sh check` in Escapement fails when
# the two copies differ. Run `scripts/tamacore-sync.sh sync` after editing the
# originals.
#
# Only tmc_core.h is public: the glue in main/tama.c needs the emulator's own
# API and nothing else. splb32.h, segmap.h and render.h are private to the
# component, so the CPU emulator's internals are not something the port can
# accidentally come to depend on.
#
# NO -Werror HERE, on purpose. This is vendored code that also builds under
# the host compiler with its own flag set; the port's own sources keep -Werror
# and these keep compiling, which is the useful result.
idf_component_register(
    SRCS "tmc_core.c" "splb32.c" "splb32_sound.c" "ht24lc08.c" "con_v3_ir.c" "segmap.c"
    INCLUDE_DIRS "." "include"
)
