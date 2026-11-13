/*
 * port.c - the guard rails that keep this component buildable on its own.
 *
 * This file contains no emulator code. It exists so that two facts about the
 * vendored core are enforced by the COMPILER rather than by a comment, because a
 * comment does not fail a build.
 *
 * FACT ONE: RETRO_GO MUST NOT BE DEFINED. `nes/utils.h` has an `#ifdef RETRO_GO`
 * branch that pulls in `rg_system.h` and routes logging through `rg_system_log`.
 * That is the whole of the core's dependency on retro-go, and the port depends on
 * the OTHER branch. If a future build ever defines RETRO_GO -- by copying a
 * retro-go CMake fragment, or by someone "tidying up" the defines -- the core
 * would start needing a header that is not in this tree, and the failure would
 * appear as an include error deep inside vendored code rather than as anything
 * to do with the change that caused it. So it is checked here, where the message
 * names the actual problem.
 *
 * FACT TWO: THE CORE ASSUMES A 32-BIT LITTLE-ENDIAN TARGET. `IS_BIG_ENDIAN` is
 * commented out in config.h, and 6502 emulation is full of 16-bit loads and
 * stores that would silently swap halves on a big-endian machine. The ESP32-S3
 * is little-endian and the assumption holds, but it is worth stating: if this
 * port were ever retargeted, this is the first thing to check and the code would
 * give wrong answers rather than fail.
 */
#include <stdint.h>

#ifdef RETRO_GO
#error "nofrendo is being built with RETRO_GO defined. This component vendors the \
core precisely so it does NOT need retro-go: see nes/utils.h, which routes \
logging through printf when the macro is absent. Remove the define rather than \
adding rg_system.h to the include path."
#endif

/* nofrendo's config.h says the target is little-endian; the ESP32-S3 agrees.
 * Checked at compile time because a silent byte swap would corrupt every 16-bit
 * value the emulated 6502 touches, and would look like a broken ROM. */
#if defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__)
_Static_assert(__BYTE_ORDER__ != __ORDER_BIG_ENDIAN__,
               "nofrendo assumes a little-endian target; enable IS_BIG_ENDIAN in config.h");
#endif

_Static_assert(sizeof(uint8_t) == 1, "uint8_t is not 8 bits");
_Static_assert(sizeof(uint16_t) == 2, "uint16_t is not 16 bits");
_Static_assert(sizeof(uint32_t) == 4, "uint32_t is not 32 bits");

/*
 * A version stamp so `nes` can report what it is running without the port
 * having to include nofrendo's own headers for the string. Deliberately not
 * `const` so it lands in .rodata only if the caller actually prints it.
 */
const char *nofrendo_core_version(void)
{
    return "nofrendo 3.0 (retro-go 4ced120, vendored)";
}
