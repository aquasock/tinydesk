#!/usr/bin/env python3
"""Check that the two personality images are actually different in the way they
are supposed to be.

    tools/check_personality.py [--desktop build-desktop] [--slotb build-slotb]

WHY THIS EXISTS. `main/CMakeLists.txt` builds the same source tree twice, once
with TD_PERSONALITY=desktop and once with TD_PERSONALITY=slotb. The point of the
split is that the DESKTOP IMAGE CONTAINS NO EMULATOR CODE. If that stops being
true -- a source file added back to the wrong list, a component pulled in
transitively, a `#include` that drags a core in -- the build still succeeds and
the images still flash. Nothing announces it. The symptom would be 50 KB of
internal DRAM quietly reappearing, in the memory this board has least of.

So this reads the LINKED ARTIFACTS and asserts the property directly:

  desktop   no gb_/pm_/tama_/nes_/maps_ entry points; board_startup_override
            is WEAK, so the desktop's no-op is what runs
  slotb     those entry points present; board_startup_override is STRONG

It also prints the internal-RAM delta, because the size of that delta is the
reason the split was done and a person should see the number rather than trust
that it is still there.

Exit status is 0 when every check passes, 1 otherwise. Run it after any change
to main/CMakeLists.txt, and after adding a source file to main/.
"""

import argparse
import glob
import os
import subprocess
import sys

# Symbols that must exist in slot B and must NOT exist in the desktop. The
# start function is the one that pulls the whole core in; if it is absent the
# core's code and .bss are absent too.
EMULATOR_SYMBOLS = [
    "gb_start",
    "pmini_start",
    "tama_start",
    "nes_start",
    "maps_start",
]

# The board hook. WEAK in the desktop is CORRECT -- main.c's weak no-op is what
# makes the desktop boot normally. STRONG in slot B is REQUIRED, or the linker
# never fetches slotb.c and slot B silently boots the desktop instead (see the
# note in slotb.c).
OVERRIDE_SYMBOL = "board_startup_override"


def find_tool(name):
    """Locate an xtensa-esp32s3-elf-* tool, from IDF_TOOLS_PATH or PATH."""
    roots = []
    if os.environ.get("IDF_TOOLS_PATH"):
        roots.append(os.path.join(os.environ["IDF_TOOLS_PATH"], "tools",
                                  "xtensa-esp-elf"))
    roots.append(os.path.expanduser("~/.espressif/tools/xtensa-esp-elf"))
    roots.append("/home/vash/esp/idf-tools/tools/xtensa-esp-elf")

    for root in roots:
        hits = sorted(glob.glob(os.path.join(root, "*", "xtensa-esp-elf",
                                             "bin", name)))
        if hits:
            return hits[-1]

    # Fall back to PATH, which is how an exported IDF environment resolves it.
    from shutil import which
    found = which(name)
    if found:
        return found
    sys.exit("check_personality: cannot find %s -- export IDF_TOOLS_PATH or "
             "source the IDF environment" % name)


def nm_symbols(nm, elf):
    """{name: type-letter} for every symbol in the ELF."""
    out = subprocess.run([nm, elf], capture_output=True, text=True)
    if out.returncode != 0:
        sys.exit("check_personality: %s failed on %s\n%s"
                 % (nm, elf, out.stderr))

    table = {}
    for line in out.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 3:
            table[parts[2]] = parts[1]
        elif len(parts) == 2:
            # Absolute symbols print as "addr name" with no type letter.
            table[parts[1]] = "A"
    return table


def section_sizes(size, elf):
    """{section: bytes} from the size tool."""
    out = subprocess.run([size, "-A", elf], capture_output=True, text=True)
    if out.returncode != 0:
        return {}

    sizes = {}
    for line in out.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0].startswith("."):
            try:
                sizes[parts[0]] = int(parts[1])
            except ValueError:
                pass
    return sizes


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--desktop", default="build-desktop")
    ap.add_argument("--slotb", default="build-slotb")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    failures = []

    def check(ok, message):
        print(("  ok    " if ok else "  FAIL  ") + message)
        if not ok:
            failures.append(message)

    nm = find_tool("xtensa-esp32s3-elf-nm")
    size = find_tool("xtensa-esp32s3-elf-size")

    print("desktop: %s" % os.path.join(here, args.desktop))
    print("slot B:  %s" % os.path.join(here, args.slotb))

    desktop_elf = os.path.join(here, args.desktop, "tinydesk.elf")
    slotb_elf = os.path.join(here, args.slotb, "tinydesk.elf")

    for elf in (desktop_elf, slotb_elf):
        if not os.path.isfile(elf):
            sys.exit("check_personality: %s is missing -- build that "
                     "personality first" % elf)

    print("\nsymbols")
    d = nm_symbols(nm, desktop_elf)
    s = nm_symbols(nm, slotb_elf)

    for sym in EMULATOR_SYMBOLS:
        check(sym not in d, "desktop does not define %s" % sym)
        check(sym in s, "slot B defines %s" % sym)

    check(d.get(OVERRIDE_SYMBOL) == "W",
          "desktop's %s is weak (%s)" % (OVERRIDE_SYMBOL,
                                         d.get(OVERRIDE_SYMBOL, "absent")))
    check(s.get(OVERRIDE_SYMBOL) == "T",
          "slot B's %s is strong (%s)" % (OVERRIDE_SYMBOL,
                                          s.get(OVERRIDE_SYMBOL, "absent")))

    print("\ninternal RAM")
    dsz = section_sizes(size, desktop_elf)
    ssz = section_sizes(size, slotb_elf)

    dbss = dsz.get(".dram0.bss", 0)
    sbss = ssz.get(".dram0.bss", 0)
    ddata = dsz.get(".dram0.data", 0)
    sdata = ssz.get(".dram0.data", 0)

    print("  desktop  .bss %7d   .data %6d" % (dbss, ddata))
    print("  slot B   .bss %7d   .data %6d" % (sbss, sdata))
    print("  freed by the split: %d bytes" % ((sbss + sdata) - (dbss + ddata)))

    # The split exists to remove emulator state from the desktop. If the
    # desktop's .bss is not substantially smaller than slot B's, the sources
    # came back and the symbol checks above should be read carefully.
    check(sbss - dbss > 20000,
          "the desktop's .bss is at least 20 KB smaller than slot B's (%d)"
          % (sbss - dbss))

    print()
    if failures:
        print("%d check(s) FAILED" % len(failures))
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
