#!/usr/bin/env python3
"""Refuse to build a module whose link addresses were not measured on THIS app.

moddata.at holds two addresses that only the board knows: the MMU base of the
`modules` partition and of `moddata`. They are read off the watch by
`make modaddr` and they move when the app image changes size. So moddata.at is
only ever as fresh as the last `make modaddr`, while the app is rebuilt here --
and nothing used to connect the two.

WHAT THAT COST. On 2026-11-17 the app was rebuilt five times for the IMU and
KLIO work and moddata.at still held entry 43's pair, 0x42ad0000/0x3cce0000,
against a live 0x42ae0000/0x3cce0000. `music` then refused every file with the
loader's stale-link message and nothing in the build said a word. The runtime
guard caught it, which is why it was one line in a status message rather than a
decoder reading garbage -- but it should never have reached the flash.

WHY THE APP IMAGE SIZE AND NOT A TIMESTAMP. A mtime check would also fire on a
rebuild that changed nothing but a comment, and a check that cries wolf on clean
builds is a check that gets deleted. The two bases are observed to move exactly
when the image size moves, so the size is the thing to compare -- and the
loader's own header comparison stays the backstop for anything this misses.

    check_addr_fresh.py <app.bin> <moddata.at>
"""
import os
import re
import sys


def main(argv):
    if len(argv) != 3:
        sys.exit("check_addr_fresh: usage: %s <app.bin> <moddata.at>" % argv[0])

    bin_path, at_path = argv[1], argv[2]

    try:
        live_size = os.path.getsize(bin_path)
    except OSError:
        sys.exit("check_addr_fresh: %s missing -- build the app first" % bin_path)

    try:
        with open(at_path) as f:
            text = f.read()
    except OSError:
        sys.exit("check_addr_fresh: %s missing -- run 'make modaddr'" % at_path)

    m = re.search(r"^app=(\d+)\s*$", text, re.MULTILINE)
    if not m:
        sys.exit(
            "check_addr_fresh: %s has no app= line, so there is no record of which\n"
            "app build these addresses were measured against. Run, with the watch\n"
            "booted and its console free:\n\n"
            "    make modaddr && make flac\n" % at_path)

    measured = int(m.group(1))
    if measured == live_size:
        return 0

    sys.exit(
        "check_addr_fresh: REFUSING TO LINK.\n\n"
        "  moddata.at was measured against an app image of %d bytes\n"
        "  the app image is now                    %d bytes\n\n"
        "Both module mmap bases move when the app image changes size, so these\n"
        "addresses describe a mapping the board no longer has. A blob linked for\n"
        "them still executes and reads its constants 64 KB away -- it does not\n"
        "crash, it reports a corrupt file.\n\n"
        "With the watch booted and nothing holding /dev/ttyACM1:\n\n"
        "    make modaddr && make flac\n\n"
        "then flash BOTH regions (0xC00000 and 0xE00000).\n"
        % (measured, live_size))


if __name__ == "__main__":
    sys.exit(main(sys.argv))
