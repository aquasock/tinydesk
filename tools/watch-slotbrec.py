#!/usr/bin/env python3
"""Read slot B's failure record out of flash.

    tools/watch-slotbrec.py                # decode the record, if any
    tools/watch-slotbrec.py --erase        # clear it first

WHY THIS EXISTS. Slot B has no console, and a picture owns the panel whenever it
is drawing, so the USB mirror is dead at exactly the moment something goes wrong
there (DSP-003). The glass carries one short line and no path. A cartridge that
"is not found" therefore leaves nothing behind to inspect -- the wearer can say
it failed, and that is all anybody knows.

So slotb_record() writes what it knows into flash sector 0x1B000: the reason, the
path, and the time. This reads it back and decodes it, because the struct is 312
bytes with a 256-byte path and reading those offsets by hand is how a confident
wrong answer happens.

    magic       "SBRC"          -- anything else means no record was written
    uptime_ms   milliseconds since slot B started
    why[48]     the reason, as the emulator or the picker worded it
    path[256]   the cartridge (or directory) it was about

WHAT IS WRITTEN, and this is what makes the pair worth reading:

    pick        <path>          the picker handed this cartridge over
    <reason>    <path>          gb_start() refused it, or the picker refused

A `pick` with no failure record after it means the cartridge opened; a `pick`
followed by a failure names the path that would not open. Both survive a reset,
so the record can be read after the watch has already rebooted to the desktop.
"""
import argparse
import os
import re
import struct
import subprocess
import sys

ADDR = 0x1B000
SIZE = 512
MAGIC = b"SBRC"
WHY_LEN = 48
PATH_LEN = 256


def find_port():
    """Locate the watch, preferring the by-id name so a re-enumeration is fine."""
    for p in sorted(os.listdir("/dev/serial/by-id")):
        if "Espressif" in p and p.endswith("if00"):
            return os.path.join("/dev/serial/by-id", p)
    for i in range(4):
        p = f"/dev/ttyACM{i}"
        if os.path.exists(p):
            return p
    raise SystemExit("watch-slotbrec: no Espressif serial port found")


def esptool(port, *args):
    exe = os.path.join(os.path.dirname(sys.executable), "esptool.py")
    cmd = [exe if os.path.exists(exe) else "esptool.py", "-p", port, "-b", "921600"]
    cmd += list(args)
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        sys.stderr.write(r.stdout + r.stderr)
        raise SystemExit(f"watch-slotbrec: esptool failed ({r.returncode})")
    return r.stdout


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--erase", action="store_true",
                    help="clear the sector before reading, so nothing stale remains")
    args = ap.parse_args()

    port = args.port or find_port()

    if args.erase:
        esptool(port, "--after", "no_reset", "erase_region", hex(ADDR), hex(0x1000))
        print(f"erased {hex(ADDR)}")

    # Read to a FILE, not to stdout: esptool's binary output is not reliably
    # clean on a pipe, and a record that decodes to noise is worse than none.
    import tempfile

    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
        tmp = f.name
    esptool(port, "--after", "no_reset", "read_flash", hex(ADDR), str(SIZE), tmp)
    with open(tmp, "rb") as f:
        raw = f.read()
    os.unlink(tmp)

    if raw[:4] != MAGIC:
        blank = raw[:4] == b"\xff\xff\xff\xff"
        print(f"no record at {hex(ADDR)}: "
              f"{'sector is erased' if blank else 'holds ' + repr(raw[:4])}")
        return 1

    uptime, = struct.unpack_from("<I", raw, 4)
    why = raw[8:8 + WHY_LEN].split(b"\0")[0].decode("utf-8", "replace")
    path = raw[8 + WHY_LEN:8 + WHY_LEN + PATH_LEN].split(b"\0")[0].decode("utf-8", "replace")

    print(f"at {uptime} ms")
    print(f"why  {why!r}")
    print(f"path {path!r}")

    # Say what it means rather than leaving the reader to work it out.
    if why == "pick":
        print("\nThis was the cartridge the picker handed over. If nothing after it")
        print("failed, it opened; check the path against the card.")
    elif not path:
        print("\nNo path: the failure was not about one cartridge.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
