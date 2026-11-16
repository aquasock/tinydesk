#!/usr/bin/env python3
"""Read the live module mapping addresses off the board and write moddata.at.

WHY THIS EXISTS. A module's .text is linked at the address the MMU gives the
`modules` partition and its .rodata at the address it gives `moddata`. Both are
chosen at boot and BOTH MOVE when the app image changes size -- measured:
0x42ae0000/0x3cbe0000 while the inline FLAC decoder was linked in,
0x42ad0000/0x3cbd0000 after it was removed. A blob linked for the old pair still
executes and then reads its constants 64 KB off, which surfaces only as a
decoder error, so the numbers have to be right and have to be easy to refresh.

The firmware writes them to /root/modaddr.txt at every boot. This reads them
without depending on the console: the file is small and its bytes sit literally
in the littlefs partition, so scanning the raw image for the pattern finds it.
No F10/Enter, no display mirror, no line discipline -- the three things that
have each cost this project a cycle.

    read_modaddr.py [--port /dev/ttyACM1] [-o modules/moddata.at]

Reads the whole `storage` partition with esptool, which takes a couple of
minutes at 921600. That is the trade: slow and certain beats fast and flaky.
"""
import argparse
import os
import re
import subprocess
import sys
import tempfile

STORAGE_OFF = 0x620000
STORAGE_LEN = 0x7E0000          # 8064 KiB

# The file's literal bytes, as modload_prepare() writes them.
PATTERN = re.compile(rb"inst=(0x[0-9a-fA-F]+)\s*\n\s*data=(0x[0-9a-fA-F]+)")


def esptool_python():
    """The interpreter that has esptool, which is the IDF venv's."""
    env = os.environ.get("IDF_PYTHON_ENV_PATH")
    if env:
        cand = os.path.join(env, "bin", "python")
        if os.path.exists(cand):
            return cand
    return sys.executable


def read_storage(port, baud):
    tmp = tempfile.NamedTemporaryFile(suffix=".img", delete=False)
    tmp.close()
    cmd = [
        esptool_python(), "-m", "esptool", "--chip", "esp32s3",
        "-p", port, "-b", str(baud),
        "--before", "default_reset", "--after", "hard_reset",
        "read_flash", hex(STORAGE_OFF), hex(STORAGE_LEN), tmp.name,
    ]
    print("reading storage (%d KiB) -- this takes a couple of minutes..."
          % (STORAGE_LEN // 1024), flush=True)
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        os.unlink(tmp.name)
        sys.exit("read_modaddr: esptool failed\n%s" % (r.stderr or r.stdout)[-800:])
    return tmp.name


def find_pair(path):
    """Last match wins: the file is rewritten at every boot, and a stale copy
    from an older version of the file may still be in an unrecycled block."""
    last = None
    with open(path, "rb") as f:
        while True:
            chunk = f.read(1 << 20)
            if not chunk:
                break
            for m in PATTERN.finditer(chunk):
                last = (m.group(1).decode(), m.group(2).decode())
    return last


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=os.environ.get("WATCH_PORT", "/dev/ttyACM1"))
    ap.add_argument("--baud", default="921600")
    ap.add_argument("-o", "--out", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "moddata.at"))
    args = ap.parse_args()

    img = read_storage(args.port, args.baud)
    try:
        pair = find_pair(img)
    finally:
        os.unlink(img)

    if not pair:
        sys.exit("read_modaddr: no inst=/data= pair in storage -- is the board "
                 "booted? modload_prepare() writes /root/modaddr.txt at boot.")

    inst, data = pair
    text = """# Addresses the MMU gave for the two module partitions.
#
# COPIED FROM THE WATCH, never typed. modload_prepare() rewrites
# /root/modaddr.txt at every boot, and both addresses move when the app image
# changes size -- 0x42ae0000/0x3cbe0000 became 0x42ad0000/0x3cbd0000 when the
# inline FLAC decoder was removed.
#
# Refresh with:
#
#     cd ports/twatchultra/modules && make modaddr
#
# which reads the file off the board and rewrites this one. A blob is only valid
# for the app build that produced these numbers: the loader compares them against
# the live mapping and refuses to call a mismatch, so a stale value is a log line
# rather than a decoder reading garbage.
inst=%s
data=%s
""" % (inst, data)

    with open(args.out, "w") as f:
        f.write(text)
    print("wrote %s\n  inst=%s\n  data=%s" % (args.out, inst, data))


if __name__ == "__main__":
    main()
