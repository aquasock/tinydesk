#!/usr/bin/env python3
"""Wait for the Nano ESP32 to appear, then capture its boot log.

Picks the Arduino ACM port (VID 2341) so the TinyTang on ACM0 is ignored.
Writes everything to /tmp/nano_boot_capture.log.
"""

import glob
import subprocess
import time

import serial

OUT = "/tmp/nano_boot_capture.log"
WAIT_SECS = 300


def is_nano(port):
    try:
        props = subprocess.run(["udevadm", "info", "-q", "property", "-n", port],
                               capture_output=True, text=True, timeout=5).stdout
        return "ID_VENDOR_ID=2341" in props
    except Exception:
        return False


def find_nano():
    for port in sorted(glob.glob("/dev/ttyACM*")):
        if is_nano(port):
            return port
    return None


def main():
    with open(OUT, "w") as fh:
        fh.write("waiting for board...\n")
        fh.flush()
        deadline = time.time() + WAIT_SECS
        port = None
        while time.time() < deadline:
            port = find_nano()
            if port:
                break
            time.sleep(0.2)
        if not port:
            fh.write("TIMEOUT: board never appeared\n")
            return 1

        fh.write(f"board appeared on {port} at {time.strftime('%H:%M:%S')}\n")
        fh.flush()

        # retry until the port is actually openable
        s = None
        for _ in range(60):
            try:
                s = serial.Serial(port, 115200, timeout=0.3)
                s.dtr = True
                break
            except Exception:
                time.sleep(0.15)
        if s is None:
            fh.write("FAIL: could not open port\n")
            return 1

        start = time.time()
        while time.time() - start < 45:
            try:
                d = s.read(1024)
            except Exception as e:
                fh.write(f"\n[reader stopped: {e}]\n")
                break
            if d:
                fh.write(d.decode("utf-8", "replace"))
                fh.flush()
        try:
            s.close()
        except Exception:
            pass
        fh.write("\n[capture done]\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
