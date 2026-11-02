#!/usr/bin/env python3
"""Persistence check for the Nano ESP32 Reticulum node.

1. list /ffat before reboot (proves RNS wrote its tables)
2. send 'r' to reboot
3. capture the next boot's "volume contents before RNS start" block
4. list again after the reboot
"""

import glob
import time

import serial

PORT_GLOB = "/dev/ttyACM*"


def open_port(timeout=25):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            port = sorted(glob.glob(PORT_GLOB))[-1]
            s = serial.Serial(port, 115200, timeout=0.3)
            s.dtr = True
            return s
        except Exception:
            time.sleep(0.25)
    raise RuntimeError("no serial port")


def read_for(s, seconds, marker=None):
    out = ""
    deadline = time.time() + seconds
    while time.time() < deadline:
        d = s.read(1024)
        if d:
            out += d.decode("utf-8", "replace")
            if marker and marker in out:
                break
    return out


def main():
    s = open_port()
    boot = read_for(s, 30, marker="RNS READY")
    if "RNS READY" not in boot:
        print("WARN: did not see RNS READY on attach")

    print("=== 1. /ffat BEFORE reboot ===")
    s.write(b"l")
    print(read_for(s, 4))

    print("=== 2. sending reboot ('r') ===")
    s.write(b"r")
    s.close()
    time.sleep(1.0)

    print("=== 3. boot log after reboot ===")
    for attempt in range(20):
        try:
            s = open_port(timeout=5)
            break
        except Exception:
            time.sleep(0.5)
    else:
        print("FAIL: port never came back")
        return 1
    after = read_for(s, 30, marker="RNS READY")
    start = after.find("[1/5]")
    print(after[start:start + 700] if start >= 0 else after[:700])

    print("=== 4. /ffat AFTER reboot ===")
    s.write(b"l")
    print(read_for(s, 4))
    s.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
