#!/usr/bin/env python3
"""Announce a destination from the host so the Nano ESP32 node learns a path.

Without a peer that announces, the board's path table is empty and there is
nothing for it to persist. This creates a destination and announces it every
few seconds for a while.
"""

import os
import sys
import time

import RNS

CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "host-config")
DURATION = int(sys.argv[1]) if len(sys.argv) > 1 else 60


def main() -> int:
    RNS.Reticulum(configdir=CONFIG_DIR, loglevel=RNS.LOG_NOTICE)

    identity = RNS.Identity()
    destination = RNS.Destination(identity, RNS.Destination.IN,
                                  RNS.Destination.SINGLE, "hosttest", "peer")
    print(f"host destination: {destination.hash.hex()}", flush=True)

    end = time.time() + DURATION
    n = 0
    while time.time() < end:
        destination.announce(app_data=b"host peer alive")
        n += 1
        print(f"announced ({n})", flush=True)
        time.sleep(5)

    print(f"done: {n} announces", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
