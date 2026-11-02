#!/usr/bin/env python3
"""End-to-end Reticulum test: host -> Nano ESP32 -> host.

Runs as a single RNS instance (no daemon), so there is no shared-instance
socket to go stale. Waits for the board's announce, then sends an encrypted
packet to its destination and waits for the cryptographic proof of delivery.

Usage: e2e_test.py <destination_hash_hex>
"""

import os
import sys
import time

import RNS

CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "host-config")
APP_NAME = "nano"
APP_ASPECT = "esp32"


def main() -> int:
    dest_hex = sys.argv[1]
    dest_hash = bytes.fromhex(dest_hex)

    RNS.Reticulum(configdir=CONFIG_DIR, loglevel=RNS.LOG_NOTICE)

    print(f"[1] waiting for path to {dest_hex}", flush=True)
    deadline = time.time() + 90
    while time.time() < deadline:
        if RNS.Transport.has_path(dest_hash):
            break
        RNS.Transport.request_path(dest_hash)
        time.sleep(3)
    else:
        print("FAIL: no path after 90s", flush=True)
        return 1

    hops = RNS.Transport.hops_to(dest_hash)
    print(f"[1] path found: {hops} hop(s)", flush=True)

    identity = RNS.Identity.recall(dest_hash)
    if identity is None:
        print("FAIL: no identity recalled for destination", flush=True)
        return 1
    print(f"[2] identity recalled, public key {len(identity.get_public_key())} bytes", flush=True)

    destination = RNS.Destination(identity, RNS.Destination.OUT,
                                  RNS.Destination.SINGLE, APP_NAME, APP_ASPECT)
    if destination.hash != dest_hash:
        print(f"FAIL: derived hash {destination.hash.hex()} != {dest_hex}", flush=True)
        return 1
    print("[2] destination hash matches what the board printed", flush=True)

    payload = b"hello nano-esp32 from the host"
    packet = RNS.Packet(destination, payload, create_receipt=True)
    packet.send()
    print(f"[3] sent {len(payload)} bytes, waiting for proof", flush=True)

    deadline = time.time() + 30
    while time.time() < deadline:
        receipt = packet.receipt
        if receipt is not None and receipt.status == RNS.PacketReceipt.DELIVERED:
            rtt = receipt.get_rtt()
            print(f"[4] PROOF RECEIVED: delivered, rtt={rtt:.3f}s", flush=True)
            print("RESULT: PASS - bidirectional Reticulum link confirmed", flush=True)
            return 0
        time.sleep(0.5)

    print("FAIL: no proof of delivery within 30s", flush=True)
    return 1


if __name__ == "__main__":
    sys.exit(main())
