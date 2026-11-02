#!/usr/bin/env python3
"""Interactive LXMF chat with the TinyNode board.

Usage:
    ~/.venvs/rns/bin/python chat.py [board_destination_hash]

Type a message and press enter. Ctrl-D or /quit to exit.
Incoming messages are printed as they arrive, with the board's own timestamp.
"""

import os
import sys
import time
from datetime import datetime, timezone

import LXMF
import RNS

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(HERE, "host-config")
BOARD_DEFAULT = "9ad89c3114e18aa322ec4a20159a9d3d"


def fmt(ts: float) -> str:
    return datetime.fromtimestamp(ts, tz=timezone.utc).strftime("%H:%M:%SZ")


def main() -> int:
    board_hash = sys.argv[1] if len(sys.argv) > 1 else BOARD_DEFAULT
    dest_hash = bytes.fromhex(board_hash)

    RNS.Reticulum(configdir=CONFIG_DIR, loglevel=RNS.LOG_WARNING)

    identity = RNS.Identity()
    router = LXMF.LXMRouter(identity=identity, storagepath="/tmp/lxmf-chat")
    local = router.register_delivery_identity(identity, display_name="host-chat")

    def on_delivery(message):
        who = "board" if message.source_hash.hex() == board_hash else message.source_hash.hex()[:8]
        print(f"\r{who} [{fmt(message.timestamp)}]: "
              f"{message.content.decode('utf-8', 'replace')}", flush=True)
        print("you> ", end="", flush=True)

    router.register_delivery_callback(on_delivery)
    router.announce(local.hash)

    print(f"local LXMF address: {local.hash.hex()}", flush=True)
    print(f"looking for board {board_hash} ...", flush=True)

    deadline = time.time() + 90
    while time.time() < deadline:
        if not RNS.Transport.has_path(dest_hash):
            RNS.Transport.request_path(dest_hash)
        if RNS.Transport.has_path(dest_hash) and RNS.Identity.recall(dest_hash) is not None:
            break
        time.sleep(3)
    else:
        print("FAIL: board not found. Is it powered, on WiFi, and announcing?", flush=True)
        return 1

    board = RNS.Destination(RNS.Identity.recall(dest_hash), RNS.Destination.OUT,
                            RNS.Destination.SINGLE, "lxmf", "delivery")
    print(f"connected ({RNS.Transport.hops_to(dest_hash)} hop). Type to chat, /quit to exit.\n", flush=True)
    router.announce(local.hash)

    while True:
        try:
            line = input("you> ")
        except EOFError:
            break
        line = line.strip()
        if not line:
            continue
        if line in ("/quit", "/exit"):
            break

        msg = LXMF.LXMessage(board, local, line, "", desired_method=LXMF.LXMessage.DIRECT)
        router.handle_outbound(msg)
        print(f"     ... sent at {fmt(time.time())}", flush=True)

    print("waiting for any outstanding replies...", flush=True)
    time.sleep(20)
    return 0


if __name__ == "__main__":
    sys.exit(main())
