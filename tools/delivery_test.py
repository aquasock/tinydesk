#!/usr/bin/env python3
"""Verify the node is receive-only and that delivery is proven by the protocol.

Sends messages with no expectation of a reply:
  - each message must reach LXMF state DELIVERED (cryptographic proof from the board)
  - no inbound reply may arrive afterwards

Run with:
    ~/.venvs/rns/bin/python delivery_test.py [board_lxmf_hash]
"""

import os
import sys
import time

import LXMF
import RNS

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(HERE, "host-config")
BOARD_DEFAULT = "9ad89c3114e18aa322ec4a20159a9d3d"

replies = []


def main() -> int:
    board_hash = sys.argv[1] if len(sys.argv) > 1 else BOARD_DEFAULT
    dest_hash = bytes.fromhex(board_hash)

    RNS.Reticulum(configdir=CONFIG_DIR, loglevel=RNS.LOG_WARNING)

    identity = RNS.Identity()
    router = LXMF.LXMRouter(identity=identity, storagepath="/tmp/lxmf-delivery")
    local = router.register_delivery_identity(identity, display_name="delivery-test")
    router.register_delivery_callback(lambda m: replies.append(m))
    router.announce(local.hash)

    deadline = time.time() + 90
    while time.time() < deadline:
        if not RNS.Transport.has_path(dest_hash):
            RNS.Transport.request_path(dest_hash)
        if RNS.Transport.has_path(dest_hash) and RNS.Identity.recall(dest_hash) is not None:
            break
        time.sleep(3)
    else:
        print("FAIL: board not reachable")
        return 1

    board = RNS.Destination(RNS.Identity.recall(dest_hash), RNS.Destination.OUT,
                            RNS.Destination.SINGLE, "lxmf", "delivery")
    print(f"board reachable, {RNS.Transport.hops_to(dest_hash)} hop(s)\n", flush=True)

    ok = 0
    sent = ("delivery check one", "delivery check two")
    for text in sent:
        result = {"state": None, "t": None}
        msg = LXMF.LXMessage(board, local, text, "", desired_method=LXMF.LXMessage.DIRECT)
        msg.register_delivery_callback(lambda m: result.update(state="DELIVERED", t=time.time()))
        msg.register_failed_callback(lambda m: result.update(state="FAILED", t=time.time()))
        started = time.time()
        router.handle_outbound(msg)
        print(f"you> {text}", flush=True)

        deadline = time.time() + 30
        while time.time() < deadline and result["state"] is None:
            time.sleep(0.5)

        if result["state"] == "DELIVERED":
            print(f"  -> DELIVERED in {result['t'] - started:.1f}s (protocol proof)\n", flush=True)
            ok += 1
        else:
            print(f"  -> {result['state'] or 'no confirmation'}\n", flush=True)
        time.sleep(2)

    print("waiting 20s to confirm the board sends nothing back...", flush=True)
    deadline = time.time() + 20
    while time.time() < deadline:
        time.sleep(1)

    if replies:
        print(f"UNEXPECTED: {len(replies)} reply/replies received:", flush=True)
        for m in replies:
            print(f"  {m.content.decode('utf-8', 'replace')}", flush=True)
    else:
        print("no replies received (as intended)", flush=True)

    passed = ok == len(sent) and not replies
    print(f"RESULT: {'PASS' if passed else 'FAIL'} "
          f"({ok}/{len(sent)} delivered, {len(replies)} replies)", flush=True)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
