#!/usr/bin/env python3
"""Test whether a reply larger than one LXMF packet survives to a Python peer.

microLXMF sends OPPORTUNISTICally, as a single packet, only when packed_size()
is at most 159 bytes (core-reference LXMF-011). Anything larger falls back to a
DIRECT link and a resource transfer. Core-reference LXMF-012 records two
benches disagreeing about whether that fallback works: this node once delivered
a 614-byte status reply as a resource, while another bench could not assemble a
607-byte link message.

This script settles it for this node. It sends a command whose reply is known
to exceed one packet and reports whether the peer assembled it, so the pass
condition is the receiver's result rather than the sender's log line.

The client:
  - attaches to the Reticulum shared instance by loading ~/.reticulum, which is
    the directory the running instance keeps its transport identity in
    (core-reference HOST-007). Loading any other directory makes every RPC fail
    with 'digest sent was rejected'.
  - uses a fresh in-memory identity, never the endpoint's, because one identity
    may have only one LXMF router at a time (HOST-007).

Usage:
  ~/.venvs/rns/bin/python tools/link_delivery_test.py [command] [timeout] [dest]
Defaults: command 'status', timeout 90 s, destination 89f60977...

P1 (bare-metal TinyNode) stores its identity at /ffat/identity and P4
(tinydesk nanoesp32-node) at /fs/identity, so the two firmwares present
different LXMF addresses on the same board. P1 was 854ff4d2...; P4 is
89f60977..., read from its own boot log.

Exit status is 0 when a reply arrived and reassembled, 1 otherwise.
"""

import os
import sys
import tempfile
import time

import LXMF
import RNS

CONFIG_DIR = os.path.expanduser("~/.reticulum")
BOARD_LXMF = "89f609774be9dc1b2640bc5f23d81714"   # P4's lxmf.delivery

command = sys.argv[1] if len(sys.argv) > 1 else "status"
timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0
if len(sys.argv) > 3:
    BOARD_LXMF = sys.argv[3]

received = []


def on_delivery(message):
    received.append(message)
    print("  [reply] assembled: %d bytes of content, %d bytes packed"
          % (len(message.content), message.packed_size()), flush=True)
    print("  [reply] method %s, representation %s"
          % (message.method, message.representation), flush=True)
    text = message.content.decode("utf-8", "replace")
    print("  [reply] first line: %s" % text.splitlines()[0] if text else "", flush=True)


def main():
    RNS.Reticulum(configdir=CONFIG_DIR, loglevel=RNS.LOG_WARNING)

    identity = RNS.Identity()
    # A throwaway storage path: the router requires one, and this test must not
    # touch the endpoint's mailbox under ~/.tinynode-agent.
    storage = tempfile.mkdtemp(prefix="lxmf-link-test-")
    router = LXMF.LXMRouter(identity=identity, storagepath=storage)
    local = router.register_delivery_identity(identity, display_name="link-delivery-test")
    router.register_delivery_callback(on_delivery)

    print("local destination: %s" % local.hash.hex())
    router.announce(local.hash)
    # Give the node a moment to learn a path back, so a link can be opened.
    time.sleep(3)

    dest_hash = bytes.fromhex(BOARD_LXMF)
    print("requesting path to the node %s" % BOARD_LXMF)
    RNS.Transport.request_path(dest_hash)
    deadline = time.time() + 30
    while not RNS.Transport.has_path(dest_hash) and time.time() < deadline:
        time.sleep(0.25)
    if not RNS.Transport.has_path(dest_hash):
        print("FAIL: no path to the node")
        return 1
    print("path found at %d hops" % RNS.Transport.hops_to(dest_hash))

    node_identity = RNS.Identity.recall(dest_hash)
    if node_identity is None:
        print("FAIL: could not recall the node's identity")
        return 1
    node_dest = RNS.Destination(node_identity, RNS.Destination.OUT,
                                RNS.Destination.SINGLE, "lxmf", "delivery")

    print("sending %r" % command)
    message = LXMF.LXMessage(node_dest, local, command,
                             desired_method=LXMF.LXMessage.OPPORTUNISTIC)
    message.try_propagation_on_fail = False
    router.handle_outbound(message)

    deadline = time.time() + timeout
    while not received and time.time() < deadline:
        time.sleep(0.25)

    if not received:
        print("FAIL: no reply within %.0f s" % timeout)
        return 1

    packed = received[0].packed_size()
    if packed > 159:
        print("PASS: reply of %d packed bytes arrived and assembled, which is"
              " above the 159-byte packet gate, so link delivery works here"
              % packed)
    else:
        print("INCONCLUSIVE: reply was only %d packed bytes, under the gate, so"
              " this did not exercise link delivery" % packed)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
