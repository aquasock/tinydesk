#!/usr/bin/env python3
"""Ask the node for telemetry the way a client does, and report what comes back.

The firmware on the board (P4, tinydesk nanoesp32-node) acts only on
FIELD_COMMANDS: a message whose content is text is ignored outright, because
the desktop UI is the operator's interface and LXMF is the machine one. A
telemetry request is therefore a list holding one single-key dict,
{TELEMETRY_REQUEST: [timebase, collector]}, with the content left empty.

This tool reports, from the node's own reply:
  - whether a FIELD_TELEMETRY map arrived at all,
  - the reply's packed_size and delivery method, which is what decides whether
    it travelled as one packet or as a link resource transfer,
  - the decoded sensor map, checked against Sideband's sensor ids.

That makes it both the bring-up check for P4's telemetry and the measurement
behind core-reference LXMF-012: if packed_size exceeds 159 and the reply still
arrived, link delivery works on this stack.

Usage:
  ~/.venvs/rns/bin/python tools/telemetry_request_test.py [dest] [timeout]
Default destination is P4's lxmf.delivery, 89f60977..., read from its boot log.
"""

import os
import sys
import tempfile
import time

import LXMF
import RNS

try:
    import umsgpack
except ImportError:
    from RNS.vendor import umsgpack

CONFIG_DIR = os.path.expanduser("~/.reticulum")
BOARD = sys.argv[1] if len(sys.argv) > 1 else "89f609774be9dc1b2640bc5f23d81714"
TIMEOUT = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0

FIELD_COMMANDS = 0x09
FIELD_TELEMETRY = 0x02
CMD_TELEMETRY_REQUEST = 0x01

SID_NAMES = {
    0x01: "TIME", 0x02: "LOCATION", 0x03: "PRESSURE", 0x04: "BATTERY",
    0x05: "PHYSICAL_LINK", 0x06: "ACCELERATION", 0x07: "TEMPERATURE",
    0x08: "HUMIDITY", 0x09: "MAGNETIC_FIELD", 0x14: "RAM", 0x15: "NVM",
    0x19: "RNS_TRANSPORT",
}

received = []


def on_delivery(message):
    received.append(message)


def main():
    RNS.Reticulum(configdir=CONFIG_DIR, loglevel=RNS.LOG_WARNING)

    identity = RNS.Identity()
    router = LXMF.LXMRouter(identity=identity,
                            storagepath=tempfile.mkdtemp(prefix="lxmf-tel-test-"))
    local = router.register_delivery_identity(identity, display_name="telemetry-test")
    router.register_delivery_callback(on_delivery)

    # The node keeps peer keys from announces; without one it cannot encrypt a
    # reply. Give it a moment to hear this identity.
    router.announce(local.hash)
    print("announced local destination %s" % local.hash.hex())
    time.sleep(8)

    dest_hash = bytes.fromhex(BOARD)
    print("path to %s ..." % BOARD)
    RNS.Transport.request_path(dest_hash)
    deadline = time.time() + 60
    while not RNS.Transport.has_path(dest_hash) and time.time() < deadline:
        time.sleep(0.5)
    if not RNS.Transport.has_path(dest_hash):
        print("FAIL: no path to the node")
        return 1
    print("  path at %d hops" % RNS.Transport.hops_to(dest_hash))

    node_identity = RNS.Identity.recall(dest_hash)
    if node_identity is None:
        print("FAIL: could not recall the node's identity")
        return 1
    node_dest = RNS.Destination(node_identity, RNS.Destination.OUT,
                                RNS.Destination.SINGLE, "lxmf", "delivery")

    fields = {FIELD_COMMANDS: [{CMD_TELEMETRY_REQUEST: [int(time.time()), False]}]}
    print("sending TELEMETRY_REQUEST in FIELD_COMMANDS")
    message = LXMF.LXMessage(node_dest, local, "", "",
                             fields=fields,
                             desired_method=LXMF.LXMessage.OPPORTUNISTIC)
    message.try_propagation_on_fail = False
    router.handle_outbound(message)

    deadline = time.time() + TIMEOUT
    while not received and time.time() < deadline:
        time.sleep(0.5)

    if not received:
        print("FAIL: no reply within %.0f s" % TIMEOUT)
        return 1

    reply = received[0]
    # Python LXMF exposes packed_size as an attribute; microLXMF exposes a method.
    packed = reply.packed_size() if callable(reply.packed_size) else reply.packed_size
    print()
    print("reply received")
    print("  packed_size   %d bytes" % packed)
    print("  method        %s" % reply.method)
    print("  representation %s" % reply.representation)
    print("  content       %r" % reply.content[:60])

    telemetry = None
    for key, value in reply.fields.items():
        # Python LXMF gives integer field keys; microLXMF gives single bytes.
        k = key[0] if isinstance(key, (bytes, bytearray)) and len(key) == 1 else key
        if k == FIELD_TELEMETRY:
            telemetry = value
    if telemetry is None:
        print("  NO FIELD_TELEMETRY in the reply (fields present: %s)"
              % [k.hex() if isinstance(k, (bytes, bytearray)) else hex(k)
                 for k in reply.fields])
        return 1

    # core-reference LXMF-010 says the firmware must store a bin holding the
    # packed map, because microLXMF re-emits fields verbatim with
    # packRawBytes(). Python LXMF unwraps such a bin when it parses the outer
    # msgpack, so what arrives here is already the map and this branch cannot
    # distinguish the two cases. Do not read it as evidence either way: the
    # firmware's own `node telemetry` path is where LXMF-010 compliance lives.
    inner = umsgpack.unpackb(telemetry)
    if isinstance(inner, (bytes, bytearray)):
        sensor_map = umsgpack.unpackb(inner)
        print("  field value   %d bytes (still bin-wrapped)" % len(telemetry))
    else:
        sensor_map = inner
        print("  field value   %d bytes (decoded to the map)" % len(telemetry))
    print()
    print("  sensor map:")
    for sid, val in sorted(sensor_map.items()):
        print("    %-16s %s" % (SID_NAMES.get(sid, "0x%02x" % sid), val))

    print()
    if packed > 159:
        print("PASS: a %d-byte reply arrived and decoded, above the 159-byte"
              % packed)
        print("      single-packet threshold, so link delivery works on this stack.")
    else:
        print("OK: %d bytes, which fits one link-free packet." % packed)
    return 0


if __name__ == "__main__":
    sys.exit(main())
