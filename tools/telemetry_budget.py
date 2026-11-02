#!/usr/bin/env python3
"""Measure what fits in one LXMF packet as FIELD_TELEMETRY.

LXMF has no maximum message size (core-reference LXMF-011). A message is
carried either as a single OPPORTUNISTIC packet with no link established, or by
DIRECT delivery over a link, where anything larger than one link packet becomes
a RESOURCE transfer. The 159-byte LORA_ENCRYPTED_PACKET_MDU is only the
threshold for the first case. Above it the message still arrives; it just costs
a link setup and a resource transfer, which over LoRa is seconds of airtime
rather than a fraction of a second.

So this tool does not report what is permitted. It reports how much of the one
packet each sensor consumes, so a sensor set can be chosen on cost.

The model is traced from the vendored library, not guessed:

  LXMessage::pack()                  microLXMF/src/LXMF/LXMessage.cpp
    _packed = destination_hash(16) + source_hash(16) + signature(64)
              + wire_payload
  wire_payload
    msgpack array(4) [ timestamp float64, title bin, content bin, fields map ]
  fields map
    packMapSize(n) then, per field, packRawBytes(key) + packRawBytes(value)

Both key and value go through packRawBytes, so each must already be valid
msgpack. A FIELD_TELEMETRY value is therefore a msgpack *bin* holding the
packed sensor map -- the rule in core-reference LXMF-010, whose violation makes
clients reject the field with "packed data must be type bytes".

Sensor value shapes are taken from Sideband 2.2.0 sbapp/sideband/sense.py:
Time is an int, Pressure/Temperature/Humidity are floats, Location is seven
items (six big-endian structs plus a last-update epoch), MagneticField is three
floats.

Every figure this tool prints was cross-checked in the same session against the
real vendored MsgPack::Packer compiled on the host, and the two agree exactly.

Usage:  ~/.venvs/rns/bin/python tools/telemetry_budget.py
The RNS virtualenv is used only because it carries umsgpack (as
RNS.vendor.umsgpack; there is no top-level umsgpack module).
"""

import struct
import sys

try:
    import umsgpack
except ImportError:
    # RNS ships the same packer under RNS.vendor; it is the copy the reference
    # LXMF implementation on this box actually uses.
    try:
        from RNS.vendor import umsgpack
    except ImportError:
        sys.exit("umsgpack not found; run with ~/.venvs/rns/bin/python")

# ---- constants traced from microLXMF/src/LXMF/Type.h ----------------------
DESTINATION_LENGTH = 16
SIGNATURE_LENGTH = 64
LORA_ENCRYPTED_PACKET_MDU = 159
FIXED_HEADER = DESTINATION_LENGTH * 2 + SIGNATURE_LENGTH  # 96

# ---- sensor ids, from Sideband sense.py ----------------------------------
SID_TIME = 0x01
SID_LOCATION = 0x02
SID_PRESSURE = 0x03
SID_TEMPERATURE = 0x07
SID_HUMIDITY = 0x08
SID_MAGNETIC_FIELD = 0x09

FIELD_TELEMETRY = 0x02


def sensor_values():
    """One representative reading per sensor, in sense.py's packed shape."""
    lat, lon, alt = 33.293893, -111.880882, 377.0
    return {
        SID_TIME: 1762000000,  # int, matching packed[SID_TIME] = int(time.time())
        SID_LOCATION: [
            struct.pack("!i", int(round(lat, 6) * 1e6)),
            struct.pack("!i", int(round(lon, 6) * 1e6)),
            struct.pack("!i", int(round(alt, 2) * 1e2)),
            struct.pack("!I", int(round(0.0, 2) * 1e2)),   # speed
            struct.pack("!i", int(round(0.0, 2) * 1e2)),   # bearing
            struct.pack("!H", int(round(8.0, 2) * 1e2)),   # accuracy
            1762000000,                                     # last_update
        ],
        SID_PRESSURE: 1013.25,
        SID_TEMPERATURE: 24.8,
        SID_HUMIDITY: 42.2,
        SID_MAGNETIC_FIELD: [35.3, 4.9, -44.2],
    }


NAMES = {
    SID_TIME: "TIME",
    SID_LOCATION: "LOCATION",
    SID_PRESSURE: "PRESSURE",
    SID_TEMPERATURE: "TEMPERATURE",
    SID_HUMIDITY: "HUMIDITY",
    SID_MAGNETIC_FIELD: "MAGNETIC_FIELD",
}

# What each sensor's value looks like on the wire, for the report.
SHAPES = {
    SID_TIME: "int epoch",
    SID_LOCATION: "array(7): 6 x bin + int",
    SID_PRESSURE: "float64",
    SID_TEMPERATURE: "float64",
    SID_HUMIDITY: "float64",
    SID_MAGNETIC_FIELD: "array(3) of float64",
}


def telemetry_map_bytes(ids, values):
    """The sensor map itself: msgpack map of SID -> value."""
    return umsgpack.packb({sid: values[sid] for sid in ids})


def field_value_bytes(ids, values):
    """The LXMF field value: a bin holding the packed sensor map (LXMF-010)."""
    return umsgpack.packb(telemetry_map_bytes(ids, values))


def fields_map_bytes(ids, values):
    """The LXMF fields map: one FIELD_TELEMETRY entry, key and value raw."""
    return (bytes([0x81])  # fixmap of one entry
            + umsgpack.packb(FIELD_TELEMETRY)
            + field_value_bytes(ids, values))


def wire_payload_bytes(ids, values, timestamp=1762000000.0):
    """msgpack array(4) [timestamp, title, content, fields]."""
    return (bytes([0x94])
            + umsgpack.packb(float(timestamp))
            + umsgpack.packb(b"")          # title
            + umsgpack.packb(b"")          # content
            + fields_map_bytes(ids, values))


def envelope_size(timestamp=1762000000.0):
    """array(4) header + timestamp + empty title + empty content.

    Everything in the wire payload except the fields map itself.
    """
    return (1                                    # fixarray of 4
            + len(umsgpack.packb(float(timestamp)))
            + len(umsgpack.packb(b""))          # title
            + len(umsgpack.packb(b"")))         # content


def packed_size(ids, values):
    """The exact quantity microLXMF compares against 159."""
    return FIXED_HEADER + len(wire_payload_bytes(ids, values))


def main():
    values = sensor_values()

    # ---- the budget ----
    envelope = envelope_size()
    map_header = 1      # fixmap, one entry
    field_key = len(umsgpack.packb(FIELD_TELEMETRY))   # 1 byte for 0x02
    bin_header = 2      # bin8 header around the sensor map
    overhead = map_header + field_key + bin_header
    sensor_map_budget = (LORA_ENCRYPTED_PACKET_MDU - FIXED_HEADER - envelope
                         - overhead)

    print("LXMF one-packet budget (single OPPORTUNISTIC packet, no link)")
    print("=" * 62)
    print("  threshold: packed_size <= %d bytes" % LORA_ENCRYPTED_PACKET_MDU)
    print("  This is NOT a size limit. Above it LXMF uses DIRECT + a RESOURCE")
    print("  transfer and the message still arrives; it just costs link setup")
    print("  and airtime. See core-reference LXMF-011.")
    print()
    print("  packed_size is built as:")
    print("    destination hash          %3d" % DESTINATION_LENGTH)
    print("    source hash               %3d" % DESTINATION_LENGTH)
    print("    signature                 %3d" % SIGNATURE_LENGTH)
    print("    msgpack envelope          %3d   array(4) + float64 ts"
          " + bin title + bin content" % envelope)
    print("    fields map header         %3d   fixmap of 1 entry" % map_header)
    print("    FIELD_TELEMETRY key       %3d" % field_key)
    print("    bin header on the value   %3d" % bin_header)
    print("    " + "-" * 40)
    print("    committed before sensors  %3d" % (FIXED_HEADER + envelope + overhead))
    print()
    print("  ==> the packed SENSOR MAP may be at most %d bytes"
          % sensor_map_budget)
    print("      (the field value bin is then %d bytes: map + %d header)"
          % (sensor_map_budget + bin_header, bin_header))
    print()

    # ---- per-sensor cost ----
    print("Cost of each sensor inside the packed sensor map")
    print("=" * 62)
    print("  %-16s %-26s %4s %5s %6s"
          % ("sensor", "wire shape", "key", "value", "total"))
    base = telemetry_map_bytes([SID_TIME], values)
    costs = {}
    for sid in (SID_TIME, SID_LOCATION, SID_PRESSURE, SID_TEMPERATURE,
                SID_HUMIDITY, SID_MAGNETIC_FIELD):
        key = len(umsgpack.packb(sid))
        val = len(umsgpack.packb(values[sid]))
        if sid == SID_TIME:
            total = len(base) - map_header
        else:
            with_it = telemetry_map_bytes([SID_TIME, sid], values)
            total = len(with_it) - len(base)
        costs[sid] = total
        print("  %-16s %-26s %4d %5d %6d"
              % (NAMES[sid], SHAPES[sid], key, val, total))
    print("  %-16s %-26s %20s" % ("", "map header (fixmap)", map_header))
    print()
    print("  A key+value pair whose total is <= %d can coexist with SID_TIME,"
          % (sensor_map_budget - costs[SID_TIME]))
    print("  which itself costs %d and cannot be omitted: Telemeter.packed()"
          % costs[SID_TIME])
    print("  always writes SID_TIME, and clients render their Timestamp from it.")
    print()

    # ---- each sensor alone, with TIME ----
    print("Each sensor beside SID_TIME, packed")
    print("=" * 62)
    print("  %-38s %6s %6s  %s" % ("reply", "map", "packed", "carried as"))
    for sid in (SID_LOCATION, SID_PRESSURE, SID_TEMPERATURE, SID_HUMIDITY,
                SID_MAGNETIC_FIELD):
        ids = [SID_TIME, sid]
        tmap = telemetry_map_bytes(ids, values)
        total = packed_size(ids, values)
        if total <= LORA_ENCRYPTED_PACKET_MDU:
            verdict = "1 packet (%d spare)" % (LORA_ENCRYPTED_PACKET_MDU - total)
        else:
            verdict = "link + resource (%d over)" % (
                total - LORA_ENCRYPTED_PACKET_MDU)
        print("  %-38s %6d %6d  %s"
              % ("TIME + " + NAMES[sid], len(tmap), total, verdict))
    print()

    # ---- useful combinations ----
    combos = [
        ("TIME + env (PRESSURE, TEMPERATURE, HUMIDITY)",
         [SID_TIME, SID_PRESSURE, SID_TEMPERATURE, SID_HUMIDITY]),
        ("TIME + LOCATION", [SID_TIME, SID_LOCATION]),
        ("LOCATION alone (drops mandatory TIME)", [SID_LOCATION]),
        ("TIME + MAGNETIC_FIELD", [SID_TIME, SID_MAGNETIC_FIELD]),
        ("TIME + LOCATION + env", [SID_TIME, SID_LOCATION, SID_PRESSURE,
                                   SID_TEMPERATURE, SID_HUMIDITY]),
        ("TIME + MAGNETIC_FIELD + env", [SID_TIME, SID_MAGNETIC_FIELD,
                                         SID_PRESSURE, SID_TEMPERATURE,
                                         SID_HUMIDITY]),
        ("TIME + LOCATION + MAGNETIC_FIELD + env",
         [SID_TIME, SID_LOCATION, SID_MAGNETIC_FIELD, SID_PRESSURE,
          SID_TEMPERATURE, SID_HUMIDITY]),
    ]
    print("Candidate replies")
    print("=" * 62)
    print("  %-44s %5s %6s  %s" % ("reply", "map", "packed", "carried as"))
    for label, ids in combos:
        tmap = telemetry_map_bytes(ids, values)
        total = packed_size(ids, values)
        if total <= LORA_ENCRYPTED_PACKET_MDU:
            verdict = "1 packet, no link (%d spare)" % (
                LORA_ENCRYPTED_PACKET_MDU - total)
        else:
            verdict = "link + resource (%d over)" % (
                total - LORA_ENCRYPTED_PACKET_MDU)
        print("  %-44s %5d %6d  %s" % (label, len(tmap), total, verdict))
    print()
    print("  Nothing here is refused. The right-hand column is the cost of the")
    print("  reading, and the choice is how often a sensor is worth it:")
    print("  a link-free packet is one transmission, while a resource transfer")
    print("  sets up a link first and then moves the whole payload, which over")
    print("  LoRa is seconds rather than a fraction of a second (FRM-005).")


if __name__ == "__main__":
    main()
