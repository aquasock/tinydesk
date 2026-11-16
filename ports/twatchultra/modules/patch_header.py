#!/usr/bin/env python3
"""Fill in the module header's build-derived fields after linking.

Four things in the header cannot be known until the link has run:

  size          how many bytes the blob occupies
  entry_offset  where mod_entry landed, as an offset from the mapped base
  linked_inst   the address .text was linked for
  linked_data   the address .rodata was linked for

Patching them here keeps them derived from the linked image and from
moddata.at rather than typed in by hand. A stale entry offset is a jump into the
middle of an instruction; a stale linked pair is worse, because the blob still
runs and only fails when it reads a constant -- see modload.h.

    patch_header.py <blob> <nm output> <link base> <data base>
"""
import struct
import sys

if len(sys.argv) < 5:
    sys.exit(__doc__)

path, nm_output = sys.argv[1], sys.argv[2]
base = int(sys.argv[3], 0)
data_base = int(sys.argv[4], 0)

entry = None
for line in nm_output.splitlines():
    parts = line.split()
    if len(parts) == 3 and parts[2] == "mod_entry":
        entry = int(parts[0], 16)

if entry is None:
    sys.exit("patch_header: mod_entry not found -- was it renamed or inlined?")

blob = bytearray(open(path, "rb").read())

# Layout of modload_header_t: magic, abi, size, entry_offset, char name[24],
# then the linked pair. name[] is 24 bytes from offset 16, so the pair follows
# at 40 and 44 with no padding.
SIZE_OFF, ENTRY_OFF, LINKED_INST, LINKED_DATA = 8, 12, 40, 44

magic, abi = struct.unpack_from("<II", blob, 0)
if magic != 0x4D4F444C:
    sys.exit("patch_header: bad magic 0x%08x at offset 0" % magic)

if len(blob) < LINKED_DATA + 4:
    sys.exit("patch_header: blob is %d bytes, too short to hold the header"
             % len(blob))

entry_off = entry - base
if not (0 <= entry_off < len(blob)):
    sys.exit("patch_header: entry at 0x%x is outside the blob (base 0x%x, %d bytes)"
             % (entry, base, len(blob)))

struct.pack_into("<II", blob, SIZE_OFF, len(blob), entry_off)
struct.pack_into("<II", blob, LINKED_INST, base, data_base)
open(path, "wb").write(blob)

print("patched: size=%d entry_offset=0x%x (mod_entry at 0x%x) "
      "linked inst=0x%x data=0x%x" % (len(blob), entry_off, entry, base, data_base))
