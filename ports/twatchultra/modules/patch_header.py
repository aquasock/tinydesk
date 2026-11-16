#!/usr/bin/env python3
"""Fill in the module header's size and entry offset after linking.

The header sits at offset 0 of the blob with those two fields as placeholders,
because neither is known until the link has placed everything. Patching them
here keeps them derivable from the linked image rather than typed in by hand --
a stale entry offset is a jump into the middle of an instruction.
"""
import re
import struct
import sys

path, nm_output = sys.argv[1], sys.argv[2]

entry = None
for line in nm_output.splitlines():
    parts = line.split()
    if len(parts) == 3 and parts[2] == "mod_entry":
        entry = int(parts[0], 16)

if entry is None:
    sys.exit("patch_header: mod_entry not found -- was it renamed or inlined?")

BASE = 0x42AE0000
data = bytearray(open(path, "rb").read())

magic, abi, size, entry_off = struct.unpack_from("<IIII", data, 0)
if magic != 0x4D4F444C:
    sys.exit("patch_header: bad magic 0x%08x at offset 0" % magic)

entry_off = entry - BASE
if not (0 <= entry_off < len(data)):
    sys.exit("patch_header: entry at 0x%x is outside the blob (base 0x%x, %d bytes)"
             % (entry, BASE, len(data)))

struct.pack_into("<II", data, 8, len(data), entry_off)
open(path, "wb").write(data)
print("patched: size=%d entry_offset=0x%x (mod_entry at 0x%x)" % (len(data), entry_off, entry))
