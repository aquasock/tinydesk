#!/bin/sh
# Regenerate bhi260_klio.img from SensorLib, so the blob in this directory is
# reproducible rather than an unexplained binary.
#
# The image is: a 12-byte header (magic 'FUMI', payload length, a 32-bit
# checksum) followed by Bosch's BHI260AP KLIO firmware, converted from the C
# array SensorLib carries.
#
# Where the firmware comes from, and why this copy rather than Bosch's own
# download: SensorLib (MIT, src/bosch/firmware/bhi260/bosch_bhi260_klio.h)
# redistributes Bosch's BHY2 SDK including this image, and its licence
# (BSD-3-Clause, see ../../bhy2/LICENSE.BSD-3-Clause) permits redistribution.
# It is the SAME image the vendor firmware uploads, which matters: the KLIO
# variant carries the fusion algorithms (orientation, step counter, gestures),
# where the "shuttle" variant is 680 KB of extras this watch has no sensors for.
set -e
here=$(cd -- "$(dirname -- "$0")" && pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

git clone -q --depth 1 https://github.com/lewisxhe/SensorLib "$tmp/SensorLib"
python3 - "$tmp/SensorLib/src/bosch/firmware/bhi260/bosch_bhi260_klio.h" \
         "$here/bhi260_klio.img" <<'PY'
import re, struct, sys, hashlib
src = open(sys.argv[1]).read()
body = src[src.index('{') + 1: src.rindex('}')]
fw = bytes(int(v, 16) for v in re.findall(r'0x([0-9a-fA-F]{1,2})', body))
check = int.from_bytes(hashlib.sha256(fw).digest()[:4], 'little')
hdr = struct.pack('<III', 0x494D5546, len(fw), check)
open(sys.argv[2], 'wb').write(hdr + fw)
print("wrote %s: header 12 + payload %d" % (sys.argv[2], len(fw)))
PY
