#!/usr/bin/env bash
set -euo pipefail

SOURCE_ROOT=/source
BUILD_ROOT=/build
OUTPUT_ROOT=/out

rm -rf "${BUILD_ROOT}/panda"
cp -a "${SOURCE_ROOT}/panda" "${BUILD_ROOT}/panda"
rm -rf "${BUILD_ROOT}/panda/board/obj"
mkdir -p "${BUILD_ROOT}/panda/board/obj" "${OUTPUT_ROOT}"
sed -i 's/\r$//' "${BUILD_ROOT}/panda/crypto/sign.py"
chmod +x "${BUILD_ROOT}/panda/crypto/sign.py"

cat > "${BUILD_ROOT}/SConstruct" <<'SCONS'
SConscript(["panda/board/SConscript"])
SCONS

echo "Toolchain:"
arm-none-eabi-gcc --version | head -n 1
echo "SCons:"
scons --version | sed -n '2p'

cd "${BUILD_ROOT}"
PEDAL=1 scons -j"$(nproc)" \
  panda/board/obj/pedal.bin.signed \
  panda/board/obj/bootstub.pedal.bin

cp panda/board/obj/pedal.bin "${OUTPUT_ROOT}/pedal.bin"
cp panda/board/obj/pedal.bin.signed "${OUTPUT_ROOT}/pedal.bin.signed"
cp panda/board/obj/bootstub.pedal.bin "${OUTPUT_ROOT}/bootstub.pedal.bin"

python3 - "${OUTPUT_ROOT}/pedal.bin.signed" <<'PY'
import struct
import sys
from pathlib import Path

path = Path(sys.argv[1])
data = path.read_bytes()
declared_length = struct.unpack_from("<I", data, 0)[0]
marker_offset = len(data) - 136
marker = data[marker_offset:marker_offset + 4]
version = struct.unpack_from("<I", data, marker_offset + 4)[0]

if marker != b"VERS" or version != 2 or declared_length != len(data) - 128:
  raise SystemExit(
    "invalid signed firmware: "
    f"marker={marker!r}, version={version}, declared_length={declared_length}"
  )

print(f"Signed firmware structure: VERS, version {version}")
PY

echo "Build artifacts:"
sha256sum \
  "${OUTPUT_ROOT}/pedal.bin.signed" \
  "${OUTPUT_ROOT}/pedal.bin" \
  "${OUTPUT_ROOT}/bootstub.pedal.bin"
