#!/usr/bin/bash
set -e

if [ "${1:-}" != "--bench" ]; then
  echo "Refusing to start without the explicit bench flag."
  echo "Disconnect the device from the vehicle, then run:"
  echo "  bash tools/equinox_sim/launch.sh --bench"
  exit 2
fi

SIM_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_BASEDIR="$(cd "$SIM_SCRIPT_DIR/../.." && pwd)"
cd "$SIM_BASEDIR"

export PASSIVE=0
export SIMULATION=1
export NOBOARD=1
export NO_CAN_TIMEOUT=1
export EQUINOX_SIMULATOR=1
# This fork's shared-memory msgq has only 10 reader slots per service and
# evicts existing subscribers during the simulator's concurrent startup.
# Use the supported ZMQ transport for every bench process instead.
export ZMQ=1
export FINGERPRINT="CHEVROLET EQUINOX NO RADAR"
export SKIP_FW_QUERY=1

echo "Starting Equinox Virtual Panda bench simulator"
echo "Bench messaging transport: ZMQ"
echo "Physical pandad is disabled. Do not use this mode in a vehicle."
exec ./launch_chffrplus.sh
