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

# Started from a shell rather than by Android, this process sits in the
# /dev/cpuset/android group, which launch_chffrplus.sh restricts to cores 0-1.
# The bench processes then pin themselves to cores 2 and 3 the way the onroad
# ones do and die with EINVAL from sched_setaffinity. Move into the cpuset
# openpilot is meant to run in (0-3); children inherit it.
if [ -w /dev/cpuset/app/tasks ]; then
  echo $$ > /dev/cpuset/app/tasks || true
  ALLOWED="$(python3 -c 'import os; print(len(os.sched_getaffinity(0)))' 2>/dev/null || echo '?')"
  if [ "$ALLOWED" = "2" ]; then
    echo "WARNING: still limited to 2 cores; core pinning will fail."
    echo "  check: cat /dev/cpuset/app/cpus   (expected 0-3)"
  fi
fi

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
