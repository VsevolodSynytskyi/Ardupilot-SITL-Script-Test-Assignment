#!/usr/bin/env bash
# Launch ArduCopter SITL (headless). SITL itself sends MAVLink over UDP, no router in between:
#   SERIAL0 -> udp:127.0.0.1:14550  ground station (QGroundControl, pymavlink scripts)
#   SERIAL1 -> udp:127.0.0.1:14551  the controller (altitude_control)
#
# The binary runs from sitl/run/ with relative paths because sim_vehicle.py breaks on
# project paths containing spaces (it splits the --defaults argument).
#
# Usage: scripts/run_sitl.sh [--wipe]   (--wipe resets EEPROM to defaults + guided_thrust.parm)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

RUN_DIR="$ROOT/sitl/run"
mkdir -p "$RUN_DIR"
cd "$RUN_DIR"

EXTRA=()
[[ "${1:-}" == "--wipe" ]] && EXTRA+=(-w)

exec ../../ardupilot/build/sitl/bin/arducopter ${EXTRA[@]+"${EXTRA[@]}"} --model + --speedup 1 -I0 \
  --defaults @ROMFS/default_params/copter.parm,../guided_thrust.parm \
  --serial0=udpclient:127.0.0.1:14550 \
  --serial1=udpclient:127.0.0.1:14551
