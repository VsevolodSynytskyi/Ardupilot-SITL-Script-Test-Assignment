#!/usr/bin/env bash
# Launch ArduCopter SITL + MAVProxy router (headless).
#
# sim_vehicle.py is not used because the project path contains a space, which breaks
# its --defaults argument. Here the binary runs from sitl/run/ with relative paths.
#
# Endpoints:
#   udp:127.0.0.1:14550  GCS / manual MAVProxy (e.g. `mavproxy.py --master udp:127.0.0.1:14550`)
#   udp:127.0.0.1:14551  our controller script
#
# --streamrate=-1: MAVProxy must not re-request stream rates, or it overrides the
# SET_MESSAGE_INTERVAL rates our controller asks for (observed: 50 Hz -> 4 Hz).
#
# Usage: scripts/run_sitl.sh [--wipe]   (--wipe resets EEPROM to defaults + guided_thrust.parm)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/.venv/bin/activate"

RUN_DIR="$ROOT/sitl/run"
mkdir -p "$RUN_DIR"
cd "$RUN_DIR"

EXTRA=()
[[ "${1:-}" == "--wipe" ]] && EXTRA+=(-w)

../../ardupilot/build/sitl/bin/arducopter ${EXTRA[@]+"${EXTRA[@]}"} --model + --speedup 1 -I0 \
  --defaults @ROMFS/default_params/copter.parm,../guided_thrust.parm \
  > arducopter.log 2>&1 &
SITL_PID=$!
trap 'kill $SITL_PID 2>/dev/null || true' EXIT INT TERM

sleep 2
mavproxy.py --master tcp:127.0.0.1:5760 \
  --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 \
  --streamrate=-1 --non-interactive --state-basedir="$RUN_DIR"
