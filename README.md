# ArduCopter SITL: thrust-only altitude PID

A C++ program that controls the altitude of an ArduCopter in SITL using thrust commands only. It:

1. Switches to GUIDED mode.
2. Arms the copter. Arming directly in GUIDED lets the thrust stream start right away, before ArduPilot's auto-disarm can trigger.
3. Climbs to 10 m and holds.
4. Descends to 5 m and holds.
5. Switches to LAND.

- **Control:** a cascaded PID controller calculates the thrust. The program sends it to ArduPilot 50 times per second in `SET_ATTITUDE_TARGET` messages, with the attitude kept level. With `GUID_OPTIONS=8`, ArduPilot applies this value directly as motor thrust.
- **Stack:** C++17, MAVSDK v4, CMake, GoogleTest. Python only for plots and SITL test scripts.

## Quick start

Requirements: a C++17 compiler, CMake 3.20+, Python 3.10+ (with venv; `python3-venv` on Ubuntu/Debian) and git. Runs on macOS and Linux; on Windows, use WSL2 with Ubuntu.

Set up once, from the project root. The SITL build takes about 2 min and the MAVSDK build about 10 min:

```bash
git clone --recurse-submodules --shallow-submodules --depth 1 https://github.com/ArduPilot/ardupilot.git ardupilot
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
source .venv/bin/activate
(cd ardupilot && ./waf configure --board sitl && ./waf copter)
scripts/build_mavsdk.sh
cmake -S . -B build && cmake --build build -j && (cd build && ctest)
```

Run it, with each step in its own terminal, from the project root:

1. Start SITL and leave it running. It sends MAVLink to UDP 14550 (ground station, e.g. QGroundControl) and UDP 14551 (controller).

   ```bash
   scripts/run_sitl.sh --wipe
   ```

2. Run the mission. Settings are in `config/mission.conf`, and the flight log goes to `logs/flight.csv`.

   ```bash
   ./build/altitude_control
   ```

3. After the flight, plot altitude, climb rate and thrust to `logs/flight.png`:

   ```bash
   source .venv/bin/activate
   python scripts/plot.py logs/flight.csv
   ```

Other tools:
- `altitude_control --set key=value` overrides any config value.
- `--run telemetry` and `--run open-loop` are bring-up checks.
- `scripts/tune.py` runs gain sweeps; `scripts/flight_metrics.py` computes step-response metrics.
- `scripts/fault_tests.py` runs fault injection.

## How it works

### Control loop

On every control tick (50 Hz) the program:

1. Reads altitude and climb rate from `LOCAL_POSITION_NED`.
2. Moves the altitude setpoint toward the target along a smooth trajectory, limited in speed and acceleration.
3. **Outer loop (P):** converts the altitude error into a climb-rate setpoint.
4. **Inner loop (PID):** converts the climb-rate error into a thrust correction.
5. Sends `MOT_THST_HOVER` plus the correction as the thrust value.

### Take-off

Until the copter is 0.3 m above the ground, thrust is fixed at hover + 0.10, with the PID held in reset. The cascade then takes over from the current altitude and speed.

### Safety

Any of these triggers LAND:
- telemetry older than 0.2 s, or a loop stall over 0.5 s
- an unexpected disarm
- altitude above 15 m
- a state timeout (60 s)
- Ctrl+C

If someone else changes the flight mode, the controller stops and leaves the vehicle to them. It refuses to start on an armed vehicle.

Exit codes: 0 done, 1 error, 3 released to the operator, 130 interrupted.

### Code structure

The controller is in `include/altctl/` (headers) and `src/`:

| Class | Main methods | Role |
|---|---|---|
| `DroneInterface` | `connect`, `state`, `set_mode`, `arm`, `land`, `send_thrust` | All MAVLink communication through MAVSDK: telemetry, parameters, mode, arming, thrust commands |
| `AltitudeController` | `reset`, `update` | Take-off phase, setpoint trajectory and cascade; `update` returns the thrust for one tick |
| `PidController` | `update`, `reset`, `set_integrator_frozen` | PID with derivative on measurement, filtered D, anti-windup, output limits |
| `MissionRunner` | `run` (→ `prepare_vehicle`, `fly_mission`, `land_and_wait_disarmed`) | Mission state machine, 50 Hz control loop, safety checks |
| `DataLogger` | `open`, `write` | Per-tick CSV log |

Other parts of the code:
- **`Config`** holds all settings and their defaults. `config/mission.conf` and `--set key=value` override them.
- **`main.cpp`** parses arguments, handles Ctrl+C, and connects the classes together.
- **`diagnostics.cpp`** contains the `--run telemetry` and `--run open-loop` bring-up checks.

`MissionRunner` uses `DroneInterface`, `AltitudeController` and `DataLogger`. `AltitudeController` uses `PidController`. The controller classes don't depend on MAVSDK, so the unit tests in `tests/` run them against a simulated vertical-dynamics model without SITL. Config parsing is tested too.
