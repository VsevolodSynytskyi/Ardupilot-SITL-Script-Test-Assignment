#!/usr/bin/env python3
"""Stage 1 smoke test: GUID_OPTIONS readback + standard GUIDED takeoff to 5 m, then LAND.

Equivalent to the MAVProxy sequence `mode guided; arm throttle; takeoff 5`.
"""
import sys
import time

from pymavlink import mavutil

URL = sys.argv[1] if len(sys.argv) > 1 else "udp:127.0.0.1:14551"

m = mavutil.mavlink_connection(URL, source_system=250)
m.wait_heartbeat(timeout=30)
print(f"heartbeat: sys={m.target_system} comp={m.target_component}")

m.mav.param_request_read_send(m.target_system, m.target_component, b"GUID_OPTIONS", -1)
p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=5)
print(f"GUID_OPTIONS = {p.param_value if p else 'NO REPLY'}")

# Wait until the EKF/GPS are ready: ArduPilot reports it via SYS_STATUS prearm bit.
t0 = time.time()
while time.time() - t0 < 120:
    s = m.recv_match(type="SYS_STATUS", blocking=True, timeout=2)
    if s and s.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK:
        print(f"pre-arm checks OK after {time.time() - t0:.0f}s")
        break
else:
    sys.exit("pre-arm checks never passed")

m.set_mode("GUIDED")
m.arducopter_arm()
m.motors_armed_wait()
print("armed")
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 5)
ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
print(f"takeoff ack: {ack.result if ack else 'none'}")

t0 = time.time()
while time.time() - t0 < 30:
    g = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    if g:
        alt = g.relative_alt / 1000.0
        print(f"t={time.time() - t0:5.1f}s rel_alt={alt:5.2f} m")
        if alt > 4.8:
            break
m.set_mode("LAND")
print("LAND; waiting for disarm")
m.motors_disarmed_wait()
print("disarmed - smoke test OK")
