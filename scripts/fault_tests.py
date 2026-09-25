#!/usr/bin/env python3
"""Fault-injection tests for altitude_control against a running SITL (scripts/run_sitl.sh).

  python scripts/fault_tests.py [mode_change link_loss stall early_interrupt]

  mode_change      at HOLD_HIGH switch to LOITER from the GCS port -> controller releases control (exit 3)
  link_loss        at HOLD_HIGH drop all controller <-> SITL packets for 3 s (UDP relay) -> ERROR
                   (stale telemetry) -> LAND once the link is back
  stall            at HOLD_HIGH freeze the controller process for 1.5 s -> ERROR (loop stalled) -> LAND
  early_interrupt  Ctrl+C during start-up -> exits quickly, never arms
  armed_start      vehicle already flying (GCS take-off) -> controller refuses to start
"""
import os
import select
import signal
import socket
import subprocess
import sys
import threading
import time

from pymavlink import mavutil

ROOT = os.path.join(os.path.dirname(__file__), "..")
BIN = os.path.join(ROOT, "build", "altitude_control")
GCS_URL = "udp:127.0.0.1:14550"


class LinkRelay:
    """UDP relay between SITL (which sends to 14551) and the controller (moved to 14552).
    Setting `drop = True` silently discards traffic in both directions: a link outage."""

    SITL_SIDE = ("127.0.0.1", 14551)
    CONTROLLER_SIDE = ("127.0.0.1", 14552)

    def __init__(self):
        self.drop = False
        self._stop = False
        self._from_sitl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._from_sitl.bind(self.SITL_SIDE)
        self._to_ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sitl_addr = None
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        socks = [self._from_sitl, self._to_ctl]
        while not self._stop:
            ready, _, _ = select.select(socks, [], [], 0.1)
            for s in ready:
                data, addr = s.recvfrom(4096)
                if s is self._from_sitl:
                    self._sitl_addr = addr
                    if not self.drop:
                        self._to_ctl.sendto(data, self.CONTROLLER_SIDE)
                elif self._sitl_addr and not self.drop:
                    self._from_sitl.sendto(data, self._sitl_addr)

    def close(self):
        self._stop = True
        time.sleep(0.2)
        self._from_sitl.close()
        self._to_ctl.close()


class Controller:
    """altitude_control subprocess with its stdout collected line by line."""

    def __init__(self, name, url=None):
        self.name = name
        self.lines = []
        args = [BIN, "--set", f"log_path=logs/fault_{name}.csv"]
        if url:
            args += ["--set", f"connection_url={url}"]
        self.p = subprocess.Popen(
            args, cwd=ROOT,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        with open(os.path.join(ROOT, "logs", f"fault_{self.name}.txt"), "w") as f:
            for line in self.p.stdout:
                self.lines.append(line.rstrip())
                f.write(line)
                f.flush()

    def wait_for(self, text, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if any(text in l for l in self.lines):
                return True
            if self.p.poll() is not None:
                return False
            time.sleep(0.05)
        return False

    def finish(self, timeout=150):
        return self.p.wait(timeout=timeout)

    def mission_log(self):
        return [l for l in self.lines if l.startswith("[mission]") and "landing:" not in l]


_gcs = None


def gcs():
    """One shared GCS connection (the UDP port can only be bound once)."""
    global _gcs
    if _gcs is None:
        _gcs = mavutil.mavlink_connection(GCS_URL, source_system=254)
        _gcs.wait_heartbeat(timeout=10)
    while _gcs.recv_match(blocking=False):  # drop stale buffered messages
        pass
    return _gcs


def altitude(m, timeout=2):
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout)
    return msg.relative_alt / 1000.0 if msg else float("nan")


def wait_disarmed(m, timeout=90):
    while m.recv_match(blocking=False):  # buffered heartbeats may predate arming
        pass
    end = time.time() + timeout
    while time.time() < end:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if (hb and hb.get_srcSystem() == 1 and hb.get_srcComponent() == 1
                and not hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
    return False


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}: {name}{(' - ' + detail) if detail else ''}")
    return ok


def test_mode_change():
    print("== mode_change")
    c = Controller("mode_change")
    if not c.wait_for("-> HOLD_HIGH", 60):
        return check("reached HOLD_HIGH", False)
    m = gcs()
    m.set_mode("LOITER")
    rc = c.finish()
    ok = check("exit code 3 (released)", rc == 3, f"rc={rc}")
    ok &= check("no LAND commanded", not any("-> LAND" in l for l in c.lines))
    m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)  # refreshes m.flightmode
    ok &= check("vehicle still in LOITER", m.flightmode == "LOITER", m.flightmode)
    print("   ", "\n    ".join(c.mission_log()[-3:]))
    m.set_mode("LAND")  # clean up
    wait_disarmed(m)
    return ok


def test_stall():
    """Freeze the controller process for 1.5 s during HOLD_HIGH."""
    print("== stall")
    c = Controller("stall")
    if not c.wait_for("-> HOLD_HIGH", 60):
        return check("reached HOLD_HIGH", False)
    m = gcs()
    alt0 = altitude(m)
    os.kill(c.p.pid, signal.SIGSTOP)
    alts = []
    end = time.time() + 1.5
    while time.time() < end:
        alts.append(altitude(m, timeout=0.5))
    os.kill(c.p.pid, signal.SIGCONT)
    rc = c.finish()
    ok = check("error detected (control loop stalled)", any("control loop stalled" in l for l in c.lines))
    ok &= check("LAND commanded and disarmed", any("landed and disarmed" in l for l in c.lines), f"rc={rc}")
    drift = max(abs(a - alt0) for a in alts if a == a)
    ok &= check("altitude held during freeze", drift < 1.0, f"max drift {drift:.2f} m")
    print("   ", "\n    ".join(c.mission_log()[-4:]))
    return ok


def test_link_loss():
    """Drop all controller <-> SITL traffic for 3 s during HOLD_HIGH (the GCS link stays up)."""
    print("== link_loss")
    relay = LinkRelay()
    try:
        c = Controller("link_loss", url=f"udpin://0.0.0.0:{LinkRelay.CONTROLLER_SIDE[1]}")
        if not c.wait_for("-> HOLD_HIGH", 90):
            return check("reached HOLD_HIGH", False)
        m = gcs()
        alt0 = altitude(m)
        relay.drop = True
        alts = []
        end = time.time() + 3.0
        while time.time() < end:
            alts.append(altitude(m, timeout=0.5))
        relay.drop = False
        rc = c.finish()
    finally:
        relay.close()
    ok = check("error detected (telemetry stale)", any("telemetry stale" in l for l in c.lines))
    ok &= check("LAND commanded and disarmed", any("landed and disarmed" in l for l in c.lines), f"rc={rc}")
    drift = max(abs(a - alt0) for a in alts if a == a)
    ok &= check("altitude held during outage", drift < 1.0, f"max drift {drift:.2f} m")
    print("   ", "\n    ".join(c.mission_log()[-4:]))
    return ok


def test_early_interrupt():
    print("== early_interrupt")
    c = Controller("early_interrupt")
    time.sleep(1.0)
    t0 = time.time()
    c.p.send_signal(signal.SIGINT)
    rc = c.finish(timeout=30)
    dt = time.time() - t0
    ok = check("exited quickly", dt < 10, f"{dt:.1f} s, rc={rc}")
    ok &= check("never armed", not any("-> CLIMB_TO_HIGH" in l for l in c.lines))
    print("   ", "\n    ".join(c.mission_log()[-2:]))
    return ok


def test_armed_start():
    print("== armed_start")
    m = gcs()
    m.set_mode("GUIDED")
    m.arducopter_arm()
    m.motors_armed_wait()
    m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 3)
    time.sleep(6)
    c = Controller("armed_start")
    rc = c.finish(timeout=30)
    ok = check("refused to start", rc == 1 and any("already armed" in l for l in c.lines), f"rc={rc}")
    ok &= check("did not stream thrust", not any("-> CLIMB_TO_HIGH" in l for l in c.lines))
    print("   ", "\n    ".join(c.mission_log()[-1:]))
    m.set_mode("LAND")
    wait_disarmed(m)
    return ok


TESTS = {"early_interrupt": test_early_interrupt, "armed_start": test_armed_start,
         "mode_change": test_mode_change, "stall": test_stall, "link_loss": test_link_loss}

if __name__ == "__main__":
    names = sys.argv[1:] or list(TESTS)
    results = {n: TESTS[n]() for n in names}
    print("\n" + ", ".join(f"{n}: {'PASS' if r else 'FAIL'}" for n, r in results.items()))
    sys.exit(0 if all(results.values()) else 1)
