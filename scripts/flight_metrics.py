#!/usr/bin/env python3
"""Step-response metrics from a flight CSV written by altitude_control (logs/flight.csv).

Usage: python scripts/flight_metrics.py [logs/flight.csv]
"""
import csv
import math
import sys


def _settle_time(t, alt, target, band):
    """Time (from the first sample) after which |alt - target| stays within band."""
    for i in range(len(alt) - 1, -1, -1):
        if abs(alt[i] - target) > band:
            return t[i + 1] - t[0] if i + 1 < len(t) else math.nan
    return 0.0


def _rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else math.nan


def compute(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        raise ValueError(f"{path}: empty log")
    num = lambda r, k: float(r[k])
    m = {}
    segments = (
        ("up", ("CLIMB_TO_HIGH", "HOLD_HIGH"), "HOLD_HIGH", +1),
        ("down", ("DESCEND_TO_LOW", "HOLD_LOW"), "HOLD_LOW", -1),
    )
    for name, states, hold_state, direction in segments:
        seg = [r for r in rows if r["state"] in states]
        if not seg:
            continue
        target = num(seg[0], "target_alt_m")
        t = [num(r, "t_s") for r in seg]
        alt = [num(r, "alt_m") for r in seg]
        m[f"overshoot_{name}_m"] = max(direction * (a - target) for a in alt)
        m[f"settle25_{name}_s"] = _settle_time(t, alt, target, 0.25)
        m[f"settle10_{name}_s"] = _settle_time(t, alt, target, 0.10)
        hold = [r for r in seg if r["state"] == hold_state]
        m[f"hold_rms_{name}_m"] = _rms([num(r, "alt_m") - target for r in hold])
        m[f"hold_max_{name}_m"] = max((abs(num(r, "alt_m") - target) for r in hold), default=math.nan)
        moving = [r for r in seg if r["state"] != hold_state and num(r, "alt_m") > 0.5]
        m[f"vel_rms_{name}_ms"] = _rms([num(r, "climb_sp_ms") - num(r, "climb_ms") for r in moving])
        thrust = [num(r, "thrust") for r in hold]
        m[f"thrust_jitter_{name}"] = _rms([b - a for a, b in zip(thrust, thrust[1:])])
    dts = [num(r, "dt_s") for r in rows[1:]]
    m["loop_dt_max_ms"] = 1000 * max(dts) if dts else math.nan
    m["thrust_max"] = max(num(r, "thrust") for r in rows)
    m["i_final"] = num(rows[-1], "i")
    return m


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "logs/flight.csv"
    for k, v in compute(path).items():
        print(f"{k:22s} {v:8.3f}")
