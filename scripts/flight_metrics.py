#!/usr/bin/env python3
"""Step-response metrics from a flight CSV written by altitude_control (logs/flight.csv).

Usage: python scripts/flight_metrics.py [logs/flight.csv]
"""
import csv
import math
import sys

SEGMENTS = (
    ("up", ("CLIMB_TO_HIGH", "HOLD_HIGH"), "HOLD_HIGH", +1),
    ("down", ("DESCEND_TO_LOW", "HOLD_LOW"), "HOLD_LOW", -1),
)


def settling_time_s(times_s, altitudes_m, target_m, band_m):
    """Time (from the first sample) after which |altitude - target| stays within the band."""
    for index in range(len(altitudes_m) - 1, -1, -1):
        if abs(altitudes_m[index] - target_m) > band_m:
            return times_s[index + 1] - times_s[0] if index + 1 < len(times_s) else math.nan
    return 0.0


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else math.nan


def compute(path):
    with open(path) as log_file:
        rows = list(csv.DictReader(log_file))
    if not rows:
        raise ValueError(f"{path}: empty log")

    def column(row, name):
        return float(row[name])

    metrics = {}
    for name, states, hold_state, direction in SEGMENTS:
        segment = [row for row in rows if row["state"] in states]
        if not segment:
            continue
        target_m = column(segment[0], "target_alt_m")
        times_s = [column(row, "time_s") for row in segment]
        altitudes_m = [column(row, "alt_m") for row in segment]
        metrics[f"overshoot_{name}_m"] = max(direction * (alt - target_m) for alt in altitudes_m)
        metrics[f"settle25_{name}_s"] = settling_time_s(times_s, altitudes_m, target_m, 0.25)
        metrics[f"settle10_{name}_s"] = settling_time_s(times_s, altitudes_m, target_m, 0.10)

        hold = [row for row in segment if row["state"] == hold_state]
        hold_errors_m = [column(row, "alt_m") - target_m for row in hold]
        metrics[f"hold_rms_{name}_m"] = rms(hold_errors_m)
        metrics[f"hold_max_{name}_m"] = max((abs(error) for error in hold_errors_m), default=math.nan)

        moving = [row for row in segment if row["state"] != hold_state and column(row, "alt_m") > 0.5]
        metrics[f"vel_rms_{name}_mps"] = rms(
            [column(row, "climb_setpoint_mps") - column(row, "climb_mps") for row in moving])
        hold_thrust = [column(row, "thrust") for row in hold]
        metrics[f"thrust_jitter_{name}"] = rms(
            [after - before for before, after in zip(hold_thrust, hold_thrust[1:])])

    loop_periods_s = [column(row, "loop_period_s") for row in rows[1:]]
    metrics["loop_dt_max_ms"] = 1000 * max(loop_periods_s) if loop_periods_s else math.nan
    metrics["thrust_max"] = max(column(row, "thrust") for row in rows)
    metrics["i_final"] = column(rows[-1], "i")
    return metrics


if __name__ == "__main__":
    log_path = sys.argv[1] if len(sys.argv) > 1 else "logs/flight.csv"
    for metric, value in compute(log_path).items():
        print(f"{metric:22s} {value:8.3f}")
