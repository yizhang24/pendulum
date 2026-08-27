#!/usr/bin/env python3
"""Compare PID and LQR logs produced by the pendulum controller."""

import csv
import math
import sys
from pathlib import Path


def load(path):
    with path.open(newline="") as stream:
        return [
            {
                key: value if key in ("controller", "log_name") else float(value)
                for key, value in row.items()
            }
            for row in csv.DictReader(stream)
        ]


def metrics(rows, disturbance_duration):
    # Exclude the exact transition sample, which can still carry the
    # disturbance because of floating-point simulation-time rounding.
    rows = [row for row in rows if row["time"] > disturbance_duration]
    if len(rows) < 2:
        raise ValueError("log has too few post-disturbance samples")

    duration = rows[-1]["time"] - rows[0]["time"]
    angle_sq = sum(row["pendulum_angle"] ** 2 for row in rows)
    cart_sq = sum(row["cart_position"] ** 2 for row in rows)
    force_sq = sum(row["force"] ** 2 for row in rows)

    angle_iae = 0.0
    for previous, current in zip(rows, rows[1:]):
        dt = current["time"] - previous["time"]
        angle_iae += 0.5 * dt * (
            abs(previous["pendulum_angle"]) + abs(current["pendulum_angle"])
        )

    settled_after = None
    last_outside = -1
    for index, row in enumerate(rows):
        if abs(row["pendulum_angle"]) > 0.01 or abs(row["cart_position"]) > 0.02:
            last_outside = index
    if last_outside + 1 < len(rows):
        settled_after = rows[last_outside + 1]["time"] - disturbance_duration

    return {
        "samples": len(rows),
        "duration_s": duration,
        "angle_rmse_rad": math.sqrt(angle_sq / len(rows)),
        "cart_rmse_m": math.sqrt(cart_sq / len(rows)),
        "angle_iae_rad_s": angle_iae,
        "peak_angle_rad": max(abs(row["pendulum_angle"]) for row in rows),
        "force_rms_n": math.sqrt(force_sq / len(rows)),
        "settling_time_s": settled_after,
    }


def improvement(pid, lqr):
    return 100.0 * (pid - lqr) / pid if pid else float("nan")


def main():
    if len(sys.argv) not in (3, 4):
        raise SystemExit(
            "usage: analyze_controller_logs.py PID.csv LQR.csv [disturbance_duration]"
        )
    disturbance_duration = float(sys.argv[3]) if len(sys.argv) == 4 else 0.15
    pid = metrics(load(Path(sys.argv[1])), disturbance_duration)
    lqr = metrics(load(Path(sys.argv[2])), disturbance_duration)

    print("metric,pid,lqr,lqr_improvement_percent")
    for key in (
        "angle_rmse_rad",
        "cart_rmse_m",
        "angle_iae_rad_s",
        "peak_angle_rad",
        "force_rms_n",
        "settling_time_s",
    ):
        pid_value = pid[key]
        lqr_value = lqr[key]
        if pid_value is None or lqr_value is None:
            print(f"{key},{pid_value},{lqr_value},n/a")
        else:
            print(
                f"{key},{pid_value:.9g},{lqr_value:.9g},"
                f"{improvement(pid_value, lqr_value):.3f}"
            )


if __name__ == "__main__":
    main()
