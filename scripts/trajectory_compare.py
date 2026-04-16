#!/usr/bin/env python3
"""Compare two trajectory CSV files (sim vs real) and produce metrics + plots.

Reads CSVs produced by trajectory_recorder.py:
    columns: time_sec, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6

Workflow:
    1. Loads both CSVs.
    2. Resamples both trajectories onto a common time grid using linear
       interpolation (motion start t=0 in both).
    3. Computes per-joint position error over time.
    4. Prints summary: max, mean, RMSE error per joint (in degrees).
    5. Optionally saves a 12-panel plot (6 overlay + 6 error).

Usage:
    python3 trajectory_compare.py sim.csv real.csv
    python3 trajectory_compare.py sim.csv real.csv --threshold-deg 2.0 --plot comparison.png

Runs anywhere Python 3 + numpy are available (no ROS needed).
matplotlib is optional — if not installed, plots are skipped.
"""

import argparse
import csv
import math
import sys

import numpy as np

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
RAD2DEG = 180.0 / math.pi


def load_csv(path):
    with open(path) as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    t = np.array([float(r["time_sec"]) for r in rows])
    q = np.array([[float(r[j]) for j in JOINT_NAMES] for r in rows])
    return t, q


def resample(t_src, q_src, t_dst):
    q_out = np.zeros((len(t_dst), q_src.shape[1]))
    for j in range(q_src.shape[1]):
        q_out[:, j] = np.interp(t_dst, t_src, q_src[:, j])
    return q_out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_a", help="First CSV (e.g. sim_traj.csv)")
    parser.add_argument("csv_b", help="Second CSV (e.g. real_traj.csv)")
    parser.add_argument(
        "--threshold-deg",
        type=float,
        default=2.0,
        help="Max allowable error per joint [deg] for PASS (default: 2.0)",
    )
    parser.add_argument("--plot", default=None, help="Save plot to this file (PNG)")
    parser.add_argument(
        "--hz",
        type=float,
        default=100.0,
        help="Resample rate for comparison [Hz]",
    )
    args = parser.parse_args()

    t_a, q_a = load_csv(args.csv_a)
    t_b, q_b = load_csv(args.csv_b)

    print(f"Loaded {args.csv_a}: {len(t_a)} samples, {t_a[-1]:.2f}s")
    print(f"Loaded {args.csv_b}: {len(t_b)} samples, {t_b[-1]:.2f}s")

    t_end = min(t_a[-1], t_b[-1])
    dt = 1.0 / args.hz
    t_common = np.arange(0, t_end + dt, dt)

    qa = resample(t_a, q_a, t_common)
    qb = resample(t_b, q_b, t_common)

    err = (qa - qb) * RAD2DEG

    print(f"\nComparison over {t_end:.2f}s ({len(t_common)} samples at {args.hz} Hz):")
    print(f"{'Joint':<10} {'Max [deg]':>10} {'Mean [deg]':>11} {'RMSE [deg]':>11}")
    print("-" * 44)

    all_pass = True
    for j, name in enumerate(JOINT_NAMES):
        e = err[:, j]
        mx = float(np.max(np.abs(e)))
        mn = float(np.mean(np.abs(e)))
        rmse = float(np.sqrt(np.mean(e**2)))
        ok = mx <= args.threshold_deg
        flag = "PASS" if ok else "FAIL"
        if not ok:
            all_pass = False
        print(f"{name:<10} {mx:10.4f} {mn:11.4f} {rmse:11.4f}   {flag}")

    print("-" * 44)
    verdict = "PASS" if all_pass else "FAIL"
    print(f"Overall: {verdict} (threshold={args.threshold_deg}° per joint)")

    if args.plot:
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            fig, axes = plt.subplots(6, 2, figsize=(14, 18), sharex="col")
            fig.suptitle(
                f"Sim vs Real trajectory comparison (threshold={args.threshold_deg}°)",
                fontsize=14,
            )

            for j, name in enumerate(JOINT_NAMES):
                ax_overlay = axes[j, 0]
                ax_overlay.plot(t_common, qa[:, j] * RAD2DEG, label="sim", linewidth=1)
                ax_overlay.plot(
                    t_common, qb[:, j] * RAD2DEG, label="real", linewidth=1, linestyle="--"
                )
                ax_overlay.set_ylabel(f"{name} [deg]")
                ax_overlay.legend(fontsize=8)
                ax_overlay.grid(True, alpha=0.3)
                if j == 0:
                    ax_overlay.set_title("Position overlay")

                ax_err = axes[j, 1]
                ax_err.plot(t_common, err[:, j], color="red", linewidth=0.8)
                ax_err.axhline(
                    args.threshold_deg, color="gray", linestyle=":", linewidth=0.5
                )
                ax_err.axhline(
                    -args.threshold_deg, color="gray", linestyle=":", linewidth=0.5
                )
                ax_err.set_ylabel(f"{name} err [deg]")
                ax_err.grid(True, alpha=0.3)
                if j == 0:
                    ax_err.set_title("Position error")

            axes[-1, 0].set_xlabel("Time [s]")
            axes[-1, 1].set_xlabel("Time [s]")
            fig.tight_layout(rect=[0, 0, 1, 0.97])
            fig.savefig(args.plot, dpi=150)
            print(f"\nPlot saved: {args.plot}")
        except ImportError:
            print(
                "\nmatplotlib not installed — skipping plot. "
                "Install with: pip install matplotlib"
            )

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
