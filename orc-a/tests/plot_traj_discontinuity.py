#!/usr/bin/env python3
"""Plot the CSV produced by traj_discontinuity_test.cpp (setpoint vs. analytic reference).

Regenerate the CSV inside the devcontainer with:

    docker exec -e ORC_TRAJ_DISCONT_DUMP_CSV=1 adoring_ishizaka \
        /workspaces/orc-a/build-ctest/bin/runTests \
        --gtest_filter='TrajDiscontinuityTest.*'

The CSV lands in the test working directory (tests/). Then run:

    python tests/plot_traj_discontinuity.py [path/to/csv]
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open() as f:
        rdr = csv.DictReader(f)
        cols: dict[str, list[float]] = {k: [] for k in rdr.fieldnames or []}
        for row in rdr:
            for k, v in row.items():
                cols[k].append(float(v))
    return {k: np.asarray(v) for k, v in cols.items()}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "csv",
        nargs="?",
        default="traj_discontinuity_setpoints.csv",
        help="Path to CSV (default: ./traj_discontinuity_setpoints.csv)",
    )
    ap.add_argument("-o", "--output", default=None, help="Save figure to PATH instead of showing")
    args = ap.parse_args()

    path = Path(args.csv)
    if not path.exists():
        print(f"error: {path} not found", file=sys.stderr)
        return 1

    d = load_csv(path)
    required = {"t", "q", "qd", "qdd", "q_ref", "qd_ref", "qdd_ref", "traj_type", "seg_id"}
    missing = required - d.keys()
    if missing:
        print(f"error: CSV missing columns {missing}", file=sys.stderr)
        return 1

    # Segment boundaries: where seg_id changes.
    seg = d["seg_id"]
    seg_changes = np.where(np.diff(seg) != 0)[0] + 1
    seg_times = d["t"][seg_changes]

    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)

    # Panel 1: position
    ax = axes[0]
    ax.plot(d["t"], d["q_ref"], color="tab:gray", lw=2, alpha=0.6, label="q_ref (analytic)")
    ax.plot(d["t"], d["q"], color="tab:blue", lw=1.2, label="q (spline setpoint)")
    ax.set_ylabel("q")
    ax.legend(loc="lower right")
    ax.grid(True, alpha=0.3)

    # Panel 2: velocity
    ax = axes[1]
    ax.plot(d["t"], d["qd_ref"], color="tab:gray", lw=2, alpha=0.6, label="qd_ref")
    ax.plot(d["t"], d["qd"], color="tab:blue", lw=1.2, label="qd")
    ax.set_ylabel("qd")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    # Panel 3: acceleration
    ax = axes[2]
    ax.plot(d["t"], d["qdd_ref"], color="tab:gray", lw=2, alpha=0.6, label="qdd_ref")
    ax.plot(d["t"], d["qdd"], color="tab:blue", lw=1.2, label="qdd")
    ax.set_ylabel("qdd")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    # Panel 4: errors on symlog scale
    ax = axes[3]
    ax.plot(d["t"], np.abs(d["q"] - d["q_ref"]), label="|q - q_ref|", lw=1.0)
    ax.plot(d["t"], np.abs(d["qd"] - d["qd_ref"]), label="|qd - qd_ref|", lw=1.0)
    ax.plot(d["t"], np.abs(d["qdd"] - d["qdd_ref"]), label="|qdd - qdd_ref|", lw=1.0)
    ax.set_yscale("symlog", linthresh=1e-6)
    ax.set_ylabel("abs error")
    ax.set_xlabel("t [s]")
    ax.legend(loc="upper right")
    ax.grid(True, which="both", alpha=0.3)

    # Mark segment hand-offs on every panel.
    for ax in axes:
        for st in seg_times:
            ax.axvline(st, color="tab:red", lw=0.6, ls="--", alpha=0.5)

    n_segs = int(seg.max()) + 1 if len(seg) else 0
    fig.suptitle(
        f"traj_discontinuity_test — setpoint vs. quintic-smoothstep reference "
        f"({n_segs} segments, red = hand-off)",
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))

    if args.output:
        fig.savefig(args.output, dpi=120)
        print(f"wrote {args.output}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
