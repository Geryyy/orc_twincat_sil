#!/usr/bin/env python3
"""Plot the CSVs produced by the two (previously skipped) spline tests in
``interpolator_comprehensive_test.cpp``:

- ``spline_nonuniform_60.csv``   — columns ``t, q, is_knot``
- ``spline_velratio_100hz.csv``  — columns ``t, q, q_expected, is_knot``

Regenerate inside the devcontainer with::

    docker exec -e ORC_INTERP_DUMP_CSV=1 adoring_ishizaka \\
        /workspaces/orc-a/build/bin/runTests \\
        --gtest_filter='SplineHighResolution.NonUniformSpacing60Points:SplineVelocityMismatch.ExtremeVelocityRatio_100Hz'

Then::

    python tests/plot_interp.py build/spline_nonuniform_60.csv
    python tests/plot_interp.py build/spline_velratio_100hz.csv
    python tests/plot_interp.py build/spline_*.csv     # both in one figure

Knots are overlaid as markers on the sampled trajectory. If
``q_expected`` is present, it is plotted as the analytic reference.
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


def plot_one(ax, data: dict[str, np.ndarray], title: str) -> None:
    if "is_knot" not in data or "t" not in data or "q" not in data:
        raise SystemExit(f"{title}: CSV missing required columns (t, q, is_knot)")

    is_knot = data["is_knot"].astype(bool)
    samples = ~is_knot

    t_s, q_s = data["t"][samples], data["q"][samples]
    t_k, q_k = data["t"][is_knot], data["q"][is_knot]

    # Sort samples by time (the dump writes knots first, then samples -- no
    # guarantee of interleaved order if either block is out of order).
    order = np.argsort(t_s)
    t_s, q_s = t_s[order], q_s[order]

    if "q_expected" in data:
        q_exp = data["q_expected"][samples][order]
        ax.plot(t_s, q_exp, color="tab:gray", lw=2, alpha=0.6, label="q_expected (analytic)")

    ax.plot(t_s, q_s, color="tab:blue", lw=1.3, label="q (spline)")
    ax.scatter(t_k, q_k, color="tab:red", s=18, zorder=5, label=f"knots (n={len(t_k)})")

    # Annotate sample-to-sample non-monotone events (|Δq| going backward by > 1e-4)
    # -- they're the thing the test's ``violations`` counter picks up.
    dq = np.diff(q_s)
    back = np.where(dq < -1e-4)[0]
    if back.size:
        ax.scatter(
            t_s[back + 1],
            q_s[back + 1],
            color="tab:orange",
            marker="x",
            s=32,
            zorder=6,
            label=f"mono violations (n={len(back)})",
        )

    ax.set_title(title)
    ax.set_ylabel("q")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "csv",
        nargs="+",
        help="Path(s) to CSV. Multiple = one subplot per file.",
    )
    ap.add_argument("-o", "--output", default=None, help="Save figure to PATH instead of showing")
    args = ap.parse_args()

    paths = [Path(p) for p in args.csv]
    missing = [p for p in paths if not p.exists()]
    if missing:
        for p in missing:
            print(f"error: {p} not found", file=sys.stderr)
        return 1

    n = len(paths)
    fig, axes = plt.subplots(n, 1, figsize=(11, 4 * n), sharex=False)
    if n == 1:
        axes = [axes]

    for ax, path in zip(axes, paths, strict=False):
        data = load_csv(path)
        plot_one(ax, data, path.name)

    axes[-1].set_xlabel("t [s]")
    fig.tight_layout()

    if args.output:
        fig.savefig(args.output, dpi=120)
        print(f"wrote {args.output}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
