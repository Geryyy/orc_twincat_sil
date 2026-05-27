"""Build a tex-friendly CSV combining sim & real kinova lemniscate runs.

Inputs:
  kinova_leminscate_plotjuggler.csv   (real robot)
  kinova_leminscate_sim.csv           (headless sim)

Output:
  data/kinova_lemniscate.csv

Columns: t, x_sim, y_sim, z_sim, x_real, y_real, z_real,
         dq0..dq6,                   (= q_sim - q_real, per joint)
         eq_sim0..eq_sim6,           (= q_d - q_act in sim, per joint)
         eq_real0..eq_real6          (= q_d - q_act on real, per joint)

Time: each run is re-zeroed at the first sample in its movement window
(when |q_act - q_start| exceeds a small threshold) and re-sampled to a
common 1kHz grid covering the overlap.
"""

from __future__ import annotations

import argparse
import os
import sys

import mujoco
import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))


def load_run(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    df = df.rename(columns=lambda c: c.lstrip("/"))
    df["__time"] = df["__time"].astype(float)
    df = df.sort_values("__time").reset_index(drop=True)
    return df


def zero_time(df: pd.DataFrame, thresh: float = 1e-3) -> pd.DataFrame:
    """Re-zero a run at the moment the *commanded* trajectory leaves home.

    q_d is the controller setpoint and is identical across runs that share the
    same sender, so this gives a deterministic alignment point regardless of
    when the recording was started.
    """
    try:
        qd = df[[f"q_d[{i}]" for i in range(7)]].to_numpy()
    except KeyError:
        print("q_d[i] is missing")
        print(f"Available columns: {df.columns}")
        sys.exit(1)
    delta = np.linalg.norm(qd - qd[0], axis=1)
    idx = int(np.argmax(delta > thresh))
    if delta[idx] <= thresh:
        idx = 0
    t0 = df["__time"].iloc[idx]
    out = df.iloc[idx:].copy()
    out["t"] = out["__time"].to_numpy() - t0
    return out.reset_index(drop=True)


def forward_kinematics(q_traj: np.ndarray, model, data, site_name: str) -> np.ndarray:
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    if sid < 0:
        raise RuntimeError(f"site {site_name!r} not in model")
    out = np.zeros((q_traj.shape[0], 3))
    for i in range(q_traj.shape[0]):
        data.qpos[:7] = q_traj[i]
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        out[i] = data.site_xpos[sid]
    return out


def resample(df: pd.DataFrame, t_grid: np.ndarray, cols: list[str]) -> np.ndarray:
    src_t = df["t"].to_numpy()
    out = np.zeros((len(t_grid), len(cols)))
    for j, c in enumerate(cols):
        out[:, j] = np.interp(t_grid, src_t, df[c].to_numpy())
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--real", default=os.path.join(HERE, "kinova_leminscate_plotjuggler.csv"))
    ap.add_argument("--sim", default=os.path.join(HERE, "kinova_leminscate_sim.csv"))
    ap.add_argument("--model", default=os.path.join(REPO, "models", "kinova3.mjb"))
    ap.add_argument("--site", default="kinova3/gripping_point")
    ap.add_argument("--out", default=None)
    ap.add_argument("--rate", type=float, default=200.0, help="output sample rate [Hz]")
    args = ap.parse_args()

    out_path = args.out or os.path.join(HERE, "data", "kinova_lemniscate.csv")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    real = load_run(args.real)
    sim = load_run(args.sim)

    real0 = zero_time(real)
    sim0 = zero_time(sim)

    T = min(real0["t"].iloc[-1], sim0["t"].iloc[-1])
    n = int(T * args.rate) + 1
    t_grid = np.linspace(0.0, T, n)

    q_cols = [f"q_act[{i}]" for i in range(7)]
    qd_cols = [f"q_d[{i}]" for i in range(7)]
    q_real = resample(real0, t_grid, q_cols)
    q_sim = resample(sim0, t_grid, q_cols)
    qd_real = resample(real0, t_grid, qd_cols)
    qd_sim = resample(sim0, t_grid, qd_cols)

    model = mujoco.MjModel.from_binary_path(args.model)
    data = mujoco.MjData(model)

    x_real = forward_kinematics(q_real, model, data, args.site)
    x_sim = forward_kinematics(q_sim, model, data, args.site)

    dq = q_sim - q_real
    eq_sim = qd_sim - q_sim
    eq_real = qd_real - q_real

    out = pd.DataFrame({"t": t_grid})
    out["x_sim"], out["y_sim"], out["z_sim"] = x_sim[:, 0], x_sim[:, 1], x_sim[:, 2]
    out["x_real"], out["y_real"], out["z_real"] = x_real[:, 0], x_real[:, 1], x_real[:, 2]
    for j in range(7):
        out[f"dq{j}"] = dq[:, j]
    for j in range(7):
        out[f"eq_sim{j}"] = eq_sim[:, j]
    for j in range(7):
        out[f"eq_real{j}"] = eq_real[:, j]

    out.to_csv(out_path, index=False, float_format="%.6f")
    print(f"wrote {len(out)} rows → {out_path}")
    print(f"duration: {T:.3f} s, rate: {args.rate:g} Hz")
    return 0


if __name__ == "__main__":
    sys.exit(main())
