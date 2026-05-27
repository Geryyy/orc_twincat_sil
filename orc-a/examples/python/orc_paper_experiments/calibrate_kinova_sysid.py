"""
Calibrate Kinova Gen3 dynamic parameters from on-robot data using the
joint-torque regressor from `mujoco-sysid`_, with a Khalil QR base-parameter
regrouping to drop the algebraically unidentifiable directions.

Excitation has two phases, sharing one CSV:

1. **Static poses** — move through ``--n-poses`` random configurations
   (uniform inside per-joint excursion bounds), holding each pose for
   ``--t-hold`` seconds. While stationary the regressor reduces to the
   gravity term, which strongly constrains link masses and first-moment
   parameters.
2. **Fourier excitation** — a single Park-form finite Fourier-series
   trajectory (random coefficients, ``Σ_k b_ik = 0`` so it starts and
   ends at home with zero velocity). Provides the velocity / acceleration
   coverage needed to identify inertia parameters.

The combined log feeds a column-normalised, ridge-regularised LS for the
base parameters; dependent and unexcited columns stay at the prior. The
calibrated model is written via ``mujoco_sysid.parameters.set_dynamic_parameters``.

.. _mujoco-sysid: https://github.com/lvjonok/mujoco-sysid
"""

import argparse
import csv
import os
import threading
import time

import mujoco
import numpy as np
import orcpy.core as oc
import orcpy.robots as orco
import scipy.linalg
from mujoco_sysid import convert as sysid_convert
from mujoco_sysid import parameters as sysid_parameters
from mujoco_sysid import regressors as sysid_regressors

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


KINOVA_CFG = {
    "model_path": os.path.join(REPO_ROOT, "models", "kinova3.mjb"),
    "starting_configuration": np.array([0.0, 0.27, 3.14, -2.27, 0.0, 0.96, 1.57]),
    "joint_limits_deg": (
        [-180.0, -128.0, -180.0, -147.8, -180.0, -120.3, -180.0],
        [180.0, 128.0, 180.0, 147.8, 180.0, 120.3, 180.0],
    ),
}


# Excitation ------------------------------------------------------------------


def fourier_excitation(coeffs, q_home, w_f, harmonics, t_traj, n_samples):
    """Park-form Fourier series. ``coeffs`` packs (a, b) of shape
    ``(dof, harmonics)`` flat. b is zero-meaned per joint so the
    trajectory starts and ends at home with zero velocity."""
    dof = q_home.size
    n = dof * harmonics
    a = coeffs[:n].reshape(dof, harmonics)
    b = coeffs[n:].reshape(dof, harmonics)
    b = b - b.mean(axis=1, keepdims=True)

    t = np.linspace(0.0, t_traj, n_samples)
    q = np.zeros((n_samples, dof))
    qd = np.zeros((n_samples, dof))
    qdd = np.zeros((n_samples, dof))
    for k in range(1, harmonics + 1):
        wk = w_f * k
        s = np.sin(wk * t)[:, None]
        c = np.cos(wk * t)[:, None]
        q += (a[:, k - 1] * (1.0 - c) + b[:, k - 1] * s) / wk
        qd += a[:, k - 1] * s + b[:, k - 1] * c
        qdd += a[:, k - 1] * wk * c - b[:, k - 1] * wk * s
    return t, q + q_home, qd, qdd


def sample_static_poses(n, q_home, joint_limits, is_infinite, max_excursion, rng):
    """Random target configurations for the static phase.

    Continuous joints get uniform ±max_excursion around home; bounded
    joints get uniform within (limit ∩ home ± max_excursion).
    """
    lo, hi = joint_limits
    is_inf = np.asarray(is_infinite, dtype=bool)
    poses = np.zeros((n, q_home.size))
    for j in range(q_home.size):
        if is_inf[j]:
            poses[:, j] = q_home[j] + rng.uniform(-max_excursion, max_excursion, n)
        else:
            j_lo = max(lo[j], q_home[j] - max_excursion)
            j_hi = min(hi[j], q_home[j] + max_excursion)
            poses[:, j] = rng.uniform(j_lo, j_hi, n)
    return poses


# Khalil base-parameter regrouping --------------------------------------------


def calibrated_body_ids(model):
    """Bodies whose inertial parameters affect joint torques.

    ``mujoco_sysid.regressors.joint_torque_regressor`` only includes the
    body directly attached to each joint. The Kinova model also has massive
    fixed children after the last arm joint (gripper/base geometry), whose
    inertia is reflected through the parent joint Jacobians. Include every
    massive body; zero-Jacobian bodies stay inactive and therefore at prior.
    """
    return [bid for bid in range(1, model.nbody) if float(model.body(bid).mass[0]) > 0.0]


def body_torque_regressor(model, data, body_ids):
    """Joint torque regressor for the selected bodies.

    The upstream helper omits gravity from ``mj_objectAcceleration`` for this
    fixed-base model, so static poses produce a zero regressor. Add gravity as
    base acceleration in body coordinates and sum contributions for all massive
    moving/fixed-child bodies.
    """
    Y = np.zeros((model.nv, 10 * len(body_ids)))
    velocity = np.zeros(6)
    accel = np.zeros(6)
    cross = np.zeros(3)

    for k, bid in enumerate(body_ids):
        velocity[:] = 0.0
        accel[:] = 0.0
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, bid, velocity, 1)
        mujoco.mj_objectAcceleration(model, data, mujoco.mjtObj.mjOBJ_BODY, bid, accel, 1)

        v, w = velocity[3:], velocity[:3]
        dv, dw = accel[3:].copy(), accel[:3].copy()
        mujoco.mju_cross(cross, w, v)
        if model.nq == model.nv:
            dv -= cross

        R = data.xmat[bid].reshape(3, 3)
        dv += -R.T @ model.opt.gravity

        Y[:, 10 * k : 10 * (k + 1)] = sysid_regressors.get_jacobian(
            model, data, bid
        ).T @ sysid_regressors.body_regressor(v, w, dv, dw)
    return Y


def compute_base_regrouping(model, n_probe=2000, tol=1e-7, rng=None):
    """QR with column pivoting on a generic regressor.

    Returns ``(base_idx, dep_idx, K, rank)`` such that
    ``W[:, dep] ≈ W[:, base] @ K`` for any ``W`` from
    ``joint_torque_regressor``.
    """
    rng = rng if rng is not None else np.random.default_rng(0)
    body_ids = calibrated_body_ids(model)
    nv, p = model.nv, 10 * len(body_ids)
    data = mujoco.MjData(model)

    W = np.zeros((n_probe * nv, p))
    for i in range(n_probe):
        data.qpos[:] = rng.uniform(-np.pi, np.pi, nv)
        data.qvel[:] = rng.normal(0.0, 2.0, nv)
        data.qacc[:] = rng.normal(0.0, 5.0, nv)
        mujoco.mj_inverse(model, data)
        W[i * nv : (i + 1) * nv, :] = body_torque_regressor(model, data, body_ids)

    col_scale = np.maximum(np.linalg.norm(W, axis=0), 1e-12)
    _, R, piv = scipy.linalg.qr(W / col_scale, pivoting=True, mode="economic")
    diag = np.abs(np.diag(R))
    rank = int(np.sum(diag > tol * diag[0]))
    base_idx = np.sort(piv[:rank])
    dep_idx = np.sort(piv[rank:])

    K, *_ = np.linalg.lstsq(W[:, base_idx], W[:, dep_idx], rcond=None)
    rec = float(
        np.linalg.norm(W[:, base_idx] @ K - W[:, dep_idx])
        / max(np.linalg.norm(W[:, dep_idx]), 1e-12)
    )
    oc.log.write_info(f"Base regrouping: rank={rank}/{p}, residual={rec:.2e}")
    return base_idx, dep_idx, K, rank


def load_or_compute_regrouping(model, model_path):
    cache = os.path.splitext(model_path)[0] + ".base_regrouping.npz"
    body_ids = np.array(calibrated_body_ids(model), dtype=int)
    if os.path.exists(cache) and os.path.getmtime(cache) >= os.path.getmtime(model_path):
        d = np.load(cache)
        if "body_ids" in d and np.array_equal(d["body_ids"], body_ids):
            oc.log.write_info(f"Loaded base regrouping from {cache} (rank={int(d['rank'])}).")
            return d["base_idx"], d["dep_idx"], d["K"], int(d["rank"])
        oc.log.write_info(f"Ignoring stale base regrouping cache {cache}.")
    base_idx, dep_idx, K, rank = compute_base_regrouping(model)
    np.savez(cache, base_idx=base_idx, dep_idx=dep_idx, K=K, rank=rank, body_ids=body_ids)
    return base_idx, dep_idx, K, rank


# CSV logging -----------------------------------------------------------------


class TauStateLogger:
    """RobotState logger that captures ``state.tau`` (commanded torque).

    Schema mirrors ``sending_leminscate.StateLogger`` so PlotJuggler /
    paper-figure post-processing can reuse the same parser.
    """

    HDR_FIELDS = (
        "e_q",
        "q_act",
        "q_d",
        "q_dot_act",
        "q_dot_d",
        "q_dotdot_d",
        "q_dotdot_act",
        "tau",
    )

    def __init__(self, robot, dof, path, period=0.001):
        self._r = robot
        self._dof = dof
        self._path = path
        self._period = period
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._n = 0

    def start(self):
        self._t.start()

    def stop(self):
        self._stop.set()
        self._t.join()

    def rows(self):
        return self._n

    def _safe(self, arr, n):
        a = np.zeros(n) if arr is None else np.asarray(arr).ravel()
        out = np.zeros(n)
        out[: min(a.size, n)] = a[:n]
        return out

    def _run(self):
        n = self._dof
        with open(self._path, "w", newline="") as f:
            w = csv.writer(f)
            cols = ["__time", "/dt"]
            for base in self.HDR_FIELDS:
                cols += [f"/{base}[{i}]" for i in range(n)]
            w.writerow(cols)

            t_prev = time.time()
            while not self._stop.is_set():
                st = self._r.state
                t = time.time()
                vals = [
                    self._safe(getattr(st, k, None), n)
                    for k in (
                        "q_act",
                        "q_set",
                        "q_dot_act",
                        "q_dot_set",
                        "q_dotdot_set",
                        "q_dotdot_act",
                        "tau",
                    )
                ]
                q_act, q_d = vals[0], vals[1]
                row = [f"{t:.6f}", f"{t - t_prev:.9f}"]
                for arr in (q_d - q_act, *vals):
                    row += [f"{v:.9f}" for v in arr]
                w.writerow(row)
                self._n += 1
                t_prev = t
                time.sleep(self._period)


def load_csv(path, dof):
    with open(path, newline="") as f:
        reader = csv.reader(f)
        h = next(reader)
        rows = [r for r in reader if r and len(r) == len(h)]
    arr = np.array(rows, dtype=float)

    def col(p):
        return arr[:, [h.index(f"/{p}[{i}]") for i in range(dof)]]

    return {
        "t": arr[:, 0],
        "q_act": col("q_act"),
        "q_dot_act": col("q_dot_act"),
        "q_dotdot_act": col("q_dotdot_act"),
        "tau": col("tau"),
    }


def savgol_diff(y, fs, window_s=0.05, polyorder=3):
    from scipy.signal import savgol_filter

    win = max(polyorder + 2, int(window_s * fs))
    if win % 2 == 0:
        win += 1
    return savgol_filter(y, win, polyorder, deriv=1, delta=1.0 / fs, axis=0)


# Identification --------------------------------------------------------------


def identify(csv_path, model_path, base_idx, dep_idx, K, *, ridge=5000.0, log_path=None):
    """Column-normalised ridge LS in base-parameter coordinates.

    Algebra: with ``W[:, dep] = W[:, base] @ K``, any θ has the same fit
    as ``W_base · θ_b`` for ``θ_b = θ[base] + K · θ[dep]``. We solve for
    θ_b and then map back: θ̂[base] = θ_b − K · θ_prior[dep].
    """
    model = mujoco.MjModel.from_binary_path(model_path)
    data = mujoco.MjData(model)
    log = load_csv(csv_path, model.nv)
    n = log["q_act"].shape[0]
    if n == 0:
        raise RuntimeError(f"No samples in {csv_path}")

    fs = 1.0 / np.median(np.diff(log["t"]))
    qdd = log["q_dotdot_act"]
    if not np.any(qdd):
        qdd = savgol_diff(log["q_dot_act"], fs=fs)

    body_ids = calibrated_body_ids(model)
    base_idx = np.asarray(base_idx)
    dep_idx = np.asarray(dep_idx)
    K = np.asarray(K)

    A = np.zeros((n * model.nv, base_idx.size))
    b = np.zeros(n * model.nv)
    for i in range(n):
        data.qpos[:] = log["q_act"][i]
        data.qvel[:] = log["q_dot_act"][i]
        data.qacc[:] = qdd[i]
        mujoco.mj_inverse(model, data)
        Y = body_torque_regressor(model, data, body_ids)
        A[i * model.nv : (i + 1) * model.nv] = Y[:, base_idx]
        # MuJoCo passive joint forces (e.g. damping) are not body inertial
        # parameters. Move them to the measured side so A theta represents
        # only the body dynamics.
        b[i * model.nv : (i + 1) * model.nv] = log["tau"][i] + data.qfrc_passive

    theta_full_prior = np.concatenate(
        [sysid_parameters.get_dynamic_parameters(model, bid) for bid in body_ids]
    )
    theta_b_prior = theta_full_prior[base_idx] + K @ theta_full_prior[dep_idx]

    # Column-normalise (mass / first moment / inertia have wildly different
    # unit scales). Hold tiny-norm columns at the prior — they correspond
    # to base parameters that this trajectory does not excite.
    cn = np.linalg.norm(A, axis=0)
    active = cn > 1e-8 * cn.max()
    n_inactive = int((~active).sum())
    if n_inactive:
        oc.log.write_info(f"{n_inactive}/{base_idx.size} base params not excited; held at prior.")
    cs = cn.copy()
    cs[~active] = 1.0
    A_norm = A / cs

    b_eff = b - A[:, ~active] @ theta_b_prior[~active]
    A_act = A_norm[:, active]
    p_act_prior = theta_b_prior[active] * cs[active]
    if ridge > 0.0:
        A_aug = np.vstack([A_act, ridge * np.eye(int(active.sum()))])
        b_aug = np.concatenate([b_eff, ridge * p_act_prior])
        x, *_ = np.linalg.lstsq(A_aug, b_aug, rcond=None)
    else:
        x, *_ = np.linalg.lstsq(A_act, b_eff, rcond=None)

    theta_b_hat = theta_b_prior.copy()
    theta_b_hat[active] = x / cs[active]

    theta_full_hat = theta_full_prior.copy()
    theta_full_hat[base_idx] = theta_b_hat - K @ theta_full_prior[dep_idx]

    cond_act = float(np.linalg.cond(A_act)) if A_act.size else float("nan")
    rms = float(np.sqrt(np.mean((A @ theta_b_hat - b) ** 2)))
    rms_prior = float(np.sqrt(np.mean((A @ theta_b_prior - b) ** 2)))
    msg = (
        f"Identified {int(active.sum())}/{base_idx.size} base params "
        f"from {n} samples; cond(A_active)={cond_act:.2e}, "
        f"rms(W θ̂_b − (τ + τ_passive))={rms:.4f} Nm, "
        f"rms(W θ_b,prior − (τ + τ_passive))={rms_prior:.4f} Nm"
    )
    oc.log.write_info(msg)

    pd_violations = []
    for k, bid in enumerate(body_ids):
        try:
            P = sysid_convert.theta2pseudo(theta_full_hat[10 * k : 10 * (k + 1)])
            eigs = np.linalg.eigvalsh(P)
            if eigs.min() <= 0:
                pd_violations.append((bid, model.body(bid).name, float(eigs.min())))
        except Exception:
            pass
    for bid, name, e in pd_violations:
        oc.log.write_warning(
            f"body {bid} ({name}): pseudo-inertia not PD (min eig {e:.3e}) — params unphysical"
        )

    if log_path:
        with open(log_path, "w") as f:
            f.write(msg + "\n")
            if pd_violations:
                f.write(f"WARNING: {len(pd_violations)} bodies non-PD\n")
            f.write("body_id,name,mass_prior,mass_hat,d_mass\n")
            for k, bid in enumerate(body_ids):
                m0, mh = theta_full_prior[10 * k], theta_full_hat[10 * k]
                f.write(f"{bid},{model.body(bid).name},{m0:.4f},{mh:.4f},{mh - m0:+.4f}\n")
    return theta_full_hat, model


def write_calibrated_model(model, theta_hat, output_mjb):
    for k, bid in enumerate(calibrated_body_ids(model)):
        sysid_parameters.set_dynamic_parameters(model, bid, theta_hat[10 * k : 10 * (k + 1)])
    mujoco.mj_saveModel(model, output_mjb)
    oc.log.write_info(f"Calibrated model written to {output_mjb}")


# Robot wiring ----------------------------------------------------------------


def build_robot(model_path, simulation):
    return (
        orco.Kinova(model_path)
        if simulation
        else orco.Kinova(model_path, "192.168.2.3", "192.168.2.10")
    )


def visit_static_poses(robot, poses, t_move, t_hold):
    for i, p in enumerate(poses):
        oc.log.write_info(f"static pose {i + 1}/{len(poses)}")
        robot.move_jointspace(p, robot.time, oc.Time(t_move), blocking_call=True)
        time.sleep(t_hold)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--no-sim", action="store_true")
    p.add_argument("--n-poses", type=int, default=8, help="Random static configurations to visit.")
    p.add_argument(
        "--t-move", type=float, default=2.5, help="Move duration between static poses [s]."
    )
    p.add_argument(
        "--t-hold", type=float, default=1.5, help="Hold duration at each static pose [s]."
    )
    p.add_argument(
        "--pose-amp",
        type=float,
        default=0.6,
        help="Per-joint excursion radius around home for random static poses [rad].",
    )
    p.add_argument(
        "--w-f",
        type=float,
        default=2.0 * np.pi * 0.1,
        help="Fourier fundamental frequency [rad/s].",
    )
    p.add_argument("--harmonics", type=int, default=5)
    p.add_argument("--n-samples", type=int, default=900, help="Dense Fourier waypoints.")
    p.add_argument(
        "--amp", type=float, default=0.3, help="Fourier per-joint amplitude scale [rad]."
    )
    p.add_argument(
        "--ridge",
        type=float,
        default=5000.0,
        help="Ridge regularisation toward the prior in "
        "column-normalised base coordinates. The default "
        "is intentionally strong because the base model is "
        "usually decent and calibration should make only "
        "small physical corrections.",
    )
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--csv", type=str, default=None)
    p.add_argument("--output-mjb", type=str, default=None)
    p.add_argument("--report", type=str, default=None)
    p.add_argument(
        "--skip-execution", action="store_true", help="Identify from an existing --csv only."
    )
    p.add_argument(
        "--static-only", action="store_true", help="Run/identify using only the static-pose phase; "
    )

    args = p.parse_args()

    cfg = KINOVA_CFG
    dof = 7
    t_traj = 2.0 * np.pi / args.w_f

    np.set_printoptions(precision=3, suppress=True)
    oc.log.start_logging(oc.log.LogLevel.Info)

    csv_path = args.csv or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "kinova_calibration_plotjuggler.csv",
    )
    output_mjb = args.output_mjb or os.path.join(REPO_ROOT, "models", "kinova3_calibrated.mjb")

    rng = np.random.default_rng(args.seed)
    model = mujoco.MjModel.from_binary_path(cfg["model_path"])
    base_idx, dep_idx, K, _ = load_or_compute_regrouping(model, cfg["model_path"])

    if not args.skip_execution:
        robot = build_robot(cfg["model_path"], not args.no_sim)
        time.sleep(0.5)

        q_home = cfg["starting_configuration"]
        if np.linalg.norm(robot.model.get_q_act() - q_home) > 0.02:
            robot.move_jointspace(
                q_home, robot.time, oc.Time(args.t_move + 2.0), blocking_call=True
            )

        lo = np.deg2rad(np.array(cfg["joint_limits_deg"][0]))
        hi = np.deg2rad(np.array(cfg["joint_limits_deg"][1]))
        is_inf = np.asarray(orco.Kinova.is_infinite, dtype=bool)
        poses = sample_static_poses(args.n_poses, q_home, (lo, hi), is_inf, args.pose_amp, rng)

        logger = TauStateLogger(robot, dof, csv_path)
        logger.start()
        try:
            visit_static_poses(robot, poses, args.t_move, args.t_hold)

            if not args.static_only:
                # Return home before Fourier excitation.
                robot.move_jointspace(q_home, robot.time, oc.Time(args.t_move), blocking_call=True)

                n = dof * args.harmonics
                coeffs = rng.normal(0.0, 1.0, 2 * n) * args.amp
                t_local, q_traj, qd_traj, qdd_traj = fourier_excitation(
                    coeffs,
                    q_home,
                    args.w_f,
                    args.harmonics,
                    t_traj,
                    args.n_samples,
                )

                t_start = robot.time.to_sec() + 1.0
                robot.send_dense_jointspace_trajectory_split(
                    oc.Time.convert_double_to_time_vector(t_local + t_start),
                    q_traj,
                    qd_traj,
                    qdd_traj,
                )
                oc.log.write_info(
                    f"Fourier sent (T={t_traj:.2f}s, K={args.harmonics}); logging to {csv_path}"
                )
                deadline = t_start + t_traj + 0.5
                while robot.time.to_sec() < deadline:
                    time.sleep(0.05)
            else:
                oc.log.write_info("Static-only mode enabled; skipping Fourier excitation.")
        finally:
            logger.stop()
            oc.log.write_info(f"Wrote {logger.rows()} rows to {csv_path}")

    theta_hat, model = identify(
        csv_path,
        cfg["model_path"],
        base_idx,
        dep_idx,
        K,
        ridge=args.ridge,
        log_path=args.report,
    )
    write_calibrated_model(model, theta_hat, output_mjb)


if __name__ == "__main__":
    main()
