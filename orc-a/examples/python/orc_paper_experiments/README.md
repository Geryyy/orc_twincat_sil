# Paper experiments — Python examples

End-to-end Cartesian-trajectory examples driven from Python via `orcpy`.
Each `sending_*.py` script computes a Cartesian trajectory, solves IK
to joint space, and streams the resulting jointspace trajectory over
UDP to either a running ORC simulator or a real robot.

## Files

- **`sending_leminscate.py`** — figure-eight (lemniscate) trajectory.
  Supports `--robot {iiwa,kinova}` plus overrides for the starting
  configuration, lemniscate axes, duration, etc. Logs the received
  `RobotState` stream to a PlotJuggler-format CSV for later analysis.
- **`sending_square_octogon.py`** — closed-loop square / octagon path
  (jointspace move + Cartesian sweep over a polygon).
- **`sending_simple.py`** — minimal hybrid force/motion example
  (approach a surface, draw a short line under constant normal force).
- **`_headless_sim.py`** — viewer-less companion simulator that runs
  the C++ Iiwa or Kinova robot + ORC controller in a MuJoCo physics
  loop. Intended as a background process for the senders so the
  examples run end-to-end inside a container / CI without a display.

## Run an example

Senders need a running endpoint on the configured UDP address — either
`_headless_sim.py` (loopback) or a real robot. To exercise the
lemniscate against the loopback simulator:

```bash
# 1. start the simulator in the background
python3 examples/python/orc_paper_experiments/_headless_sim.py \
    --robot kinova --duration 25 &
sleep 2

# 2. run the sender
python3 examples/python/orc_paper_experiments/sending_leminscate.py \
    --robot kinova --t-traj 10 --n-points 50
```

Other senders work the same way — start `_headless_sim.py` first, then
run the script. To target real hardware instead of the loopback
simulator, pass `--no-sim` (and skip the `_headless_sim.py` step).

## Kinova system-identification math

`calibrate_kinova_sysid.py` identifies the rigid-body inertial
parameters of the Kinova model from logged joint positions, velocities,
accelerations, and commanded torques. The model is linear in each body's
10 inertial parameters, following the standard robot dynamic
identification form used by Khalil and Dombre [1]:

```text
theta_i = [m, h_x, h_y, h_z, I_xx, I_xy, I_yy, I_xz, I_yz, I_zz]^T
```

where `h = m c` is the first moment about the body frame origin and `I`
is the inertia about the same origin. For a set of calibrated bodies,
the stacked parameter vector is

```text
theta = [theta_1^T, theta_2^T, ..., theta_N^T]^T .
```

For each logged sample, the script builds a joint-torque regressor
`Y(q, qdot, qddot)` such that

```text
Y(q, qdot, qddot) theta = tau_body .
```

The per-body contribution is the standard spatial Newton-Euler
regressor. In body coordinates,

```text
f_i = Y_i(v_i, omega_i, a_i, alpha_i) theta_i
```

and the generalized torque contribution is projected through the body
Jacobian:

```text
tau_i = J_i(q)^T f_i .
```

The full regressor concatenates all body columns:

```text
Y = [J_1^T Y_1, J_2^T Y_2, ..., J_N^T Y_N] .
```

For the Kinova `.mjb`, `N` is not just the seven joint bodies. The wrist
and gripper include massive fixed children, and their inertias are still
reflected through the parent joint Jacobians. The script therefore uses
all massive bodies in the model as candidate calibrated bodies. Bodies
whose Jacobians are zero or whose columns are algebraically dependent are
handled by the base-parameter reduction.

One MuJoCo detail matters for the sanity check: for this fixed-base
model, `mj_objectAcceleration` does not provide the gravity acceleration
needed by the inertial regressor in static poses. The script adds it in
body coordinates:

```text
a_i <- a_i - R_i^T g .
```

With `qdot = 0` and `qddot = 0`, this makes the regressor reproduce the
model's gravity torque. In simulation, a static log row should therefore
satisfy

```text
Y(q, 0, 0) theta_model = tau_logged
```

up to numerical precision.

MuJoCo passive joint forces, such as the damping terms in
`model.dof_damping`, are not part of `theta`. MuJoCo's dynamics can be
written as

```text
tau_applied + tau_passive = tau_body .
```

So the least-squares target in the script is

```text
Y theta ~= tau_logged + qfrc_passive .
```

This sign convention is why the residual log prints
`W theta - (tau + tau_passive)`: the regressor is fitting rigid-body
inertial dynamics only, while passive joint effects are moved to the
measured side.

The base-parameter regrouping follows the usual base-inertial-parameter
identification idea: the full inertial vector contains linearly
dependent directions, so only a smaller base parameter vector is
identifiable from joint torques [1, 2]. In this script, a numerical probe
matrix `W` is built from random states, its columns are scaled, and QR
with column pivoting separates independent and dependent columns:

```text
W_dep ~= W_base K .
```

For any full parameter vector,

```text
theta_base = theta[base] + K theta[dep] .
```

The script solves in these base coordinates with column normalization and
ridge regularization toward the model prior:

```text
min_x ||W_base x - b||_2^2 + lambda^2 ||S (x - theta_base,prior)||_2^2 .
```

The default `--ridge` is intentionally strong. The Kinova `.mjb` is
already a reasonable base model, and many inertial directions are weakly
observable or coupled by the base-parameter regrouping, so calibration is
treated as a small correction around the prior rather than an unconstrained
rewrite of individual body inertias. Lower `--ridge` values can reduce the
torque residual slightly, but they may also produce non-physical
pseudo-inertias.

After solving, dependent parameters are kept at the prior and the
independent entries are mapped back:

```text
theta_hat[base] = theta_base,hat - K theta_prior[dep] .
```

When the logged data comes from the same MuJoCo model, the residual is
expected to be essentially zero if `q`, `qdot`, `qddot`, torque, and
passive-force conventions are all exact. Older calibration CSVs in this
folder have `q_dotdot_act` logged as all zeros, so dynamic rows require
Savitzky-Golay differentiation of `q_dot_act`; that reconstructed
acceleration leaves a small nonzero dynamic residual. Static rows remain
a useful exact sanity check because they do not depend on acceleration
reconstruction.

The Fourier excitation phase is the same broad family as Park's
Fourier-based excitation trajectories for robot dynamic identification
[3]. This script uses random coefficients rather than optimizing them;
Gautier and Khalil's trajectory-design paper is the relevant next step
for choosing excitation trajectories that improve the conditioning of
the identification matrix [2].

### References

[1] W. Khalil and E. Dombre, *Modeling, Identification and Control of
Robots*, HPS, 2002. ISBN 9781903996133.

[2] M. Gautier and W. Khalil, "Exciting Trajectories for the
Identification of Base Inertial Parameters of Robots," *The International
Journal of Robotics Research*, 11(4), pp. 362-375, 1992.
doi: `10.1177/027836499201100408`.

[3] K.-J. Park, "Fourier-based optimal excitation trajectories for the
dynamic identification of robots," *Robotica*, 24(5), pp. 625-633, 2006.
doi: `10.1017/S0263574706002712`.

## In a container

If `orcpy` is not installed locally, the project's devcontainer image
already has all C++ dependencies. From the repo root:

```bash
IMG=$(docker images --format '{{.Repository}}' | grep '^vsc-orc-a-' | head -1)
docker run --rm --net=host -v "$PWD:/workspaces/orc-a" -w /workspaces/orc-a "$IMG" bash -c '
  pip install -e . mujoco==3.3.2 --quiet
  python3 examples/python/orc_paper_experiments/_headless_sim.py --robot kinova --duration 25 &
  sleep 2
  python3 examples/python/orc_paper_experiments/sending_leminscate.py --robot kinova --t-traj 10
'
```
