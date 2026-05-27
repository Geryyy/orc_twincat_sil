"""Headless (no viewer) MuJoCo simulator for Iiwa or Kinova.

Intended as a background companion to ``sending_leminscate.py``: it runs the
same physics + ORC control loop as ``simulate_{iiwa,kinova}.py`` but without
the interactive viewer, so it can run inside a container / CI.
"""

import argparse
import os
import sys
import time

import mujoco
import numpy as np
import orcpy.core as oc

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


def run_iiwa(mjb_path: str, duration: float) -> None:
    mj_model = mujoco.MjModel.from_binary_path(mjb_path)
    mj_data = mujoco.MjData(mj_model)
    Ts = mj_model.opt.timestep

    ctr_param = oc.robots.iiwa.IiwaContrParam(True)
    f_c_slow_norm = ctr_param.f_c_slow * (2 * Ts)
    js_param = oc.robots.robot7.JointCTParameter(ctr_param)
    ts_param = oc.robots.robot7.CartesianCTParameter(ctr_param)
    gc_param = oc.robots.robot7.GravityCompParameter(ctr_param.D_gc)

    iiwa = oc.robots.Iiwa(mjb_path, js_param, ts_param, gc_param, oc.Time(Ts), f_c_slow_norm)
    q0 = np.zeros(7)
    iiwa.start(oc.Time(mj_data.time), q0, q0, oc.Time(2.0))

    com_server = oc.robots.iiwa.TrajectoryServer(iiwa)

    t_end = time.time() + duration
    while time.time() < t_end:
        step_start = time.time()
        com_server.poll()
        iiwa.set_q_act_filtered_derivatives(mj_data.qpos)
        ret = iiwa.update(oc.Time(mj_data.time), False)
        if ret:
            mj_data.ctrl = iiwa.get_tau_act()
        mujoco.mj_step(mj_model, mj_data)
        com_server.send_robot_data(oc.Time(mj_data.time))
        dt_sleep = Ts - (time.time() - step_start)
        if dt_sleep > 0:
            time.sleep(dt_sleep)


def run_kinova(mjb_path: str, duration: float, q_home: np.ndarray | None = None) -> None:
    mj_model = mujoco.MjModel.from_binary_path(mjb_path)
    mj_data = mujoco.MjData(mj_model)
    Ts = mj_model.opt.timestep

    ctr_param = oc.robots.kinova.KinovaContrParam(True)
    js_param = oc.robots.robot7.JointPDPParameter(ctr_param.KP_PDP, ctr_param.KD_PDP)
    gc_param = oc.robots.robot7.GravityCompParameter(ctr_param.D_gc)

    kinova = oc.robots.Kinova(mjb_path, js_param, gc_param, oc.Time(Ts))
    if q_home is None:
        # Match Kinova::q_home (include/orc/robots/Kinova.h) so the sim starts
        # at the same configuration as the real robot's home pose.
        q_home = np.array([0.0, 0.27, 3.14, -2.27, 0.0, 0.96, 1.57])
    kinova.start(oc.Time(mj_data.time), q_home, q_home, oc.Time(2.0))

    com_server = oc.robots.kinova.TrajectoryServer(kinova)
    mj_data.qpos[:7] = q_home

    t_end = time.time() + duration
    while time.time() < t_end:
        step_start = time.time()
        com_server.poll()
        kinova.set_q_act(mj_data.qpos)
        kinova.set_q_dot_act(mj_data.qvel)
        kinova.set_tau_sens(np.zeros(7))
        kinova.set_tau_motor(np.zeros(7))
        ret = kinova.update(oc.Time(mj_data.time))
        if ret:
            mj_data.ctrl = kinova.get_tau_act()
        mujoco.mj_step(mj_model, mj_data)
        com_server.send_robot_data(oc.Time(mj_data.time))
        dt_sleep = Ts - (time.time() - step_start)
        if dt_sleep > 0:
            time.sleep(dt_sleep)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=["iiwa", "kinova"], required=True)
    parser.add_argument("--duration", type=float, default=30.0, help="Wall-clock run time [s].")
    parser.add_argument("--mjb", type=str, default=None)
    parser.add_argument(
        "--q-home", type=str, default=None, help="Comma-separated home configuration in radians."
    )
    args = parser.parse_args()

    oc.log.start_logging()

    q_home = np.fromstring(args.q_home, sep=",") if args.q_home else None
    if args.robot == "iiwa":
        mjb = args.mjb or os.path.join(REPO_ROOT, "models", "iiwa_hanging.mjb")
        run_iiwa(mjb, args.duration)
    else:
        mjb = args.mjb or os.path.join(REPO_ROOT, "models", "kinova3.mjb")
        run_kinova(mjb, args.duration, q_home=q_home)
    return 0


if __name__ == "__main__":
    sys.exit(main())
