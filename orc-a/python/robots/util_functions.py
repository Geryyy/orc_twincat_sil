import os
from pathlib import Path

import mujoco
import numpy as np


def default_model_path(name: str) -> str:
    """Resolve a bundled model filename to an absolute path.

    Search order:
      1. ``$ORCPY_MODELS_DIR`` environment variable (explicit override).
      2. ``<repo_root>/models/<name>`` when running from a checkout.
      3. ``<cwd>/models/<name>`` (legacy behaviour, run-from-repo-root).
      4. Return ``name`` unchanged (caller may rely on their own lookup).

    Args:
        name: Basename (``"iiwa_hanging.mjb"``) or relative path below
            ``models/`` (``"presets/iiwa_hanging_adapter_pen.mjb"``).

    Returns:
        Absolute filesystem path when found, otherwise the original input.
    """
    if os.path.isabs(name) and os.path.exists(name):
        return name

    env = os.environ.get("ORCPY_MODELS_DIR")
    candidates = []
    if env:
        candidates.append(Path(env) / name)

    # Repo layout: python/robots/util_functions.py -> ../../models/<name>
    here = Path(__file__).resolve()
    for parent in (here.parents[2], here.parents[1]):
        candidates.append(parent / "models" / name)

    candidates.append(Path.cwd() / "models" / name)
    candidates.append(Path.cwd() / name)

    # Walk up from CWD looking for a sibling models/ dir. Handles the common
    # case of running an example from a subdirectory of a repo checkout while
    # orcpy itself is installed under site-packages.
    for parent in Path.cwd().resolve().parents:
        candidates.append(parent / "models" / name)

    for p in candidates:
        if p.exists():
            return str(p)
    return name


def inverse_kinematics_residual(
    x, model, data, end_effector_site, target_site, reg_target, radius=0.04, reg=1e-3
):
    """Residual for inverse kinematics. Taken from MuJoCo's minimize example from readme
    https://mujoco.readthedocs.io/en/stable/python.html#minimize

    Args:
    x: joint angles.
    model: mjModel.
    data: mjData.
    end_effector_site: end effector site.
    target_site: target site.
    reg_target: Regularization target for joint angles.
    radius: Scaling of the 3D cross. Defaults to 0.04.
    reg: Regularization weight. Defaults to 1e-3.

    Returns:
    The residual of the Inverse Kinematics task.
    """

    # Set qpos, compute forward kinematics.
    res = []
    for i in range(x.shape[1]):
        data.qpos = x[:, i]
        mujoco.mj_kinematics(model, data)

        # Position residual.
        res_pos = end_effector_site.xpos - target_site.xpos

        # iiwa/pen/pen_tip quat, use mju_mat2quat.
        effector_quat = np.empty(4)
        mujoco.mju_mat2Quat(effector_quat, end_effector_site.xmat)

        # Target quat, exploit the fact that the site is aligned with the body.
        target_quat = np.zeros(4)
        mujoco.mju_mat2Quat(target_quat, target_site.xmat)

        # Orientation residual: quaternion difference.
        res_quat = np.empty(3)
        mujoco.mju_subQuat(res_quat, target_quat, effector_quat)
        res_quat *= radius

        # Regularization residual.
        res_reg = reg * (x[:, i] - reg_target)

        res_i = np.hstack((res_pos, res_quat, res_reg))
        res.append(np.atleast_2d(res_i).T)

    return np.hstack(res)


def inverse_kinematics_residual_posrot(
    x: np.ndarray,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    end_effector_site,
    pos_target: np.ndarray,
    rot_target: np.ndarray,
    reg_target: np.ndarray,
    radius: float = 0.04,
    reg: float = 1e-3,
):
    """
    Residual for inverse kinematics using position + rotation matrix targets.

    Args:
        x: np.ndarray of shape (n_joints,) or (n_joints, n_steps) joint angles.
        model: MuJoCo model.
        data: MuJoCo data.
        end_effector_site: End-effector site in the model.
        pos_target: np.ndarray (3,) desired end-effector position in world frame.
        rot_target: np.ndarray (3,3) desired end-effector rotation matrix.
        reg_target: np.ndarray (n_joints,) regularization target for joints.
        radius: float, scaling factor for orientation residual.
        reg: float, regularization weight.

    Returns:
        np.ndarray of residuals.
    """
    # Set qpos, compute forward kinematics.
    res = []
    for i in range(x.shape[1]):
        data.qpos = x[:, i]
        mujoco.mj_kinematics(model, data)

        # Position residual.
        res_pos = end_effector_site.xpos - pos_target

        # iiwa/pen/pen_tip quat, use mju_mat2quat.
        effector_quat = np.empty(4)
        mujoco.mju_mat2Quat(effector_quat, end_effector_site.xmat)

        # Target quat, exploit the fact that the site is aligned with the body.
        target_quat = np.zeros(4)
        mujoco.mju_mat2Quat(target_quat, rot_target)

        # Orientation residual: quaternion difference.
        res_quat = np.empty(3)
        mujoco.mju_subQuat(res_quat, target_quat, effector_quat)
        res_quat *= radius

        # Regularization residual.
        res_reg = reg * (x[:, i] - reg_target)

        res_i = np.hstack((res_pos, res_quat, res_reg))
        res.append(np.atleast_2d(res_i).T)

    return np.hstack(res)


def wrap_to_180(angles):
    """
    Conversion of angles from [0, 360] to [-180, 180]

    :param angles: Angle array in radians.
    """
    return (angles + 180) % 360 - 180


def wrap_to_pi(angles):
    """
    Conversion of angles from [0, 2*pi] to [-pi, pi]

    :param angles: Angle array in radians.
    """
    return (angles + np.pi) % (2 * np.pi) - np.pi


def wrap_to_2pi(angles):
    """
    Conversion of angles from [-pi, pi] to [0, 2*pi]

    :param angles: Angle array in radians.
    """
    return angles % (2 * np.pi)
