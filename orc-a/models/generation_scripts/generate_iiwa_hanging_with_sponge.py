"""Generate models/presets/iiwa_hanging_with_sponge.mjb.

Mirrors ``build_sponge_model`` in
``examples/python/hybrid_force_motion/sponge/simulate_iiwa.py``.

Run from the repo root:

    python3 models/generation_scripts/generate_iiwa_hanging_with_sponge.py
"""

import os
import sys

import mujoco
import numpy as np
import orcpy.core as oc
from MujocoEnvCreator import CustomEnvironment

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(REPO, "models", "presets", "iiwa_hanging_with_sponge.mjb")
START_POINT = np.array([-0.51903, 0.0, 0.8])


def main() -> int:
    x_regal = -0.7
    sponge_geom_prefix = "iiwa/sponge/sponge_collision"

    oc.log.start_logging()
    env = CustomEnvironment("Iiwa with sponge")

    iiwa = env.get_model_from_xml("environment_parts/iiwa_arc/iiwa.xml")
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, np.array([0.0, np.pi, np.pi / 2.0]), "xyz")
    env.add_model_to_arena(iiwa, "iiwa", [0, 0, 1.7], quat)

    sponge = env.get_model_from_xml("environment_parts/sponge/sponge_sensor.xml")
    env.add_model_to_site(sponge, "iiwa/gripper_attachment")
    sponge_collision = env.get_model_from_xml("environment_parts/sponge/sponge_collision.xml")
    env.add_model_to_site(sponge_collision, "iiwa/sponge/sponge_plate")

    box = env.get_model_from_xml("environment_parts/Regal_braun/regal_braun.xml")
    quat_box = np.zeros(4)
    mujoco.mju_euler2Quat(quat_box, np.array([0.0, 0.0, np.pi / 2.0]), "xyz")
    env.add_model_to_arena(box, "Regal_braun", [x_regal, 0, 0], quat_box)

    quat_pt = np.zeros(4)
    mujoco.mju_euler2Quat(quat_pt, np.array([0.0, -np.pi / 2.0, 0.0]), "xyz")
    env.arena.worldbody.add(
        "site",
        name="handover_point",
        pos=START_POINT,
        size=[0.03],
        type="sphere",
        quat=quat_pt,
        rgba=[1, 0, 0, 1],
    )

    model, _ = env.compile_model()

    # Match runtime softening from simulate_iiwa.py: zero-friction, raised
    # priority on every sponge_collision geom so the contact is compliant.
    geom_names = [
        model.geom(i).name
        for i in range(model.ngeom)
        if model.geom(i).name.startswith(sponge_geom_prefix)
    ]
    for name in geom_names:
        model.geom(name).priority = 1
        model.geom(name).friction = np.zeros(3)

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    mujoco.mj_saveModel(model, OUT)
    print(f"saved {OUT} ({os.path.getsize(OUT) / 1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
