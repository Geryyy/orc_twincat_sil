"""Generate models/presets/iiwa_hanging_adapter_pen.mjb.

Mirrors the in-script ``build_model_with_MMM(add_whiteboard=False)`` from
``examples/python/hybrid_force_motion/experiment_whiteboard/simulate.py``
so the preset can be regenerated standalone (e.g. by tests/CI).

Run from the repo root:

    python3 models/generation_scripts/generate_iiwa_hanging_adapter_pen.py
"""

import os
import sys

import mujoco
import numpy as np
import orcpy.core as oc
from MujocoEnvCreator import CustomEnvironment

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(REPO, "models", "presets", "iiwa_hanging_adapter_pen.mjb")


def main() -> int:
    oc.log.start_logging()
    env = CustomEnvironment("Iiwa hanging + adapter_mini40 + pen_force_sensor")

    iiwa = env.get_model_from_xml("environment_parts/iiwa_arc/iiwa.xml")
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, np.array([0.0, np.pi, np.pi / 2.0]), "xyz")
    env.add_model_to_arena(iiwa, "iiwa", [0, 0, 1.7], quat)

    adapter = env.get_model_from_xml("environment_parts/adapter_mini40/adapter_mini40.xml")
    env.add_model_to_site(adapter, "iiwa/gripper_attachment")

    pen = env.get_model_from_xml("environment_parts/pen/pen_force_sensor.xml")
    env.add_model_to_site(pen, "iiwa/adapter_mini40/adapter_attachment")

    model, _ = env.compile_model()

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    mujoco.mj_saveModel(model, OUT)
    print(f"saved {OUT} ({os.path.getsize(OUT) / 1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
