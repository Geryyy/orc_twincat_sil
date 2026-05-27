"""
Generate models/iiwa_hanging_meshed.mjb and models/iiwa_standing_meshed.mjb.
"""

import argparse
import os

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--hanging", action="store_true", default=False, help="Generate hanging meshed Iiwa if set."
    )
    args = parser.parse_args()

    import mujoco
    import numpy as np
    import orcpy.core as oc
    from MujocoEnvCreator import CustomEnvironment

    oc.log.start_logging()

    appendix = "hanging" if args.hanging else "standing"
    OUT = os.path.join(REPO, "models", f"iiwa_{appendix}_meshed.mjb")

    env = CustomEnvironment(f"Iiwa {appendix}")

    iiwa = env.get_model_from_xml("environment_parts/iiwa_arc/iiwa_orc.xml")
    quat = np.zeros(4)

    if args.hanging:
        mujoco.mju_euler2Quat(quat, np.array([0.0, np.pi, np.pi / 2.0]), "xyz")
        env.add_model_to_arena(iiwa, "", [0, 0, 1.744], quat)
    else:
        env.add_model_to_arena(iiwa, "", [0, 0, 0])
    model, _ = env.compile_model()

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    mujoco.mj_saveModel(model, OUT)
    print(f"saved {OUT} ({os.path.getsize(OUT) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
