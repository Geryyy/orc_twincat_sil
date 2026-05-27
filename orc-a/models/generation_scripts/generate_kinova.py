import argparse

import mujoco
from MujocoEnvCreator import CustomEnvironment
from orcpy.robots import util_functions

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--add_mocap_box", action="store_false", default="Adds box to scene for collision detection"
)
args = parser.parse_args()


def save_mjb_from_mujocomeshmanager(add_mocap_box: bool):
    xml_kinova = "environment_parts/kinova3/kinova3_fixed_gripper_optimized.xml"
    # xml_gripper = "environment_parts/UR_gripper/ur_gripper_fixed.xml"
    xml_box = "environment_parts/box_red_medium/box_red_medium_no_sites.xml"

    if add_mocap_box:
        mjb_path = util_functions.default_model_path("kinova3_box.mjb")
    else:
        mjb_path = util_functions.default_model_path("kinova3.mjb")

    # Create MJB from MujocoMeshManager, needed for ORC
    env = CustomEnvironment(model_name="Kinova Gen3")
    model_kinova = env.get_model_from_xml(xml_kinova)
    env.add_model_to_arena(model_kinova, "kinova3", [0, 0, 0])
    # model_gripper = env.get_model_from_xml(xml_gripper)
    # env.add_model_to_site(model_gripper, "kinova3/gripper_mount")

    if add_mocap_box:
        model_box = env.get_model_from_xml(xml_box)
        env.add_model_to_arena(model_box, "mocap_box", [0.5, 0.2, 0], is_mocap=True)

    model, _ = env.compile_model()
    # env.display_environment()
    mujoco.mj_saveModel(model, mjb_path)
    print(f"Saved {mjb_path}")


if __name__ == "__main__":
    save_mjb_from_mujocomeshmanager(not args.add_mocap_box)
