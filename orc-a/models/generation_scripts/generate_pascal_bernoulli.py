import mujoco
import orcpy.core as oc
from MujocoEnvCreator import CustomEnvironment

mjb_path = "models/presets/pascal_bernoulli.mjb"
iiwa_xml_path = "environment_parts/iiwa_arc/iiwa_orc.xml"

if __name__ == "__main__":
    env = CustomEnvironment("Pascal Bernoulli")
    oc.log.start_logging()

    # Load XML files
    xml_file_cage = "environment_parts/cage/cage.xml"
    model_cage = env.get_model_from_xml(xml_file_cage)
    env.add_model_to_arena(model_cage, "cage", [0, 0, 0], None, is_mocap=False)

    iiwa_left = env.get_model_from_xml(iiwa_xml_path)
    env.add_model_to_site(iiwa_left, "cage/robot_attach_left")

    iiwa_right = env.get_model_from_xml(iiwa_xml_path)
    env.add_model_to_site(iiwa_right, "cage/robot_attach_right")

    # Compile model
    model, data = env.compile_model()
    oc.log.write_info("Model is compiled!")
    mujoco.mj_saveModel(model, mjb_path)
    oc.log.write_info(f"Model saved to {mjb_path}!")
