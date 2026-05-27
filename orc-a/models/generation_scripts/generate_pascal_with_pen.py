import mujoco
import orcpy.core as oc
from MujocoEnvCreator import CustomEnvironment

oc.log.start_logging()
add_pascal = True  # if True add right robot, else Bernoulli is added
mjb_path = "models/presets/pascal_with_pen.mjb"
env = CustomEnvironment("Pascal with pen")

xml_file_cage = "environment_parts/cage/cage.xml"
model_cage = env.get_model_from_xml(xml_file_cage)
env.add_model_to_arena(model_cage, "cage", [0, 0, 0], None, is_mocap=False)

if add_pascal:
    attach_site = "cage/robot_attach_right"
else:
    attach_site = "cage/robot_attach_left"

xml_robot_right = "environment_parts/iiwa_arc/iiwa.xml"
iiwa_right = env.get_model_from_xml(xml_robot_right)
env.add_model_to_site(iiwa_right, attach_site)
pen_model = env.get_model_from_xml("environment_parts/pen/pen_force_sensor.xml")
env.add_model_to_site(pen_model, "cage/iiwa/gripper_attachment")

model, data = env.compile_model()
oc.log.write_info("Model is compiled!")
mujoco.mj_saveModel(model, mjb_path)
oc.log.write_info(f"Model saved to {mjb_path}!")
