"""
Example showcasing how to send JointCtrContrParamTrajectory and CartesianCtrParamTrajectory to change controller parameters online.
Call this script with an additional console argument 'j' or 't'.
- 'j' sends a JointCtrContrParamTrajectory
- 't' sends a CartesianCtrParamTrajectory
"""

import sys

import orcpy.core as oc
import orcpy.robots as orco

simulation = True
mjb_path = orco.util_functions.default_model_path("iiwa_hanging.mjb")

oc.log.start_logging()
if simulation:
    iiwa = orco.Iiwa(mjb_path)
else:
    iiwa = orco.Iiwa(mjb_path, "192.168.2.3", "192.168.2.10")

if len(sys.argv) != 2:
    oc.log.write_error(
        "Pass additional console argument 'j' or 't' to send joint CT or cartesian CT controller parameters"
    )
    exit()

ctr_param = oc.robots.iiwa.IiwaContrParam(simulation)
if sys.argv[1] == "j":
    # Send JointCtrParameterTrajectory
    K0_new = ctr_param.K0_joint / 2
    K1_new = ctr_param.K1_joint / 2
    KI_new = ctr_param.KI_joint / 2
    iiwa.send_joint_ctr_parameter_trajectory(iiwa.time, K0_new, K1_new, KI_new)
elif sys.argv[1] == "t":
    # Send CartesianCtrParameterTrajectory
    K0_new = ctr_param.K0_cart / 2
    K1_new = ctr_param.K1_cart / 2
    K0N_new = ctr_param.K0_N_cart / 2
    K1N_new = ctr_param.K1_N_cart / 2
    iiwa.send_cartesian_ctr_parameter_trajectory(iiwa.time, K0_new, K1_new, K0N_new, K1N_new)
else:
    oc.log.write_error("Only console argument 'j' or 't' is supported!")
