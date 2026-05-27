#pragma once

#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"

namespace orc::control {
template <int DOF>
struct ControllerParameter {
    DECLARE_ROBOT_TRAITS_USINGS(DOF)

    // cartesian CT controller
    CartesianMatrix K0_cart = CartesianMatrix::Zero();
    CartesianMatrix K1_cart = CartesianMatrix::Zero();

    // cartesian nullspace controller
    JointMatrix K0_N_cart = JointMatrix::Zero();
    JointMatrix K1_N_cart = JointMatrix::Zero();

    // joint CT controller
    JointMatrix K0_joint = JointMatrix::Zero();
    JointMatrix K1_joint = JointMatrix::Zero();
    JointMatrix KI_joint = JointMatrix::Zero();

    // Joint PDP controller
    JointMatrix KP_PDP = JointMatrix::Zero();
    JointMatrix KD_PDP = JointMatrix::Zero();

    // Rotor inertia
    JointVector B = JointVector::Zero();  // Vector of rotor inertias

    // singular perturbation controller paramterts
    JointMatrix K_sp = JointMatrix::Zero();  // tau singular perturbation contoller K value
    JointMatrix D_sp = JointMatrix::Zero();  // tau singular perturbation contoller D value

    // friction compensation
    JointVector L_fc = JointVector::Zero();

    // gravity compensation
    JointMatrix D_gc = JointMatrix::Zero();

    // torque sensor
    JointVector torque_sensor_offset = JointVector::Zero();

    JointArray f_c_fast = JointArray::Zero();
    JointArray f_c_slow = JointArray::Zero();
};
}  // namespace orc::control
