
#pragma once

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>

#include <orc/robots/Robot.h>

#include "ControllerParameter_bindings.h"
#include "RobotData_bindings.h"
#include "RobotState_bindings.h"
#include "com/FlatBufferSerializer_bindings.h"
#include "interpolator/Interpolator_bindings.h"
#include "trajectory/HybridForceMotionTrajectory_bindings.h"

namespace nb = nanobind;
using namespace nb::literals;

template <int DOF>
void add_Robot(nb::module_& m, const char* module_name, const char* class_name) {
    using RobotX = orc::robots::Robot<DOF>;
    using JointVector = RobotX::JointVector;
    using PoseVector = orc::PoseVector;
    using Time = orc::Time;

    nb::module_ robotX_module = m.def_submodule(module_name);

    // RobotStatus
    add_RobotState<RobotX>(robotX_module);

    add_ControllerParameter<DOF>(robotX_module);

    // Add control parameters
    add_JointCTParameter<DOF>(robotX_module);
    add_JointPDPParameter<DOF>(robotX_module);
    add_CartesianCTParameter<DOF>(robotX_module);
    add_SingularPertrubationParameter<DOF>(robotX_module);
    add_FrictionCompParameter<DOF>(robotX_module);
    add_HybridForceMotionParameter<DOF>(robotX_module);
    add_GravityCompensationParameter<DOF>(robotX_module);
    add_CoulombFrictionCompParameter<DOF>(robotX_module);

    // Add interpolators
    add_SplineJointInterpolator<DOF>(robotX_module);

    // Add RobotData
    add_RobotData<DOF>(robotX_module);

    // Add trajectories
    add_HybridForceMotionTrajectory<DOF>(robotX_module);

    nb::class_<RobotX>(m, class_name)
        .def(nb::init<const char*, Time, std::string>(), "mjb_path"_a, "Ta"_a,
             "name_link_e"_a = ""_s)

        .def_rw("robot_data", &RobotX::robot_data)

        .def("register_JointCTController", &RobotX::register_JointCTController)
        .def("register_JointPDPController", &RobotX::register_JointPDPController)
        .def("register_VelocityController", &RobotX::register_VelocityController)
        .def("register_CartesianCTController", &RobotX::register_CartesianCTController)
        .def("register_GravityCompController", &RobotX::register_GravityCompController)
        .def("register_SingularPertrubationController",
             &RobotX::register_SingularPertrubationController)
        .def("register_FrictionCompController", &RobotX::register_FrictionCompController)
        .def("register_HybridForceMotionController", &RobotX::register_HybridForceMotionController)
        .def("register_CoulombFrictionCompController",
             &RobotX::register_CoulombFrictionCompController)

        .def_rw("serializer", &RobotX::serializer_)

        .def(
            "update", nb::overload_cast<Time, bool>(&RobotX::update), "t"_a,
            "grav_comp_only"_a =
                false)  // Default arguments must be repeated in the bindings
                        // (https://nanobind.readthedocs.io/en/latest/basics.html#keyword-and-default-args)

        .def("start", &RobotX::start, "t"_a, "q_act"_a, "q_set"_a, "T_traj"_a)

        .def("get_endeffector_site_id", &RobotX::get_endeffector_site_id)

        .def("get_q_act", &RobotX::get_q_act)
        .def("get_q_dot_act", &RobotX::get_q_dot_act)
        .def("get_q_dotdot_act", &RobotX::get_q_dotdot_act)

        .def("add_jointspace_trajectory",
             nb::overload_cast<JointVector, JointVector, Time, Time>(
                 &RobotX::add_jointspace_trajectory),
             "q0"_a, "q1"_a, "t0"_a, "t1"_a)
        .def("add_jointspace_trajectory",
             nb::overload_cast<std::vector<JointVector>&, std::vector<Time>&>(
                 &RobotX::add_jointspace_trajectory),
             "joint_poses"_a, "time_points"_a)
        .def("add_taskspace_trajectory",
             nb::overload_cast<PoseVector, PoseVector, Time, Time>(
                 &RobotX::add_taskspace_trajectory),
             "pose0"_a, "pose1"_a, "t0"_a, "t1"_a)
        .def("add_taskspace_trajectory",
             nb::overload_cast<std::vector<PoseVector>&, std::vector<Time>&>(
                 &RobotX::add_taskspace_trajectory),
             "pose_vec"_a, "time_points"_a)

        .def("add_hybrid_force_motion_trajectory", &RobotX::add_hybrid_force_motion_trajectory)

        .def("set_q_d_nullspace", &RobotX::set_q_d_nullspace, "q_d_NS"_a)

        .def("get_joint_error", &RobotX::get_joint_error)
        .def("get_joint_error_dot", &RobotX::get_joint_error_dot)
        .def("get_cartesian_error", &RobotX::get_cartesian_error)
        .def("get_cartesian_error_dot", &RobotX::get_cartesian_error_dot)

        .def("get_pose_set", &RobotX::get_pose_set)
        .def("get_x_dot_set", &RobotX::get_x_dot_set)
        .def("get_x_dotdot_set", &RobotX::get_x_dotdot_set)

        .def("get_q_NS_set", &RobotX::get_q_NS_set)

        .def("get_pose_act", &RobotX::get_pose_act)
        .def("get_x_dot_act", &RobotX::get_x_dot_act)
        .def("get_x_dotdot_act", &RobotX::get_x_dotdot_act)

        .def("get_q_set", &RobotX::get_q_set)
        .def("get_q_dot_set", &RobotX::get_q_dot_set)
        .def("get_q_dotdot_set", &RobotX::get_q_dotdot_set)

        .def("get_tau_act", &RobotX::get_tau_act)

        .def("is_taskspace_traj_active", &RobotX::is_taskspace_traj_active)
        .def("is_jointspace_traj_active", &RobotX::is_jointspace_traj_active)

        .def("get_model_id", &RobotX::get_model_id)

        .def("set_t", &RobotX::set_t)
        .def("set_q_act", &RobotX::set_q_act)
        .def("set_q_dot_act", &RobotX::set_q_dot_act)
        .def("set_q_dotdot_act", &RobotX::set_q_dotdot_act)
        .def("set_tau_motor", &RobotX::set_tau_motor)
        .def("set_tau_sens", &RobotX::set_tau_sens)
        .def("set_q_d_nullspace", &RobotX::set_q_d_nullspace)
        .def("get_force_filtered", &RobotX::get_force_filtered)

        .def("get_current_H_0_e", &RobotX::get_current_H_0_e)
        .def("get_current_jacobian", &RobotX::get_current_jacobian)
        .def("get_current_jacobian_dot", &RobotX::get_current_jacobian_dot)
        .def("get_current_inverse_jacobian", &RobotX::get_current_inverse_jacobian)
        .def("get_current_mass_matrix", &RobotX::get_current_mass_matrix);
}
