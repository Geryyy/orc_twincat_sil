#pragma once

/**
 * @file FlatBufferSerializer_bindings.h
 * @brief Python bindings for FlatBuffer serialization
 */

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>

#include <orc/OrcTypes.h>
#include <orc/RobotTraits.h>

#include <orc/com/flatbuffers/FlatBufferDeserializer.h>
#include <orc/com/flatbuffers/FlatBufferRobotState.h>
#include <orc/com/flatbuffers/FlatBufferSerializer.h>

namespace nb = nanobind;
using namespace nb::literals;

template <int DOF>
inline void add_FlatBufferSerializer(nb::module_& m) {
    using FlatBufferSerializer = orc::com::fb::FlatBufferSerializer<DOF>;
    using FlatBufferDeserializer = orc::com::fb::FlatBufferDeserializer<DOF>;
    using JointVector = typename orc::RobotTraits<DOF>::JointVector;

    std::string serializer_name = "FlatBufferSerializer" + std::to_string(DOF);
    std::string deserializer_name = "FlatBufferDeserializer" + std::to_string(DOF);

    // Serializer
    nb::class_<FlatBufferSerializer>(m, serializer_name.c_str())
        .def(nb::init<>())
        .def("serialize_joint_trajectory", &FlatBufferSerializer::serialize_joint_trajectory,
             "time_pts"_a, "joint_pts"_a,
             "Serialize a joint trajectory to FlatBuffer format.\n\n"
             "Args:\n"
             "    time_pts: List of time points (float)\n"
             "    joint_pts: List of joint position vectors\n\n"
             "Returns:\n"
             "    bytes: Serialized FlatBuffer data ready for UDP transmission")
        .def("serialize_dense_joint_trajectory",
             &FlatBufferSerializer::serialize_dense_joint_trajectory, "time_pts"_a, "q_pts"_a,
             "q_dot_pts"_a, "q_dotdot_pts"_a, "tau_ff_pts"_a,
             "Serialize a dense joint trajectory with feedforward torques.")
        .def("serialize_cartesian_trajectory",
             &FlatBufferSerializer::serialize_cartesian_trajectory, "time_pts"_a, "pose_pts"_a,
             "Serialize a Cartesian/taskspace trajectory.\n\n"
             "Args:\n"
             "    time_pts: List of time points\n"
             "    pose_pts: List of pose vectors (x, y, z, qw, qx, qy, qz)\n\n"
             "Returns:\n"
             "    bytes: Serialized FlatBuffer data")
        .def("serialize_nullspace_trajectory",
             &FlatBufferSerializer::serialize_nullspace_trajectory, "time"_a, "q_nullspace"_a,
             "Serialize a nullspace configuration.\n\n"
             "Args:\n"
             "    time: Time point\n"
             "    q_nullspace: Nullspace joint configuration\n\n"
             "Returns:\n"
             "    bytes: Serialized FlatBuffer data")
        .def("serialize_jointctrparam_trajectory",
             &FlatBufferSerializer::serialize_jointctrparam_trajectory, "time"_a, "param"_a,
             "Serialize a joint-space CT controller parameter update.\n\n"
             "Args:\n"
             "    time: Time point at which the new gains take effect\n"
             "    param: JointCTParameter (only diagonals of K0/K1/KI are carried)\n\n"
             "Returns:\n"
             "    bytes: Serialized FlatBuffer data")
        .def("serialize_cartesianctrparam_trajectory",
             &FlatBufferSerializer::serialize_cartesianctrparam_trajectory, "time"_a, "param"_a,
             "Serialize a Cartesian CT controller parameter update.\n\n"
             "Args:\n"
             "    time: Time point at which the new gains take effect\n"
             "    param: CartesianCTParameter (only diagonals of K0/K1 are carried;\n"
             "           nullspace gains K0N/K1N are not transmitted)\n\n"
             "Returns:\n"
             "    bytes: Serialized FlatBuffer data")
        .def("serialize_stop", &FlatBufferSerializer::serialize_stop,
             "Serialize a stop command.\n\n"
             "Returns:\n"
             "    bytes: Serialized FlatBuffer data")
        .def("serialize_jointspace_velocity_trajectory",
             &FlatBufferSerializer::serialize_jointspace_velocity_trajectory, "time_pts"_a,
             "velocity_pts"_a,
             "Serialize a jointspace velocity trajectory.\n\n"
             "Args:\n"
             "    time_pts: List of time points\n"
             "    velocity_pts: List of joint velocity vectors\n\n"
             "Returns:\n"
             "    bytes: Serialized FlatBuffer data")
        .def("serialize_cartesian_velocity_trajectory",
             &FlatBufferSerializer::serialize_cartesian_velocity_trajectory, "time_pts"_a,
             "velocity_pts"_a,
             "Serialize a Cartesian (6-DOF) velocity trajectory.\n\n"
             "Args:\n"
             "    time_pts: List of time points\n"
             "    velocity_pts: List of 6-DOF Cartesian velocities (vx, vy, vz, wx, wy, wz)\n\n"
             "Returns:\n"
             "    bytes: Serialized FlatBuffer data")
        .def("serialize_hybrid_force_motion_trajectory",
             &FlatBufferSerializer::serialize_hybrid_force_motion_trajectory, "time_pts"_a,
             "pose_pts"_a, "force_pts"_a, "Serialize a hybrid force/motion trajectory.")
        // Split variants: each returns a list[bytes], one element per UDP
        // datagram. user_cap == 0 means "use MTU-derived cap only".
        .def("serialize_joint_trajectory_split",
             &FlatBufferSerializer::serialize_joint_trajectory_split, "time_pts"_a, "joint_pts"_a,
             "max_pts_per_split"_a = 0, "Split joint trajectory; list of UDP-sized buffers.")
        .def("serialize_dense_joint_trajectory_split",
             &FlatBufferSerializer::serialize_dense_joint_trajectory_split, "time_pts"_a, "q_pts"_a,
             "q_dot_pts"_a, "q_dotdot_pts"_a, "tau_ff_pts"_a, "max_pts_per_split"_a = 0,
             "Split dense joint trajectory; list of UDP-sized buffers.")
        .def("serialize_cartesian_trajectory_split",
             &FlatBufferSerializer::serialize_cartesian_trajectory_split, "time_pts"_a,
             "pose_pts"_a, "max_pts_per_split"_a = 0,
             "Split cartesian trajectory; list of UDP-sized buffers.")
        .def("serialize_jointspace_velocity_trajectory_split",
             &FlatBufferSerializer::serialize_jointspace_velocity_trajectory_split, "time_pts"_a,
             "velocity_pts"_a, "max_pts_per_split"_a = 0,
             "Split jointspace velocity trajectory; list of UDP-sized buffers.")
        .def("serialize_cartesian_velocity_trajectory_split",
             &FlatBufferSerializer::serialize_cartesian_velocity_trajectory_split, "time_pts"_a,
             "velocity_pts"_a, "max_pts_per_split"_a = 0,
             "Split cartesian velocity trajectory; list of UDP-sized buffers.")
        .def("serialize_hybrid_force_motion_trajectory_split",
             &FlatBufferSerializer::serialize_hybrid_force_motion_trajectory_split, "time_pts"_a,
             "pose_pts"_a, "force_pts"_a, "max_pts_per_split"_a = 0,
             "Split hybrid force/motion trajectory; list of UDP-sized buffers.");

    // Deserializer (for receiving robot state on PC)
    nb::class_<FlatBufferDeserializer>(m, deserializer_name.c_str())
        .def(nb::init<>())
        .def(
            "verify_buffer",
            [](const FlatBufferDeserializer& self, nb::bytes data) {
                return self.verify_buffer(data.c_str(), data.size());
            },
            "data"_a,
            "Check if a buffer contains valid FlatBuffer data.\n\n"
            "Args:\n"
            "    data: Raw bytes received from UDP\n\n"
            "Returns:\n"
            "    bool: True if valid FlatBuffer message")
        .def(
            "get_trajectory_type",
            [](const FlatBufferDeserializer& self, nb::bytes data) {
                auto result = self.get_trajectory_type(data.c_str(), data.size());
                return nb::make_tuple(static_cast<int>(result.type), result.valid);
            },
            "data"_a,
            "Get the trajectory type from a FlatBuffer message.\n\n"
            "Args:\n"
            "    data: Raw bytes received from UDP\n\n"
            "Returns:\n"
            "    tuple: (trajectory_type_id, is_valid)");
}

template <int DOF>
inline void add_RobotStateReader(nb::module_& m) {
    using RobotStateReader = orc::com::fb::RobotStateReader<DOF>;

    std::string class_name = "RobotStateReader" + std::to_string(DOF);

    nb::class_<RobotStateReader>(m, class_name.c_str())
        .def(nb::init<>())
        .def(
            "init",
            [](RobotStateReader& self, nb::bytes data) {
                return self.init(data.c_str(), data.size());
            },
            "data"_a,
            "Initialize reader from FlatBuffer data.\n\n"
            "Args:\n"
            "    data: Raw bytes received from UDP\n\n"
            "Returns:\n"
            "    bool: True if valid robot state message")
        .def("time", &RobotStateReader::time, "Get timestamp")
        .def(
            "status", [](const RobotStateReader& self) { return static_cast<int>(self.status()); },
            "Get robot status as int")
        .def("model_id", &RobotStateReader::model_id, "Get model ID")
        .def("q_act", &RobotStateReader::q_act, "joint"_a, "Get actual joint position")
        .def("q_dot_act", &RobotStateReader::q_dot_act, "joint"_a, "Get actual joint velocity")
        .def("q_dotdot_act", &RobotStateReader::q_dotdot_act, "joint"_a,
             "Get actual joint acceleration")
        .def("tau", &RobotStateReader::tau, "joint"_a, "Get actual joint torque")
        .def("q_set", &RobotStateReader::q_set, "joint"_a, "Get joint position setpoint")
        .def("q_dot_set", &RobotStateReader::q_dot_set, "joint"_a, "Get joint velocity setpoint")
        .def("q_dotdot_set", &RobotStateReader::q_dotdot_set, "joint"_a,
             "Get joint acceleration setpoint");
}
