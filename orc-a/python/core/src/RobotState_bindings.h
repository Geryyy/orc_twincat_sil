#pragma once

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <orc/OrcTypes.h>
#include <orc/RobotStatus.h>
#include <orc/com/RobotState.h>
#include <orc/robots/Iiwa.h>
#include <orc/robots/Robot.h>

namespace nb = nanobind;
using namespace nb::literals;
using MatrixXdR = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

template <typename RobotType>
void add_RobotState(nb::module_& m) {
    /* RobotState */
    using RobotState = orc::com::RobotState<RobotType>;
    nb::class_<RobotState>(m, "RobotState", "Represents the state of a Iiwa robot.")
        .def(nb::init<>(), "Default constructor for LBRIiwaState.")

        .def(nb::init<RobotType&, double, orc::logic::RobotStatus>(), "iiwa_object"_a, "time"_a,
             "status"_a, "Constructs a RobotState object from an Iiwa robot, time, and status.")

        .def(nb::init<RobotType&, double>(), "iiwa_object"_a, "time"_a,
             "Constructs a RobotState object from an Iiwa robot and time.")

        .def(nb::init<std::vector<unsigned char>&>(), "bytearr"_a,
             "Constructs an LBRIiwaState object from a byte array.")

        // Manually changed function signature, because of problems with bytearray and const char *
        .def_static("deserialize", &RobotState::deserialize,
                    "Deserialize the RobotState from a pointer and size")
        .def_static(
            "deserialize",
            [](nb::bytearray data) { return RobotState::deserialize(data.c_str(), data.size()); },
            "Deserialize the RobotState from a bytearray")

        .def_rw("time", &RobotState::time_)
        .def_rw("status", &RobotState::status)
        .def_rw("q_act", &RobotState::q_act)
        .def_rw("q_dot_act", &RobotState::q_dot_act)
        .def_rw("q_dotdot_act", &RobotState::q_dotdot_act)
        .def_rw("tau", &RobotState::tau)
        .def_rw("q_set", &RobotState::q_set)
        .def_rw("q_dot_set", &RobotState::q_dot_set)
        .def_rw("q_dotdot_set", &RobotState::q_dotdot_set)
        .def_rw("x_set", &RobotState::x_set)
        .def_rw("x_dot_set", &RobotState::x_dot_set)
        .def_rw("x_dotdot_set", &RobotState::x_dotdot_set)
        .def_rw("q_d_NS", &RobotState::q_d_NS)
        .def_rw("model_id", &RobotState::model_id)

        .def("__repr__", [](RobotState&) { return "RobotState()"; })

        .def("serialize", &RobotState::serialize, "Serialize the RobotState to a vector of chars.")

        .def("__str__",
             [](RobotState& state) {
                 std::ostringstream r;
                 r << "RobotState(\n"
                   << "time=" << state.time_ << "\n"
                   << "q_act=" << state.q_act.transpose() << "\n"
                   << "q_dot_act=" << state.q_dot_act.transpose() << "\n"
                   << "q_dotdot_act=" << state.q_dotdot_act.transpose() << "\n"
                   << "tau=" << state.tau.transpose() << "\n"
                   << "q_set=" << state.q_set.transpose() << "\n"
                   << "q_dot_set=" << state.q_dot_set.transpose() << "\n"
                   << "q_dotdot_set=" << state.q_dotdot_set.transpose() << "\n"
                   << "x_set=" << state.x_set.transpose() << "\n"
                   << "x_dot_set=" << state.x_dot_set.transpose() << "\n"
                   << "x_dotdot_set=" << state.x_dotdot_set.transpose() << "\n"
                   << "q_d_NS=" << state.q_d_NS.transpose() << "\n"
                   << ")";
                 return r.str();
             })

        .def_prop_ro_static(
            "DATA_SIZE", [](nb::handle /*unused*/) { return RobotState::DATA_SIZE; },
            "Size of receive buffer for state data.");
}
