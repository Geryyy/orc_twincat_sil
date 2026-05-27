#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <orc/OrcTypes.h>
#include <orc/com/TrajectoryServer.h>
#include <orc/com/flatbuffers/FlatBufferDeserializer.h>
#include <orc/com/flatbuffers/FlatBufferSerializer.h>
#include <orc/control/ControllerParameter.h>
#include <orc/robots/LinearAxis.h>
#include <orc/robots/Robot.h>

#include <ControllerParameter_bindings.h>
#include <RobotState_bindings.h>
#include <interpolator/Interpolator_bindings.h>

namespace nb = nanobind;
using namespace nb::literals;
using MatrixXdR = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

void add_linear_axis(nb::module_& m) {
    using LinearAxis = orc::robots::LinearAxis;
    using Time = orc::Time;
    using JointVector = orc::robots::LinearAxis::JointVector;
    using JointMatrix = orc::robots::LinearAxis::JointMatrix;
    using Robot2 = orc::robots::Robot<LinearAxis::DOF>;

    nb::module_ linear_axis_submodule = m.def_submodule("linear_axis", "LinearAxis robot module");

    // RobotState
    add_RobotState<LinearAxis>(linear_axis_submodule);

    // LinearAxis class
    nb::class_<LinearAxis, Robot2>(m, "LinearAxis")
        .def(nb::init<const char*, Time, JointMatrix>(), "mjb_path"_a, "Ta"_a, "K0"_a)
        .def(nb::init<const char*>(), "mjb_path"_a)
        .def("update", nb::overload_cast<Time, bool>(&LinearAxis::update), "t"_a,
             "grav_comp_only"_a = false)
        .def("reset", &LinearAxis::reset, "q_act"_a)
        .def("copy", [](const LinearAxis& self) { return LinearAxis(self); })
        .def("__repr__", [](const LinearAxis&) { return "<LinearAxis Robot>"; })
        .def_ro_static("CLIENT_PORT", &LinearAxis::CLIENT_PORT)
        .def_ro_static("SERVER_PORT", &LinearAxis::SERVER_PORT)
        .def_ro_static("SIL_MODEL_PORT", &LinearAxis::SIL_MODEL_PORT)
        .def_ro_static("SIL_CONTROLLER_PORT", &LinearAxis::SIL_CONTROLLER_PORT);

    // Trajectory server
    using LinearAxisTrajectoryServer =
        orc::com::TrajectoryServer<LinearAxis, orc::robots::LinearAxis::SERVER_PORT,
                                   orc::robots::LinearAxis::CLIENT_PORT>;
    nb::class_<LinearAxisTrajectoryServer>(linear_axis_submodule, "TrajectoryServer")
        .def(nb::init<std::shared_ptr<LinearAxis>, uint16_t, uint16_t>(), nb::arg("contr"),
             nb::arg("server_port") = LinearAxis::SERVER_PORT,
             nb::arg("client_port") = LinearAxis::CLIENT_PORT)
        .def("run", &LinearAxisTrajectoryServer::run, nb::call_guard<nb::gil_scoped_release>())
        .def("poll", &LinearAxisTrajectoryServer::poll, nb::call_guard<nb::gil_scoped_release>())
        .def("send_robot_data", &LinearAxisTrajectoryServer::send_robot_data, nb::arg("time"),
             nb::call_guard<nb::gil_scoped_release>());
}
