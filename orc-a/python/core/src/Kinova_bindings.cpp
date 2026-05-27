#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <orc/OrcTypes.h>
#include <orc/com/RobotState.h>
#include <orc/com/TrajectoryServer.h>
#include <orc/com/flatbuffers/FlatBufferDeserializer.h>
#include <orc/com/flatbuffers/FlatBufferSerializer.h>
#include <orc/control/ControllerParameter.h>
#include <orc/robots/Kinova.h>
#include <orc/robots/Robot.h>

#include <ControllerParameter_bindings.h>
#include <RobotState_bindings.h>
#include <interpolator/Interpolator_bindings.h>

namespace nb = nanobind;
using namespace nb::literals;
using MatrixXdR = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

void add_Kinova(nb::module_& m) {
    using Kinova = orc::robots::Kinova;
    using Time = orc::Time;
    using JointVector = orc::robots::Kinova::JointVector;

    nb::module_ kinova_submodule = m.def_submodule("kinova", "Kinova robot module");

    nb::class_<Kinova, orc::robots::Robot<7>>(m, "Kinova")
        .def(nb::init<const char*, std::string>(), "mjb_path"_a,
             "endeffector_body_name"_a = Kinova::name_link_e)
        .def(nb::init<const char*, typename Kinova::JointPDPParameter,
                      typename Kinova::GravityCompParameter, Time, std::string>(),
             "mjb_path"_a, "js_param"_a, "gc_param"_a, "Ta"_a,
             "endeffector_body_name"_a = Kinova::name_link_e)
        .def_static("get_q_home", &Kinova::get_q_home)
        .def("copy", [](const Kinova& self) { return Kinova(self); })
        .def("__repr__", [](const Kinova&) { return "<Kinova Robot>"; })
        .def_ro_static("DOF", &Kinova::DOF);

    kinova_submodule.attr("SERVER_PORT") = nb::int_(orc::robots::Kinova::SERVER_PORT);
    kinova_submodule.attr("CLIENT_PORT") = nb::int_(orc::robots::Kinova::CLIENT_PORT);

    using KinovaTrajectoryServer =
        orc::com::TrajectoryServer<Kinova, orc::robots::Kinova::SERVER_PORT,
                                   orc::robots::Kinova::CLIENT_PORT>;
    nb::class_<KinovaTrajectoryServer>(kinova_submodule, "TrajectoryServer")
        .def(nb::init<std::shared_ptr<Kinova>, uint16_t, uint16_t>(), nb::arg("contr"),
             nb::arg("server_port") = orc::robots::Kinova::SERVER_PORT,
             nb::arg("client_port") = orc::robots::Kinova::CLIENT_PORT,
             R"pbdoc(
       A UDP server that listens for incoming trajectories and sends back robot state updates.

       Args:
           contr (RobotType): A pointer to the robot controller.
           server_port (int, optional): The port number to listen on. Defaults to SERVER_PORT.
           client_port (int, optional): The port number of the client that sends the trajectories. Defaults to CLIENT_PORT.
       )pbdoc")
        .def("run", &KinovaTrajectoryServer::run,
             R"pbdoc(
       Start the server and wait for incoming trajectories.
       )pbdoc",
             nb::call_guard<nb::gil_scoped_release>())
        .def("poll", &KinovaTrajectoryServer::poll,
             R"pbdoc(
       Check for incoming trajectories without blocking.
       )pbdoc",
             nb::call_guard<nb::gil_scoped_release>())
        .def("send_robot_data", &KinovaTrajectoryServer::send_robot_data, nb::arg("time"),
             R"pbdoc(
       Send the current robot state to the client.

       Args:
           time (float): The current time in seconds.
       )pbdoc",
             nb::call_guard<nb::gil_scoped_release>());

    using KinovaContrParam = orc::robots::Kinova::KinovaContrParam;
    using ControllerParameter = orc::control::ControllerParameter<Kinova::DOF>;
    nb::class_<KinovaContrParam, ControllerParameter>(kinova_submodule, "KinovaContrParam")
        .def(nb::init<bool>(), "simulation"_a = true);
}
