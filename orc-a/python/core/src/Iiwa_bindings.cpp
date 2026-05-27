/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   iiwa_bindings.cpp                                  :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: anonymous                                  +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */  
/*   Created: 2023/04/26 10:40:15 by anonymous         #+#    #+#             */
/*   Updated: 2024/04/11 09:59:51 by anonymous        ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

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
#include <orc/robots/Iiwa.h>
#include <orc/robots/Robot.h>

#include <ControllerParameter_bindings.h>
#include <RobotState_bindings.h>

namespace nb = nanobind;
using namespace nb::literals;
using MatrixXdR = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

void add_iiwa(nb::module_& m) {
    using Iiwa = orc::robots::Iiwa;
    using Time = orc::Time;
    using JointVector = orc::robots::Iiwa::JointVector;
    using Robot7 = orc::robots::Robot<7>;

    nb::module_ iiwa_submodule = m.def_submodule("iiwa", "Iiwa robot module");

    // Communication defaults
    iiwa_submodule.attr("SERVER_PORT") = nb::int_(orc::robots::Iiwa::SERVER_PORT);
    iiwa_submodule.attr("CLIENT_PORT") = nb::int_(orc::robots::Iiwa::CLIENT_PORT);

    // Iiwa class
    nb::class_<Iiwa, Robot7>(m, "Iiwa")
        .def(nb::init<const char*, std::string, std::string>(), "mjb_path"_a,
             "endeffector_body_name"_a = Iiwa::name_link_e, "force_sensor_name"_a = "")
        .def(nb::init<const char*, Time, std::string, std::string>(), "mjb_path"_a, "Ta"_a,
             "endeffector_body_name"_a = Iiwa::name_link_e, "force_sensor_name"_a = "")
        .def(nb::init<const char*, typename Iiwa::JointCTParameter,
                      typename Iiwa::CartesianCTParameter,
                      typename Iiwa::SingularPerturbationParameter,
                      typename Iiwa::FrictionCompParameter, typename Iiwa::GravityCompParameter,
                      Time, typename Iiwa::JointArray, std::string, std::string>(),
             "mjb_path"_a, "js_param"_a, "ts_param"_a, "sp_param"_a, "fc_param"_a, "gc_param"_a,
             "Ta"_a, "f_cutoff_norm"_a, "endeffector_body_name"_a = Iiwa::name_link_e,
             "force_sensor_name"_a = "")
        .def(nb::init<const char*, typename Iiwa::JointCTParameter,
                      typename Iiwa::CartesianCTParameter, typename Iiwa::GravityCompParameter,
                      Time, typename Iiwa::JointArray>(),
             "mjb_path"_a, "js_param"_a, "ts_param"_a, "gc_param"_a, "Ta"_a, "f_cutoff_norm"_a)
        .def(nb::init<const char*, typename Iiwa::JointCTParameter,
                      typename Iiwa::CartesianCTParameter, typename Iiwa::GravityCompParameter,
                      Time, typename Iiwa::JointArray, std::string, std::string>(),
             "mjb_path"_a, "js_param"_a, "ts_param"_a, "gc_param"_a, "Ta"_a, "f_cutoff_norm"_a,
             "endeffector_body_name"_a = Iiwa::name_link_e, "force_sensor_name"_a = "")

        .def("copy", [](const Iiwa& self) { return Iiwa(self); })
        .def("set_q_act_filtered_derivatives", &Iiwa::set_q_act_filtered_derivatives)
        .def("reset", &Iiwa::reset, "q_act"_a)
        .def("__repr__", [](const Iiwa&) { return "<Iiwa Robot>"; })
        .def_ro_static("CLIENT_PORT", &Iiwa::CLIENT_PORT)
        .def_ro_static("SERVER_PORT", &Iiwa::SERVER_PORT)
        .def_ro_static("SIL_MODEL_PORT", &Iiwa::SIL_MODEL_PORT)
        .def_ro_static("SIL_CONTROLLER_PORT", &Iiwa::SIL_CONTROLLER_PORT);

    using IiwaContrParam = orc::robots::Iiwa::IiwaContrParam;
    using ControllerParameter = orc::control::ControllerParameter<Iiwa::DOF>;
    nb::class_<IiwaContrParam, ControllerParameter>(iiwa_submodule, "IiwaContrParam")
        .def(nb::init<bool>(), nb::arg("simulation") = true)
        .def_rw("K0_cart", &IiwaContrParam::K0_cart)
        .def_rw("K1_cart", &IiwaContrParam::K1_cart)
        .def_rw("K0_N_cart", &IiwaContrParam::K0_N_cart)
        .def_rw("K1_N_cart", &IiwaContrParam::K1_N_cart)
        .def_rw("K0_joint", &IiwaContrParam::K0_joint)
        .def_rw("K1_joint", &IiwaContrParam::K1_joint)
        .def_rw("KI_joint", &IiwaContrParam::KI_joint)
        .def_rw("B", &IiwaContrParam::B)
        .def_rw("K_sp", &IiwaContrParam::K_sp)
        .def_rw("D_sp", &IiwaContrParam::D_sp)
        .def_rw("L_fc", &IiwaContrParam::L_fc)
        .def_rw("D_gc", &IiwaContrParam::D_gc)
        .def_rw("f_c_fast", &IiwaContrParam::f_c_fast)
        .def_rw("f_c_slow", &IiwaContrParam::f_c_slow)
        .def_rw("torque_sensor_offset", &IiwaContrParam::torque_sensor_offset)
        .def("__str__", [](const IiwaContrParam& self) -> std::string {
            std::stringstream ss;
            ss << "K0_cart:\n" << self.K0_cart << "\n";
            ss << "K1_cart:\n" << self.K1_cart << "\n";
            ss << "K0_N_cart:\n" << self.K0_N_cart << "\n";
            ss << "K1_N_cart:\n" << self.K1_N_cart << "\n";
            ss << "K0_joint:\n" << self.K0_joint << "\n";
            ss << "K1_joint:\n" << self.K1_joint << "\n";
            ss << "KI_joint:\n" << self.KI_joint << "\n";
            ss << "K_sp:\n" << self.K_sp << "\n";
            ss << "D_sp:\n" << self.D_sp << "\n";
            ss << "L_fc:\n" << self.L_fc << "\n";
            ss << "D_gc:\n" << self.D_gc << "\n";
            ss << "f_c_fast:\n" << self.f_c_fast << "\n";
            ss << "f_c_slow:\n" << self.f_c_slow << "\n";
            ss << "torque_sensor_offset:\n" << self.torque_sensor_offset << "\n";
            return ss.str();
        });

    // Trajectory server
    using LBRIiwaTrajectoryServer = orc::com::TrajectoryServer<Iiwa, orc::robots::Iiwa::SERVER_PORT,
                                                               orc::robots::Iiwa::CLIENT_PORT>;
    nb::class_<LBRIiwaTrajectoryServer>(iiwa_submodule, "TrajectoryServer")
        .def(nb::init<std::shared_ptr<Iiwa>, uint16_t, uint16_t>(), nb::arg("contr"),
             nb::arg("server_port") = orc::robots::Iiwa::SERVER_PORT,
             nb::arg("client_port") = orc::robots::Iiwa::CLIENT_PORT,
             R"pbdoc(
       A UDP server that listens for incoming trajectories and sends back robot state updates.

       Args:
           contr (RobotType): A pointer to the robot controller.
           server_port (int, optional): The port number to listen on. Defaults to SERVER_PORT.
           client_port (int, optional): The port number of the client that sends the trajectories. Defaults to CLIENT_PORT.
       )pbdoc")
        .def("run", &LBRIiwaTrajectoryServer::run,
             R"pbdoc(
       Start the server and wait for incoming trajectories.
       )pbdoc",
             nb::call_guard<nb::gil_scoped_release>())
        .def("poll", &LBRIiwaTrajectoryServer::poll,
             R"pbdoc(
       Check for incoming trajectories without blocking.
       )pbdoc",
             nb::call_guard<nb::gil_scoped_release>())
        .def("send_robot_data", &LBRIiwaTrajectoryServer::send_robot_data, nb::arg("time"),
             R"pbdoc(
       Send the current robot state to the client.

       Args:
           time (float): The current time in seconds.
       )pbdoc",
             nb::call_guard<nb::gil_scoped_release>());
}
