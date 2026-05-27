/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   bindings.cpp                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: anonymous                                  +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2023/04/26 10:41:02 by anonymous         #+#    #+#             */
/*   Updated: 2024/04/12 13:24:44 by anonymous        ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "mujoco/mujoco.h"
#include "orc/RobotStatus.h"
#include "orc/util/Logger.h"
#include "orc/util/Time.h"

#include "Logger_bindings.h"
#include "Robot_bindings.h"
#include "interpolator/Interpolator_bindings.h"

// FlatBuffers support
#include "com/FlatBufferSerializer_bindings.h"

namespace nb = nanobind;
using namespace nb::literals;
using MatrixXdR = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

using namespace orc;

void add_iiwa(nb::module_& m);
void add_linear_axis(nb::module_& m);
void add_Kinova(nb::module_& m);

NB_MODULE(_core, m) {
    // Disable leak warnings
    nb::set_leak_warnings(false);

    m.doc() = R"(ORC - Open Robot Control python module)";

    // Bind submodules
    nb::module_ logic_module = m.def_submodule("logic");
    nb::module_ robots_module = m.def_submodule("robots");
    nb::module_ interpolator_module = m.def_submodule("interpolator");
    nb::module_ com_module = m.def_submodule("com");
    nb::module_ log_module = m.def_submodule("log");

    // Bind logging capability
    add_Logger(log_module);

    // RobotStatus
    nb::enum_<orc::logic::RobotStatus>(logic_module, "RobotStatus")
        .value("OFF", orc::logic::RobotStatus::OFF)
        .value("ENABLE", orc::logic::RobotStatus::ENABLE)
        .value("GRAVCOMP", orc::logic::RobotStatus::GRAVCOMP)
        .export_values();

    // Add DOF independent interpolators
    add_CartesianPoseInterpolator(interpolator_module);

    // This ensures the types are registered when Robot members are bound
    nb::module_ fb_module = com_module.def_submodule(
        "flatbuffers", "FlatBuffers-based serialization for efficient zero-copy communication");

    // Add 7-DOF serializer (Iiwa)
    add_FlatBufferSerializer<7>(fb_module);

    // Add 2-DOF serializer (LinearAxis)
    add_FlatBufferSerializer<2>(fb_module);

    // Add RobotState readers
    add_RobotStateReader<7>(fb_module);
    add_RobotStateReader<2>(fb_module);

    // Indicate FlatBuffers is available
    fb_module.def(
        "is_available", []() { return true; }, "Check if FlatBuffers support is available");

    // Bind specified Robot class
    // This is necessary for Robot base class functions to work in deriving classes, i.e.,
    // LinearAxis and Iiwa.
    add_Robot<2>(robots_module, "robot2", "Robot2");
    add_Robot<7>(robots_module, "robot7", "Robot7");

    // Bind classes deriving from Robot<DOF>
    add_linear_axis(robots_module);
    add_iiwa(robots_module);
    add_Kinova(robots_module);

    // Bind Time class
    nb::class_<Time>(m, "Time")
        // Constructors
        .def(nb::init<>())
        .def(nb::init<int64_t, int64_t>())
        .def(nb::init<double>())
        // Member functions
        .def("__repr__",
             [](Time& time) {
                 return "Time(" + std::to_string(time.get_sec()) + ", " +
                        std::to_string(time.get_nsec()) + ")";
             })
        .def("to_sec", &Time::toSec)
        .def("quantize", &Time::quantize)
        .def("get_sec", &Time::get_sec)
        .def("get_nsec", &Time::get_nsec)
        .def("normalize", &Time::normalize)
        // .def("__add__", &Time::operator+, nb::arg<Time&>())
        .def("__add__", [](const Time& self, const Time& other) { return self + other; })
        .def("__add__", [](const Time& self, double value) { return self + value; })
        .def("__radd__", [](double value, const Time& self) { return value + self; })
        .def("__sub__", [](const Time& self, const Time& other) { return self - other; })
        .def("__sub__", [](const Time& self, double value) { return self - value; })
        .def("__rsub__", [](double value, const Time& self) { return value - self; })
        .def("__mul__", [](const Time& self, const Time& other) { return self * other; })
        .def("__mul__", [](const Time& self, double value) { return self * value; })
        .def("__rmul__", [](double value, const Time& self) { return value * self; })
        .def("__truediv__", [](const Time& self, const Time& other) { return self / other; })
        .def("__truediv__", [](const Time& self, double value) { return self / value; })
        .def("__rtruediv__", [](double value, const Time& self) { return value / self; })
        .def("__neg__", [](const Time& self, const Time& other) { return self - other; })
        .def("__neg__", [](const Time& self, double value) { return self - value; })
        .def("__eq__", [](const Time& self, const Time& other) { return self == other; })
        .def("__ne__", [](const Time& self, const Time& other) { return self != other; })
        .def("__lt__", [](const Time& self, const Time& other) { return self < other; })
        .def("__le__", [](const Time& self, const Time& other) { return self <= other; })
        .def("__gt__", [](const Time& self, const Time& other) { return self > other; })
        .def("__ge__", [](const Time& self, const Time& other) { return self >= other; })
        .def("__double__", &Time::operator double)
        .def("__int__", &Time::operator int64_t)
        // Operator overloads
        .def_static("convert_time_to_double_vector", &Time::convertTimeToDoubleVector)
        .def_static("convert_double_to_time_vector", &Time::convertDoubleToTimeVector)
        .def("toString", &Time::toString)
        .def("__repr__", [](const Time& time) { return time.toString(); });
}
