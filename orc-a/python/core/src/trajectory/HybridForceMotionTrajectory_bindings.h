#pragma once

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "orc/OrcTypes.h"
#include "orc/trajectory/HybridForceMotionTrajectory.h"

namespace nb = nanobind;
using namespace nb::literals;

template <int DOF>
inline void add_HybridForceMotionTrajectory(nb::module_& m) {
    using namespace orc;
    using HybridForceMotionTrajectory = typename orc::trajectory::HybridForceMotionTrajectory<DOF>;

    nb::class_<HybridForceMotionTrajectory>(m, "HybridForceMotionTrajectory")
        .def(nb::init<std::vector<orc::PoseVector>&, std::vector<double>&,
                      std::vector<orc::Time>&>(),
             "pose_vectors"_a, "forces"_a, "time_points"_a)
        .def("init", nb::overload_cast<>(&HybridForceMotionTrajectory::init))
        .def("get_start_time", &HybridForceMotionTrajectory::get_start_time)
        .def("update", nb::overload_cast<orc::Time>(&HybridForceMotionTrajectory::update), "t"_a)
        .def("update",
             nb::overload_cast<orc::Time, typename orc::robots::RobotData<DOF>&>(
                 &HybridForceMotionTrajectory::update),
             "t"_a, "data"_a)
        .def("get_pose", &HybridForceMotionTrajectory::get_pose)
        .def("get_x_dot", &HybridForceMotionTrajectory::get_x_dot)
        .def("get_x_dotdot", &HybridForceMotionTrajectory::get_x_dotdot)
        .def("get_force", &HybridForceMotionTrajectory::get_force);
}
