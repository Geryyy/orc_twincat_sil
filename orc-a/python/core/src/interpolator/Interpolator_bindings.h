#pragma once

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/stl/vector.h>

#include <orc/OrcTypes.h>
#include <orc/RobotTraits.h>
#include <orc/interpolator/cartesian/CartesianPoseInterpolator.h>
#include <orc/interpolator/jointspace/SplineJointInterpolator.h>

namespace nb = nanobind;
using namespace nb::literals;

// This solution is used, as the base class ManifoldBaseInterpolator is templated. Handle with care!
#define MANIFOLDINTERPOLATORBASEFUNCTIONALITY(CLASS_NAME)                 \
    .def("update", &CLASS_NAME::update, "t"_a)                            \
        .def("get_start_point", &CLASS_NAME::get_start_point)             \
        .def("get_end_point", &CLASS_NAME::get_end_point)                 \
        .def("get_point", &CLASS_NAME::get_point)                         \
        .def("get_derivative", &CLASS_NAME::get_derivative)               \
        .def("get_second_derivative", &CLASS_NAME::get_second_derivative) \
        .def("get_third_derivative", &CLASS_NAME::get_third_derivative)   \
        .def("get_trajectory_points", &CLASS_NAME::get_trajectory_points)

inline void add_CartesianPoseInterpolator(nb::module_& m) {
    using CartesianPoseInterpolator = orc::interpolator::CartesianPoseInterpolator;
    using PoseVector = orc::PoseVector;
    using CartesianVector = orc::CartesianVector;
    using Time = orc::Time;
    using ManifoldInterpolatorBase =
        orc::interpolator::ManifoldInterpolatorBase<Time, PoseVector, CartesianVector>;

    nb::class_<CartesianPoseInterpolator>(m, "CartesianPoseInterpolator")
        .def(nb::init<PoseVector, PoseVector, Time, Time>(), "pose0"_a, "pose1"_a, "t0"_a, "t1"_a)
        .def("init",
             nb::overload_cast<PoseVector&, CartesianVector&, CartesianVector&>(
                 &CartesianPoseInterpolator::init),
             "pose_now"_a, "x_dot_now"_a, "x_dotdot_now"_a)
        .def("init", nb::overload_cast<>(&CartesianPoseInterpolator::init))
            MANIFOLDINTERPOLATORBASEFUNCTIONALITY(CartesianPoseInterpolator);
}

template <int DOF>
inline void add_SplineJointInterpolator(nb::module_& m) {
    using SplineJointInterpolator = orc::interpolator::SplineJointInterpolator<DOF>;
    using JointVector = typename orc::RobotTraits<DOF>::JointVector;
    using Time = orc::Time;
    using ManifoldInterpolatorBase =
        orc::interpolator::ManifoldInterpolatorBase<Time, JointVector, JointVector>;

    nb::class_<SplineJointInterpolator>(m, "SplineJointInterpolator")
        .def(nb::init<JointVector, JointVector, Time, Time>(), "x0"_a, "x1"_a, "t0"_a, "t1"_a)
        .def("init",
             nb::overload_cast<JointVector&, JointVector&, JointVector&>(
                 &SplineJointInterpolator::init),
             "q_now"_a, "q_dot_now"_a, "q_dot_dot_now"_a)
            MANIFOLDINTERPOLATORBASEFUNCTIONALITY(SplineJointInterpolator);
}

#undef MANIFOLDINTERPOLATORBASEFUNCTIONALITY
