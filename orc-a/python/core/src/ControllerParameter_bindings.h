#pragma once

#include <type_traits>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <orc/OrcTypes.h>
#include <orc/RobotTraits.h>
#include <orc/control/ControllerParameter.h>
#include <orc/control/controller/CoulombFrictionCompController.h>
#include <orc/control/controller/FrictionCompController.h>
#include <orc/control/controller/GravityCompController.h>
#include <orc/control/controller/HybridForceMotionController.h>
#include <orc/control/controller/cartesian/CartesianCTController.h>
#include <orc/control/controller/joint/JointCTController.h>
#include <orc/robots/Iiwa.h>
#include <orc/robots/Robot.h>

namespace nb = nanobind;
using namespace nb::literals;

template <int DOF>
inline void add_JointCTParameter(nb::module_& m) {
    using JointCTParameter = orc::control::JointCTParameter<DOF>;
    nb::class_<JointCTParameter>(m, "JointCTParameter")
        .def(nb::init<>(), "Default constructor.")
        .def(
            nb::init<typename JointCTParameter::JointMatrix, typename JointCTParameter::JointMatrix,
                     typename JointCTParameter::JointMatrix>(),
            "Constructor that sets the K0, K1, and KI values.", nb::arg("K0"), nb::arg("K1"),
            nb::arg("KI"))
        .def(nb::init<orc::control::ControllerParameter<DOF>>(),
             "Constructor that sets the K0, K1, and KI values using an ControllerParameter object.",
             nb::arg("contr_param"))
        .def_rw("K0", &JointCTParameter::K0,
                "The diagonal matrix of proportional gains in the joint space.")
        .def_rw("K1", &JointCTParameter::K1,
                "The diagonal matrix of derivative gains in the joint space.")
        .def_rw("KI", &JointCTParameter::KI,
                "The diagonal matrix of integral gains in the joint space.")
        .def(
            "__str__",
            [](const JointCTParameter& param) {
                std::ostringstream ss;
                ss << "JointCTParameter:\n"
                   << "K0:\n"
                   << param.K0 << "\nK1:\n"
                   << param.K1 << "\nKI:\n"
                   << param.KI;
                return ss.str();
            },
            "Returns a string representation of the JointCTParameter object.");
}

template <int DOF>
inline void add_JointPDPParameter(nb::module_& m) {
    using JointPDPParameter = orc::control::JointPDPParameter<DOF>;
    nb::class_<JointPDPParameter>(m, "JointPDPParameter")
        .def(nb::init<>(), "Default constructor.")
        .def(nb::init<typename JointPDPParameter::JointMatrix,
                      typename JointPDPParameter::JointMatrix>(),
             "Constructor that sets the Kp and Kd values.", nb::arg("Kp"), nb::arg("Kd"))
        .def(nb::init<orc::control::ControllerParameter<DOF>>(),
             "Constructor that sets the Kp and Kd values using an IiwaContrParam object.",
             nb::arg("contr_param"))
        .def_rw("Kp", &JointPDPParameter::Kp,
                "The diagonal matrix of proportional gains in the joint space.")
        .def_rw("Kd", &JointPDPParameter::Kd,
                "The diagonal matrix of derivative gains in the joint space.")
        .def(
            "__str__",
            [](const JointPDPParameter& param) {
                std::ostringstream ss;
                ss << "JointPDPParameter:\n" << "Kp:\n" << param.Kp << "\nKd:\n" << param.Kd;
                return ss.str();
            },
            "Returns a string representation of the JointPDPParameter object.");
}

template <int DOF>
inline void add_CartesianCTParameter(nb::module_& m) {
    // Define the CartesianCTParameter class bindings.
    using CartesianCTParameter = orc::control::CartesianCTParameter<DOF>;
    nb::class_<CartesianCTParameter>(m, "CartesianCTParameter")
        .def(nb::init<>(),
             R"(
            Constructs a new `CartesianCTParameter` object with default values for `K0`, `K1`, `K0N`, and `K1N`.
         )")
        .def(nb::init<orc::CartesianMatrix, orc::CartesianMatrix,
                      typename CartesianCTParameter::JointMatrix,
                      typename CartesianCTParameter::JointMatrix>(),
             nb::arg("K_0"), nb::arg("K_1"), nb::arg("K_0N"), nb::arg("K_1N"),
             R"(
            Constructs a new `CartesianCTParameter` object with the given values of `K0`, `K1`, `K0N`, and `K1N`.

            :param K_0: The proportional gain matrix for the error in the Cartesian position.
            :type K_0: CartesianMatrix
            :param K_1: The derivative gain matrix for the error in the Cartesian position.
            :type K_1: CartesianMatrix
            :param K_0N: The proportional gain matrix for the error in the joint positions.
            :type K_0N: JointMatrix
            :param K_1N: The derivative gain matrix for the error in the joint positions.
            :type K_1N: JointMatrix
         )")
        .def(nb::init<orc::control::ControllerParameter<DOF>>(), nb::arg("param"),
             R"(
            Constructs a new `CartesianCTParameter` object from a `ControllerParameter` object.

            :param param: The `ControllerParameter` object from which to construct the `CartesianCTParameter` object.
            :type param: ControllerParameter
         )")
        .def_rw("K0", &CartesianCTParameter::K0,
                R"(
                      The proportional gain matrix for the error in the Cartesian position.

                      :type: CartesianMatrix
                   )")
        .def_rw("K1", &CartesianCTParameter::K1,
                R"(
                      The derivative gain matrix for the error in the Cartesian position.

                      :type: CartesianMatrix
                   )")
        .def_rw("K0N", &CartesianCTParameter::K0N,
                R"(
                      The proportional gain matrix for the error in the joint positions.

                      :type: JointMatrix
                   )")
        .def_rw("K1N", &CartesianCTParameter::K1N,
                R"(
                      The derivative gain matrix for the error in the joint positions.

                      :type: JointMatrix
                   )")
        .def(
            "__str__",
            [](const CartesianCTParameter& param) {
                std::ostringstream ss;
                ss << "CartesianCTParameter:\n"
                   << "K0:\n"
                   << param.K0 << "\n"
                   << "K1:\n"
                   << param.K1 << "\n"
                   << "K0N:\n"
                   << param.K0N << "\n"
                   << "K1N:\n"
                   << param.K1N;
                return ss.str();
            },
            R"(
            Returns a string representation of the `CartesianCTParameter` object.
         )");
}

template <int DOF>
inline void add_SingularPertrubationParameter(nb::module_& m) {
    using SingularPerturbationParameter = orc::control::SingularPerturbationParameter<DOF>;
    nb::class_<SingularPerturbationParameter>(m, "SingularPerturbationParameter")
        .def(nb::init<>(),
             R"(
            Constructs a new `SingularPerturbationParameter` object with default values of zero for `K` and `D`, and an array of ones for `f_norm`.
         )")
        .def(nb::init<typename SingularPerturbationParameter::JointMatrix,
                      typename SingularPerturbationParameter::JointMatrix,
                      typename SingularPerturbationParameter::JointVector,
                      typename SingularPerturbationParameter::JointArray>(),
             nb::arg("K_sp"), nb::arg("D_sp"), nb::arg("B_sp"), nb::arg("f_cutoff_norm"),
             R"(
            Constructs a new `SingularPerturbationParameter` object with the given values of `K`, `D`, and `f_norm`.

            :param K_sp: The proportional gain matrix for the singular perturbation.
            :type K_sp: JointMatrix
            :param D_sp: The derivative gain matrix for the singular perturbation.
            :type D_sp: JointMatrix
            :param B_sp: Rotor inertia.
            :type B_sp: JointVector
            :param f_cutoff_norm: The cutoff frequencies for the PT1 and DT1 filters applied to the torque signals to calculate tau and tau_dot.
            :type f_cutoff_norm: JointArray
         )")
        .def_rw("K", &SingularPerturbationParameter::K,
                R"(
                      The proportional gain matrix for the singular perturbation.

                      :type: JointMatrix
                   )")
        .def_rw("D", &SingularPerturbationParameter::D,
                R"(
                      The derivative gain matrix for the singular perturbation.

                      :type: JointMatrix
                   )")
        .def_rw("B", &SingularPerturbationParameter::B,
                R"(
            Rotor inertia.

            :type: JointVector
         )")
        .def_rw("f_norm", &SingularPerturbationParameter::f_norm,
                R"(
                      The cutoff frequencies for the PT1 and DT1 filters applied to the torque signals to calculate tau and tau_dot.

                      :type: JointArray
                   )")
        .def(
            "__str__",
            [](const SingularPerturbationParameter& param) {
                std::ostringstream ss;
                ss << "SingularPerturbationParameter:\n"
                   << "K:\n"
                   << param.K << "\n"
                   << "D:\n"
                   << param.D << "\n"
                   << "f_norm:\n"
                   << param.f_norm;
                return ss.str();
            },
            R"(
            Returns a string representation of the `SingularPerturbationParameter` object.
         )");
}

template <int DOF>
inline void add_FrictionCompParameter(nb::module_& m) {
    using FrictionCompParameter = orc::control::FrictionCompParameter<DOF>;
    nb::class_<FrictionCompParameter>(m, "FrictionCompParameter")
        .def(nb::init<>(),
             R"(
            Constructs a new `FrictionCompParameter` object with default values of zero for `L=0` and `B=0`, and an array for `f_cutoff_norm=1`.
         )")
        .def(nb::init<typename FrictionCompParameter::JointVector,
                      typename FrictionCompParameter::JointVector,
                      typename FrictionCompParameter::JointArray>(),
             nb::arg("L_fric"), nb::arg("B_fric"), nb::arg("f_cutoff_norm"),
             R"(
            Constructs a new `FrictionCompParameter` object with the given values of `L`, `B`, and `f_cutoff_norm`.

            :param L_fric: Positive controller gain for the viscous friction compensation.
            :type L_fric: JointVector
            :param B_fric: Moments of inertia of the rotors.
            :type B_fric: JointVector
            :param f_cutoff_norm: The cutoff frequencies for the PT1 and DT1 filters applied to the motor torque signals and the rotor position signals theta.
            :type f_cutoff_norm: JointArray
         )")
        .def_rw("L", &FrictionCompParameter::L,
                R"(
                      Positive controller gain for the viscous friction compensation.

                      :type: JointVector
                   )")
        .def_rw("B", &FrictionCompParameter::B,
                R"(
                      Moments of inertia of the rotors.

                      :type: JointVector
                   )")
        .def_rw("f_cutoff_norm", &FrictionCompParameter::f_norm,
                R"(
                      The cutoff frequencies for the PT1 and DT1 filters applied to the motor torque signals and the rotor position signals theta.

                      :type: JointArray
                   )")
        .def(
            "__str__",
            [](const FrictionCompParameter& param) {
                std::ostringstream ss;
                ss << "FrictionCompParameter:\n"
                   << "L:\n"
                   << param.L << "\n"
                   << "B:\n"
                   << param.B << "\n"
                   << "f_norm:\n"
                   << param.f_norm;
                return ss.str();
            },
            R"(
            Returns a string representation of the `FrictionCompParameter` object.
         )");
}

template <int DOF>
inline void add_GravityCompensationParameter(nb::module_& m) {
    using GravityCompParameter = orc::control::GravityCompParameter<DOF>;
    nb::class_<GravityCompParameter>(m, "GravityCompParameter")
        .def(nb::init<>(),
             R"(
            Constructs a new `GravityCompParameter` object with a default value of zero for `D`.
         )")
        .def(nb::init<typename GravityCompParameter::JointMatrix>(), nb::arg("D"),
             R"(
            Constructs a new `GravityCompParameter` object with the given value of `D`.

            :param D: Viscous damping matrix.
            :type D: JointMatrix
         )")
        .def_rw("D", &GravityCompParameter::D,
                R"(
                      Viscous damping matrix.

                      :type: JointMatrix
                   )")
        .def(
            "__str__",
            [](const GravityCompParameter& param) {
                std::ostringstream ss;
                ss << "GravityCompParameter:\n"
                   << "D:\n"
                   << param.D;
                return ss.str();
            },
            R"(
            Returns a string representation of the `GravityCompParameter` object.
         )");
}

template <int DOF>
inline void add_HybridForceMotionParameter(nb::module_& m) {
    using HybridForceMotionParameter = orc::control::HybridForceMotionParameter<DOF>;
    using Matrix3 = HybridForceMotionParameter::Matrix3;
    using JointMatrix = HybridForceMotionParameter::JointMatrix;
    using Array3D = HybridForceMotionParameter::Array3D;

    nb::class_<HybridForceMotionParameter>(m, "HybridForceMotionParameter")
        .def(nb::init<Matrix3, Matrix3, Matrix3, Matrix3, Matrix3, Matrix3, double, JointMatrix,
                      JointMatrix, Array3D>(),
             "KPf"_a, "KIf"_a, "KP"_a, "KD"_a, "KI"_a, "KOmega"_a, "Ko"_a, "Kpn"_a, "Kdn"_a,
             "f_c_force_norm"_a)
        .def(nb::init<bool>(), "simulation"_a = true)
        .def_rw("KPf", &HybridForceMotionParameter::KPf)
        .def_rw("KIf", &HybridForceMotionParameter::KIf)
        .def_rw("KP", &HybridForceMotionParameter::KP)
        .def_rw("KD", &HybridForceMotionParameter::KD)
        .def_rw("KI", &HybridForceMotionParameter::KI)
        .def_rw("KOmega", &HybridForceMotionParameter::KOmega)
        .def_rw("Ko", &HybridForceMotionParameter::Ko)
        .def_rw("Kpn", &HybridForceMotionParameter::Kpn)
        .def_rw("Kdn", &HybridForceMotionParameter::Kdn)
        .def_rw("f_c_force_norm", &HybridForceMotionParameter::f_c_force_norm);
}

// CoulombFrictionCompParameter
template <int DOF>
inline void add_CoulombFrictionCompParameter(nb::module_& m) {
    using CoulombFrictionCompParameter = orc::control::CoulombFrictionCompParameter<DOF>;
    nb::class_<CoulombFrictionCompParameter>(m, "CoulombFrictionCompParameter")
        .def(nb::init<>())
        .def(nb::init<typename CoulombFrictionCompParameter::JointVector,
                      typename CoulombFrictionCompParameter::JointVector,
                      typename CoulombFrictionCompParameter::JointVector>(),
             nb::arg("Fc"), nb::arg("B"), nb::arg("f_cutoff_norm"))
        .def_rw("Fc", &CoulombFrictionCompParameter::Fc)
        .def_rw("B", &CoulombFrictionCompParameter::B)
        .def_rw("f_cutoff_norm", &CoulombFrictionCompParameter::f_cutoff_norm);
}

// ControllerParameter
template <int DOF>
inline void add_ControllerParameter(nb::module_& m) {
    using ControllerParameter = orc::control::ControllerParameter<DOF>;

    nb::class_<ControllerParameter>(m, "ControllerParameter")
        .def(nb::init<>())
        .def_rw("K0_cart", &ControllerParameter::K0_cart)
        .def_rw("K1_cart", &ControllerParameter::K1_cart)
        .def_rw("K0_N_cart", &ControllerParameter::K0_N_cart)
        .def_rw("K1_N_cart", &ControllerParameter::K1_N_cart)
        .def_rw("K0_joint", &ControllerParameter::K0_joint)
        .def_rw("K1_joint", &ControllerParameter::K1_joint)
        .def_rw("KI_joint", &ControllerParameter::KI_joint)
        .def_rw("KP_PDP", &ControllerParameter::KP_PDP)
        .def_rw("KD_PDP", &ControllerParameter::KD_PDP)
        .def_rw("B", &ControllerParameter::B)
        .def_rw("K_sp", &ControllerParameter::K_sp)
        .def_rw("D_sp", &ControllerParameter::D_sp)
        .def_rw("L_fc", &ControllerParameter::L_fc)
        .def_rw("D_gc", &ControllerParameter::D_gc)
        .def_rw("torque_sensor_offset", &ControllerParameter::torque_sensor_offset)
        .def_rw("f_c_fast", &ControllerParameter::f_c_fast)
        .def_rw("f_c_slow", &ControllerParameter::f_c_slow);
}
