#pragma once

#include <orc/util/quatutil.h>
#include <cmath>
#include <vector>
#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/control/ControllerParameter.h"
#include "orc/sig/filter.h"
#include "orc/util/import_eigen.h"

namespace orc::control {
template <int DOF>
struct SingularPerturbationParameter {
    DECLARE_ROBOT_TRAITS_USINGS(DOF)

    JointMatrix K, D;
    JointVector B;
    JointArray f_norm;

    /**
     * @brief Paramters for singular perturbation controller.
     *
     * @param K_sp Controller stiffness matrix
     * @param D_sp Controller damping matrix
     * @param B_sp Vector of rotor inertias
     * @param f_cutoff_norm Cutoff frequency for torque, and torque derivative filters
     */
    SingularPerturbationParameter(JointMatrix K_sp, JointMatrix D_sp, JointVector B_sp,
                                  JointArray f_cutoff_norm) {
        K = K_sp;
        D = D_sp;
        B = B_sp;
        f_norm = f_cutoff_norm;
    }

    /**
     * @brief Default constructor.
     *
     */
    SingularPerturbationParameter()
        : SingularPerturbationParameter(JointMatrix::Zero(), JointMatrix::Zero(),
                                        JointVector::Zero(), JointArray::Ones()) {}
};

/**
 * @brief Implementation of a singular peturbation controller. This controller adapts the mass
 * matrix of the system. The offset mass matrix M_off is calculated by get_M_off() and should be
 * written into robot_data by the registering function.
 *
 * @tparam DOF
 */
template <int DOF>
class SingularPerturbationController : public ControllerBase<DOF> {
    DECLARE_ROBOT_TRAITS_USINGS(DOF)
    using SingularPerturbationParameter = typename orc::control::SingularPerturbationParameter<DOF>;
    using RobotData = orc::robots::RobotData<DOF>;

    sig::PT1<JointArray> tau_filter;
    sig::DT1<JointArray> tau_dot_filter;
    JointArray f_cutoff_norm_;

public:
    using ControllerBase<DOF>::robot_data;
    JointMatrix K_, D_;
    JointVector B;

public:
    /**
     * @brief Construct a new Singular Perturbation Controller object.
     *
     * @param robot_data
     * @param K
     * @param D
     * @param B
     * @param f_cutoff_norm
     */
    SingularPerturbationController(const RobotData& robot_data, JointMatrix K, JointMatrix D,
                                   JointVector B, JointVector f_cutoff_norm)
        : ControllerBase<DOF>(robot_data),
          tau_filter(f_cutoff_norm, robot_data.Ta),
          tau_dot_filter(f_cutoff_norm, robot_data.Ta),
          f_cutoff_norm_(f_cutoff_norm),
          K_(K),
          D_(D),
          B(B) {}

    /**
     * @brief Construct a new Singular Perturbation Controller object.
     *
     * @param robot_data RobotData reference from which robot data will be taken.
     * @param param
     */
    SingularPerturbationController(const RobotData& robot_data,
                                   SingularPerturbationParameter& param)
        : SingularPerturbationController(robot_data, param.K, param.D, param.B, param.f_norm) {}

    JointVector update() {
        JointVector tau_act_f = tau_filter.update(robot_data.tau_sens);
        JointVector tau_dot_f = tau_dot_filter.update(robot_data.tau_sens);

        JointVector tau = -K_ * (tau_act_f - robot_data.tau_primary) - D_ * tau_dot_f;
        return tau;
    }

    void reset(JointVector tau) {
        tau_filter.reset(tau, tau);
        tau_dot_filter.reset(tau, JointArray::Zero());
    }

    void reset() override { reset(JointVector::Zero()); }

    /**
     * @brief Calculates offset mass matrix. This should be written to the robot_data member M_off.
     *
     * @return JointMatrix
     */
    JointMatrix get_M_off() {
        /*	if singular perturbation controller is active, modify mass matrix based on Ott2008, Eq.
         * (5.28) */
        JointMatrix A = (JointMatrix::Identity() + K_);
        JointMatrix b = JointMatrix::Zero();
        b.diagonal() = B;
        Eigen::FullPivLU<JointMatrix> lu(A);
        // Ott 2008 Eq. (5.28): M_off = (I + K)^{-1} · B
        return lu.solve(b);
    }

    void set_K(JointMatrix K) { K_ = K; }

    void set_D(JointMatrix D) { D_ = D; }
};
}  // namespace orc::control
