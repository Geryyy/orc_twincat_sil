#pragma once

#include <cmath>
#include <limits>
#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/sig/filter.h"
#include "orc/util/Logger.h"
#include "orc/util/import_eigen.h"
#include "orc/util/import_mujoco.h"

#include "orc/control/ControllerBases.h"
#include "orc/control/ControllerParameter.h"

namespace orc::control {
template <int DOF>
struct HybridForceMotionParameter {
    using JointMatrix = typename orc::RobotTraits<DOF>::JointMatrix;
    using Matrix3 = Eigen::Matrix<double, 3, 3>;
    using Array3D = Eigen::Array<double, 3, 1>;

    Matrix3 KPf, KIf;               // Force control gains
    Matrix3 KDf = Matrix3::Zero();  // Force control velocity damping
    Matrix3 KP, KD, KI;             // Motion control gains
    Matrix3 KOmega;                 // Rotational motion control gains
    double Ko;                      // Orientation motion control gain
    JointMatrix Kpn, Kdn;           // Nullspace gains
    Array3D f_c_force_norm;         // Normalized cutoff frequency for force measurement PT1 filter

    HybridForceMotionParameter(Matrix3 KPf, Matrix3 KIf, Matrix3 KP, Matrix3 KD, Matrix3 KI,
                               Matrix3 KOmega, double Ko, JointMatrix Kpn, JointMatrix Kdn,
                               Array3D f_c_force_norm)
        : KPf(KPf),
          KIf(KIf),
          KP(KP),
          KD(KD),
          KI(KI),
          KOmega(KOmega),
          Ko(Ko),
          Kpn(Kpn),
          Kdn(Kdn),
          f_c_force_norm(f_c_force_norm) {}

    HybridForceMotionParameter(bool simulation = true) {
        if (simulation) {
            // Default simulation parameters
            KPf = Matrix3::Identity() * 1.;
            KIf = Matrix3::Identity() * 8.;

            KP = Matrix3::Identity() * 550.;
            KD = Matrix3::Identity() * 120.;
            KI = Matrix3::Identity() * 0.;

            Ko = 64000.;
            KOmega = Matrix3::Identity() * 1600.;

            Kpn = JointMatrix::Identity() * 50.;
            Kdn = JointMatrix::Identity() * 20.;
            f_c_force_norm << .05, .05, .05;
        } else {
            // Default real robot parameters
            KPf = Matrix3::Identity() * 1.0;
            KIf = Matrix3::Identity() * 1.0;
            KDf = Matrix3::Identity() * 300.;

            KP = Matrix3::Identity() * 1600.;
            KD = Matrix3::Identity() * 80.;
            KI = Matrix3::Zero();

            Ko = 1600.;
            KOmega = Matrix3::Identity() * 80.;

            Kpn = JointMatrix::Identity() * 10.;
            Kdn = JointMatrix::Identity() * 100.;
            f_c_force_norm = 10 * Array3D::Ones() * 2 * 0.000125;
        }
    }
};

/**
 * @brief Implementation of a Hybrid force/motion controller as described in Weingartshofer et al.,
 * 2024
 *
 * @tparam DOF
 */
template <int DOF>
class HybridForceMotionController : public ControllerBase<DOF> {
    DECLARE_ROBOT_TRAITS_USINGS(DOF)
    using RobotData = orc::robots::RobotData<DOF>;
    using Vec3D = orc::Vec3D;
    using Array3D = Eigen::Array<double, 3, 1>;  // Used for filtering

    static constexpr double FORCE_MEASURED_THRESHOLD = 0.1;
    static constexpr double POS_ERR_I_MAX =
        10.0;  // Anti-windup clamp for position-error integral [m*s]
    static constexpr double M_COND_MIN_DET = 1e-9;  // Mass-matrix invertibility guard (|det(M)|)
    static constexpr double J_COND_MAX =
        1e6;  // Nullspace projection guard (Jacobian condition number)

    HybridForceMotionParameter<DOF> param;
    using ControllerBase<DOF>::robot_data;
    orc::sig::PT1<Array3D> filter_force;
    bool force_filter_initialized = false;
    Vec3D f_err_I;    // Integral of force error
    Vec3D pos_err_I;  // Integral of position error
    JointVector tau_prev_ =
        JointVector::Zero();  // previous torque for fallback when inversion/projection unsafe

    // Selection matrices
    const Eigen::Matrix<double, 6, 3> Y_f = [] {
        Eigen::Matrix<double, 6, 3> m;
        m.setZero();
        m(2, 2) = 1.0;
        return m;
    }();

    const Eigen::Matrix<double, 6, 6> Y_v = [] {
        Eigen::Matrix<double, 6, 6> m;
        m.setIdentity();
        m(2, 2) = 0.0;
        return m;
    }();

public:
    // For debugging
    JointVector tau_1_ = JointVector::Zero();
    JointVector tau_2_ = JointVector::Zero();
    Vec3D v_f_ = Vec3D::Zero();
    Vec6D v_c_breve_ = Vec6D::Zero();
    Vec3D v_c_breve_omega = Vec3D::Zero();
    Vec3D v_c_breve_quat = Vec3D::Zero();
    Vec3D omega_breve_ = Vec3D::Zero();
    Vec3D e0_breve_ = Vec3D::Zero();

    Vec3D force_filtered;  // Filtered force measurement

    HybridForceMotionController(const RobotData& data, HybridForceMotionParameter<DOF> param)
        : ControllerBase<DOF>(data, ControllerType::HYBRID_FORCE_MOTION),
          param(param),
          f_err_I(Vec3D::Zero()),
          pos_err_I(Vec3D::Zero()),
          filter_force(param.f_c_force_norm, data.Ta),
          force_filtered(Vec3D::Zero()) {}

    JointVector update() {
        // Filter force measurement
        Vec3D force_d;
        Array3D force_measurement;
        force_d << 0, 0, robot_data.force_d;
        force_measurement << 0, 0, robot_data.force_compensated(2);
        if (force_filter_initialized) {
            force_filtered = filter_force.update(force_measurement.array()).matrix();
        } else {
            filter_force.reset(force_measurement.array(), force_measurement.array());
            force_filtered = force_measurement;
            force_filter_initialized = true;
        }

        CartesianMatrix T_BC;  // (36)
        T_BC.setIdentity();
        T_BC.topLeftCorner(3, 3) = robot_data.H_0_e.topLeftCorner(3, 3);

        Quatd quat_d = Quatd(robot_data.pose_d(3), robot_data.pose_d(4), robot_data.pose_d(5),
                             robot_data.pose_d(6));
        Quatd quat_act_inv = Quatd(robot_data.pose_act(3), -robot_data.pose_act(4),
                                   -robot_data.pose_act(5), -robot_data.pose_act(6));
        Quatd quat_product = quat_d * quat_act_inv;
        Vec3D e_0 = quat_product.vec();  // This is equal to e_0 from the paper

        // Motion control (29)
        Vec3D p_breve = robot_data.H_0_e.topLeftCorner(3, 3).transpose() *
                        (robot_data.pose_d.topRows(3) - robot_data.pose_act.topRows(3));
        Vec3D p_dot_breve = robot_data.H_0_e.topLeftCorner(3, 3).transpose() *
                            (robot_data.x_dot_d.topRows(3) - robot_data.x_dot_act.topRows(3));
        Vec3D omega_breve = T_BC.transpose().bottomRightCorner(3, 3) *
                            (robot_data.x_dot_d.bottomRows(3) - robot_data.x_dot_act.bottomRows(3));
        Vec3D e0_breve = T_BC.transpose().bottomRightCorner(3, 3) * e_0;

        // Force control (39)
        Vec3D p_dot_f =
            robot_data.H_0_e.topLeftCorner(3, 3).transpose() * (-robot_data.x_dot_act.topRows(3));
        Vec3D v_f = force_d + param.KPf * (force_d - force_filtered) + param.KIf * f_err_I +
                    param.KDf * p_dot_f;

        Vec6D v_c_breve;
        v_c_breve.head<3>() = robot_data.x_dotdot_d.topRows(3) + param.KD * p_dot_breve +
                              param.KP * p_breve + param.KI * pos_err_I;
        v_c_breve.tail<3>() =
            robot_data.x_dotdot_d.bottomRows(3) + param.KOmega * omega_breve + param.Ko * e0_breve;

        // Control input (35)
        JacobianInverseMatrix J_pinv =
            robot_data.J_inv;  // Use the damped pseudoinverse from robot data

        // H-6: Guard against singular mass matrix before inversion. Use LDLT solve when
        // well-conditioned.
        double M_det = robot_data.M.determinant();
        if (std::abs(M_det) < M_COND_MIN_DET) {
            orc::log::write_error(
                "HybridForceMotionController: mass matrix near-singular, returning previous "
                "torque");
            return tau_prev_;
        }

        // Instead of computing M^{-1} explicitly then multiplying, solve
        // M * X = J^T for X = M^{-1} J^T. Numerically more stable and
        // typically faster than forming the inverse.
        // M_inv_JT = M^{-1} * J^T has shape DOF x 6 (J is 6 x DOF), so its type is
        // JacobianInverseMatrix, not JointMatrix (DOF x DOF). Using JointMatrix here was a
        // compile-time size mismatch that blocked instantiation for every DOF.
        Eigen::LDLT<JointMatrix> M_ldlt(robot_data.M);
        JacobianInverseMatrix M_inv_JT = M_ldlt.solve(robot_data.J.transpose());

        JointVector a = J_pinv * T_BC * Y_v * v_c_breve + M_inv_JT * T_BC * Y_f * v_f -
                        J_pinv * robot_data.J_dot * robot_data.q_dot_act;

        // Inverse dynamics control law (34)
        Vec6D f_hat;
        f_hat.setZero();
        f_hat.head<3>() = force_filtered;
        JointVector tau_1 =
            robot_data.M * a + robot_data.qfrc_bias - robot_data.J.transpose() * f_hat;

        // M-11: Nullspace control (40) — bound contribution when Jacobian condition number high.
        JointVector tau_2;
        Eigen::JacobiSVD<JacobianMatrix> svd(robot_data.J);
        double sigma_max = svd.singularValues()(0);
        double sigma_min = svd.singularValues()(svd.singularValues().size() - 1);
        double cond =
            (sigma_min > 0.0) ? (sigma_max / sigma_min) : std::numeric_limits<double>::infinity();
        if (cond > J_COND_MAX || !std::isfinite(cond)) {
            orc::log::write_error(
                "HybridForceMotionController: Jacobian ill-conditioned, disabling nullspace "
                "contribution");
            tau_2 = JointVector::Zero();
        } else {
            tau_2 = robot_data.M * (JointMatrix::Identity() - J_pinv * robot_data.J) *
                    (-param.Kdn * robot_data.q_dot_act -
                     param.Kpn * (robot_data.q_act - robot_data.q_d_NS));
        }

        // Update integrators
        // force error anti-windup
        if (force_filtered(2) > FORCE_MEASURED_THRESHOLD) {
            // when measured force is lower than threshold (i.e. no contact) stop the integrator
            f_err_I = f_err_I + (force_d - force_filtered) * robot_data.Ta.toSec();
        }
        // H-4: position-error integral anti-windup (mirror of force-error guard): clamp
        // element-wise.
        pos_err_I = pos_err_I + p_breve * robot_data.Ta.toSec();
        pos_err_I = pos_err_I.cwiseMax(-POS_ERR_I_MAX).cwiseMin(POS_ERR_I_MAX);

        // For debugging
        tau_1_ = tau_1;
        tau_2_ = tau_2;
        v_f_ = v_f;
        v_c_breve_ = v_c_breve;
        v_c_breve_omega = param.KOmega * omega_breve;
        v_c_breve_quat = param.Ko * e0_breve;
        omega_breve_ = omega_breve;
        e0_breve_ = e0_breve;

        tau_prev_ = tau_1 + tau_2;
        return tau_1 + tau_2;
    }

    Vec3D get_integral_force_error() { return f_err_I; }

    /**
     * @brief Resets integrators and force filter to zero.
     *
     */
    void reset() override {
        if (force_filter_initialized) {
            orc::log::write_error("Reset HFM controller");
            f_err_I.setZero();
            pos_err_I.setZero();
            force_filtered.setZero();
            filter_force.reset(Vec3D::Zero(), Vec3D::Zero());
            force_filter_initialized = false;
        }
    }
};
}  // namespace orc::control
