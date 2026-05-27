#pragma once

#include <cmath>
#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/util/import_mujoco.h"

#include "orc/control/ControllerBases.h"
#include "orc/control/ControllerParameter.h"

namespace orc::control {
/**
 * Compute the orientation error vector used by the Cartesian CT
 * controller from the actual and desired rotation matrices.
 *
 * Returns 2*sign(w)*vec(R*R_d^T) so that the result is continuous across
 * the quaternion hemisphere boundary (w=0) and has magnitude close to
 * the rotation angle.
 */
inline orc::Vec3D cartesian_ct_orientation_error(const orc::RotationMatrix& R,
                                                 const orc::RotationMatrix& R_d) {
    // Use 2*sign(w)*vec to keep the error single-valued and well-behaved
    // across the quaternion hemisphere boundary (w=0 at 180 deg).
    auto quat_e = orc::Quatd(R * R_d.transpose());
    double s = (quat_e.w() >= 0.0) ? 1.0 : -1.0;
    return 2.0 * s * quat_e.vec();
}

template <int DOF>
struct CartesianCTParameter {
    DECLARE_ROBOT_TRAITS_USINGS(DOF)
    using ControllerParameter = orc::control::ControllerParameter<DOF>;

    CartesianMatrix K0, K1;
    JointMatrix K0N, K1N;

    CartesianCTParameter() {
        K0 = CartesianMatrix::Zero();
        K0.diagonal() = CartesianVector::Ones();
        K1 = K0;
        K0N = JointMatrix::Zero();
        K0N.diagonal() = JointVector::Ones();
        K1N = K0N;
    }

    CartesianCTParameter(CartesianMatrix K_0, CartesianMatrix K_1, JointMatrix K_0N,
                         JointMatrix K_1N)
        : K0(K_0), K1(K_1), K0N(K_0N), K1N(K_1N) {}

    explicit CartesianCTParameter(ControllerParameter param) {
        K0 = param.K0_cart;
        K1 = param.K1_cart;
        K0N = param.K0_N_cart;
        K1N = param.K1_N_cart;
    }
};

template <int DOF>
class CartesianCTController : public PoseTrackingController<DOF> {
    DECLARE_ROBOT_TRAITS_USINGS(DOF)
    using CartesianCTParameter = orc::control::CartesianCTParameter<DOF>;
    using RobotData = orc::robots::RobotData<DOF>;

private:
    CartesianMatrix K0_, K1_;
    JointMatrix K0N_, K1N_;
    using ControllerBase<DOF>::robot_data;

public:
    CartesianCTController(const RobotData& data, CartesianCTParameter& controller_param)
        : PoseTrackingController<DOF>(data, ControllerType::CARTESIAN_CT),
          K0_(controller_param.K0),
          K1_(controller_param.K1),
          K0N_(controller_param.K0N),
          K1N_(controller_param.K1N) {}

    JointVector update() {
        CartesianVector e;
        CartesianPositionVector p, p_d;

        /* extract quaternion from desired and actual pose, respectively */
        Quatd quat_d(robot_data.pose_d[3], robot_data.pose_d[4], robot_data.pose_d[5],
                     robot_data.pose_d[6]);  // w,x,y,z
        RotationMatrix R_d = quat_d.toRotationMatrix();
        RotationMatrix R = robot_data.H_0_e.template block<3, 3>(0, 0);

        /* control error */
        p = robot_data.H_0_e.template block<3, 1>(0, 3);
        p_d = robot_data.pose_d.template block<3, 1>(0, 0);
        e.template block<3, 1>(0, 0) = p - p_d;
        e.template block<3, 1>(3, 0) = cartesian_ct_orientation_error(R, R_d);

        /* control error - first derivative */
        CartesianVector e_dot = robot_data.x_dot_act - robot_data.x_dot_d;

        /* control error - second derivative */
        CartesianVector e_dotdot =
            robot_data.x_dotdot_d -
            robot_data.x_dotdot_act;  // flipped order: (desired - actual) intended!

        /* new input v */
        JointVector v =
            (robot_data.J_inv * (e_dotdot - K1_ * e_dot - K0_ * e) +
             (JointMatrix::Identity() - robot_data.J_inv * robot_data.J) *
                 (-K1N_ * robot_data.q_dot_act - K0N_ * (robot_data.q_act - robot_data.q_d_NS)));

        /* computed torque controller */
        return (robot_data.M + robot_data.M_off) * v + robot_data.qfrc_bias;
    }

    void set_parameter(CartesianCTParameter param) {
        K0_ = param.K0;
        K1_ = param.K1;
        K0N_ = param.K0N;
        K1N_ = param.K1N;
    }

    void reset() override {}
};

}  // namespace orc::control
