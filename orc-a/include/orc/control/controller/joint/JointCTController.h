#pragma once

#include <cmath>
#include "orc/RobotTraits.h"
#include "orc/util/Angle.h"
#include "orc/util/import_mujoco.h"

/* model headers */
#include <orc/control/ControllerParameter.h>
#include "orc/control/ControllerBases.h"

namespace orc::control {
template <int DOF>
struct JointCTParameter {
    DECLARE_ROBOT_TRAITS_USINGS(DOF)
    using ControllerParameter = typename orc::control::ControllerParameter<DOF>;

    JointMatrix K0, K1, KI;
    JointVector xq_I_max =
        JointVector::Constant(1.0e6); /**< Element-wise anti-windup clamp for integrator */

    JointCTParameter() {
        K0 = JointMatrix::Zero();
        K0.diagonal() = JointVector::Ones();
        K1 = K0;
        KI = JointMatrix::Zero();
    }

    JointCTParameter(JointMatrix K_0, JointMatrix K_1, JointMatrix K_I)
        : K0(K_0), K1(K_1), KI(K_I) {}

    explicit JointCTParameter(ControllerParameter param) {
        K0 = param.K0_joint;
        K1 = param.K1_joint;
        KI = param.KI_joint;
    }
};

template <int DOF>
class JointCTController : public JointTrackingController<DOF> {
public:
    DECLARE_ROBOT_TRAITS_USINGS(DOF)
    using JointCTParameter = typename orc::control::JointCTParameter<DOF>;
    using RobotData = orc::robots::RobotData<DOF>;

private:
    JointVector xq_I_;
    JointVector xq_I_max_;
    JointMatrix K0_, K1_, KI_;
    using ControllerBase<DOF>::robot_data;

public:
    JointCTController(const RobotData& data, JointCTParameter& controller_param)
        : JointTrackingController<DOF>(data, ControllerType::JOINT_CT),
          K0_(controller_param.K0),
          K1_(controller_param.K1),
          KI_(controller_param.KI),
          xq_I_(JointVector::Zero()),
          xq_I_max_(controller_param.xq_I_max) {}

    JointVector update() {
        /* control error */
        JointVector e = (robot_data.q_act - robot_data.q_d).unaryExpr(orc::util::wrap_to_pi);
        JointVector e_dot = robot_data.q_dot_act - robot_data.q_dot_d;

        /* new input v */
        JointVector v = robot_data.q_dotdot_d - K0_ * e - K1_ * e_dot - KI_ * xq_I_;

        /* integrator with element-wise anti-windup clamp */
        xq_I_ = xq_I_ + robot_data.Ta.toSec() * e;
        xq_I_ = xq_I_.cwiseMax(-xq_I_max_).cwiseMin(xq_I_max_);

        /* computed torque controller */
        return (robot_data.M + robot_data.M_off) * v + robot_data.qfrc_bias;
    }

    void reset() override { xq_I_ = JointVector::Zero(); }

    void set_parameter(JointCTParameter param) {
        K0_ = param.K0;
        K1_ = param.K1;
        KI_ = param.KI;
        xq_I_max_ = param.xq_I_max;
    }
};
}  // namespace orc::control
