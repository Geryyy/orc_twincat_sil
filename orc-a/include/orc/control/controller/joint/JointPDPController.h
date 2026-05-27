#pragma once

#include <cmath>
#include "orc/RobotTraits.h"
#include "orc/util/import_mujoco.h"

#include "orc/util/Angle.h"

/* model headers */
#include <orc/control/ControllerParameter.h>
#include "orc/control/ControllerBases.h"

namespace orc::control {
template <int DOF>
struct JointPDPParameter {
    DECLARE_ROBOT_TRAITS_USINGS(DOF)
    using ControllerParameter = typename orc::control::ControllerParameter<DOF>;

    JointMatrix Kp, Kd;

    JointPDPParameter() {
        Kp = JointMatrix::Zero();
        Kp.diagonal() = JointVector::Ones();
        Kd = Kp;
    }

    JointPDPParameter(JointMatrix K_p, JointMatrix K_d) : Kp(K_p), Kd(K_d) {}

    explicit JointPDPParameter(ControllerParameter param) {
        Kp = param.K0_joint;
        Kd = param.K1_joint;
    }
};

template <int DOF>
class JointPDPController : public JointTrackingController<DOF> {
public:
    DECLARE_ROBOT_TRAITS_USINGS(DOF)
    using JointPDPParameter = typename orc::control::JointPDPParameter<DOF>;
    using RobotData = orc::robots::RobotData<DOF>;

private:
    JointMatrix Kp_, Kd_;
    using ControllerBase<DOF>::robot_data;

public:
    JointPDPController(const RobotData& data, JointPDPParameter& controller_param)
        : JointTrackingController<DOF>(data), Kp_(controller_param.Kp), Kd_(controller_param.Kd) {}

    JointVector update() {
        /* control error */
        JointVector e = (robot_data.q_act - robot_data.q_d).unaryExpr(orc::util::wrap_to_pi);
        JointVector e_dot = robot_data.q_dot_act - robot_data.q_dot_d;

        /* computed torque controller */
        JointVector tau = (robot_data.M + robot_data.M_off) * robot_data.q_dotdot_d +
                          robot_data.qfrc_bias - Kp_ * e - Kd_ * e_dot;

        return tau;
    }

    void reset() {}

    void set_parameter(JointPDPParameter param) {
        Kp_ = param.Kp;
        Kd_ = param.Kd;
    }
};
}  // namespace orc::control
