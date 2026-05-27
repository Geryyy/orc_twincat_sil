#pragma once

#include <orc/control/ControllerParameter.h>
#include <orc/util/quatutil.h>
#include <cmath>
#include <vector>
#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/control/ControllerBases.h"
#include "orc/robots/RobotData.h"
#include "orc/sig/filter.h"
#include "orc/util/import_eigen.h"

namespace orc::control {
template <int DOF>
struct CoulombFrictionCompParameter : public RobotTraits<DOF> {
    DECLARE_ROBOT_TRAITS_USINGS(DOF)

    JointVector Fc;
    JointVector B;
    JointVector f_cutoff_norm;

    CoulombFrictionCompParameter()
        : CoulombFrictionCompParameter(JointVector::Zero(), JointVector::Zero(),
                                       JointVector::Zero()) {}

    CoulombFrictionCompParameter(JointVector Fc_, JointVector B_, JointVector f_cutoff_norm_)
        : Fc(Fc_), B(B_), f_cutoff_norm(f_cutoff_norm_) {}
};

template <int DOF>
class CoulombFrictionCompController : public ControllerBase<DOF> {
    DECLARE_ROBOT_TRAITS_USINGS(DOF)
    using RobotData = orc::robots::RobotData<DOF>;
    using CoulombFrictionCompParameter = typename orc::control::CoulombFrictionCompParameter<DOF>;

    using ControllerBase<DOF>::robot_data;
    JointVector Fc;
    JointVector B;

    // private:
    //     JointVector theta_dot_filtered;

public:
    CoulombFrictionCompController(const RobotData& robot_data, JointVector Fc, JointVector B,
                                  JointVector f_cutoff_norm)
        : ControllerBase<DOF>(robot_data), Fc(Fc), B(B) {}

    CoulombFrictionCompController(const RobotData& robot_data, CoulombFrictionCompParameter& param)
        : ControllerBase<DOF>(robot_data), Fc(param.Fc), B(param.B) {}

    JointVector update() {
        // Coulomb friction uses a smoothed sign (tanh) of joint velocity; viscous term is linear in
        // velocity.
        constexpr double k_smooth = 100.0;
        JointVector sgn_qdot = (k_smooth * robot_data.q_dot_act.array()).tanh().matrix();
        JointVector tau_fm_hat = Fc.cwiseProduct(sgn_qdot) + B.cwiseProduct(robot_data.q_dot_act);
        return tau_fm_hat;
    }

    void reset() override {}
};

}  // namespace orc::control
