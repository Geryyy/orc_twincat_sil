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
struct FrictionCompParameter : public RobotTraits<DOF> {
    DECLARE_ROBOT_TRAITS_USINGS(DOF)

    JointMatrix L;
    JointMatrix B;
    JointArray f_norm;

    FrictionCompParameter()
        : FrictionCompParameter(JointVector::Zero(), JointVector::Zero(), JointArray::Ones()) {}

    FrictionCompParameter(JointVector L_fc, JointVector B_fc, JointArray f_cutoff_norm) {
        L = JointMatrix::Zero();
        B = JointMatrix::Zero();
        B.diagonal() = B_fc;
        L.diagonal() = L_fc;
        f_norm = f_cutoff_norm;
    }
};

template <int DOF>
class FrictionCompController : public ControllerBase<DOF> {
    DECLARE_ROBOT_TRAITS_USINGS(DOF)
    using RobotData = orc::robots::RobotData<DOF>;
    using FrictionCompParameter = typename orc::control::FrictionCompParameter<DOF>;

    using ControllerBase<DOF>::robot_data;
    sig::PT1<JointArray> theta_filter;
    sig::PT1<JointArray> tau_sensor_filter;
    sig::PT1<JointArray> tau_motor_filter;
    sig::DT1<JointArray> theta_dot_filter;
    JointVector f_cutoff_norm_;

    JointMatrix L_;
    JointMatrix B_;
    JointVector theta_dot_hat;

public:
    FrictionCompController(const RobotData& robot_data, JointMatrix L, JointMatrix B,
                           JointVector f_cutoff_norm)
        : ControllerBase<DOF>(robot_data),
          theta_filter(f_cutoff_norm, robot_data.Ta),
          tau_sensor_filter(f_cutoff_norm, robot_data.Ta),
          tau_motor_filter(f_cutoff_norm, robot_data.Ta),
          theta_dot_filter(f_cutoff_norm, robot_data.Ta),
          f_cutoff_norm_(f_cutoff_norm),
          L_(L),
          B_(B),
          theta_dot_hat(JointVector::Zero()) {}

    FrictionCompController(const RobotData& robot_data, FrictionCompParameter& param)
        : FrictionCompController(robot_data, param.L, param.B, param.f_norm) {}

    JointVector update() {
        // JointVector theta_f = theta_filter.update(theta);
        JointVector theta_dot_f = theta_dot_filter.update(robot_data.q_act);
        JointVector tau_motor_f = tau_motor_filter.update(robot_data.tau_motor);
        JointVector tau_sens_f = tau_sensor_filter.update(robot_data.tau_sens);

        JointVector tau_fm_hat = -L_ * B_ * (theta_dot_f - theta_dot_hat);
        theta_dot_hat +=
            robot_data.Ta.toSec() * B_.inverse() * (tau_motor_f - tau_sens_f - tau_fm_hat);

        return tau_fm_hat;
    }

    void reset(JointVector tau_motor, JointVector tau_sens, JointVector theta) {
        theta_dot_hat = JointVector::Zero();
        tau_motor_filter.reset(tau_motor, tau_motor);
        tau_sensor_filter.reset(tau_sens, tau_sens);
        theta_dot_filter.reset(theta, JointArray::Zero());
        theta_filter.reset(theta, theta);
    }

    void reset() override { reset(JointVector::Zero(), JointVector::Zero(), JointVector::Zero()); }

    void set_L(JointMatrix L) { L_ = L; }

    void set_B(JointMatrix B) { B_ = B; }
};

}  // namespace orc::control
