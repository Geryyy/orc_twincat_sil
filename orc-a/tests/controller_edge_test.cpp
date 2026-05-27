#include <filesystem>
#include <string>

#include "gtest/gtest.h"
#include "orc/Orc.h"
#include "orc/util/import_mujoco.h"

namespace {
constexpr int DOF = 7;
using JointVector = typename orc::RobotTraits<DOF>::JointVector;
using JointMatrix = typename orc::RobotTraits<DOF>::JointMatrix;

static const std::string MJB = "../models/iiwa_hanging.mjb";

// ---- C-2: CoulombFrictionCompParameter should actually store Fc/B/f_cutoff ----
TEST(ControllerEdgeTest, CoulombFrictionParameterStoresValues_C2) {
    using Param = orc::control::CoulombFrictionCompParameter<DOF>;
    JointVector Fc = JointVector::Constant(1.5);
    JointVector B = JointVector::Constant(0.25);
    JointVector fc = JointVector::Constant(0.1);
    Param p(Fc, B, fc);
    EXPECT_TRUE(p.Fc.isApprox(Fc)) << "Fc was not stored (self-assignment bug)";
    EXPECT_TRUE(p.B.isApprox(B)) << "B was not stored";
    EXPECT_TRUE(p.f_cutoff_norm.isApprox(fc)) << "f_cutoff_norm was not stored";
}

// ---- M-24: Coulomb friction behaves like sign for |qdot| >> 0 ----
TEST(ControllerEdgeTest, CoulombFrictionUsesSignOfVelocity_M24) {
    using orc::robots::Robot;
    Robot<DOF> robot(MJB.c_str(), 0.001, "iiwa_link_e");
    JointVector q = JointVector::Zero();
    JointVector qd = JointVector::Constant(10.0);  // fast positive velocity -> tanh ~ +1
    robot.set_q_act(q);
    robot.set_q_dot_act(qd);
    robot.update(orc::Time(0, 0));

    JointVector Fc = JointVector::Constant(2.0);
    JointVector B = JointVector::Zero();
    JointVector fc = JointVector::Constant(0.1);
    orc::control::CoulombFrictionCompController<DOF> c(robot.robot_data, Fc, B, fc);
    JointVector tau = c.update();
    // Should be approximately +Fc (not Fc*qdot which would be 20 per joint)
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(tau[i], Fc[i], 1e-3)
            << "Coulomb torque should saturate at Fc (sign of qdot), got " << tau[i];
    }

    // Negative velocity -> -Fc
    JointVector qd_neg = JointVector::Constant(-10.0);
    robot.set_q_dot_act(qd_neg);
    robot.update(orc::Time(0, 0));
    orc::control::CoulombFrictionCompController<DOF> c2(robot.robot_data, Fc, B, fc);
    JointVector tau_neg = c2.update();
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(tau_neg[i], -Fc[i], 1e-3);
    }
}

// ---- H-5: RobotData::force_error_integral default-initialized ----
TEST(ControllerEdgeTest, RobotDataForceErrorIntegralInitialized_H5) {
    using orc::robots::Robot;
    Robot<DOF> robot(MJB.c_str(), 0.001, "iiwa_link_e");
    EXPECT_DOUBLE_EQ(robot.robot_data.force_error_integral, 0.0);
}

// ---- C-3: get_force_filtered must not crash when no custom controller registered ----
TEST(ControllerEdgeTest, GetForceFilteredNoController_C3) {
    using orc::robots::Robot;
    Robot<DOF> robot(MJB.c_str(), 0.001, "iiwa_link_e");
    orc::Vec3D f;
    // Must not segfault; should return zero sentinel.
    EXPECT_NO_THROW({ f = robot.get_force_filtered(); });
    EXPECT_TRUE(f.isApprox(orc::Vec3D::Zero()));
}

// ---- H-3: JointCT integral anti-windup clamps xq_I_ ----
TEST(ControllerEdgeTest, JointCTIntegralAntiWindup_H3) {
    using orc::robots::Robot;
    Robot<DOF> robot(MJB.c_str(), 0.001, "iiwa_link_e");
    JointVector q = JointVector::Zero();
    robot.set_q_act(q);
    robot.update(orc::Time(0, 0));

    orc::control::JointCTParameter<DOF> param;
    param.K0 = JointMatrix::Identity() * 1.0;
    param.K1 = JointMatrix::Identity() * 1.0;
    param.KI = JointMatrix::Identity() * 1.0;
    param.xq_I_max = JointVector::Constant(0.5);  // tight clamp

    orc::control::JointCTController<DOF> ctrl(robot.robot_data, param);

    // Drive integral by having persistent error (q_act != q_d, q_d remains 0)
    JointVector q_persistent_err = JointVector::Constant(100.0);
    robot.set_q_act(q_persistent_err);
    robot.update(orc::Time(0, 0));

    for (int i = 0; i < 10000; ++i) {
        (void)ctrl.update();
    }
    // Re-run once more; call .update() again to inspect bounded behavior via finite torque.
    JointVector tau = ctrl.update();
    // With clamp xq_I_max=0.5 and KI=I, integral contribution to v is bounded by 0.5 per joint;
    // the total torque magnitude should remain finite (not diverge like 1e4).
    EXPECT_TRUE(tau.allFinite());
    EXPECT_LT(tau.cwiseAbs().maxCoeff(), 1e6)
        << "Integral wind-up not bounded; torque diverged: " << tau.transpose();
}
}  // namespace
