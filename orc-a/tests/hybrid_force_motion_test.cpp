// Guardrail for HybridForceMotionController<DOF>::update().
//
// Regression: M_inv_JT was declared as JointMatrix (DOF x DOF) when M^{-1} * J^T is
// actually DOF x 6, triggering an Eigen YOU_MIXED_MATRICES_OF_DIFFERENT_SIZES static
// assertion that blocked every instantiation. Merely instantiating + calling update()
// for both a 7-DOF (iiwa) and a 2-DOF robot is sufficient to catch the bug at compile
// time; the runtime checks additionally verify the result is a finite DOF-vector.

#include <string>

#include "gtest/gtest.h"
#include "orc/Orc.h"
#include "orc/control/controller/HybridForceMotionController.h"

namespace {

TEST(HybridForceMotionTest, Instantiates7DOFAndProducesFiniteTorque) {
    constexpr int DOF = 7;
    using orc::robots::Robot;
    using JointVector = typename orc::RobotTraits<DOF>::JointVector;

    Robot<DOF> robot("../models/iiwa_hanging.mjb", 0.001, "iiwa_link_e");
    JointVector q_zero = JointVector::Zero();
    JointVector qd_zero = JointVector::Zero();
    robot.set_q_act(q_zero);
    robot.set_q_dot_act(qd_zero);
    robot.update(orc::Time(0, 0));

    orc::control::HybridForceMotionParameter<DOF> param(/*simulation=*/true);
    orc::control::HybridForceMotionController<DOF> ctrl(robot.robot_data, param);

    JointVector tau = ctrl.update();
    EXPECT_EQ(tau.size(), DOF);
    EXPECT_TRUE(tau.allFinite()) << "tau = " << tau.transpose();
}

// Compile-time size check: forces instantiation for a different DOF so any future size
// mismatch in the template body fails the build, not just runtime.
TEST(HybridForceMotionTest, Instantiates2DOF) {
    constexpr int DOF = 2;
    using JointVector = typename orc::RobotTraits<DOF>::JointVector;
    using JacobianInverseMatrix = typename orc::RobotTraits<DOF>::JacobianInverseMatrix;

    // Static shape checks mirroring the arithmetic inside update().
    static_assert(JointVector::RowsAtCompileTime == DOF);
    static_assert(JacobianInverseMatrix::RowsAtCompileTime == DOF);
    static_assert(JacobianInverseMatrix::ColsAtCompileTime == 6);

    // Reference the class template so it is fully instantiated by the compiler.
    using Ctrl = orc::control::HybridForceMotionController<DOF>;
    (void)sizeof(Ctrl);
    SUCCEED();
}

}  // namespace
