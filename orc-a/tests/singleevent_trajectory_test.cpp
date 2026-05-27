/**
 * @file singleevent_trajectory_test.cpp
 * @brief Tests for NullspaceTrajectory, JointCtrParamTrajectory, CartesianCtrParamTrajectory.
 */
#include <gtest/gtest.h>
#include "orc/Orc.h"
#include "orc/control/controller/cartesian/CartesianCTController.h"
#include "orc/control/controller/joint/JointCTController.h"
#include "orc/trajectory/singleevent/CartesianCtrParamTrajectory.h"
#include "orc/trajectory/singleevent/JointCtrParamTrajectory.h"
#include "orc/trajectory/singleevent/NullspaceTrajectory.h"
#include "orc/util/Logger.h"
namespace {
constexpr int DOF = 7;
using Time = orc::Time;
using JointVector = orc::RobotTraits<DOF>::JointVector;
using JointMatrix = orc::RobotTraits<DOF>::JointMatrix;
using CartesianMatrix = orc::CartesianMatrix;
using TrajectoryType = orc::trajectory::TrajectoryType;
using NullspaceTrajectory = orc::trajectory::NullspaceTrajectory<DOF>;
using JointCtrParamTrajectory = orc::trajectory::JointCtrParamTrajectory<DOF>;
using CartesianCtrParamTrajectory = orc::trajectory::CartesianCtrParamTrajectory<DOF>;
using JointCTParameter = orc::control::JointCTParameter<DOF>;
using CartesianCTParameter = orc::control::CartesianCTParameter<DOF>;
using TrajectoryPointStorage = orc::trajectory::TrajectoryPointStorage<DOF>;
// ===========================================================================
// NullspaceTrajectory
// ===========================================================================
TEST(SingleEventTest, Nullspace_TrajectoryType) {
    JointVector q_ns = JointVector::Ones() * 0.5;
    NullspaceTrajectory traj(Time(1.0), q_ns);
    EXPECT_EQ(traj.get_trajectory_type(), TrajectoryType::NULLSPACE);
}
TEST(SingleEventTest, Nullspace_GetStartTime) {
    JointVector q_ns = JointVector::Zero();
    NullspaceTrajectory traj(Time(2.5), q_ns);
    EXPECT_EQ(traj.get_start_time(), Time(2.5));
}
TEST(SingleEventTest, Nullspace_GetNullspaceJointState) {
    JointVector q_ns;
    q_ns << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
    NullspaceTrajectory traj(Time(0.0), q_ns);
    JointVector recovered = traj.get_nullspace_joint_state();
    for (int i = 0; i < DOF; ++i) {
        EXPECT_DOUBLE_EQ(recovered(i), q_ns(i));
    }
}
TEST(SingleEventTest, Nullspace_SaveStateReturnsDefault) {
    JointVector q_ns = JointVector::Ones();
    NullspaceTrajectory traj(Time(0.0), q_ns);
    TrajectoryPointStorage saved = traj.save_state(Time(1.0));
    EXPECT_EQ(saved.previous_type, TrajectoryType::INVALID);
}
TEST(SingleEventTest, Nullspace_InitDoesNotCrash) {
    JointVector q_ns = JointVector::Ones();
    NullspaceTrajectory traj(Time(0.0), q_ns);
    traj.init();
    TrajectoryPointStorage saved;
    traj.init(saved);
    // Just ensure no crash
    EXPECT_EQ(traj.get_trajectory_type(), TrajectoryType::NULLSPACE);
}
TEST(SingleEventTest, Nullspace_ZeroJointConfig) {
    JointVector q_ns = JointVector::Zero();
    NullspaceTrajectory traj(Time(0.0), q_ns);
    JointVector recovered = traj.get_nullspace_joint_state();
    EXPECT_NEAR(recovered.norm(), 0.0, 1e-12);
}
// ===========================================================================
// JointCtrParamTrajectory
// ===========================================================================
TEST(SingleEventTest, JointCtrParam_TrajectoryType) {
    JointCTParameter param;
    JointCtrParamTrajectory traj(Time(1.0), param);
    EXPECT_EQ(traj.get_trajectory_type(), TrajectoryType::JOINT_CTR_PARAM);
}
TEST(SingleEventTest, JointCtrParam_GetStartTime) {
    JointCTParameter param;
    JointCtrParamTrajectory traj(Time(3.0), param);
    EXPECT_EQ(traj.get_start_time(), Time(3.0));
}
TEST(SingleEventTest, JointCtrParam_GetParameterRoundTrip) {
    JointCTParameter param;
    param.K0 = JointMatrix::Identity() * 100;
    param.K1 = JointMatrix::Identity() * 50;
    param.KI = JointMatrix::Identity() * 10;
    JointCtrParamTrajectory traj(Time(0.0), param);
    JointCTParameter recovered = traj.get_parameter();
    for (int i = 0; i < DOF; ++i) {
        EXPECT_DOUBLE_EQ(recovered.K0(i, i), 100.0);
        EXPECT_DOUBLE_EQ(recovered.K1(i, i), 50.0);
        EXPECT_DOUBLE_EQ(recovered.KI(i, i), 10.0);
    }
}
TEST(SingleEventTest, JointCtrParam_DefaultGains) {
    JointCTParameter param;  // default: K0=I, K1=I, KI=0
    JointCtrParamTrajectory traj(Time(0.0), param);
    JointCTParameter recovered = traj.get_parameter();
    for (int i = 0; i < DOF; ++i) {
        EXPECT_DOUBLE_EQ(recovered.K0(i, i), 1.0);
        EXPECT_DOUBLE_EQ(recovered.K1(i, i), 1.0);
        EXPECT_DOUBLE_EQ(recovered.KI(i, i), 0.0);
    }
}
TEST(SingleEventTest, JointCtrParam_SaveStateReturnsDefault) {
    JointCTParameter param;
    JointCtrParamTrajectory traj(Time(0.0), param);
    TrajectoryPointStorage saved = traj.save_state(Time(1.0));
    EXPECT_EQ(saved.previous_type, TrajectoryType::INVALID);
}
// ===========================================================================
// CartesianCtrParamTrajectory
// ===========================================================================
TEST(SingleEventTest, CartCtrParam_TrajectoryType) {
    CartesianCTParameter param;
    CartesianCtrParamTrajectory traj(Time(1.0), param);
    EXPECT_EQ(traj.get_trajectory_type(), TrajectoryType::CART_CTR_PARAM);
}
TEST(SingleEventTest, CartCtrParam_GetStartTime) {
    CartesianCTParameter param;
    CartesianCtrParamTrajectory traj(Time(4.0), param);
    EXPECT_EQ(traj.get_start_time(), Time(4.0));
}
TEST(SingleEventTest, CartCtrParam_GetParameterRoundTrip) {
    CartesianCTParameter param;
    param.K0 = CartesianMatrix::Identity() * 200;
    param.K1 = CartesianMatrix::Identity() * 80;
    param.K0N = JointMatrix::Identity() * 50;
    param.K1N = JointMatrix::Identity() * 20;
    CartesianCtrParamTrajectory traj(Time(0.0), param);
    CartesianCTParameter recovered = traj.get_parameter();
    for (int i = 0; i < 6; ++i) {
        EXPECT_DOUBLE_EQ(recovered.K0(i, i), 200.0);
        EXPECT_DOUBLE_EQ(recovered.K1(i, i), 80.0);
    }
    for (int i = 0; i < DOF; ++i) {
        EXPECT_DOUBLE_EQ(recovered.K0N(i, i), 50.0);
        EXPECT_DOUBLE_EQ(recovered.K1N(i, i), 20.0);
    }
}
TEST(SingleEventTest, CartCtrParam_DefaultGains) {
    CartesianCTParameter param;
    CartesianCtrParamTrajectory traj(Time(0.0), param);
    CartesianCTParameter recovered = traj.get_parameter();
    for (int i = 0; i < 6; ++i) {
        EXPECT_DOUBLE_EQ(recovered.K0(i, i), 1.0);
        EXPECT_DOUBLE_EQ(recovered.K1(i, i), 1.0);
    }
}
TEST(SingleEventTest, CartCtrParam_SaveStateReturnsDefault) {
    CartesianCTParameter param;
    CartesianCtrParamTrajectory traj(Time(0.0), param);
    TrajectoryPointStorage saved = traj.save_state(Time(1.0));
    EXPECT_EQ(saved.previous_type, TrajectoryType::INVALID);
}
}  // namespace
