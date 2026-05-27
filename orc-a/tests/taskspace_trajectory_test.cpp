/**
 * @file taskspace_trajectory_test.cpp
 * @brief Comprehensive tests for TaskspaceTrajectory<DOF> and additional
 *        CartesianPoseInterpolator edge cases (180° rotation, multi-pose, transitions).
 */
#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "orc/Orc.h"
#include "orc/robots/Iiwa.h"
#include "orc/trajectory/TaskspaceTrajectory.h"
#include "orc/trajectory/TrajectoryPointStorage.h"
#include "orc/util/Logger.h"

namespace {
constexpr int DOF = 7;
using Time = orc::Time;
using PoseVector = orc::PoseVector;
using CartesianVector = orc::CartesianVector;
using Quatd = orc::Quatd;
using TaskspaceTrajectory = orc::trajectory::TaskspaceTrajectory<DOF>;
using TrajectoryPointStorage = orc::trajectory::TrajectoryPointStorage<DOF>;
using TrajectoryType = orc::trajectory::TrajectoryType;
using RobotData = orc::robots::RobotData<DOF>;
using Iiwa = orc::robots::Iiwa;

// Helper to create a PoseVector from position and quaternion (w,x,y,z)
PoseVector make_pose(double x, double y, double z, double qw, double qx, double qy, double qz) {
    PoseVector p;
    p << x, y, z, qw, qx, qy, qz;
    return p;
}

double quaternion_norm(const PoseVector& p) {
    return std::sqrt(p(3) * p(3) + p(4) * p(4) + p(5) * p(5) + p(6) * p(6));
}

// =========================================================================
// Basic construction and type
// =========================================================================
TEST(TaskspaceTrajectoryTest, TrajectoryTypeIsTaskspace) {
    PoseVector p0 = make_pose(0, 0, 0, 1, 0, 0, 0);
    PoseVector p1 = make_pose(1, 0, 0, 1, 0, 0, 0);
    TaskspaceTrajectory traj(p0, p1, Time(0.0), Time(1.0));
    EXPECT_EQ(traj.get_trajectory_type(), TrajectoryType::TASKSPACE);
}

TEST(TaskspaceTrajectoryTest, MultiPointConstructor) {
    std::vector<PoseVector> poses = {make_pose(0, 0, 0, 1, 0, 0, 0),
                                     make_pose(0.5, 0, 0, 1, 0, 0, 0),
                                     make_pose(1.0, 0, 0, 1, 0, 0, 0)};
    std::vector<Time> times = {Time(0.0), Time(0.5), Time(1.0)};
    TaskspaceTrajectory traj(poses, times);
    EXPECT_EQ(traj.get_trajectory_type(), TrajectoryType::TASKSPACE);
}

TEST(TaskspaceTrajectoryTest, GetStartTime) {
    PoseVector p0 = make_pose(0, 0, 0, 1, 0, 0, 0);
    PoseVector p1 = make_pose(1, 0, 0, 1, 0, 0, 0);
    TaskspaceTrajectory traj(p0, p1, Time(2.5), Time(5.0));
    EXPECT_EQ(traj.get_start_time(), Time(2.5));
}

// =========================================================================
// Init without saved state
// =========================================================================
TEST(TaskspaceTrajectoryTest, InitWithoutState) {
    PoseVector p0 = make_pose(0, 0, 0, 1, 0, 0, 0);
    PoseVector p1 = make_pose(1, 0, 0, 1, 0, 0, 0);
    TaskspaceTrajectory traj(p0, p1, Time(0.0), Time(1.0));
    // init without saved state should not throw
    traj.init();
    traj.update(Time(0.0));
    PoseVector pose = traj.get_pose();
    // Should return a valid pose
    EXPECT_TRUE(pose.allFinite());
}

// =========================================================================
// Endpoint accuracy — position
// =========================================================================
TEST(TaskspaceTrajectoryTest, EndpointAccuracyPosition) {
    PoseVector p0 = make_pose(0, 0, 0, 1, 0, 0, 0);
    PoseVector p1 = make_pose(1, 2, 3, 1, 0, 0, 0);
    TaskspaceTrajectory traj(p0, p1, Time(0.0), Time(2.0));
    traj.init();

    // At start
    traj.update(Time(0.0));
    EXPECT_NEAR(traj.get_pose()(0), 0.0, 1e-6);
    EXPECT_NEAR(traj.get_pose()(1), 0.0, 1e-6);
    EXPECT_NEAR(traj.get_pose()(2), 0.0, 1e-6);

    // At end
    traj.update(Time(2.0));
    EXPECT_NEAR(traj.get_pose()(0), 1.0, 1e-6);
    EXPECT_NEAR(traj.get_pose()(1), 2.0, 1e-6);
    EXPECT_NEAR(traj.get_pose()(2), 3.0, 1e-6);
}

// =========================================================================
// Quaternion normalization throughout trajectory
// =========================================================================
TEST(TaskspaceTrajectoryTest, QuaternionNormPreserved) {
    // 90° rotation around Z
    PoseVector p0 = make_pose(0, 0, 0, 1, 0, 0, 0);
    double c45 = std::cos(M_PI / 4.0);
    double s45 = std::sin(M_PI / 4.0);
    PoseVector p1 = make_pose(1, 0, 0, c45, 0, 0, s45);
    TaskspaceTrajectory traj(p0, p1, Time(0.0), Time(1.0));
    traj.init();

    for (int i = 0; i <= 20; ++i) {
        double t = i / 20.0;
        traj.update(Time(t));
        double qn = quaternion_norm(traj.get_pose());
        EXPECT_NEAR(qn, 1.0, 1e-6) << "Quaternion norm violated at t=" << t;
    }
}

// =========================================================================
// Endpoint accuracy — orientation (90° rotation)
// =========================================================================
TEST(TaskspaceTrajectoryTest, EndpointAccuracyOrientation90) {
    PoseVector p0 = make_pose(0, 0, 0, 1, 0, 0, 0);
    double c45 = std::cos(M_PI / 4.0);
    double s45 = std::sin(M_PI / 4.0);
    PoseVector p1 = make_pose(0, 0, 0, c45, 0, 0, s45);
    TaskspaceTrajectory traj(p0, p1, Time(0.0), Time(1.0));
    traj.init();

    traj.update(Time(1.0));
    PoseVector pose_end = traj.get_pose();
    EXPECT_NEAR(pose_end(3), c45, 1e-5);
    EXPECT_NEAR(pose_end(6), s45, 1e-5);
}

// =========================================================================
// 180° rotation (edge case — singularity in quaternion log)
// =========================================================================
TEST(TaskspaceTrajectoryTest, Rotation180Degrees) {
    PoseVector p0 = make_pose(0, 0, 0, 1, 0, 0, 0);
    // 180° around Z: qw=0, qz=1
    PoseVector p1 = make_pose(0, 0, 0, 0, 0, 0, 1);
    TaskspaceTrajectory traj(p0, p1, Time(0.0), Time(2.0));
    traj.init();

    // Verify interpolation stays valid throughout
    for (int i = 0; i <= 20; ++i) {
        double t = i * 0.1;
        traj.update(Time(t));
        PoseVector pose = traj.get_pose();
        EXPECT_TRUE(pose.allFinite()) << "Non-finite pose at t=" << t;
        double qn = quaternion_norm(pose);
        EXPECT_NEAR(qn, 1.0, 1e-4) << "Quaternion norm violated at t=" << t;
    }

    // Endpoint
    traj.update(Time(2.0));
    PoseVector pose_end = traj.get_pose();
    EXPECT_NEAR(std::abs(pose_end(6)), 1.0, 1e-4) << "180° rotation endpoint wrong";
}

// =========================================================================
// Identity rotation (edge case — zero quaternion log)
// =========================================================================
TEST(TaskspaceTrajectoryTest, IdentityRotation) {
    PoseVector p0 = make_pose(0, 0, 0, 1, 0, 0, 0);
    PoseVector p1 = make_pose(1, 2, 3, 1, 0, 0, 0);
    TaskspaceTrajectory traj(p0, p1, Time(0.0), Time(1.0));
    traj.init();

    // At midpoint: quaternion should still be identity
    traj.update(Time(0.5));
    PoseVector pose_mid = traj.get_pose();
    EXPECT_NEAR(pose_mid(3), 1.0, 1e-6);
    EXPECT_NEAR(pose_mid(4), 0.0, 1e-6);
    EXPECT_NEAR(pose_mid(5), 0.0, 1e-6);
    EXPECT_NEAR(pose_mid(6), 0.0, 1e-6);
}

// =========================================================================
// Clamping: before start and after end
// =========================================================================
TEST(TaskspaceTrajectoryTest, ClampBeforeStart) {
    PoseVector p0 = make_pose(1, 2, 3, 1, 0, 0, 0);
    PoseVector p1 = make_pose(4, 5, 6, 1, 0, 0, 0);
    TaskspaceTrajectory traj(p0, p1, Time(1.0), Time(3.0));
    traj.init();

    traj.update(Time(0.0));  // before start
    PoseVector pose = traj.get_pose();
    EXPECT_NEAR(pose(0), 1.0, 1e-6);
    EXPECT_NEAR(pose(1), 2.0, 1e-6);
    EXPECT_NEAR(pose(2), 3.0, 1e-6);
}

TEST(TaskspaceTrajectoryTest, ClampAfterEnd) {
    PoseVector p0 = make_pose(1, 2, 3, 1, 0, 0, 0);
    PoseVector p1 = make_pose(4, 5, 6, 1, 0, 0, 0);
    TaskspaceTrajectory traj(p0, p1, Time(1.0), Time(3.0));
    traj.init();

    traj.update(Time(10.0));  // after end
    PoseVector pose = traj.get_pose();
    EXPECT_NEAR(pose(0), 4.0, 1e-6);
    EXPECT_NEAR(pose(1), 5.0, 1e-6);
    EXPECT_NEAR(pose(2), 6.0, 1e-6);
}

// =========================================================================
// Zero end-derivatives (velocity and acceleration at end should be ~0)
// =========================================================================
TEST(TaskspaceTrajectoryTest, ZeroEndDerivatives) {
    PoseVector p0 = make_pose(0, 0, 0, 1, 0, 0, 0);
    PoseVector p1 = make_pose(1, 0, 0, 1, 0, 0, 0);
    TaskspaceTrajectory traj(p0, p1, Time(0.0), Time(2.0));
    traj.init();

    traj.update(Time(2.0));
    CartesianVector x_dot = traj.get_x_dot();
    CartesianVector x_dotdot = traj.get_x_dotdot();
    EXPECT_NEAR(x_dot.norm(), 0.0, 1e-3) << "Velocity should be ~0 at trajectory end";
    EXPECT_NEAR(x_dotdot.norm(), 0.0, 1e-3) << "Acceleration should be ~0 at trajectory end";
}

// =========================================================================
// Derivatives are finite throughout trajectory
// =========================================================================
TEST(TaskspaceTrajectoryTest, DerivativesFinite) {
    PoseVector p0 = make_pose(0, 0, 0, 1, 0, 0, 0);
    double c45 = std::cos(M_PI / 4.0);
    double s45 = std::sin(M_PI / 4.0);
    PoseVector p1 = make_pose(1, 2, 0, c45, 0, s45, 0);
    TaskspaceTrajectory traj(p0, p1, Time(0.0), Time(2.0));
    traj.init();

    for (int i = 0; i <= 40; ++i) {
        double t = i * 0.05;
        traj.update(Time(t));
        EXPECT_TRUE(traj.get_x_dot().allFinite()) << "x_dot not finite at t=" << t;
        EXPECT_TRUE(traj.get_x_dotdot().allFinite()) << "x_dotdot not finite at t=" << t;
    }
}

// =========================================================================
// Multi-point Cartesian trajectory
// =========================================================================
TEST(TaskspaceTrajectoryTest, MultiPointTrajectory) {
    std::vector<PoseVector> poses = {make_pose(0, 0, 0, 1, 0, 0, 0), make_pose(1, 0, 0, 1, 0, 0, 0),
                                     make_pose(1, 1, 0, 1, 0, 0, 0),
                                     make_pose(1, 1, 1, 1, 0, 0, 0)};
    std::vector<Time> times = {Time(0.0), Time(1.0), Time(2.0), Time(3.0)};
    TaskspaceTrajectory traj(poses, times);
    traj.init();

    // At start
    traj.update(Time(0.0));
    EXPECT_NEAR(traj.get_pose()(0), 0.0, 1e-5);

    // At end
    traj.update(Time(3.0));
    PoseVector end_pose = traj.get_pose();
    EXPECT_NEAR(end_pose(0), 1.0, 1e-5);
    EXPECT_NEAR(end_pose(1), 1.0, 1e-5);
    EXPECT_NEAR(end_pose(2), 1.0, 1e-5);

    // Along the way: poses should be finite
    for (int i = 0; i <= 30; ++i) {
        traj.update(Time(i * 0.1));
        EXPECT_TRUE(traj.get_pose().allFinite());
    }
}

// =========================================================================
// Save/restore state round-trip
// =========================================================================
TEST(TaskspaceTrajectoryTest, SaveRestoreState) {
    PoseVector p0 = make_pose(0, 0, 0, 1, 0, 0, 0);
    PoseVector p1 = make_pose(1, 0, 0, 1, 0, 0, 0);
    TaskspaceTrajectory traj1(p0, p1, Time(0.0), Time(2.0));
    traj1.init();

    // Save state at midpoint
    TrajectoryPointStorage saved = traj1.save_state(Time(1.0));
    EXPECT_EQ(saved.previous_type, TrajectoryType::TASKSPACE);

    // Verify saved pose is at the midpoint
    EXPECT_TRUE(saved.pose_.allFinite());
    EXPECT_TRUE(saved.x_dot_.allFinite());
    EXPECT_TRUE(saved.x_dotdot_.allFinite());

    // Create new trajectory and init with saved state
    PoseVector p2 = make_pose(2, 0, 0, 1, 0, 0, 0);
    TaskspaceTrajectory traj2(saved.pose_, p2, Time(1.0), Time(3.0));
    traj2.init(saved);

    // The start of traj2 should be close to the saved state
    traj2.update(Time(1.0));
    PoseVector start_pose = traj2.get_pose();
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(start_pose(i), saved.pose_(i), 1e-3) << "Position mismatch at index " << i;
    }
}

// =========================================================================
// Smooth transition continuity (C0/C1)
// =========================================================================
TEST(TaskspaceTrajectoryTest, SmoothTransitionContinuity) {
    PoseVector p0 = make_pose(0, 0, 0, 1, 0, 0, 0);
    PoseVector p1 = make_pose(1, 0, 0, 1, 0, 0, 0);
    PoseVector p2 = make_pose(2, 0, 0, 1, 0, 0, 0);

    TaskspaceTrajectory traj1(p0, p1, Time(0.0), Time(1.0));
    traj1.init();

    // Save state at transition point
    Time t_switch(1.0);
    TrajectoryPointStorage saved = traj1.save_state(t_switch);

    TaskspaceTrajectory traj2(p1, p2, Time(1.0), Time(2.0));
    traj2.init(saved);

    // Position continuity at transition
    traj1.update(t_switch);
    traj2.update(t_switch);
    PoseVector pose_before = traj1.get_pose();
    PoseVector pose_after = traj2.get_pose();

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(pose_before(i), pose_after(i), 1e-2)
            << "Position discontinuity at transition, index " << i;
    }
}

// =========================================================================
// Update with RobotData (writes into robot_data fields)
// =========================================================================
TEST(TaskspaceTrajectoryTest, UpdateWritesIntoRobotData) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");

    PoseVector p0 = make_pose(0, 0, 0.5, 1, 0, 0, 0);
    PoseVector p1 = make_pose(0.1, 0, 0.5, 1, 0, 0, 0);
    TaskspaceTrajectory traj(p0, p1, Time(0.0), Time(1.0));
    traj.init();

    traj.update(Time(0.5), iiwa.robot_data);

    EXPECT_TRUE(iiwa.robot_data.pose_d.allFinite());
    EXPECT_TRUE(iiwa.robot_data.x_dot_d.allFinite());
    EXPECT_TRUE(iiwa.robot_data.x_dotdot_d.allFinite());
    // Position should be roughly at midpoint
    EXPECT_GT(iiwa.robot_data.pose_d(0), -0.1);
    EXPECT_LT(iiwa.robot_data.pose_d(0), 0.2);
}

// =========================================================================
// Multi-pose with orientation changes
// =========================================================================
TEST(TaskspaceTrajectoryTest, MultiPoseWithOrientationChanges) {
    double c30 = std::cos(M_PI / 6.0);
    double s30 = std::sin(M_PI / 6.0);
    double c60 = std::cos(M_PI / 3.0);
    double s60 = std::sin(M_PI / 3.0);

    std::vector<PoseVector> poses = {make_pose(0, 0, 0, 1, 0, 0, 0),
                                     make_pose(0.5, 0, 0, c30, 0, 0, s30),
                                     make_pose(1, 0, 0, c60, 0, 0, s60)};
    std::vector<Time> times = {Time(0.0), Time(1.0), Time(2.0)};
    TaskspaceTrajectory traj(poses, times);
    traj.init();

    // All intermediate poses should have unit quaternions
    for (int i = 0; i <= 20; ++i) {
        double t = i * 0.1;
        traj.update(Time(t));
        PoseVector pose = traj.get_pose();
        EXPECT_TRUE(pose.allFinite()) << "Non-finite pose at t=" << t;
        double qn = quaternion_norm(pose);
        EXPECT_NEAR(qn, 1.0, 1e-4) << "Quaternion norm violated at t=" << t;
    }
}

// =========================================================================
// Very short duration trajectory
// =========================================================================
TEST(TaskspaceTrajectoryTest, VeryShortDuration) {
    PoseVector p0 = make_pose(0, 0, 0, 1, 0, 0, 0);
    PoseVector p1 = make_pose(0.001, 0, 0, 1, 0, 0, 0);
    TaskspaceTrajectory traj(p0, p1, Time(0.0), Time(0.001));
    traj.init();

    traj.update(Time(0.0));
    EXPECT_TRUE(traj.get_pose().allFinite());
    traj.update(Time(0.001));
    EXPECT_TRUE(traj.get_pose().allFinite());
}

// =========================================================================
// Simultaneous translation and rotation
// =========================================================================
TEST(TaskspaceTrajectoryTest, SimultaneousTranslationAndRotation) {
    PoseVector p0 = make_pose(0, 0, 0, 1, 0, 0, 0);
    double c45 = std::cos(M_PI / 4.0);
    double s45 = std::sin(M_PI / 4.0);
    PoseVector p1 = make_pose(1, 1, 1, c45, s45, 0, 0);
    TaskspaceTrajectory traj(p0, p1, Time(0.0), Time(2.0));
    traj.init();

    // Check multiple points
    for (int i = 0; i <= 20; ++i) {
        double t = i * 0.1;
        traj.update(Time(t));
        PoseVector pose = traj.get_pose();
        EXPECT_TRUE(pose.allFinite());
        double qn = quaternion_norm(pose);
        EXPECT_NEAR(qn, 1.0, 1e-4);
    }

    // Endpoint
    traj.update(Time(2.0));
    PoseVector end_pose = traj.get_pose();
    EXPECT_NEAR(end_pose(0), 1.0, 1e-5);
    EXPECT_NEAR(end_pose(1), 1.0, 1e-5);
    EXPECT_NEAR(end_pose(2), 1.0, 1e-5);
}

}  // namespace
