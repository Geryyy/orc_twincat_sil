/**
 * @file hybrid_trajectory_test.cpp
 * @brief Comprehensive tests for HybridForceMotionTrajectory<DOF>.
 */
#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "orc/Orc.h"
#include "orc/robots/Iiwa.h"
#include "orc/trajectory/HybridForceMotionTrajectory.h"
#include "orc/trajectory/TrajectoryPointStorage.h"
#include "orc/util/Logger.h"
namespace {
constexpr int DOF = 7;
using Time = orc::Time;
using PoseVector = orc::PoseVector;
using CartesianVector = orc::CartesianVector;
using HybridForceMotionTrajectory = orc::trajectory::HybridForceMotionTrajectory<DOF>;
using TrajectoryPointStorage = orc::trajectory::TrajectoryPointStorage<DOF>;
using TrajectoryType = orc::trajectory::TrajectoryType;
using Iiwa = orc::robots::Iiwa;
PoseVector make_pose(double x, double y, double z, double qw, double qx, double qy, double qz) {
    PoseVector p;
    p << x, y, z, qw, qx, qy, qz;
    return p;
}
TEST(HybridTrajectoryTest, TrajectoryTypeIsHybridForceMotion) {
    std::vector<PoseVector> poses = {make_pose(0, 0, 0, 1, 0, 0, 0),
                                     make_pose(0, 0, 0.05, 1, 0, 0, 0),
                                     make_pose(0, 0, 0.1, 1, 0, 0, 0)};
    std::vector<double> forces = {0.0, 5.0, 10.0};
    std::vector<Time> times = {Time(0.0), Time(1.0), Time(2.0)};
    HybridForceMotionTrajectory traj(poses, forces, times);
    EXPECT_EQ(traj.get_trajectory_type(), TrajectoryType::HYBRID_FORCE_MOTION);
}
TEST(HybridTrajectoryTest, GetStartTime) {
    std::vector<PoseVector> poses = {make_pose(0, 0, 0, 1, 0, 0, 0),
                                     make_pose(0, 0, 0.05, 1, 0, 0, 0),
                                     make_pose(0, 0, 0.1, 1, 0, 0, 0)};
    std::vector<double> forces = {0.0, 5.0, 10.0};
    std::vector<Time> times = {Time(1.0), Time(2.0), Time(3.0)};
    HybridForceMotionTrajectory traj(poses, forces, times);
    EXPECT_EQ(traj.get_start_time(), Time(1.0));
}
TEST(HybridTrajectoryTest, ForceAtStartAndEnd) {
    std::vector<PoseVector> poses = {make_pose(0, 0, 0, 1, 0, 0, 0),
                                     make_pose(0, 0, 0.05, 1, 0, 0, 0),
                                     make_pose(0, 0, 0.1, 1, 0, 0, 0)};
    std::vector<double> forces = {0.0, 5.0, 10.0};
    std::vector<Time> times = {Time(0.0), Time(1.0), Time(2.0)};
    HybridForceMotionTrajectory traj(poses, forces, times);
    traj.init();
    traj.update(Time(0.0));
    EXPECT_NEAR(traj.get_force(), 0.0, 1e-3);
    traj.update(Time(2.0));
    EXPECT_NEAR(traj.get_force(), 10.0, 1e-3);
}
TEST(HybridTrajectoryTest, ForceFiniteThroughout) {
    std::vector<PoseVector> poses = {make_pose(0, 0, 0, 1, 0, 0, 0),
                                     make_pose(0, 0, 0.05, 1, 0, 0, 0),
                                     make_pose(0, 0, 0.1, 1, 0, 0, 0)};
    std::vector<double> forces = {0.0, 5.0, 10.0};
    std::vector<Time> times = {Time(0.0), Time(1.0), Time(2.0)};
    HybridForceMotionTrajectory traj(poses, forces, times);
    traj.init();
    for (int i = 0; i <= 20; ++i) {
        double t = i * 0.1;
        traj.update(Time(t));
        EXPECT_TRUE(std::isfinite(traj.get_force())) << "Force not finite at t=" << t;
    }
}
TEST(HybridTrajectoryTest, PoseAndForceSynchronized) {
    std::vector<PoseVector> poses = {make_pose(0, 0, 0, 1, 0, 0, 0),
                                     make_pose(0.5, 0, 0, 1, 0, 0, 0),
                                     make_pose(1.0, 0, 0, 1, 0, 0, 0)};
    std::vector<double> forces = {0.0, 50.0, 100.0};
    std::vector<Time> times = {Time(0.0), Time(1.0), Time(2.0)};
    HybridForceMotionTrajectory traj(poses, forces, times);
    traj.init();
    for (int i = 0; i <= 20; ++i) {
        double t = i * 0.1;
        traj.update(Time(t));
        EXPECT_TRUE(traj.get_pose().allFinite()) << "Pose not finite at t=" << t;
        EXPECT_TRUE(std::isfinite(traj.get_force())) << "Force not finite at t=" << t;
    }
}
TEST(HybridTrajectoryTest, ConstantForce) {
    std::vector<PoseVector> poses = {make_pose(0, 0, 0, 1, 0, 0, 0),
                                     make_pose(0, 0, 0.05, 1, 0, 0, 0),
                                     make_pose(0, 0, 0.1, 1, 0, 0, 0)};
    std::vector<double> forces = {5.0, 5.0, 5.0};
    std::vector<Time> times = {Time(0.0), Time(1.0), Time(2.0)};
    HybridForceMotionTrajectory traj(poses, forces, times);
    traj.init();
    for (int i = 0; i <= 20; ++i) {
        traj.update(Time(i * 0.1));
        EXPECT_NEAR(traj.get_force(), 5.0, 0.05)
            << "Constant force interpolation should be exact (or very close) at t=" << i * 0.1;
    }
}
TEST(HybridTrajectoryTest, SaveRestoreState) {
    std::vector<PoseVector> poses = {make_pose(0, 0, 0, 1, 0, 0, 0),
                                     make_pose(0, 0, 0.05, 1, 0, 0, 0),
                                     make_pose(0, 0, 0.1, 1, 0, 0, 0)};
    std::vector<double> forces = {0.0, 5.0, 10.0};
    std::vector<Time> times = {Time(0.0), Time(1.0), Time(2.0)};
    HybridForceMotionTrajectory traj1(poses, forces, times);
    traj1.init();
    TrajectoryPointStorage saved = traj1.save_state(Time(1.0));
    EXPECT_EQ(saved.previous_type, TrajectoryType::HYBRID_FORCE_MOTION);
    EXPECT_NEAR(saved.force_, 5.0, 1.0);
    EXPECT_TRUE(saved.pose_.allFinite());
}
TEST(HybridTrajectoryTest, InitWithSavedState) {
    std::vector<PoseVector> poses = {make_pose(0, 0, 0, 1, 0, 0, 0),
                                     make_pose(0, 0, 0.05, 1, 0, 0, 0),
                                     make_pose(0, 0, 0.1, 1, 0, 0, 0)};
    std::vector<double> forces = {0.0, 5.0, 10.0};
    std::vector<Time> times = {Time(0.0), Time(1.0), Time(2.0)};
    HybridForceMotionTrajectory traj1(poses, forces, times);
    traj1.init();
    TrajectoryPointStorage saved = traj1.save_state(Time(1.0));
    std::vector<PoseVector> poses2 = {make_pose(0, 0, 0.05, 1, 0, 0, 0),
                                      make_pose(0, 0, 0.1, 1, 0, 0, 0),
                                      make_pose(0, 0, 0.15, 1, 0, 0, 0)};
    std::vector<double> forces2 = {5.0, 10.0, 15.0};
    std::vector<Time> times2 = {Time(1.0), Time(2.0), Time(3.0)};
    HybridForceMotionTrajectory traj2(poses2, forces2, times2);
    traj2.init(saved);
    traj2.update(Time(1.0));
    EXPECT_TRUE(traj2.get_pose().allFinite());
    EXPECT_TRUE(std::isfinite(traj2.get_force()));
}
TEST(HybridTrajectoryTest, InitWithNonHybridSavedState) {
    TrajectoryPointStorage saved;
    saved.previous_type = TrajectoryType::TASKSPACE;
    std::vector<PoseVector> poses = {make_pose(0, 0, 0, 1, 0, 0, 0),
                                     make_pose(0, 0, 0.05, 1, 0, 0, 0),
                                     make_pose(0, 0, 0.1, 1, 0, 0, 0)};
    std::vector<double> forces = {0.0, 5.0, 10.0};
    std::vector<Time> times = {Time(0.0), Time(1.0), Time(2.0)};
    HybridForceMotionTrajectory traj(poses, forces, times);
    traj.init(saved);
    traj.update(Time(0.0));
    EXPECT_TRUE(traj.get_pose().allFinite());
    EXPECT_TRUE(std::isfinite(traj.get_force()));
}
TEST(HybridTrajectoryTest, UpdateWritesIntoRobotData) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    std::vector<PoseVector> poses = {make_pose(0, 0, 0.5, 1, 0, 0, 0),
                                     make_pose(0, 0, 0.55, 1, 0, 0, 0),
                                     make_pose(0, 0, 0.6, 1, 0, 0, 0)};
    std::vector<double> forces = {0.0, 5.0, 10.0};
    std::vector<Time> times = {Time(0.0), Time(1.0), Time(2.0)};
    HybridForceMotionTrajectory traj(poses, forces, times);
    traj.init();
    traj.update(Time(1.0), iiwa.robot_data);
    EXPECT_TRUE(iiwa.robot_data.pose_d.allFinite());
    EXPECT_TRUE(std::isfinite(iiwa.robot_data.force_d));
}
TEST(HybridTrajectoryTest, ZeroForce) {
    std::vector<PoseVector> poses = {make_pose(0, 0, 0, 1, 0, 0, 0),
                                     make_pose(0.5, 0, 0, 1, 0, 0, 0),
                                     make_pose(1.0, 0, 0, 1, 0, 0, 0)};
    std::vector<double> forces = {0.0, 0.0, 0.0};
    std::vector<Time> times = {Time(0.0), Time(1.0), Time(2.0)};
    HybridForceMotionTrajectory traj(poses, forces, times);
    traj.init();
    for (int i = 0; i <= 20; ++i) {
        traj.update(Time(i * 0.1));
        EXPECT_NEAR(traj.get_force(), 0.0, 0.05)
            << "Zero force interpolation should stay at zero at t=" << i * 0.1;
    }
}
TEST(HybridTrajectoryTest, NegativeForce) {
    std::vector<PoseVector> poses = {make_pose(0, 0, 0, 1, 0, 0, 0), make_pose(0, 0, 0, 1, 0, 0, 0),
                                     make_pose(0, 0, 0, 1, 0, 0, 0)};
    std::vector<double> forces = {-10.0, -5.0, 0.0};
    std::vector<Time> times = {Time(0.0), Time(1.0), Time(2.0)};
    HybridForceMotionTrajectory traj(poses, forces, times);
    traj.init();
    traj.update(Time(0.0));
    EXPECT_NEAR(traj.get_force(), -10.0, 1e-3);
    traj.update(Time(2.0));
    EXPECT_NEAR(traj.get_force(), 0.0, 1e-3);
}
}  // namespace
