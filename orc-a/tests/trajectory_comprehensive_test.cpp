#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/trajectory/JointspaceTrajectory.h"
#include "orc/trajectory/TrajectoryBase.h"
#include "orc/trajectory/TrajectoryPointStorage.h"
#include "orc/trajectory/TrajectoryQueue.h"
#include "orc/trajectory/TrajectoryType.h"
#include "orc/util/Logger.h"
namespace {
constexpr int DOF = 5;
using JointVector = orc::RobotTraits<DOF>::JointVector;
using JointspaceTrajectory = orc::trajectory::JointspaceTrajectory<DOF>;
using TrajectoryQueue = orc::trajectory::TrajectoryQueue<DOF>;
using TrajectoryBase = orc::trajectory::TrajectoryBase<DOF>;
using TrajectoryType = orc::trajectory::TrajectoryType;
using TrajectoryPointStorage = orc::trajectory::TrajectoryPointStorage<DOF>;
using Time = orc::Time;
// =========================================================================
// TrajectoryPointStorage tests
// =========================================================================
TEST(TrajectoryPointStorageTest, DefaultConstructorZeros) {
    TrajectoryPointStorage storage;
    EXPECT_EQ(storage.previous_type, TrajectoryType::INVALID);
    EXPECT_DOUBLE_EQ(storage.q_.norm(), 0.0);
    EXPECT_DOUBLE_EQ(storage.q_dot_.norm(), 0.0);
    EXPECT_DOUBLE_EQ(storage.q_dotdot_.norm(), 0.0);
    EXPECT_DOUBLE_EQ(storage.pose_.norm(), 0.0);
    EXPECT_DOUBLE_EQ(storage.x_dot_.norm(), 0.0);
    EXPECT_DOUBLE_EQ(storage.x_dotdot_.norm(), 0.0);
    EXPECT_DOUBLE_EQ(storage.force_, 0.0);
}
TEST(TrajectoryPointStorageTest, JointConstructor) {
    JointVector q = JointVector::Ones();
    JointVector q_dot = 2.0 * JointVector::Ones();
    JointVector q_dotdot = 3.0 * JointVector::Ones();
    TrajectoryPointStorage storage(q, q_dot, q_dotdot);
    for (int i = 0; i < DOF; i++) {
        EXPECT_DOUBLE_EQ(storage.q_(i), 1.0);
        EXPECT_DOUBLE_EQ(storage.q_dot_(i), 2.0);
        EXPECT_DOUBLE_EQ(storage.q_dotdot_(i), 3.0);
    }
    // Cartesian fields should be zero
    EXPECT_DOUBLE_EQ(storage.pose_.norm(), 0.0);
}
TEST(TrajectoryPointStorageTest, CartesianConstructor) {
    orc::PoseVector pose;
    pose << 1, 2, 3, 1, 0, 0, 0;
    orc::CartesianVector x_dot = orc::CartesianVector::Ones();
    orc::CartesianVector x_dotdot = orc::CartesianVector::Zero();
    TrajectoryPointStorage storage(pose, x_dot, x_dotdot, 5.0);
    EXPECT_DOUBLE_EQ(storage.pose_(0), 1.0);
    EXPECT_DOUBLE_EQ(storage.force_, 5.0);
    // Joint fields should be zero
    EXPECT_DOUBLE_EQ(storage.q_.norm(), 0.0);
}
// =========================================================================
// JointspaceTrajectory save/restore state
// =========================================================================
TEST(JointspaceTrajectoryTest, SaveAndRestoreState) {
    orc::log::start_logging(orc::log::Level::Error);
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones();
    JointspaceTrajectory traj1(q0, q1, Time(0.0), Time(2.0));
    traj1.init();
    // Update to midpoint
    traj1.update(Time(1.0));
    JointVector q_mid = traj1.get_q();
    // Save state at t=1.0
    TrajectoryPointStorage saved = traj1.save_state(Time(1.0));
    EXPECT_EQ(saved.previous_type, TrajectoryType::JOINTSPACE);
    // Position at save point should match midpoint
    for (int i = 0; i < DOF; i++) {
        EXPECT_NEAR(saved.q_(i), q_mid(i), 1e-9);
    }
    // Create new trajectory starting from saved state
    JointVector q2 = 2.0 * JointVector::Ones();
    JointspaceTrajectory traj2(q1, q2, Time(1.0), Time(3.0));
    traj2.init(saved);
    // Update new trajectory at start — should match saved state position
    traj2.update(Time(1.0));
    JointVector q_start2 = traj2.get_q();
    for (int i = 0; i < DOF; i++) {
        EXPECT_NEAR(q_start2(i), saved.q_(i), 1e-3) << "Smooth transition failed at joint " << i;
    }
}
TEST(JointspaceTrajectoryTest, EndpointAccuracy) {
    orc::log::start_logging(orc::log::Level::Error);
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones();
    JointspaceTrajectory traj(q0, q1, Time(0.0), Time(1.0));
    traj.init();
    // Check start
    traj.update(Time(0.0));
    for (int i = 0; i < DOF; i++)
        EXPECT_NEAR(traj.get_q()(i), 0.0, 1e-9);
    // Check end
    traj.update(Time(1.0));
    for (int i = 0; i < DOF; i++)
        EXPECT_NEAR(traj.get_q()(i), 1.0, 1e-9);
}
TEST(JointspaceTrajectoryTest, ZeroDerivativesAtEnd) {
    orc::log::start_logging(orc::log::Level::Error);
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones();
    JointspaceTrajectory traj(q0, q1, Time(0.0), Time(1.0));
    traj.init();
    // At end, velocity and acceleration should be zero (forced boundary condition)
    traj.update(Time(1.0));
    for (int i = 0; i < DOF; i++) {
        EXPECT_NEAR(traj.get_q_dot()(i), 0.0, 1e-6);
        EXPECT_NEAR(traj.get_q_dotdot()(i), 0.0, 1e-6);
    }
}
TEST(JointspaceTrajectoryTest, TrajectoryType) {
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones();
    JointspaceTrajectory traj(q0, q1, Time(0.0), Time(1.0));
    EXPECT_EQ(traj.get_trajectory_type(), TrajectoryType::JOINTSPACE);
}
// =========================================================================
// TrajectoryQueue comprehensive tests
// =========================================================================
TEST(TrajectoryQueueComprehensive, MultipleTrajectoryHandoff) {
    orc::log::start_logging(orc::log::Level::Error);
    TrajectoryQueue queue;
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones();
    JointVector q2 = 2.0 * JointVector::Ones();
    JointspaceTrajectory traj1(q0, q1, Time(0.0), Time(1.0));
    JointspaceTrajectory traj2(q1, q2, Time(1.0), Time(2.0));
    JointspaceTrajectory traj3(q2, q0, Time(2.0), Time(3.0));
    queue.add_jointspace_trajectory(traj1);
    queue.add_jointspace_trajectory(traj2);
    queue.add_jointspace_trajectory(traj3);
    // Initially 3 trajectories queued
    EXPECT_EQ(queue.get_queue_size(), 3u);
    // Activate first trajectory
    TrajectoryBase* current = queue.update(Time(0.0));
    ASSERT_NE(current, nullptr);
    EXPECT_EQ(current->get_trajectory_type(), TrajectoryType::JOINTSPACE);
    EXPECT_EQ(queue.get_queue_size(), 2u);
    // Transition to second
    current = queue.update(Time(1.0));
    ASSERT_NE(current, nullptr);
    EXPECT_EQ(queue.get_queue_size(), 1u);
    // Transition to third
    current = queue.update(Time(2.0));
    ASSERT_NE(current, nullptr);
    EXPECT_EQ(queue.get_queue_size(), 0u);
}
TEST(TrajectoryQueueComprehensive, QueueSizeTracking) {
    TrajectoryQueue queue;
    EXPECT_EQ(queue.get_queue_size(), 0u);
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones();
    JointspaceTrajectory traj(q0, q1, Time(1.0), Time(2.0));
    queue.add_jointspace_trajectory(traj);
    EXPECT_EQ(queue.get_queue_size(), 1u);
    queue.add_jointspace_trajectory(traj);
    EXPECT_EQ(queue.get_queue_size(), 2u);
    queue.clear();
    EXPECT_EQ(queue.get_queue_size(), 0u);
}
TEST(TrajectoryQueueComprehensive, ClearDuringExecution) {
    TrajectoryQueue queue;
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones();
    JointspaceTrajectory traj(q0, q1, Time(0.0), Time(2.0));
    queue.add_jointspace_trajectory(traj);
    // Start trajectory
    TrajectoryBase* current = queue.update(Time(0.5));
    ASSERT_NE(current, nullptr);
    // Clear while trajectory is active
    queue.clear();
    // After clear, update should return nullptr
    current = queue.update(Time(1.0));
    EXPECT_EQ(current, nullptr);
    EXPECT_EQ(queue.get_current_trajectory_type(), TrajectoryType::INVALID);
}
TEST(TrajectoryQueueComprehensive, UpdateBeforeAnyTrajectoryStarts) {
    TrajectoryQueue queue;
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones();
    JointspaceTrajectory traj(q0, q1, Time(5.0), Time(7.0));
    queue.add_jointspace_trajectory(traj);
    // Update before trajectory start time
    TrajectoryBase* current = queue.update(Time(0.0));
    EXPECT_EQ(current, nullptr);
    EXPECT_EQ(queue.get_queue_size(), 1u);
}
TEST(TrajectoryQueueComprehensive, AddTrajectoryViaUniquePtr) {
    TrajectoryQueue queue;
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones();
    auto traj = std::make_unique<JointspaceTrajectory>(q0, q1, Time(0.0), Time(1.0));
    queue.add_trajectory(std::move(traj));
    EXPECT_EQ(queue.get_queue_size(), 1u);
    TrajectoryBase* current = queue.update(Time(0.0));
    ASSERT_NE(current, nullptr);
}
TEST(TrajectoryQueueComprehensive, TrajectoryStaysActiveAfterEnd) {
    TrajectoryQueue queue;
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones();
    JointspaceTrajectory traj(q0, q1, Time(0.0), Time(1.0));
    queue.add_jointspace_trajectory(traj);
    // Start trajectory
    queue.update(Time(0.0));
    // Update well past end time — trajectory should still be returned
    // (it stays as current trajectory until a new one replaces it)
    TrajectoryBase* current = queue.update(Time(10.0));
    EXPECT_NE(current, nullptr);
}
}  // namespace
