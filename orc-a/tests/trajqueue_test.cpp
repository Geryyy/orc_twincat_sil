#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <vector>

#include <orc/OrcTypes.h>
#include <orc/control/controller/joint/JointCTController.h>
#include <orc/trajectory/JointspaceTrajectory.h>
#include <orc/trajectory/TrajectoryBase.h>
#include <orc/trajectory/TrajectoryQueue.h>
#include <orc/trajectory/TrajectoryType.h>
#include <orc/util/ExecutionTimer.h>
#include <orc/util/Logger.h>

namespace {
const int DOF = 5;
using TrajectoryQueue = orc::trajectory::TrajectoryQueue<DOF>;
using JointVector = typename orc::RobotTraits<DOF>::JointVector;
using JointspaceTrajectory = orc::trajectory::JointspaceTrajectory<DOF>;
using TrajectoryType = orc::trajectory::TrajectoryType;
using TrajectoryBase = orc::trajectory::TrajectoryBase<DOF>;
using Time = orc::Time;

TEST(TrajectoryQueueTest, CheckInitialConditions) {
    TrajectoryQueue traj_queue;
    EXPECT_TRUE(traj_queue.get_current_trajectory_type() == TrajectoryType::INVALID);

    TrajectoryBase* current_traj = traj_queue.update(0.0);
    EXPECT_TRUE(current_traj == nullptr);
}

TEST(TrajectoryQueueTest, TestAddingJointspaceTrajectory) {
    TrajectoryQueue traj_queue;

    JointVector q0, q1;
    q0.setZero();
    q1.setOnes();

    JointspaceTrajectory traj(q0, q1, Time(1.0), Time(3.0));
    traj_queue.add_jointspace_trajectory(traj);

    TrajectoryBase* current_traj = traj_queue.update(0.5);
    EXPECT_TRUE(current_traj == nullptr);  // Trajectory should not start yet

    current_traj = traj_queue.update(1.0);
    EXPECT_TRUE(current_traj != nullptr);  // Trajectory should start now
    EXPECT_TRUE(current_traj->get_trajectory_type() == TrajectoryType::JOINTSPACE);

    current_traj = traj_queue.update(4.0);
    EXPECT_NE(current_traj, nullptr);  // Trajectory should be still active
}

TEST(TrajectoryQueueTest, TestClearingQueue) {
    TrajectoryQueue traj_queue;

    JointVector q0, q1;
    q0.setZero();
    q1.setOnes();

    JointspaceTrajectory traj1(q0, q1, Time(1.0), Time(3.0));
    JointspaceTrajectory traj2(q1, q0, Time(4.0), Time(6.0));
    traj_queue.add_jointspace_trajectory(traj1);
    traj_queue.add_jointspace_trajectory(traj2);

    TrajectoryBase* current_traj = traj_queue.update(1.5);
    EXPECT_TRUE(current_traj != nullptr);  // First trajectory should be active

    traj_queue.clear();
    current_traj = traj_queue.update(5.0);
    EXPECT_TRUE(current_traj == nullptr);  // Queue cleared, no active trajectory
}
}  // namespace
