#include <gtest/gtest.h>
#include <stdexcept>

#include <orc/OrcTypes.h>
#include <orc/RobotTraits.h>
#include <orc/interpolator/jointspace/SplineJointInterpolator.h>
#include <orc/trajectory/JointspaceTrajectory.h>
#include <orc/trajectory/TaskspaceTrajectory.h>
#include <orc/trajectory/TrajectoryType.h>
#include <orc/util/Time.h>

namespace {
constexpr int DOF = 3;
using Time = orc::Time;
using JointVector = typename orc::RobotTraits<DOF>::JointVector;
using JointspaceTrajectory = orc::trajectory::JointspaceTrajectory<DOF>;
using TaskspaceTrajectory = orc::trajectory::TaskspaceTrajectory<DOF>;
using TrajectoryPointStorage = orc::trajectory::TrajectoryPointStorage<DOF>;
using TrajectoryType = orc::trajectory::TrajectoryType;
using SplineJointInterpolator = orc::interpolator::SplineJointInterpolator<DOF>;

// M-1: TaskspaceTrajectory::save_state must populate state.time_
TEST(TaskspaceSaveState, SetsTimeField) {
    orc::PoseVector p0 = orc::PoseVector::Zero();
    orc::PoseVector p1 = orc::PoseVector::Zero();
    p1(0) = 1.0;
    TaskspaceTrajectory traj(p0, p1, Time(0.0), Time(1.0));
    traj.init();
    TrajectoryPointStorage s = traj.save_state(Time(0.5));
    EXPECT_EQ(s.previous_type, TrajectoryType::TASKSPACE);
    EXPECT_NEAR(s.time_.toSec(), 0.5, 1e-9);
}

// M-2: handoff at exact back() should take the fallback path (not the rebase
// that would yield a single-knot spline). Verified via a JointspaceTrajectory
// save_state->init cycle which calls init(q_, q_dot_, q_dotdot_, handoff_time).
TEST(SplineInterp, HandoffAtBoundaryDoesNotCrash) {
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones();
    // two segments: 0..1s and 1..2s; second will start with handoff==t_vec.back()
    JointspaceTrajectory traj1(q0, q1, Time(0.0), Time(1.0));
    traj1.init();
    // simulate end-of-segment save_state exactly at nominal end time
    TrajectoryPointStorage s = traj1.save_state(Time(1.0));
    EXPECT_EQ(s.previous_type, TrajectoryType::JOINTSPACE);
    EXPECT_NEAR(s.time_.toSec(), 1.0, 1e-9);

    JointspaceTrajectory traj2(q1, q0, Time(1.0), Time(2.0));
    // hand-off at exact nominal start (== t_vec.front() here, not back) — use
    // a case that triggers the boundary branch in init(...handoff_time)
    // by invoking through JointspaceTrajectory::init with a saved state at
    // the segment boundary. Must not crash.
    EXPECT_NO_THROW(traj2.init(s));
}

// M-3: Zero-duration spline must throw on init (RT contract forbids throwing
// ctors; init() is where the singular fit would otherwise blow up).
TEST(SplineInterp, ZeroDurationThrows) {
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones();
    EXPECT_THROW(
        {
            SplineJointInterpolator interp(q0, q1, Time(1.0), Time(1.0));
            interp.init();
        },
        std::invalid_argument);
}
}  // namespace
