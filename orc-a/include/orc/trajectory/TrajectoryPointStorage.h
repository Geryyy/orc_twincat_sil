#pragma once

#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/trajectory/TrajectoryType.h"
#include "orc/util/Time.h"

namespace orc::trajectory {
template <int DOF>
class TrajectoryPointStorage {
    using JointVector = typename RobotTraits<DOF>::JointVector;

public:
    TrajectoryType previous_type = TrajectoryType::INVALID;

    /** absolute time at which the hand-off occurs (i.e. time at which
     *  q_, q_dot_, q_dotdot_ were sampled on the outgoing trajectory).
     *  Used by the new trajectory to rebase its internal t0 so the
     *  spline is continuous at the hand-off. */
    orc::Time time_ = orc::Time();

    JointVector q_;
    JointVector q_dot_;
    JointVector q_dotdot_;

    PoseVector pose_;
    CartesianVector x_dot_;
    CartesianVector x_dotdot_;

    // For HybridForceMotionTrajectory
    double force_ = 0.;

    /** empty save state */
    TrajectoryPointStorage() {
        q_ = JointVector::Zero();
        q_dot_ = JointVector::Zero();
        q_dotdot_ = JointVector::Zero();

        pose_ = PoseVector::Zero();
        x_dot_ = CartesianVector::Zero();
        x_dotdot_ = CartesianVector::Zero();
    }

    /** save jointspace trajectory state */
    TrajectoryPointStorage(JointVector q, JointVector q_dot, JointVector q_dotdot)
        : q_(q), q_dot_(q_dot), q_dotdot_(q_dotdot) {
        pose_ = PoseVector::Zero();
        x_dot_ = CartesianVector::Zero();
        x_dotdot_ = CartesianVector::Zero();
    }

    /** save taskspace and hybrid force motion trajectory state */
    TrajectoryPointStorage(PoseVector pose, CartesianVector x_dot, CartesianVector x_dotdot,
                           double force = 0.)
        : pose_(pose), x_dot_(x_dot), x_dotdot_(x_dotdot), force_(force) {
        q_ = JointVector::Zero();
        q_dot_ = JointVector::Zero();
        q_dotdot_ = JointVector::Zero();
    }
};
}  // namespace orc::trajectory
