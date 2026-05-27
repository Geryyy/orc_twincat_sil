#pragma once

#include <vector>

#include "orc/RobotTraits.h"
#include "orc/interpolator/jointspace/SplineJointInterpolator.h"
#include "orc/robots/RobotData.h"
#include "orc/trajectory/TrajectoryBase.h"
#include "orc/trajectory/TrajectoryPointStorage.h"
#include "orc/trajectory/TrajectoryType.h"

namespace orc::trajectory {

/**
 * @brief Jointspace velocity trajectory
 *
 * Interpolates a commanded joint-space velocity over time. On update() the
 * velocity (and its derivative, which is a joint-space acceleration) is
 * written into RobotData. The commanded joint configuration q_d is obtained
 * by integrating the velocity starting from whatever was active when the
 * trajectory began.
 */
template <int DOF>
class JointspaceVelocityTrajectory : public TrajectoryBase<DOF> {
    using JointVector = typename RobotTraits<DOF>::JointVector;
    using RobotData = orc::robots::RobotData<DOF>;
    using SplineJointInterpolator = orc::interpolator::SplineJointInterpolator<DOF>;
    using TrajectoryPointStorage = orc::trajectory::TrajectoryPointStorage<DOF>;

    SplineJointInterpolator interp_;

    JointVector q_d_;  // running commanded configuration (integrated)
    JointVector q_dot_;
    JointVector q_dotdot_;
    Time last_t_;
    bool integrated_initialized_ = false;

public:
    JointspaceVelocityTrajectory(std::vector<JointVector>& velocity_points,
                                 std::vector<Time>& time_points)
        : TrajectoryBase<DOF>(), interp_(time_points, velocity_points) {
        this->traj_type = TrajectoryType::JOINTSPACE_VELOCITY;
        q_d_.setZero();
        q_dot_.setZero();
        q_dotdot_.setZero();
    }

    void update(Time t) {
        interp_.update(t);
        q_dot_ = interp_.get_point();
        q_dotdot_ = interp_.get_derivative();
        if (integrated_initialized_) {
            const double dt = (t - last_t_).toSec();
            q_d_ += q_dot_ * dt;
        }
        last_t_ = t;
        integrated_initialized_ = true;
    }

    void update(Time t, RobotData& robot_data) {
        if (!integrated_initialized_) {
            // Start integration from whatever the controller last commanded.
            q_d_ = robot_data.q_d;
        }
        update(t);
        robot_data.q_d = q_d_;
        robot_data.q_dot_d = q_dot_;
        robot_data.q_dotdot_d = q_dotdot_;
    }

    [[nodiscard]] JointVector get_q_dot() const { return q_dot_; }
    [[nodiscard]] JointVector get_q_dotdot() const { return q_dotdot_; }
    [[nodiscard]] JointVector get_q_integrated() const { return q_d_; }

    [[nodiscard]] Time get_start_time() override { return interp_.get_start_time(); }

    [[nodiscard]] std::vector<JointVector> get_velocity_points() {
        return interp_.get_trajectory_points();
    }

    [[nodiscard]] std::vector<Time> get_time_points() { return interp_.get_time_points(); }

    void init(TrajectoryPointStorage) {
        interp_.init();
        integrated_initialized_ = false;
    }

    void init() {
        interp_.init();
        integrated_initialized_ = false;
    }

    TrajectoryPointStorage save_state(Time) { return TrajectoryPointStorage(); }
};

}  // namespace orc::trajectory
