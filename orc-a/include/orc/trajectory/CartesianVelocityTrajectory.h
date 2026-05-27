#pragma once

#include <vector>

#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/interpolator/jointspace/SplineJointInterpolator.h"
#include "orc/robots/RobotData.h"
#include "orc/trajectory/TrajectoryBase.h"
#include "orc/trajectory/TrajectoryPointStorage.h"
#include "orc/trajectory/TrajectoryType.h"

namespace orc::trajectory {

/**
 * @brief Cartesian (task-space) velocity trajectory
 *
 * Interpolates a 6-DOF Cartesian velocity (linear + angular) over time.
 * update(t, robot_data) writes x_dot_d and x_dotdot_d into RobotData; the
 * pose setpoint pose_d is obtained by Euler-integrating the linear part and
 * leaving the orientation untouched (a full SE(3) integration would need
 * an angular-velocity-to-quaternion update; call sites that need exact
 * orientation tracking should use TaskspaceTrajectory instead).
 */
template <int DOF>
class CartesianVelocityTrajectory : public TrajectoryBase<DOF> {
    using RobotData = orc::robots::RobotData<DOF>;
    using SplineJointInterpolator = orc::interpolator::SplineJointInterpolator<6>;
    using TrajectoryPointStorage = orc::trajectory::TrajectoryPointStorage<DOF>;

    SplineJointInterpolator interp_;

    orc::CartesianVector x_dot_;
    orc::CartesianVector x_dotdot_;
    orc::PoseVector pose_d_;
    Time last_t_;
    bool integrated_initialized_ = false;

public:
    CartesianVelocityTrajectory(std::vector<orc::CartesianVector>& velocity_points,
                                std::vector<Time>& time_points)
        : TrajectoryBase<DOF>(), interp_(time_points, velocity_points) {
        this->traj_type = TrajectoryType::CARTESIAN_VELOCITY;
        x_dot_.setZero();
        x_dotdot_.setZero();
        pose_d_.setZero();
    }

    void update(Time t) {
        interp_.update(t);
        x_dot_ = interp_.get_point();
        x_dotdot_ = interp_.get_derivative();
        if (integrated_initialized_) {
            const double dt = (t - last_t_).toSec();
            // Translate the position part of the pose by the linear velocity.
            // Orientation (quaternion, indices 3..6) is left unchanged here;
            // callers needing angular tracking should use TaskspaceTrajectory.
            pose_d_.template head<3>() += x_dot_.template head<3>() * dt;
        }
        last_t_ = t;
        integrated_initialized_ = true;
    }

    void update(Time t, RobotData& robot_data) {
        if (!integrated_initialized_) {
            pose_d_ = robot_data.pose_d;
        }
        update(t);
        robot_data.pose_d = pose_d_;
        robot_data.x_dot_d = x_dot_;
        robot_data.x_dotdot_d = x_dotdot_;
    }

    [[nodiscard]] orc::CartesianVector get_x_dot() const { return x_dot_; }
    [[nodiscard]] orc::CartesianVector get_x_dotdot() const { return x_dotdot_; }

    [[nodiscard]] Time get_start_time() override { return interp_.get_start_time(); }

    [[nodiscard]] std::vector<orc::CartesianVector> get_velocity_points() {
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
