#pragma once

#include <cmath>
#include <vector>

#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/control/Controller.h"
#include "orc/interpolator/cartesian/CartesianPoseInterpolator.h"
#include "orc/robots/RobotData.h"
#include "orc/trajectory/TrajectoryBase.h"
#include "orc/trajectory/TrajectoryPointStorage.h"
#include "orc/trajectory/TrajectoryType.h"

namespace orc::trajectory {
template <int DOF>
class TaskspaceTrajectory : public TrajectoryBase<DOF> {
    DECLARE_ROBOT_TRAITS_USINGS(DOF)
    using RobotData = orc::robots::RobotData<DOF>;
    using CartesianCTController = typename orc::control::CartesianCTController<DOF>;
    using TrajectoryPointStorage = typename orc::trajectory::TrajectoryPointStorage<DOF>;
    using TrajectoryBase = typename orc::trajectory::TrajectoryBase<DOF>;
    using CartesianPoseInterpolator = orc::interpolator::CartesianPoseInterpolator;
    using PoseVector = orc::PoseVector;

    CartesianPoseInterpolator interp_;

    PoseVector pose_;
    CartesianVector x_dot_, x_dotdot_;

public:
    TaskspaceTrajectory(PoseVector pose0, PoseVector pose1, Time t0, Time t1)
        : TrajectoryBase(), interp_(pose0, pose1, t0, t1) {
        this->traj_type = TrajectoryType::TASKSPACE;
    }

    TaskspaceTrajectory(std::vector<PoseVector>& pose_vec, std::vector<Time>& time_points)
        : TrajectoryBase(), interp_(time_points, pose_vec) {
        this->traj_type = TrajectoryType::TASKSPACE;
    }

    /**
     * @brief Run the interpolator at given time and write desired
     * configuration, velocity and acceleration directly into members.
     *
     * @param t
     */
    void update(Time t) {
        interp_.update(t);
        pose_ = interp_.get_point();
        x_dot_ = interp_.get_derivative();
        x_dotdot_ = interp_.get_second_derivative();
    }

    /**
     * @brief Run the interpolator at given time and write desired
     * configuration, velocity and acceleration directly into members and robot_data.
     *
     * @param t
     * @param robot_data
     */
    void update(Time t, RobotData& robot_data) {
        update(t);

        robot_data.pose_d = pose_;
        robot_data.x_dot_d = x_dot_;
        robot_data.x_dotdot_d = x_dotdot_;
    }

    [[nodiscard]] PoseVector get_pose() const { return pose_; }
    [[nodiscard]] CartesianVector get_x_dot() const { return x_dot_; }
    [[nodiscard]] CartesianVector get_x_dotdot() const { return x_dotdot_; }

    void init(TrajectoryPointStorage saved_state) {
        if (saved_state.previous_type == TrajectoryType::TASKSPACE) {
            /** Start new trajectory where last trajectory ended
             *  initialize new trajectory with pose and first derivative for smooth transition */
            pose_ = saved_state.pose_;
            x_dot_ = saved_state.x_dot_;
            x_dotdot_ = saved_state.x_dotdot_;

            interp_.init(pose_, x_dot_, x_dotdot_);
        } else {
            /** state of last trajectory was not saved, no change to new trajectory */
            interp_.init();
        }
    }

    void init() {
        // without previous state
        interp_.init();
    }

    [[nodiscard]] Time get_start_time() override { return interp_.get_start_time(); }

    [[nodiscard]] std::vector<Time> get_time_points() { return interp_.get_time_points(); }

    [[nodiscard]] std::vector<PoseVector> get_pose_vector() {
        return interp_.get_trajectory_points();
    }

    TrajectoryPointStorage save_state(Time time) {
        TrajectoryPointStorage state;

        // update interpolator to get next time step = start time of new traj.
        interp_.update(time);

        // save last trajectory point
        state = TrajectoryPointStorage(interp_.get_point(), interp_.get_derivative(),
                                       interp_.get_second_derivative());
        state.previous_type = TrajectoryType::TASKSPACE;
        state.time_ = time;

        return state;
    }
};

}  // namespace orc::trajectory
