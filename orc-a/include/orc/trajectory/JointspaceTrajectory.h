#pragma once

#include <cmath>
#include <vector>

#include "orc/RobotTraits.h"
#include "orc/control/Controller.h"
#include "orc/interpolator/jointspace/SplineJointInterpolator.h"
#include "orc/robots/RobotData.h"
#include "orc/trajectory/TrajectoryBase.h"
#include "orc/trajectory/TrajectoryPointStorage.h"
#include "orc/trajectory/TrajectoryType.h"

namespace orc::trajectory {
template <int DOF>
class JointspaceTrajectory : public orc::trajectory::TrajectoryBase<DOF> {
    using JointVector = typename RobotTraits<DOF>::JointVector;
    using RobotData = orc::robots::RobotData<DOF>;
    using TrajectoryBase = typename orc::trajectory::TrajectoryBase<DOF>;
    using JointCTController = typename orc::control::JointCTController<DOF>;
    using SplineJointInterpolator = typename orc::interpolator::SplineJointInterpolator<DOF>;
    using TrajectoryPointStorage = typename orc::trajectory::TrajectoryPointStorage<DOF>;

    SplineJointInterpolator interp_;

    JointVector q_, q_dot_, q_dotdot_;

public:
    JointspaceTrajectory(JointVector q0, JointVector q1, Time t0, Time t1)
        : TrajectoryBase(), interp_(q0, q1, t0, t1) {
        this->traj_type = TrajectoryType::JOINTSPACE;
    }

    JointspaceTrajectory(std::vector<JointVector>& joint_poses, std::vector<Time>& time_points)
        : TrajectoryBase(), interp_(time_points, joint_poses) {
        this->traj_type = TrajectoryType::JOINTSPACE;
    }

    /**
     * @brief Run the interpolator at given time and write desired
     * configuration, velocity and acceleration directly into members.
     *
     * @param t
     */
    void update(Time t) {
        interp_.update(t);
        q_ = interp_.get_point();
        q_dot_ = interp_.get_derivative();
        q_dotdot_ = interp_.get_second_derivative();
    }

    /**
     * @brief Run the interpolator at given time and write desired
     * configuration, velocity and acceleration directly into members and robot_data.
     *
     * @param t
     * @param robot_data
     */
    void update(Time t, RobotData& robot_data) {
        interp_.update(t);
        robot_data.q_d = interp_.get_point();
        robot_data.q_dot_d = interp_.get_derivative();
        robot_data.q_dotdot_d = interp_.get_second_derivative();
    }

    [[nodiscard]] JointVector get_q() const { return q_; }
    [[nodiscard]] JointVector get_q_dot() const { return q_dot_; }
    [[nodiscard]] JointVector get_q_dotdot() const { return q_dotdot_; }

    void init(TrajectoryPointStorage saved_state) {
        if (saved_state.previous_type == TrajectoryType::JOINTSPACE) {
            /** Start new trajectory where last trajectory ended
             *  initialize new trajectory with pose and first derivative for smooth transition */
            q_ = saved_state.q_;
            q_dot_ = saved_state.q_dot_;
            q_dotdot_ = saved_state.q_dotdot_;
            /* initialize new trajectory with pose and start derivatives for smooth transition.
             * Rebase the interpolator to the hand-off time so the spline is
             * continuous at t = saved_state.time_ (not at the segment's nominal t0). */
            interp_.init(q_, q_dot_, q_dotdot_, saved_state.time_);
            // orc::log::write_debug("JointspaceTrajectory::init: saved state");
        } else {
            /** state of last trajectory was not saved, no change to new trajectory */
            interp_.init();
            // orc::log::write_debug("JointspaceTrajectory::init: no saved state");
        }
    }

    void init() {
        // without previous state
        interp_.init();
    }

    [[nodiscard]] Time get_start_time() override { return interp_.get_start_time(); }

    [[nodiscard]] std::vector<JointVector> get_joint_poses() {
        return interp_.get_trajectory_points();
    }

    [[nodiscard]] std::vector<Time> get_time_points() { return interp_.get_time_points(); }

    TrajectoryPointStorage save_state(Time time) {
        TrajectoryPointStorage state;

        // update interpolator to get same timestep as start of next trajectory
        interp_.update(time);

        // save last trajectory point
        state = TrajectoryPointStorage(interp_.get_point(), interp_.get_derivative(),
                                       interp_.get_second_derivative());
        state.previous_type = TrajectoryType::JOINTSPACE;
        state.time_ = time;
        return state;
    }
};

}  // namespace orc::trajectory
