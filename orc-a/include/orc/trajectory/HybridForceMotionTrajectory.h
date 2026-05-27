#pragma once

#include <cmath>
#include <vector>

#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/control/Controller.h"
#include "orc/interpolator/jointspace/SplineJointInterpolator.h"
#include "orc/robots/RobotData.h"
#include "orc/trajectory/TaskspaceTrajectory.h"
#include "orc/trajectory/TrajectoryBase.h"
#include "orc/trajectory/TrajectoryPointStorage.h"
#include "orc/trajectory/TrajectoryType.h"
#include "orc/util/Logger.h"

namespace orc::trajectory {
template <int DOF>
class HybridForceMotionTrajectory : public TrajectoryBase<DOF> {
    using TrajectoryPointStorage = typename orc::trajectory::TrajectoryPointStorage<DOF>;
    using TaskspaceTrajectory = orc::trajectory::TaskspaceTrajectory<DOF>;
    using ForceInterpolator = orc::interpolator::SplineJointInterpolator<1>;
    using RobotData = orc::robots::RobotData<DOF>;

    using PoseVector = orc::PoseVector;
    using ForceVector =
        typename orc::RobotTraits<1>::JointVector; /**< Dummy class needed for interpolator to work
                                                      on 1D inputs */

    std::vector<ForceVector> converted_forces;
    TaskspaceTrajectory ts_traj;
    ForceInterpolator force_interp;

    double force_ = 0.;
    PoseVector pose_;
    CartesianVector x_dot_, x_dotdot_;

public:
    HybridForceMotionTrajectory(std::vector<PoseVector>& pose_vectors, std::vector<double>& forces,
                                std::vector<Time>& time_points)
        : TrajectoryBase<DOF>(),
          converted_forces(transform_elements_to_vector(forces)),
          ts_traj(pose_vectors, time_points),
          force_interp(time_points, converted_forces),
          pose_(PoseVector::Zero()),
          x_dot_(CartesianVector::Zero()),
          x_dotdot_(CartesianVector::Zero()) {
        this->traj_type = TrajectoryType::HYBRID_FORCE_MOTION;
    }

    void init() {
        ts_traj.init();
        force_interp.init();
    }

    void init(TrajectoryPointStorage saved_state) {
        ForceVector f_zero;
        f_zero(0, 0) = 0.;

        if (saved_state.previous_type == TrajectoryType::HYBRID_FORCE_MOTION) {
            orc::log::write_debug("HybridForceMotionTrajectory::init: saved state");
            ts_traj.init(saved_state);
            ForceVector force_vec;
            force_vec << saved_state.force_;
            force_interp.init(force_vec, f_zero, f_zero);
        } else {
            ForceVector f_init = converted_forces[0];
            orc::log::write_debug("HybridForceMotionTrajectory::init: no saved state");
            ts_traj.init();
            force_interp.init(f_init, f_zero, f_zero);
        }
    }

    [[nodiscard]] Time get_start_time() override { return force_interp.get_start_time(); }

    TrajectoryPointStorage save_state(Time time) {
        TrajectoryPointStorage state;
        update(time);
        state = TrajectoryPointStorage(pose_, x_dot_, x_dotdot_, force_);
        state.previous_type = TrajectoryType::HYBRID_FORCE_MOTION;
        return state;
    }

    void update(Time t) {
        ts_traj.update(t);
        force_interp.update(t);

        pose_ = ts_traj.get_pose();
        x_dot_ = ts_traj.get_x_dot();
        x_dotdot_ = ts_traj.get_x_dotdot();
        force_ = force_interp.get_point().value();
    }

    void update(Time t, RobotData& data) {
        update(t);

        data.pose_d = pose_;
        data.x_dot_d = x_dot_;
        data.x_dotdot_d = x_dotdot_;
        data.force_d = force_;
    }

    [[nodiscard]] PoseVector get_pose() const { return ts_traj.get_pose(); }
    [[nodiscard]] CartesianVector get_x_dot() const { return ts_traj.get_x_dot(); }
    [[nodiscard]] CartesianVector get_x_dotdot() const { return ts_traj.get_x_dotdot(); }
    [[nodiscard]] double get_force() const { return force_; }

private:
    std::vector<ForceVector> transform_elements_to_vector(std::vector<double> vec_in) {
        std::vector<ForceVector> eigen_forces;
        eigen_forces.reserve(vec_in.size());
        for (double f : vec_in) {
            ForceVector m;
            m(0, 0) = f;
            eigen_forces.push_back(m);
        }
        return eigen_forces;
    }
};
}  // namespace orc::trajectory
