#pragma once

#include <memory>

#include "orc/RobotTraits.h"
#include "orc/trajectory/Trajectories.h"
#include "orc/trajectory/TrajectoryBase.h"
#include "orc/trajectory/TrajectoryPointStorage.h"
#include "orc/trajectory/TrajectoryType.h"

namespace orc::trajectory {
template <int DOF>
class TrajectoryQueue {
    using JointVector = typename RobotTraits<DOF>::JointVector;

    using TrajectoryBase = typename orc::trajectory::TrajectoryBase<DOF>;
    using JointspaceTrajectory = typename orc::trajectory::JointspaceTrajectory<DOF>;
    using TaskspaceTrajectory = typename orc::trajectory::TaskspaceTrajectory<DOF>;
    using HybridForceMotionTrajectory = typename orc::trajectory::HybridForceMotionTrajectory<DOF>;
    using TrajectoryPointStorage = typename orc::trajectory::TrajectoryPointStorage<DOF>;

    // using JointspaceJerkTrajectory = typename orc::JointspaceJerkTrajectory<DOF>;
    // using CartesianVelocityTrajectory = typename orc::CartesianVelocityTrajectory<DOF>;
    // using JointspaceVelocityTrajectory = typename orc::JointspaceVelocityTrajectory<DOF>;

    std::vector<std::unique_ptr<TrajectoryBase>> queue;
    std::unique_ptr<TrajectoryBase> pcurrent_traj;

public:
    TrajectoryQueue() : queue(), pcurrent_traj(nullptr) {}

    void add_trajectory(std::unique_ptr<TrajectoryBase> traj_ptr) {
        queue.push_back(std::move(traj_ptr));
    }

    void add_jointspace_trajectory(JointspaceTrajectory& traj) {
        queue.push_back(std::make_unique<JointspaceTrajectory>(traj));
    }

    // void add_jointspace_jerk_trajectory(JointspaceJerkTrajectory &traj)
    // {
    // 	queue.push_back(std::make_unique<JointspaceJerkTrajectory>(traj));
    // }

    void add_taskspace_trajectory(TaskspaceTrajectory& traj) {
        queue.push_back(std::make_unique<TaskspaceTrajectory>(traj));
    }

    // void add_cartesian_velocity_trajectory(CartesianVelocityTrajectory &traj)
    // {
    // 	queue.push_back(std::make_unique<CartesianVelocityTrajectory>(traj));
    // }

    // void add_jointspace_velocity_trajectory(JointspaceVelocityTrajectory &traj)
    // {
    // 	queue.push_back(std::make_unique<JointspaceVelocityTrajectory>(traj));
    // }

    void add_hybrid_force_motion_trajectory(HybridForceMotionTrajectory& traj) {
        queue.push_back(std::make_unique<HybridForceMotionTrajectory>(traj));
    }

    void clear() {
        if (pcurrent_traj != nullptr)
            pcurrent_traj.release();

        auto it = queue.begin();

        while (it != queue.end())
            it = queue.erase(it);
    }

    TrajectoryBase* update(Time time) {
        // check queue if next trajectory should start
        auto it = queue.begin();

        while (it != queue.end()) {
            if ((*it)->get_start_time() <= time) {
                /* for single event trajectories (e.g. setting tool parameters, set nullspace in
                cartesian control, ...) only the init function is called.  Afterwards the previous
                trajectory (i.e. joint- or taskspace trajectory) is continued. */
                if ((*it)->get_trajectory_type() == TrajectoryType::NULLSPACE ||
                    (*it)->get_trajectory_type() == TrajectoryType::JOINT_CTR_PARAM ||
                    (*it)->get_trajectory_type() == TrajectoryType::CART_CTR_PARAM) {
                    // execute single event and destroy trajectory afterwards.
                    pcurrent_traj = std::move(*it);
                } else {
                    TrajectoryPointStorage saved_traj_state;

                    if (pcurrent_traj != nullptr)  // save state of old trajectory
                        saved_traj_state = pcurrent_traj->save_state(time);

                    // new trajectory
                    pcurrent_traj = std::move(*it);

                    if (pcurrent_traj != nullptr)  // init trajectory
                        pcurrent_traj->init(saved_traj_state);
                }
                // remove traj pointer from queue
                it = std::move(queue.erase(it));
            } else
                ++it;
        }

        return pcurrent_traj.get();
    }

    [[nodiscard]] TrajectoryType get_current_trajectory_type() const {
        if (pcurrent_traj != NULL)
            return pcurrent_traj->get_trajectory_type();
        else
            return TrajectoryType::INVALID;
    }

    [[nodiscard]] size_t get_queue_size() const { return queue.size(); }
};
}  // namespace orc::trajectory
