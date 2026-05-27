#pragma once

#include <cmath>
#include <vector>
#include "orc/util/import_eigen.h"

#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/Splines"
#include "orc/control/Controller.h"
#include "orc/interpolator/Interpolator.h"
#include "orc/trajectory/TrajectoryBase.h"
#include "orc/trajectory/TrajectoryPointStorage.h"
#include "orc/trajectory/TrajectoryType.h"

namespace orc::trajectory {

template <int DOF>
class NullspaceTrajectory : public TrajectoryBase<DOF> {
    using JointVector = typename RobotTraits<DOF>::JointVector;
    using CartesianCTController = typename orc::control::CartesianCTController<DOF>;
    using TrajectoryPointStorage = typename orc::trajectory::TrajectoryPointStorage<DOF>;
    using TrajectoryBase = typename orc::trajectory::TrajectoryBase<DOF>;

    Time t_start_;
    JointVector q_d_NS_;

public:
    NullspaceTrajectory(Time t_start, JointVector q_d_NS)
        : TrajectoryBase(), t_start_(t_start), q_d_NS_(q_d_NS) {
        this->traj_type = TrajectoryType::NULLSPACE;
    }
    void init(TrajectoryPointStorage saved_state) {}

    void init() {}

    Time get_start_time() { return t_start_; }

    JointVector get_nullspace_joint_state() { return q_d_NS_; }

    TrajectoryPointStorage save_state(Time time) { return TrajectoryPointStorage(); }
};

}  // namespace orc::trajectory
