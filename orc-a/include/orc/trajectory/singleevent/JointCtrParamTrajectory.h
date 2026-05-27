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
class JointCtrParamTrajectory : public TrajectoryBase<DOF> {
    using JointVector = typename RobotTraits<DOF>::JointVector;
    using JointCTController = orc::control::JointCTController<DOF>;
    using TrajectoryPointStorage = orc::trajectory::TrajectoryPointStorage<DOF>;
    using TrajectoryBase = orc::trajectory::TrajectoryBase<DOF>;
    using JointCTParameter = orc::control::JointCTParameter<DOF>;

    Time t_start_;
    JointCTParameter param_;

public:
    JointCtrParamTrajectory(Time t_start, JointCTParameter param)
        : TrajectoryBase(), t_start_(t_start), param_(param) {
        this->traj_type = TrajectoryType::JOINT_CTR_PARAM;
    }

    void init(TrajectoryPointStorage saved_state) {}

    void init() {}

    Time get_start_time() { return t_start_; }

    JointCTParameter get_parameter() { return param_; }

    TrajectoryPointStorage save_state(Time time) { return TrajectoryPointStorage(); }
};

}  // namespace orc::trajectory
