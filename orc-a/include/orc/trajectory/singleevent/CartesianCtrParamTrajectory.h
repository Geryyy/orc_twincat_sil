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
class CartesianCtrParamTrajectory : public TrajectoryBase<DOF> {
    using JointVector = typename RobotTraits<DOF>::JointVector;
    using CartesianCTController = orc::control::CartesianCTController<DOF>;
    using TrajectoryPointStorage = orc::trajectory::TrajectoryPointStorage<DOF>;
    using TrajectoryBase = orc::trajectory::TrajectoryBase<DOF>;
    using CartesianCTParameter = orc::control::CartesianCTParameter<DOF>;

    Time t_start_;
    CartesianCTParameter param_;

public:
    CartesianCtrParamTrajectory(Time t_start, CartesianCTParameter param)
        : TrajectoryBase(), t_start_(t_start), param_(param) {
        this->traj_type = TrajectoryType::CART_CTR_PARAM;
    }

    void init(TrajectoryPointStorage saved_state) {}

    void init() {}

    Time get_start_time() { return t_start_; }

    CartesianCTParameter get_parameter() { return param_; }

    TrajectoryPointStorage save_state(Time time) { return TrajectoryPointStorage(); }
};

}  // namespace orc::trajectory
