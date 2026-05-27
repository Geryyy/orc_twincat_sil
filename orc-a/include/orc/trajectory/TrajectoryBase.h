#pragma once

#include "orc/util/import_mujoco.h"

#include "orc/RobotTraits.h"
#include "orc/trajectory/TrajectoryPointStorage.h"
#include "orc/trajectory/TrajectoryType.h"

namespace orc::trajectory {
template <int DOF>
class TrajectoryBase {
    using JointVector = typename RobotTraits<DOF>::JointVector;
    using TrajectoryPointStorage = typename orc::trajectory::TrajectoryPointStorage<DOF>;

protected:
    TrajectoryType traj_type = TrajectoryType::INVALID;

public:
    explicit TrajectoryBase() {}

    virtual ~TrajectoryBase() = default;

    virtual void init(TrajectoryPointStorage saved_state) = 0;

    virtual void init() = 0;

    [[nodiscard]] virtual Time get_start_time() = 0;

    virtual TrajectoryPointStorage save_state(Time time) = 0;

    [[nodiscard]] TrajectoryType get_trajectory_type() const { return traj_type; }
};

}  // namespace orc::trajectory
