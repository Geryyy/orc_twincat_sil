#pragma once

#include "orc/RobotTraits.h"
#include "orc/control/ControllerBases.h"

namespace orc::control {
template <int DOF>
class VelocityController : public JointTrackingController<DOF> {
    DECLARE_ROBOT_TRAITS_USINGS(DOF)
    using RobotData = orc::robots::RobotData<DOF>;

private:
    JointMatrix K0; /**< Position error feedback matrix */
    using ControllerBase<DOF>::robot_data;

public:
    /**
     * @brief Construct a new Velocity Controller object
     *
     * @param data RobotData object
     * @param K0 Position error feedback matrix
     */
    VelocityController(const RobotData& data, JointMatrix K0)
        : JointTrackingController<DOF>(data), K0(K0) {}

    /**
     * @brief Returns the desired velocity computed.
     *
     * @return JointVector Desired velocity
     */
    JointVector update() {
        JointVector vel_d = robot_data.q_dot_d - K0 * (robot_data.q_act - robot_data.q_d);
        return vel_d;
    }

    void reset() {}
};
}  // namespace orc::control
