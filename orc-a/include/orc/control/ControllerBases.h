#pragma once

#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/control/ControllerType.h"
#include "orc/robots/RobotData.h"

namespace orc::control {
template <int DOF>
class ControllerBase {
public:
    DECLARE_ROBOT_TRAITS_USINGS(DOF)
    using RobotData = orc::robots::RobotData<DOF>;

protected:
    // For inheriting classes robot_data has to be brought into scope with "using
    // ControllerBase<DOF>::robot_data;" This is a nuisance, because the base class is templated,
    // and thus not automatically in scope.
    const RobotData& robot_data;
    ControllerType type;

public:
    ControllerBase(const RobotData& robot_data, ControllerType type = ControllerType::INVALID)
        : robot_data(robot_data), type(type) {}

    ControllerType get_type() { return type; }

    virtual ~ControllerBase() = default;

    /**
     * @brief Returns joint efforts computed by controller on basis of RobotDatat
     *
     * @return JointVector Joint efforts
     */
    virtual JointVector update() = 0;

    /**
     * @brief Resets the controller to initial state
     *
     */
    virtual void reset() = 0;
};

template <int DOF>
class JointTrackingController : public ControllerBase<DOF> {
public:
    using typename ControllerBase<DOF>::JointVector;
    using typename ControllerBase<DOF>::RobotData;

    JointTrackingController(const RobotData& robot_data,
                            ControllerType type = ControllerType::INVALID)
        : ControllerBase<DOF>(robot_data, type) {}

    /**
     * @brief Resets the controller to initial state
     *
     */
    virtual void reset() = 0;
};

template <int DOF>
class PoseTrackingController : public ControllerBase<DOF> {
public:
    using typename ControllerBase<DOF>::JointVector;
    using typename ControllerBase<DOF>::RobotData;
    using CartesianVector = orc::CartesianVector;
    using PoseVector = orc::PoseVector;

    PoseTrackingController(const RobotData& robot_data,
                           ControllerType type = ControllerType::INVALID)
        : ControllerBase<DOF>(robot_data, type) {}

    /**
     * @brief Resets the controller to initial state
     *
     */
    virtual void reset() = 0;
};
}  // namespace orc::control
