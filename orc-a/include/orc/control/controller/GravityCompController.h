#pragma once

#include <cmath>
#include <vector>

#include "orc/util/import_mujoco.h"

#include <orc/util/quatutil.h>
#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/control/ControllerBases.h"
#include "orc/control/ControllerParameter.h"
#include "orc/sig/filter.h"
#include "orc/util/import_eigen.h"

namespace orc::control {
template <int DOF>
struct GravityCompParameter {
    using JointMatrix = typename orc::RobotTraits<DOF>::JointMatrix;
    JointMatrix D;

    /**
     * @brief Intialize GravityCompParameter with zero damping
     *
     */
    GravityCompParameter() : GravityCompParameter(JointMatrix::Zero()) {}

    /**
     * @brief Construct a new Gravity Comp Parameter object
     *
     * @param D_gc Damping matrix
     */
    explicit GravityCompParameter(JointMatrix D_gc) { D = D_gc; }
};

/**
 * @brief Implementation of a gravity compensation controller with damping.
 *
 * @tparam DOF
 */
template <int DOF>
class GravityCompController : public ControllerBase<DOF> {
    DECLARE_ROBOT_TRAITS_USINGS(DOF)
    using GravityCompParameter = typename orc::control::GravityCompParameter<DOF>;
    using RobotData = orc::robots::RobotData<DOF>;

private:
    using ControllerBase<DOF>::robot_data;
    JointMatrix D;

public:
    /**
     * @brief Construct a new Gravity Comp Controller object
     *
     * @param data RobotData
     * @param controller_param Controller parameter to be used
     */
    GravityCompController(const RobotData& data, GravityCompParameter& controller_param)
        : ControllerBase<DOF>(data), D(controller_param.D) {}

    JointVector update() {
        // TODO: TEST OUT GRAVCOMP
        /* gravity compensation with damping */
        return robot_data.G - D * robot_data.q_dot_act;
    }

    void reset() override {}
};
}  // namespace orc::control
