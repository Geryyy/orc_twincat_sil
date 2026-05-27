#pragma once

namespace orc::control {
enum class ControllerType {
    INVALID = 0,
    JOINT_CT,
    CARTESIAN_CT,
    JOINT_PDP,
    VELOCITY,
    HYBRID_FORCE_MOTION,
    SINGULAR_PERTURBATION,
    FRICTION_COMPENSATION,
    GRAVITY_COMPENSATION
};
}
