#pragma once

#include <cstdint>

namespace orc::trajectory {
enum class TrajectoryType : uint16_t {
    INVALID = 0,
    STOP,
    /* normal trajectories */
    JOINTSPACE,
    TASKSPACE,
    /* single event trajectories */
    NULLSPACE,
    TOOLPARAM,
    JOINT_CTR_PARAM,
    CART_CTR_PARAM,
    /* fancy trajectories */
    JOINTSPACE_JERK,
    TASKSPACE_JERK,
    CARTESIAN_VELOCITY,
    JOINTSPACE_VELOCITY,
    /* Dense trajectories (no interpolation) */
    DENSE_JOINTSPACE,
    /* Hybrid Force/Motion trajectory*/
    HYBRID_FORCE_MOTION,
};
}  // namespace orc::trajectory
