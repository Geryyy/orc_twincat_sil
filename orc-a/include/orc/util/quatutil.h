#pragma once

#include <cmath>

#include "orc/OrcTypes.h"

namespace orc::util {

/**
 * @brief Calculates quaternionen logarithm
 *
 * @param q (w, x, y, z) with q = w + x*i + y*j + z*k
 * @return Quatd
 */
inline Quatd quaternion_log(Quatd& q) {
    Quatd psi;
    q.normalize();
    double a = q.w();
    double v_norm = q.vec().norm();
    psi.w() = 0.0;
    double theta = std::atan2(v_norm, a);
    if (v_norm != 0.0) {
        psi.vec() = theta / v_norm * q.vec();
    } else {
        // log(identity) = zero rotation (both scalar and vector parts zero).
        psi.vec() = 0.0 * q.vec();
    }
    return psi;
}

/**
 * @brief Calculates quaternionen exponential
 *
 * @param q (w, x, y, z) with q = w + x*i + y*j + z*k
 * @return Quatd
 */
inline Quatd quaternion_exp(Quatd& q) {
    Quatd q_exp;
    double a = q.w();
    double v_norm = q.vec().norm();
    double exp_a = std::exp(a);
    double cos_v_norm = std::cos(v_norm);
    double sin_v_norm = 0.0;
    if (v_norm != 0.0) {
        sin_v_norm = std::sin(v_norm) / v_norm;
        q_exp.vec() = exp_a * sin_v_norm * q.vec();
        q_exp.w() = exp_a * cos_v_norm;
    } else {
        q_exp = Quatd(1.0, 0.0, 0.0, 0.0);  // w,x,y,z
    }
    return q_exp;
}

inline PoseVector set_pose(Vec3D& position, Quatd& orientation) {
    PoseVector pose;
    pose.template block<3, 1>(0, 0) = position;
    pose[3] = orientation.w();
    pose.template block<3, 1>(4, 0) = orientation.vec();
    return pose;
}

}  // namespace orc::util
