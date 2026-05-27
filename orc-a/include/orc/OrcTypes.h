#pragma once

#include "orc/util/Time.h"
#include "orc/util/import_eigen.h"

namespace orc {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#if TC_VER
#ifndef EXIT_SUCCESS
#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1
#endif  // !EXIT_SUCCESS
#endif

using HomogeneousTransformation = typename Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using CartesianVector = typename Eigen::Matrix<double, 6, 1>;
using PoseVector = typename Eigen::Matrix<double, 7, 1>;  //(x, y, z, quaternion w, quaternion x,
                                                          // quaternion y, quaternion z)
using CartesianMatrix = typename Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;
using CartesianPositionVector = typename Eigen::Matrix<double, 3, 1>;
using RotationMatrix = typename Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
using Quatd = typename Eigen::Quaternion<double>;
using Vec3D = typename Eigen::Matrix<double, 3, 1>;
using Vec6D = typename Eigen::Matrix<double, 6, 1>;
using Arr3D = typename Eigen::Array<double, 3, 1>;

using HybridVector =
    typename Eigen::Matrix<double, 8, 1>;  // Vector type for hybrid force-motion trajectory (7
                                           // elements for pose, 1 for force)
}  // namespace orc
