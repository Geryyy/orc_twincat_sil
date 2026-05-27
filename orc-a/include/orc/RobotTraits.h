#pragma once

#include "orc/util/import_eigen.h"

#define DECLARE_ROBOT_TRAITS_USINGS(DOF)                                   \
    using JointVector = typename orc::RobotTraits<DOF>::JointVector;       \
    using JointArray = typename orc::RobotTraits<DOF>::JointArray;         \
    using JointMatrix = typename orc::RobotTraits<DOF>::JointMatrix;       \
    using JacobianMatrix = typename orc::RobotTraits<DOF>::JacobianMatrix; \
    using JacobianInverseMatrix = typename orc::RobotTraits<DOF>::JacobianInverseMatrix;

namespace orc {
template <int DOF>
struct RobotTraits {
    using JointVector = Eigen::Matrix<double, DOF, 1>;
    using JointArray = Eigen::Array<double, DOF, 1>;
    using JointMatrix = Eigen::Matrix<double, DOF, DOF, Eigen::RowMajor>;
    using JacobianMatrix = Eigen::Matrix<double, 6, DOF, Eigen::RowMajor>;
    using JacobianInverseMatrix = Eigen::Matrix<double, DOF, 6, Eigen::RowMajor>;
};
}  // namespace orc
