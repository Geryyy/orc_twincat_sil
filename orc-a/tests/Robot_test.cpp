#include <algorithm>
#include <filesystem>
#include <string>

#include "gtest/gtest.h"
#include "orc/Orc.h"
#include "orc/util/import_mujoco.h"

namespace {
namespace fs = std::filesystem;
constexpr int DOF = 7;
using JointVector = typename orc::RobotTraits<DOF>::JointVector;

/**
 * @brief Test if all loading MJB with different number of DOF into robots::Robot exits cleanly.
 *
 */
TEST(RobotTest, CheckDOFTest) {
    using orc::robots::Robot;

    std::string file = "../models/iiwa_hanging.mjb";

    mjModel* m = mj_loadModel(file.c_str(), NULL);

    EXPECT_THROW(Robot<2>(file.c_str(), 0.001, "iiwa_link_e"), std::runtime_error);
    mj_deleteModel(m);
}

/**
 * @brief Test if forward kinematics works for robots::Robot.
 *
 */
TEST(RobotTest, ForwardKinematicsTest) {
    using orc::robots::Robot;
    orc::log::start_logging(orc::log::Level::Info);
    std::string file = "../models/iiwa_hanging.mjb";
    mjModel* m = mj_loadModel(file.c_str(), NULL);
    Robot<7> robot(file.c_str(), 0.001, "iiwa_link_e");
    JointVector q;
    q.setZero();
    robot.set_q_act(q);
    robot.update(orc::Time(0, 0));
    orc::HomogeneousTransformation h0e = robot.get_current_H_0_e();
    double det = h0e.block<3, 3>(0, 0).determinant();
    EXPECT_NEAR(det, 1.0, 1e-9)
        << "Forward kinematics calculated invalid homogeneous transformation!";
    mj_deleteModel(m);
}
}  // namespace
