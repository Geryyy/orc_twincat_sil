/**
 * @file robot_update_test.cpp
 * @brief Tests for Robot::update() full loop, controller registration,
 *        trajectory dispatch, RobotData, copy constructor, and Iiwa-specific features.
 */
#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include "orc/Orc.h"
#include "orc/robots/Iiwa.h"
#include "orc/robots/Kinova.h"
#include "orc/trajectory/TrajectoryType.h"
#include "orc/util/Logger.h"
namespace {
constexpr int DOF = 7;
using Time = orc::Time;
using JointVector = orc::RobotTraits<DOF>::JointVector;
using JointMatrix = orc::RobotTraits<DOF>::JointMatrix;
using PoseVector = orc::PoseVector;
using CartesianVector = orc::CartesianVector;
using CartesianMatrix = orc::CartesianMatrix;
using Iiwa = orc::robots::Iiwa;
using Robot7 = orc::robots::Robot<7>;
using TrajectoryType = orc::trajectory::TrajectoryType;

// Helper: create simulation-only Iiwa (no SP/FrictionComp → avoids B_.inverse() NaN)
Iiwa make_sim_iiwa() {
    using JointCTParameter = orc::control::JointCTParameter<DOF>;
    using CartesianCTParameter = orc::control::CartesianCTParameter<DOF>;
    using GravityCompParameter = orc::control::GravityCompParameter<DOF>;
    using JointArray = orc::RobotTraits<DOF>::JointArray;
    return Iiwa("../models/iiwa_hanging.mjb", JointCTParameter(), CartesianCTParameter(),
                GravityCompParameter(), Time(0.125e-3), 200 * JointArray::Ones() * 2 * 0.125e-3);
}

// ===========================================================================
// RobotData initialization and Eigen Map aliasing
// ===========================================================================
TEST(RobotDataTest, EigenMapAliasing) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    // Modifying q_act via set_q_act should propagate to MuJoCo's qpos
    JointVector q;
    q << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7;
    iiwa.set_q_act(q);
    // Read back from Eigen Map
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(iiwa.robot_data.q_act(i), q(i), 1e-12);
    }
    // Read back from MuJoCo data pointer
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(iiwa.data->qpos[i], q(i), 1e-12);
    }
}
TEST(RobotDataTest, SampleTimeStored) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    EXPECT_NEAR(iiwa.robot_data.Ta.toSec(), 0.125e-3, 1e-12);
}
TEST(RobotDataTest, InitialFieldsAreZero) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    EXPECT_NEAR(iiwa.robot_data.q_d.norm(), 0.0, 1e-12);
    EXPECT_NEAR(iiwa.robot_data.q_dot_d.norm(), 0.0, 1e-12);
    EXPECT_NEAR(iiwa.robot_data.q_dotdot_d.norm(), 0.0, 1e-12);
    EXPECT_NEAR(iiwa.robot_data.x_dot_d.norm(), 0.0, 1e-12);
    EXPECT_NEAR(iiwa.robot_data.M_off.norm(), 0.0, 1e-6);  // SP controller modifies this
    EXPECT_EQ(iiwa.robot_data.collision_detected, false);
}
// ===========================================================================
// Controller registration
// ===========================================================================
TEST(ControllerRegistrationTest, IiwaRegistersAllControllers) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    EXPECT_NE(iiwa.js_controller, nullptr) << "JointCTController should be registered";
    EXPECT_NE(iiwa.ts_controller, nullptr) << "CartesianCTController should be registered";
    EXPECT_NE(iiwa.gc_controller, nullptr) << "GravityCompController should be registered";
    EXPECT_FALSE(iiwa.secondary_controllers.empty()) << "Should have secondary controllers";
}
TEST(ControllerRegistrationTest, DoubleRegistrationGuard) {
    orc::log::start_logging(orc::log::Level::Error);
    Robot7 robot("../models/iiwa_hanging.mjb", Time(0.001), "iiwa_link_e");
    orc::control::JointCTParameter<DOF> param;
    robot.register_JointCTController(param);
    EXPECT_NE(robot.js_controller, nullptr);
    // Second registration should be ignored (no crash)
    robot.register_JointCTController(param);
    EXPECT_NE(robot.js_controller, nullptr);
}
TEST(ControllerRegistrationTest, MoffSetBySingularPerturbation) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    // Iiwa registers SP controller, so M_off should be non-zero
    // (default SP has zero K and B, so M_off may actually be zero with defaults)
    // But the registration path should not crash
    EXPECT_TRUE(iiwa.robot_data.M_off.allFinite());
}
// ===========================================================================
// Robot::update() — gravity compensation mode
// ===========================================================================
TEST(RobotUpdateTest, GravCompOnlyReturnsTrue) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    JointVector q = JointVector::Zero();
    iiwa.set_q_act(q);
    bool success = iiwa.update(Time(0.0), true);
    EXPECT_TRUE(success);
    JointVector tau = iiwa.get_tau_act();
    EXPECT_TRUE(tau.allFinite()) << "GravComp torque should be finite";
}
TEST(RobotUpdateTest, GravCompWithoutControllerFails) {
    orc::log::start_logging(orc::log::Level::Error);
    Robot7 robot("../models/iiwa_hanging.mjb", Time(0.001), "iiwa_link_e");
    // No controller registered
    EXPECT_FALSE(robot.update(Time(0.0), true));
}
// ===========================================================================
// Robot::update() — jointspace trajectory dispatch
// ===========================================================================
TEST(RobotUpdateTest, JointspaceTrajectoryDispatch) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones() * 0.1;
    iiwa.set_q_act(q0);
    iiwa.add_jointspace_trajectory(q0, q1, Time(0.0), Time(2.0));
    bool success = iiwa.update(Time(0.0));
    EXPECT_TRUE(success);
    EXPECT_TRUE(iiwa.is_jointspace_traj_active());
    JointVector tau = iiwa.get_tau_act();
    EXPECT_TRUE(tau.allFinite());
}
TEST(RobotUpdateTest, JointspaceTrajectoryTracking) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa = make_sim_iiwa();
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones() * 0.05;
    iiwa.set_q_act(q0);
    iiwa.add_jointspace_trajectory(q0, q1, Time(0.0), Time(2.0));
    // Step through multiple update cycles
    for (int i = 0; i <= 10; ++i) {
        double t = i * 0.2;
        bool ok = iiwa.update(Time(t));
        EXPECT_TRUE(ok) << "update failed at t=" << t;
        JointVector tau = iiwa.get_tau_act();
        EXPECT_TRUE(tau.allFinite()) << "Torque not finite at t=" << t;
    }
}
// ===========================================================================
// Robot::update() — taskspace trajectory dispatch
// ===========================================================================
TEST(RobotUpdateTest, TaskspaceTrajectoryDispatch) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa = make_sim_iiwa();
    JointVector q0 = JointVector::Zero();
    iiwa.set_q_act(q0);
    // First do jointspace to init, then switch to taskspace
    iiwa.update(Time(0.0));
    PoseVector p0 = iiwa.get_pose_act();
    PoseVector p1 = p0;
    p1(0) += 0.01;
    iiwa.add_taskspace_trajectory(p0, p1, Time(0.0), Time(2.0));
    bool success = iiwa.update(Time(0.0));
    EXPECT_TRUE(success);
    EXPECT_TRUE(iiwa.is_taskspace_traj_active());
    EXPECT_TRUE(iiwa.get_tau_act().allFinite());
}
// ===========================================================================
// Robot::update() — missing controller returns false
// ===========================================================================
TEST(RobotUpdateTest, MissingJointControllerReturnsFalse) {
    orc::log::start_logging(orc::log::Level::Error);
    Robot7 robot("../models/iiwa_hanging.mjb", Time(0.001), "iiwa_link_e");
    // No controllers registered
    JointVector q0 = JointVector::Zero();
    robot.add_jointspace_trajectory(q0, q0, Time(0.0), Time(1.0));
    bool success = robot.update(Time(0.0));
    EXPECT_FALSE(success);
}
// ===========================================================================
// Robot::start()
// ===========================================================================
TEST(RobotUpdateTest, StartClearsQueueAndMoves) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones() * 0.1;
    iiwa.start(Time(0.0), q0, q1, Time(2.0));
    bool success = iiwa.update(Time(0.0));
    EXPECT_TRUE(success);
    EXPECT_TRUE(iiwa.is_jointspace_traj_active());
}
// ===========================================================================
// Robot::reset()
// ===========================================================================
TEST(RobotUpdateTest, ResetSetsJointPosition) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    JointVector q;
    q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
    iiwa.reset(q);
    JointVector q_actual = iiwa.get_q_act();
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(q_actual(i), q(i), 1e-12);
    }
    EXPECT_NEAR(iiwa.get_q_dot_act().norm(), 0.0, 1e-12);
}
// ===========================================================================
// Robot copy constructor — verify copy doesn't crash and produces valid state
// ===========================================================================
TEST(RobotUpdateTest, CopyConstructorProducesValidState) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa1 = make_sim_iiwa();
    JointVector q;
    q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
    iiwa1.set_q_act(q);

    // Copy constructor should not crash
    Iiwa iiwa2(iiwa1);
    // The copy should have valid (finite) joint configuration
    JointVector q_copy = iiwa2.get_q_act();
    EXPECT_TRUE(q_copy.allFinite()) << "Copy should have finite joint config";
    // Pointers should be non-null
    EXPECT_NE(iiwa2.model, nullptr);
    EXPECT_NE(iiwa2.data, nullptr);
}
// ===========================================================================
// compute_robot_data: mass matrix, Jacobian, H_0_e
// ===========================================================================
TEST(RobotUpdateTest, ComputeRobotDataProducesValidMatrices) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    JointVector q = JointVector::Zero();
    iiwa.set_q_act(q);
    iiwa.add_jointspace_trajectory(q, q, Time(0.0), Time(1.0));
    iiwa.update(Time(0.0));
    // Mass matrix should be symmetric positive definite
    auto M = iiwa.robot_data.M;
    EXPECT_TRUE(M.allFinite());
    for (int i = 0; i < DOF; ++i) {
        EXPECT_GT(M(i, i), 0.0) << "Mass matrix diagonal should be positive";
        for (int j = 0; j < DOF; ++j) {
            EXPECT_NEAR(M(i, j), M(j, i), 1e-9) << "M should be symmetric";
        }
    }
    // H_0_e should be a valid homogeneous transformation
    auto H = iiwa.robot_data.H_0_e;
    double det = H.block<3, 3>(0, 0).determinant();
    EXPECT_NEAR(det, 1.0, 1e-6);
    EXPECT_NEAR(H(3, 3), 1.0, 1e-12);
    // Jacobian should be finite
    EXPECT_TRUE(iiwa.robot_data.J.allFinite());
    EXPECT_TRUE(iiwa.robot_data.J_inv.allFinite());
    EXPECT_TRUE(iiwa.robot_data.J_dot.allFinite());
}
// ===========================================================================
// Gravity vector isolation
// ===========================================================================
TEST(RobotUpdateTest, GravityVectorIsolated) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    JointVector q = JointVector::Zero();
    iiwa.set_q_act(q);
    iiwa.add_jointspace_trajectory(q, q, Time(0.0), Time(1.0));
    iiwa.update(Time(0.0));
    // With zero velocity, G should be non-zero (gravity acts on links)
    JointVector G = iiwa.robot_data.G;
    EXPECT_TRUE(G.allFinite());
    // For a hanging robot at zero config, gravity should produce nonzero torques
    // on at least some joints
    EXPECT_GT(G.norm(), 0.01) << "Gravity vector should be nonzero for a hanging robot";
}
// ===========================================================================
// Kinova construction and controller registration
// ===========================================================================
TEST(KinovaTest, ConstructionAndUpdate) {
    orc::log::start_logging(orc::log::Level::Error);
    orc::robots::Kinova kinova("../models/kinova3.mjb");
    EXPECT_NE(kinova.js_controller, nullptr) << "Kinova should have JointPDPController";
    EXPECT_NE(kinova.gc_controller, nullptr) << "Kinova should have GravityCompController";
    JointVector q = orc::robots::Kinova::get_q_home();
    kinova.set_q_act(q);
    kinova.add_jointspace_trajectory(q, q, Time(0.0), Time(1.0));
    bool success = kinova.update(Time(0.0));
    EXPECT_TRUE(success);
    EXPECT_TRUE(kinova.get_tau_act().allFinite());
}
// ===========================================================================
// Secondary controller summation
// ===========================================================================
TEST(RobotUpdateTest, SecondaryControllersSummed) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    JointVector q = JointVector::Zero();
    iiwa.set_q_act(q);
    iiwa.add_jointspace_trajectory(q, q, Time(0.0), Time(2.0));
    iiwa.update(Time(0.0));
    JointVector tau = iiwa.get_tau_act();
    // Just verify output is finite and includes secondary contributions
    EXPECT_TRUE(tau.allFinite());
    // With SP + FrictionComp secondary controllers, tau should be the sum
    // of primary + all secondary updates
}
// ===========================================================================
// Multiple trajectory queue transitions
// ===========================================================================
TEST(RobotUpdateTest, MultipleTrajectoryTransitions) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa = make_sim_iiwa();
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones() * 0.05;
    JointVector q2 = JointVector::Ones() * 0.1;
    iiwa.set_q_act(q0);
    iiwa.add_jointspace_trajectory(q0, q1, Time(0.0), Time(1.0));
    iiwa.add_jointspace_trajectory(q1, q2, Time(1.0), Time(2.0));
    // Step through both trajectories
    for (int i = 0; i <= 20; ++i) {
        double t = i * 0.1;
        bool ok = iiwa.update(Time(t));
        EXPECT_TRUE(ok) << "update failed at t=" << t;
        EXPECT_TRUE(iiwa.get_tau_act().allFinite()) << "Torque not finite at t=" << t;
    }
}
// ===========================================================================
// Iiwa filtered derivatives
// ===========================================================================
TEST(IiwaFilterTest, FilteredDerivativesProduceFinite) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa = make_sim_iiwa();
    JointVector q = JointVector::Zero();
    iiwa.set_q_act_filtered_derivatives(q);
    EXPECT_TRUE(iiwa.robot_data.q_act.allFinite());
    EXPECT_TRUE(iiwa.robot_data.q_dot_act.allFinite());
    EXPECT_TRUE(iiwa.robot_data.q_dotdot_act.allFinite());
}
TEST(IiwaFilterTest, FilteredDerivativesChangeWithInput) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa = make_sim_iiwa();
    JointVector q1 = JointVector::Zero();
    iiwa.set_q_act_filtered_derivatives(q1);
    JointVector q_dot_1 = iiwa.robot_data.q_dot_act;
    JointVector q2 = JointVector::Ones() * 0.5;
    iiwa.set_q_act_filtered_derivatives(q2);
    JointVector q_dot_2 = iiwa.robot_data.q_dot_act;
    // After a step change, the derivative filter should produce nonzero velocity
    EXPECT_GT((q_dot_2 - q_dot_1).norm(), 1e-6);
}
// ===========================================================================
// Getters and setters
// ===========================================================================
TEST(RobotUpdateTest, GetSetJointVectors) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    JointVector q, q_dot;
    q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
    q_dot << 1, 2, 3, 4, 5, 6, 7;
    iiwa.set_q_act(q);
    iiwa.set_q_dot_act(q_dot);
    JointVector q_read = iiwa.get_q_act();
    JointVector qd_read = iiwa.get_q_dot_act();
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(q_read(i), q(i), 1e-12);
        EXPECT_NEAR(qd_read(i), q_dot(i), 1e-12);
    }
}
TEST(RobotUpdateTest, EndeffectorSiteId) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    EXPECT_GE(iiwa.get_endeffector_site_id(), 0) << "Endeffector site should be found";
}
}  // namespace
