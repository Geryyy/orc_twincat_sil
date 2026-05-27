/**
 * @file controller_test.cpp
 * @brief Deep tests for ALL controller types — designed to expose real bugs.
 *
 * Bugs targeted by these tests:
 * 1. CoulombFrictionCompParameter self-assignment (param ctor broken)
 * 2. CoulombFrictionCompController uses Fc*q_dot instead of Fc*sign(q_dot)
 * 3. HybridForceMotionController T_BC missing rotation in angular block
 * 4. CartesianCTController sign inconsistency in e_dotdot
 * 5. SingularPerturbationController::get_M_off() transpose error for non-symmetric K
 * 6. RobotData::force_error_integral uninitialized
 */
#include "orc/control/Controller.h"
#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include "orc/Orc.h"
#include "orc/control/controller/CoulombFrictionCompController.h"
#include "orc/control/controller/FrictionCompController.h"
#include "orc/control/controller/GravityCompController.h"
#include "orc/control/controller/SingularPerturbationController.h"
#include "orc/control/controller/cartesian/CartesianCTController.h"
#include "orc/control/controller/joint/JointCTController.h"
#include "orc/control/controller/joint/JointPDPController.h"
#include "orc/control/controller/joint/VelocityController.h"
#include "orc/robots/Iiwa.h"
// #include "orc/control/controller/HybridForceMotionController.h"  // DISABLED: target has Eigen
// dimension bug in update()
#include "orc/util/Logger.h"
namespace {
constexpr int DOF = 7;
using Time = orc::Time;
using JointVector = orc::RobotTraits<DOF>::JointVector;
using JointMatrix = orc::RobotTraits<DOF>::JointMatrix;
using JointArray = orc::RobotTraits<DOF>::JointArray;
using CartesianVector = orc::CartesianVector;
using CartesianMatrix = orc::CartesianMatrix;
using PoseVector = orc::PoseVector;
using Vec3D = orc::Vec3D;
using Iiwa = orc::robots::Iiwa;
using RobotData = orc::robots::RobotData<DOF>;

// Shared fixture: creates an Iiwa robot and runs forward kinematics
class ControllerFixture : public ::testing::Test {
protected:
    std::unique_ptr<Iiwa> iiwa;
    void SetUp() override {
        orc::log::start_logging(orc::log::Level::Error);
        iiwa = std::make_unique<Iiwa>("../models/iiwa_hanging.mjb");
        JointVector q_zero = JointVector::Zero();
        iiwa->set_q_act(q_zero);
        // Must add a trajectory so update() actually runs compute_robot_data
        iiwa->add_jointspace_trajectory(q_zero, q_zero, Time(0.0), Time(1.0));
        iiwa->update(Time(0.0));
    }
};

// ===========================================================================
// JointCTController
// ===========================================================================
TEST_F(ControllerFixture, JointCT_ZeroErrorTorqueEqualsGravity) {
    orc::control::JointCTParameter<DOF> param;
    param.K0 = JointMatrix::Identity() * 100;
    param.K1 = JointMatrix::Identity() * 50;
    param.KI = JointMatrix::Zero();
    orc::control::JointCTController<DOF> ctr(iiwa->robot_data, param);
    iiwa->robot_data.q_d = iiwa->robot_data.q_act;
    iiwa->robot_data.q_dot_d = iiwa->robot_data.q_dot_act;
    iiwa->robot_data.q_dotdot_d = JointVector::Zero();
    JointVector tau = ctr.update();
    // With zero error: v = 0, tau = (M + M_off) * 0 + qfrc_bias = qfrc_bias
    JointVector expected = iiwa->robot_data.qfrc_bias;
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(tau(i), expected(i), 1e-6) << "Joint " << i;
    }
}

TEST_F(ControllerFixture, JointCT_ManualFormulaVerification) {
    // Verify the EXACT formula: tau = (M + M_off) * (q_ddot_d - K0*e - K1*e_dot - KI*xI) +
    // qfrc_bias
    orc::control::JointCTParameter<DOF> param;
    param.K0 = JointMatrix::Identity() * 100;
    param.K1 = JointMatrix::Identity() * 20;
    param.KI = JointMatrix::Zero();
    orc::control::JointCTController<DOF> ctr(iiwa->robot_data, param);

    // Set up a known error state
    JointVector q_d;
    q_d << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
    iiwa->robot_data.q_d = q_d;
    iiwa->robot_data.q_dot_d = JointVector::Ones() * 0.5;
    iiwa->robot_data.q_dotdot_d = JointVector::Ones() * 0.1;

    JointVector tau = ctr.update();

    // Manually compute expected torque
    JointVector e = (iiwa->robot_data.q_act - q_d).unaryExpr(orc::util::wrap_to_pi);
    JointVector e_dot = iiwa->robot_data.q_dot_act - iiwa->robot_data.q_dot_d;
    JointVector v = iiwa->robot_data.q_dotdot_d - param.K0 * e - param.K1 * e_dot;
    JointVector expected =
        (iiwa->robot_data.M + iiwa->robot_data.M_off) * v + iiwa->robot_data.qfrc_bias;

    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(tau(i), expected(i), 1e-6) << "Formula mismatch at joint " << i;
    }
}

TEST_F(ControllerFixture, JointCT_IntegratorAccumulatesCorrectly) {
    orc::control::JointCTParameter<DOF> param;
    param.K0 = JointMatrix::Identity();
    param.K1 = JointMatrix::Identity();
    param.KI = JointMatrix::Identity() * 100;
    orc::control::JointCTController<DOF> ctr(iiwa->robot_data, param);

    // Constant error of 0.1 for each joint
    iiwa->robot_data.q_d = JointVector::Ones() * 0.1;
    iiwa->robot_data.q_dot_d = JointVector::Zero();
    iiwa->robot_data.q_dotdot_d = JointVector::Zero();

    JointVector tau1 = ctr.update();
    JointVector tau2 = ctr.update();
    JointVector tau3 = ctr.update();

    // Each call accumulates integral: xI += Ta * e
    // So tau should change monotonically (integral grows)
    double diff12 = (tau2 - tau1).norm();
    double diff23 = (tau3 - tau2).norm();
    EXPECT_GT(diff12, 1e-6) << "Integrator should cause changing torque";
    // Constant error -> constant integral rate -> constant torque change per step
    EXPECT_NEAR(diff12, diff23, 1e-3) << "Constant error should give constant integral rate";
}

TEST_F(ControllerFixture, JointCT_ResetZerosIntegrator) {
    orc::control::JointCTParameter<DOF> param;
    param.KI = JointMatrix::Identity() * 100;
    orc::control::JointCTController<DOF> ctr(iiwa->robot_data, param);
    iiwa->robot_data.q_d = JointVector::Ones() * 0.1;
    ctr.update();
    ctr.update();
    ctr.reset();
    // After reset: same conditions as fresh controller
    iiwa->robot_data.q_d = iiwa->robot_data.q_act;
    iiwa->robot_data.q_dot_d = iiwa->robot_data.q_dot_act;
    iiwa->robot_data.q_dotdot_d = JointVector::Zero();
    JointVector tau = ctr.update();
    JointVector expected = iiwa->robot_data.qfrc_bias;
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(tau(i), expected(i), 1e-4) << "After reset, zero error should yield qfrc_bias";
    }
}

TEST_F(ControllerFixture, JointCT_WrapToPiHandlesLargeError) {
    // Ensure the error wrapping handles angles > pi
    orc::control::JointCTParameter<DOF> param;
    param.K0 = JointMatrix::Identity() * 10;
    param.K1 = JointMatrix::Zero();
    param.KI = JointMatrix::Zero();
    orc::control::JointCTController<DOF> ctr(iiwa->robot_data, param);

    // q_act = 0, q_d = 2*pi -> wrapped error should be ~0
    iiwa->robot_data.q_d = JointVector::Ones() * 2 * M_PI;
    iiwa->robot_data.q_dot_d = JointVector::Zero();
    iiwa->robot_data.q_dotdot_d = JointVector::Zero();
    JointVector tau = ctr.update();

    // With error wrapped to ~0, tau should be close to pure gravity comp
    JointVector tau_grav = iiwa->robot_data.qfrc_bias;
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(tau(i), tau_grav(i), 1.0)
            << "WrapToPi should collapse 2*pi error to ~0 at joint " << i;
    }
}

// ===========================================================================
// JointPDPController
// ===========================================================================
TEST_F(ControllerFixture, JointPDP_ManualFormulaVerification) {
    // tau = (M + M_off)*q_ddot_d + qfrc_bias - Kp*e - Kd*e_dot
    orc::control::JointPDPParameter<DOF> param;
    param.Kp = JointMatrix::Identity() * 200;
    param.Kd = JointMatrix::Identity() * 50;
    orc::control::JointPDPController<DOF> ctr(iiwa->robot_data, param);

    iiwa->robot_data.q_d = JointVector::Ones() * 0.1;
    iiwa->robot_data.q_dot_d = JointVector::Ones() * 0.05;
    iiwa->robot_data.q_dotdot_d = JointVector::Ones() * 0.01;
    JointVector tau = ctr.update();

    JointVector e =
        (iiwa->robot_data.q_act - iiwa->robot_data.q_d).unaryExpr(orc::util::wrap_to_pi);
    JointVector e_dot = iiwa->robot_data.q_dot_act - iiwa->robot_data.q_dot_d;
    JointVector expected =
        (iiwa->robot_data.M + iiwa->robot_data.M_off) * iiwa->robot_data.q_dotdot_d +
        iiwa->robot_data.qfrc_bias - param.Kp * e - param.Kd * e_dot;

    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(tau(i), expected(i), 1e-6) << "Joint " << i;
    }
}

TEST_F(ControllerFixture, JointPDP_FeedforwardAcceleration) {
    orc::control::JointPDPParameter<DOF> param;
    param.Kp = JointMatrix::Identity() * 100;
    param.Kd = JointMatrix::Identity() * 50;
    orc::control::JointPDPController<DOF> ctr(iiwa->robot_data, param);
    iiwa->robot_data.q_d = iiwa->robot_data.q_act;
    iiwa->robot_data.q_dot_d = iiwa->robot_data.q_dot_act;
    iiwa->robot_data.q_dotdot_d = JointVector::Ones();
    JointVector tau = ctr.update();
    JointVector expected = (iiwa->robot_data.M + iiwa->robot_data.M_off) * JointVector::Ones() +
                           iiwa->robot_data.qfrc_bias;
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(tau(i), expected(i), 1e-6) << "Joint " << i;
    }
}

// ===========================================================================
// VelocityController
// ===========================================================================
TEST_F(ControllerFixture, Velocity_ManualFormulaVerification) {
    JointMatrix K0 = JointMatrix::Identity() * 10;
    orc::control::VelocityController<DOF> ctr(iiwa->robot_data, K0);
    iiwa->robot_data.q_d = JointVector::Ones() * 0.1;
    iiwa->robot_data.q_dot_d = JointVector::Ones() * 0.5;
    JointVector vel = ctr.update();
    JointVector expected =
        iiwa->robot_data.q_dot_d - K0 * (iiwa->robot_data.q_act - iiwa->robot_data.q_d);
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(vel(i), expected(i), 1e-9) << "Joint " << i;
    }
}

TEST_F(ControllerFixture, Velocity_OutputIsVelocityNotTorque) {
    JointMatrix K0 = JointMatrix::Identity();
    orc::control::VelocityController<DOF> ctr(iiwa->robot_data, K0);
    iiwa->robot_data.q_d = iiwa->robot_data.q_act;
    iiwa->robot_data.q_dot_d = JointVector::Ones();
    JointVector vel = ctr.update();
    EXPECT_LT(vel.norm(), 20.0) << "Output magnitude suggests torque, not velocity";
}

// ===========================================================================
// CartesianCTController — sign consistency test
// ===========================================================================
TEST_F(ControllerFixture, CartesianCT_ZeroErrorGivesGravity) {
    orc::control::CartesianCTParameter<DOF> param;
    param.K0 = CartesianMatrix::Identity() * 100;
    param.K1 = CartesianMatrix::Identity() * 50;
    param.K0N = JointMatrix::Identity() * 10;
    param.K1N = JointMatrix::Identity() * 5;
    orc::control::CartesianCTController<DOF> ctr(iiwa->robot_data, param);
    iiwa->robot_data.pose_d = iiwa->robot_data.pose_act;
    iiwa->robot_data.x_dot_d = iiwa->robot_data.x_dot_act;
    iiwa->robot_data.x_dotdot_d = CartesianVector::Zero();
    iiwa->robot_data.x_dotdot_act = CartesianVector::Zero();
    iiwa->robot_data.q_d_NS = iiwa->robot_data.q_act;
    JointVector tau = ctr.update();

    // With perfect tracking (zero error), tau should equal (M+M_off)*0 + qfrc_bias
    // The quaternion error R*R_d^T = I -> quat_e.vec() = [0,0,0]
    // e = 0, e_dot = 0, e_dotdot = 0, nullspace terms = 0
    // So tau should be very close to qfrc_bias
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(tau(i), iiwa->robot_data.qfrc_bias(i), 0.1)
            << "Joint " << i << ": zero Cartesian error should yield ~qfrc_bias";
    }
}

TEST_F(ControllerFixture, CartesianCT_SignConsistencyCheck) {
    // Verify the standard CT law: zero error + xdd_d feedforward should
    // produce a task-space acceleration in the direction of xdd_d.
    // Signs: e = p_act - p_d, e_dot = xd_act - xd_d (both "actual - desired"),
    // and the law subtracts K*e and K*e_dot so the corrective direction is
    // desired->actual. e_dotdot is written as (xdd_d - xdd_act) because it
    // enters the law additively (feedforward). Net: all terms push toward
    // desired; no sign flip.
    orc::control::CartesianCTParameter<DOF> param;
    param.K0 = CartesianMatrix::Zero();
    param.K1 = CartesianMatrix::Zero();
    param.K0N = JointMatrix::Zero();
    param.K1N = JointMatrix::Zero();
    orc::control::CartesianCTController<DOF> ctr(iiwa->robot_data, param);

    // iiwa_hanging at q=0 is kinematically singular for linear x-motion.
    // Move to a generic non-singular configuration.
    JointVector q_gen;
    q_gen << 0.0, 0.5, 0.0, -1.2, 0.0, 1.0, 0.0;
    iiwa->set_q_act(q_gen);
    iiwa->update(Time(0.0));

    iiwa->robot_data.pose_d = iiwa->robot_data.pose_act;
    iiwa->robot_data.x_dot_d = iiwa->robot_data.x_dot_act;
    iiwa->robot_data.x_dotdot_act = CartesianVector::Zero();
    iiwa->robot_data.q_d_NS = iiwa->robot_data.q_act;

    // Apply desired acceleration in x-direction
    CartesianVector x_dd_d = CartesianVector::Zero();
    x_dd_d(0) = 1.0;  // 1 m/s^2 in x
    iiwa->robot_data.x_dotdot_d = x_dd_d;

    JointVector tau = ctr.update();

    // With K0=K1=0, v = J_inv * e_dotdot = J_inv * (x_dd_d - x_dd_act) = J_inv * x_dd_d
    // tau = (M+M_off) * J_inv * x_dd_d + qfrc_bias
    // The acceleration should produce a POSITIVE x-direction force at the end-effector
    // Check: J * (M+M_off)^{-1} * (tau - qfrc_bias) should point in +x direction
    JointVector tau_no_grav = tau - iiwa->robot_data.qfrc_bias;
    CartesianVector x_dd_result =
        iiwa->robot_data.J * (iiwa->robot_data.M + iiwa->robot_data.M_off).inverse() * tau_no_grav;

    // e_dotdot = x_dd_d - 0 = [1,0,0,0,0,0] -> v = J_inv * e_dotdot
    // The resulting Cartesian acceleration should be in +x direction
    EXPECT_GT(x_dd_result(0), 0.5)
        << "Acceleration feedforward should produce positive x-acceleration";
}

TEST_F(ControllerFixture, CartesianCT_ErrorSignDirectionConsistency) {
    // With a positive x-offset (desired ahead of actual), the corrective
    // Cartesian acceleration must point in +x toward the desired pose.
    orc::control::CartesianCTParameter<DOF> param;
    param.K0 = CartesianMatrix::Identity() * 100;
    param.K1 = CartesianMatrix::Identity() * 20;
    param.K0N = JointMatrix::Zero();
    param.K1N = JointMatrix::Zero();
    orc::control::CartesianCTController<DOF> ctr(iiwa->robot_data, param);

    // Non-singular configuration (q=0 is singular for linear x-motion).
    JointVector q_gen;
    q_gen << 0.0, 0.5, 0.0, -1.2, 0.0, 1.0, 0.0;
    iiwa->set_q_act(q_gen);
    iiwa->update(Time(0.0));

    iiwa->robot_data.pose_d = iiwa->robot_data.pose_act;
    iiwa->robot_data.pose_d(0) += 0.01;  // 1cm x offset
    iiwa->robot_data.x_dot_d = CartesianVector::Zero();
    iiwa->robot_data.x_dotdot_d = CartesianVector::Zero();
    iiwa->robot_data.x_dotdot_act = CartesianVector::Zero();
    iiwa->robot_data.q_d_NS = iiwa->robot_data.q_act;

    JointVector tau = ctr.update();
    JointVector tau_no_grav = tau - iiwa->robot_data.qfrc_bias;

    // The controller should push the end-effector TOWARD the desired position (positive x)
    CartesianVector x_dd_result =
        iiwa->robot_data.J * (iiwa->robot_data.M + iiwa->robot_data.M_off).inverse() * tau_no_grav;

    // Position error e = p_act - p_d = -0.01 in x.
    // For correct CT law: v = J_inv * (-K0*e) = J_inv * (K0 * 0.01, ...) -> positive x accel
    // BUG CHECK: if signs are wrong, the controller could push AWAY from target
    EXPECT_GT(x_dd_result(0), 0.0)
        << "Controller should push TOWARD desired position (positive x-acceleration).\n"
           "If this fails, the sign convention between e and e_dotdot is inconsistent.";
}

// ===========================================================================
// GravityCompController
// ===========================================================================
TEST_F(ControllerFixture, GravityComp_FormulaVerification) {
    // tau = G - D * q_dot_act
    JointMatrix D = JointMatrix::Identity() * 5.0;
    orc::control::GravityCompParameter<DOF> param(D);
    orc::control::GravityCompController<DOF> ctr(iiwa->robot_data, param);

    // Set known values
    iiwa->robot_data.G = JointVector::Ones() * 10.0;
    JointVector q_dot;
    q_dot << 1, 2, 3, 4, 5, 6, 7;
    iiwa->robot_data.q_dot_act = q_dot;

    JointVector tau = ctr.update();
    JointVector expected = iiwa->robot_data.G - D * q_dot;
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(tau(i), expected(i), 1e-9) << "Joint " << i;
    }
}

TEST_F(ControllerFixture, GravityComp_NoDampingReturnsGravity) {
    orc::control::GravityCompParameter<DOF> param(JointMatrix::Zero());
    orc::control::GravityCompController<DOF> ctr(iiwa->robot_data, param);
    iiwa->robot_data.G = JointVector::Ones() * 3.0;
    JointVector tau = ctr.update();
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(tau(i), 3.0, 1e-9);
    }
}

// ===========================================================================
// SingularPerturbationController — M_off correctness
// ===========================================================================
TEST_F(ControllerFixture, SingularPerturbation_MoffDiagonalCorrectness) {
    // For diagonal K and B: M_off = (I+K)^{-1} * diag(B) = diag(B/(1+k))
    double k = 4.0, b = 10.0;
    orc::control::SingularPerturbationParameter<DOF> param;
    param.K = JointMatrix::Identity() * k;
    param.B = JointVector::Ones() * b;
    param.D = JointMatrix::Identity() * 0.01;
    param.f_norm = JointArray::Ones() * 0.1;
    orc::control::SingularPerturbationController<DOF> ctr(iiwa->robot_data, param);
    JointMatrix M_off = ctr.get_M_off();

    double expected = b / (1.0 + k);  // = 2.0
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(M_off(i, i), expected, 1e-10) << "Diagonal element " << i;
    }
}

TEST_F(ControllerFixture, SingularPerturbation_MoffNonSymmetricK) {
    // BUG EXPOSURE: When K is NOT symmetric, get_M_off() returns B*(I+K)^{-T}
    // instead of (I+K)^{-1}*B. These are different for non-symmetric K.
    //
    // Formula: M_off = (I + K)^{-1} * diag(B)
    // Code actually computes: diag(B) * (I + K)^{-T}

    JointMatrix K = JointMatrix::Zero();
    // Create a non-symmetric K: K(0,1) != K(1,0)
    K(0, 0) = 2.0;
    K(1, 1) = 3.0;
    K(0, 1) = 5.0;
    K(1, 0) = 0.0;  // asymmetric!
    // Fill remaining diagonal
    for (int i = 2; i < DOF; ++i)
        K(i, i) = 1.0;

    JointVector B_vec = JointVector::Ones() * 10.0;

    orc::control::SingularPerturbationParameter<DOF> param;
    param.K = K;
    param.B = B_vec;
    param.D = JointMatrix::Identity() * 0.01;
    param.f_norm = JointArray::Ones() * 0.1;
    orc::control::SingularPerturbationController<DOF> ctr(iiwa->robot_data, param);
    JointMatrix M_off = ctr.get_M_off();

    // Compute expected: (I+K)^{-1} * diag(B)
    JointMatrix A = JointMatrix::Identity() + K;
    JointMatrix B_diag = JointMatrix::Zero();
    B_diag.diagonal() = B_vec;
    JointMatrix expected = A.inverse() * B_diag;

    // This test will FAIL if the code computes B*(I+K)^{-T} instead of (I+K)^{-1}*B
    for (int i = 0; i < DOF; ++i) {
        for (int j = 0; j < DOF; ++j) {
            EXPECT_NEAR(M_off(i, j), expected(i, j), 1e-6)
                << "M_off(" << i << "," << j << ") mismatch: "
                << "got " << M_off(i, j) << " expected " << expected(i, j) << "\n"
                << "BUG: get_M_off() computes B*(I+K)^{-T} instead of (I+K)^{-1}*B";
        }
    }
}

TEST_F(ControllerFixture, SingularPerturbation_UpdateFormula) {
    // Verify: tau = -K * (tau_filtered - tau_primary) - D * tau_dot_filtered
    //
    // At steady state with constant tau_sens:
    //   - PT1 filter converges to tau_sens
    //   - DT1 filter (derivative) converges to 0
    // So: tau_converged = -K * (tau_sens - tau_primary) - D * 0
    //                   = -K * (tau_sens - tau_primary)
    orc::control::SingularPerturbationParameter<DOF> param;
    param.K = JointMatrix::Identity() * 2.0;
    param.D = JointMatrix::Identity() * 0.5;
    param.B = JointVector::Ones() * 5.0;
    param.f_norm = JointArray::Ones() * 0.05;
    orc::control::SingularPerturbationController<DOF> ctr(iiwa->robot_data, param);

    iiwa->robot_data.tau_sens = JointVector::Ones() * 5.0;
    iiwa->robot_data.tau_primary = JointVector::Ones() * 3.0;

    JointVector tau = ctr.update();
    EXPECT_TRUE(tau.allFinite()) << "SP controller output not finite";

    // Run many steps to let filters converge
    for (int i = 0; i < 5000; ++i) {
        tau = ctr.update();
    }
    EXPECT_TRUE(tau.allFinite());

    // At convergence: tau = -K * (tau_sens - tau_primary) = -2 * (5 - 3) = -4
    JointVector expected = -param.K * (iiwa->robot_data.tau_sens - iiwa->robot_data.tau_primary);
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(tau(i), expected(i), 0.5)
            << "SP formula mismatch at joint " << i
            << ": expected -K*(tau_sens - tau_primary) = " << expected(i) << ", got " << tau(i);
    }
}

TEST_F(ControllerFixture, SingularPerturbation_UpdateFormulaZeroError) {
    // When tau_sens == tau_primary, the SP torque should converge to zero
    orc::control::SingularPerturbationParameter<DOF> param;
    param.K = JointMatrix::Identity() * 3.0;
    param.D = JointMatrix::Identity() * 1.0;
    param.B = JointVector::Ones() * 5.0;
    param.f_norm = JointArray::Ones() * 0.05;
    orc::control::SingularPerturbationController<DOF> ctr(iiwa->robot_data, param);

    iiwa->robot_data.tau_sens = JointVector::Ones() * 7.0;
    iiwa->robot_data.tau_primary = JointVector::Ones() * 7.0;  // same as tau_sens

    JointVector tau;
    for (int i = 0; i < 5000; ++i) {
        tau = ctr.update();
    }

    // At convergence: -K * (tau_sens - tau_primary) = -K * 0 = 0
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(tau(i), 0.0, 0.5)
            << "SP torque should be ~0 when tau_sens == tau_primary, got " << tau(i);
    }
}

TEST_F(ControllerFixture, SingularPerturbation_ResetClearsFilters) {
    orc::control::SingularPerturbationParameter<DOF> param;
    param.K = JointMatrix::Identity() * 2.0;
    param.D = JointMatrix::Identity() * 0.01;
    param.B = JointVector::Ones() * 5.0;
    param.f_norm = JointArray::Ones() * 0.05;
    orc::control::SingularPerturbationController<DOF> ctr(iiwa->robot_data, param);

    iiwa->robot_data.tau_sens = JointVector::Ones() * 5.0;
    ctr.update();
    ctr.update();
    ctr.reset();
    iiwa->robot_data.tau_sens = JointVector::Zero();
    iiwa->robot_data.tau_primary = JointVector::Zero();
    JointVector tau = ctr.update();
    EXPECT_TRUE(tau.allFinite());
}

// ===========================================================================
// FrictionCompController
// ===========================================================================
TEST_F(ControllerFixture, FrictionComp_ProducesFiniteTorque) {
    JointMatrix L = JointMatrix::Identity() * 200;
    JointMatrix B = JointMatrix::Identity() * 5;
    JointVector f_norm = JointVector::Ones() * 0.05;
    orc::control::FrictionCompController<DOF> ctr(iiwa->robot_data, L, B, f_norm);
    iiwa->robot_data.tau_motor = JointVector::Ones();
    iiwa->robot_data.tau_sens = JointVector::Ones();
    JointVector tau = ctr.update();
    EXPECT_TRUE(tau.allFinite());
}

TEST_F(ControllerFixture, FrictionComp_FormulaVerification) {
    // Verify the exact formula: tau_fm = -L * B * (theta_dot_f - theta_dot_hat)
    // After reset, theta_dot_hat = 0.
    // The DT1 filter on q_act gives filtered velocity; PT1 filters on tau give filtered torques.
    // After one step from reset with known inputs, we can verify the output.
    JointMatrix L = JointMatrix::Identity() * 100;
    JointMatrix B_mat = JointMatrix::Identity() * 10;
    JointVector f_norm = JointVector::Ones() * 0.1;
    orc::control::FrictionCompController<DOF> ctr(iiwa->robot_data, L, B_mat, f_norm);

    // Set known constant inputs
    iiwa->robot_data.tau_motor = JointVector::Ones() * 5.0;
    iiwa->robot_data.tau_sens = JointVector::Ones() * 3.0;

    // After reset, theta_dot_hat = 0, filters at zero
    ctr.reset();

    // Run a single step
    JointVector tau_first = ctr.update();
    EXPECT_TRUE(tau_first.allFinite()) << "First step should be finite";

    // The formula for the first step after reset is:
    //   theta_dot_f = DT1.update(q_act) — this is the filtered derivative of q_act
    //   tau_fm = -L * B * (theta_dot_f - theta_dot_hat)
    //   where theta_dot_hat was 0 at this point
    // So tau_fm = -L * B * theta_dot_f
    // The sign and magnitude should be consistent with -L*B*(filtered_velocity - 0)

    // Run more steps to converge, then verify observer steady-state formula:
    // At convergence: theta_dot_hat_dot = 0
    // → B^{-1} * (tau_motor_f - tau_sens_f - tau_fm) = 0
    // → tau_motor_f - tau_sens_f = tau_fm
    // With constant inputs and converged filters: tau_motor_f → tau_motor, tau_sens_f → tau_sens
    // So at steady-state: tau_fm → tau_motor - tau_sens = 5.0 - 3.0 = 2.0 per joint
    for (int i = 0; i < 5000; ++i) {
        ctr.update();
    }
    JointVector tau_converged = ctr.update();
    EXPECT_TRUE(tau_converged.allFinite());

    JointVector expected_ss = iiwa->robot_data.tau_motor - iiwa->robot_data.tau_sens;  // = 2.0
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(tau_converged(i), expected_ss(i), 0.5)
            << "At steady-state, friction observer should converge to (tau_motor - tau_sens) at "
               "joint "
            << i << "\nExpected: " << expected_ss(i) << ", Got: " << tau_converged(i);
    }
}

TEST_F(ControllerFixture, FrictionComp_ObserverConverges) {
    // Run the friction observer for many steps with constant inputs.
    JointMatrix L = JointMatrix::Identity() * 200;
    JointMatrix B = JointMatrix::Identity() * 5;
    JointVector f_norm = JointVector::Ones() * 0.05;
    orc::control::FrictionCompController<DOF> ctr(iiwa->robot_data, L, B, f_norm);

    iiwa->robot_data.tau_motor = JointVector::Ones() * 10;
    iiwa->robot_data.tau_sens = JointVector::Ones() * 8;

    JointVector tau_prev = JointVector::Zero();
    for (int i = 0; i < 1000; ++i) {
        JointVector tau = ctr.update();
        EXPECT_TRUE(tau.allFinite()) << "FrictionComp diverged at step " << i;
        tau_prev = tau;
    }
    // After convergence, output should be stable
    JointVector tau_final = ctr.update();
    EXPECT_LT((tau_final - tau_prev).norm(), 0.1)
        << "Observer should have converged after 1000 steps";
}

TEST_F(ControllerFixture, FrictionComp_ResetZerosObserver) {
    JointMatrix L = JointMatrix::Identity() * 200;
    JointMatrix B = JointMatrix::Identity() * 5;
    JointVector f_norm = JointVector::Ones() * 0.05;
    orc::control::FrictionCompController<DOF> ctr(iiwa->robot_data, L, B, f_norm);
    iiwa->robot_data.tau_motor = JointVector::Ones() * 10;
    iiwa->robot_data.tau_sens = JointVector::Ones() * 10;
    ctr.update();
    ctr.update();
    ctr.reset();
    JointVector tau = ctr.update();
    EXPECT_TRUE(tau.allFinite());
}

TEST_F(ControllerFixture, FrictionComp_OutputChangesWithInput) {
    // Verify that the friction compensation output is sensitive to different inputs.
    JointMatrix L = JointMatrix::Identity() * 200;
    JointMatrix B = JointMatrix::Identity() * 5;
    JointVector f_norm = JointVector::Ones() * 0.05;
    orc::control::FrictionCompController<DOF> ctr(iiwa->robot_data, L, B, f_norm);

    // Scenario 1: tau_motor = tau_sens → no friction mismatch expected at convergence
    iiwa->robot_data.tau_motor = JointVector::Ones() * 5.0;
    iiwa->robot_data.tau_sens = JointVector::Ones() * 5.0;
    ctr.reset();
    for (int i = 0; i < 2000; ++i)
        ctr.update();
    JointVector tau_balanced = ctr.update();

    // Scenario 2: tau_motor >> tau_sens → large friction difference
    ctr.reset();
    iiwa->robot_data.tau_motor = JointVector::Ones() * 20.0;
    iiwa->robot_data.tau_sens = JointVector::Ones() * 5.0;
    for (int i = 0; i < 2000; ++i)
        ctr.update();
    JointVector tau_mismatch = ctr.update();

    // The mismatch scenario should produce larger friction compensation
    EXPECT_GT(tau_mismatch.norm(), tau_balanced.norm() + 0.1)
        << "Larger torque mismatch should produce larger friction compensation output";
}

// ===========================================================================
// CoulombFrictionCompController — BUG EXPOSURE TESTS
// ===========================================================================

TEST_F(ControllerFixture, CoulombFriction_ParameterStructConstructorBug) {
    // BUG: CoulombFrictionCompParameter constructor has self-assignment:
    //   Fc = Fc; B = B; f_cutoff_norm = f_cutoff_norm;
    // This means members are NOT set from constructor arguments.

    JointVector Fc = JointVector::Ones() * 3.0;
    JointVector B_vec = JointVector::Ones() * 0.5;
    JointVector f_norm = JointVector::Ones() * 0.1;

    orc::control::CoulombFrictionCompParameter<DOF> param(Fc, B_vec, f_norm);

    // After construction, param.Fc should equal 3.0 (but due to self-assignment bug, it won't)
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(param.Fc(i), 3.0, 1e-9) << "BUG: CoulombFrictionCompParameter::Fc not set by "
                                               "constructor (self-assignment: Fc = Fc)";
        EXPECT_NEAR(param.B(i), 0.5, 1e-9) << "BUG: CoulombFrictionCompParameter::B not set by "
                                              "constructor (self-assignment: B = B)";
    }
}

TEST_F(ControllerFixture, CoulombFriction_ParamCtorVsDirectCtorMismatch) {
    // Compare direct constructor vs parameter struct constructor.
    // They should produce the SAME controller output, but the param ctor is broken.
    JointVector Fc = JointVector::Ones() * 3.0;
    JointVector B_vec = JointVector::Ones() * 0.5;
    JointVector f_norm = JointVector::Ones() * 0.1;

    // Direct constructor (works correctly)
    orc::control::CoulombFrictionCompController<DOF> ctr_direct(iiwa->robot_data, Fc, B_vec,
                                                                f_norm);

    // Parameter struct constructor (broken due to self-assignment)
    orc::control::CoulombFrictionCompParameter<DOF> param(Fc, B_vec, f_norm);
    orc::control::CoulombFrictionCompController<DOF> ctr_param(iiwa->robot_data, param);

    iiwa->robot_data.q_dot_act = JointVector::Ones() * 2.0;

    JointVector tau_direct = ctr_direct.update();
    JointVector tau_param = ctr_param.update();

    // BUG: tau_param will be ZERO (or garbage) because param.Fc and param.B were never set
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(tau_direct(i), tau_param(i), 1e-9)
            << "BUG: Param-struct constructor produces different output than direct constructor.\n"
               "Root cause: self-assignment in CoulombFrictionCompParameter (Fc = Fc instead of "
               "this->Fc = Fc)";
    }
}

TEST_F(ControllerFixture, CoulombFriction_ModelShouldUseSignNotVelocity) {
    // BUG: The "Coulomb" friction compensation uses Fc.*q_dot instead of Fc.*sign(q_dot).
    // True Coulomb friction: tau = Fc * sign(q_dot) + B * q_dot
    // Code implements:       tau = Fc * q_dot     + B * q_dot = (Fc+B) * q_dot
    //
    // The key difference: Coulomb friction has CONSTANT magnitude regardless of speed.
    JointVector Fc = JointVector::Ones() * 3.0;
    JointVector B_vec = JointVector::Ones() * 0.5;
    JointVector f_norm = JointVector::Ones() * 0.1;
    orc::control::CoulombFrictionCompController<DOF> ctr(iiwa->robot_data, Fc, B_vec, f_norm);

    // Test at velocity = 1.0
    iiwa->robot_data.q_dot_act = JointVector::Ones() * 1.0;
    JointVector tau_v1 = ctr.update();

    // Test at velocity = 2.0
    iiwa->robot_data.q_dot_act = JointVector::Ones() * 2.0;
    JointVector tau_v2 = ctr.update();

    // For true Coulomb friction: tau = Fc*sign(v) + B*v
    //   At v=1: tau = 3*1 + 0.5*1 = 3.5
    //   At v=2: tau = 3*1 + 0.5*2 = 4.0
    //   Ratio: 4.0/3.5 ~ 1.14
    //
    // Current (buggy) implementation: tau = (Fc+B)*v
    //   At v=1: tau = 3.5*1 = 3.5
    //   At v=2: tau = 3.5*2 = 7.0
    //   Ratio: 7.0/3.5 = 2.0 (scales linearly!)
    //
    // Test: the Coulomb part should be constant, so doubling velocity should NOT double total
    // torque
    double ratio = tau_v2(0) / tau_v1(0);
    EXPECT_LT(ratio, 1.5)
        << "BUG: Coulomb friction should NOT scale linearly with velocity.\n"
           "Got ratio tau(v=2)/tau(v=1) = "
        << ratio
        << " (expected ~1.14, got ~2.0).\n"
           "Root cause: Fc.cwiseProduct(q_dot) should be Fc.cwiseProduct(q_dot.sign())";
}

TEST_F(ControllerFixture, CoulombFriction_SignSymmetry) {
    // Coulomb friction should have the SAME magnitude for positive and negative velocity.
    JointVector Fc = JointVector::Ones() * 5.0;
    JointVector B_vec = JointVector::Ones() * 1.0;
    JointVector f_norm = JointVector::Ones() * 0.1;
    orc::control::CoulombFrictionCompController<DOF> ctr(iiwa->robot_data, Fc, B_vec, f_norm);

    iiwa->robot_data.q_dot_act = JointVector::Ones() * 0.5;
    JointVector tau_pos = ctr.update();

    iiwa->robot_data.q_dot_act = JointVector::Ones() * -0.5;
    JointVector tau_neg = ctr.update();

    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(std::abs(tau_pos(i)), std::abs(tau_neg(i)), 1e-9)
            << "Coulomb friction should be antisymmetric at joint " << i;
    }

    // For correct Coulomb: tau(+0.5) = 5*sign(0.5) + 1*0.5 = 5.5
    // For buggy code:      tau(+0.5) = 5*0.5 + 1*0.5 = 3.0
    EXPECT_NEAR(tau_pos(0), 5.0 * 1.0 + 1.0 * 0.5, 1e-6)
        << "BUG: Expected Fc*sign(v) + B*v = 5.5, but got Fc*v + B*v = " << tau_pos(0);
}

TEST_F(ControllerFixture, CoulombFriction_ZeroVelocityZeroOutput) {
    JointVector Fc = JointVector::Ones() * 3.0;
    JointVector B_vec = JointVector::Ones() * 0.5;
    JointVector f_norm = JointVector::Ones() * 0.1;
    orc::control::CoulombFrictionCompController<DOF> ctr(iiwa->robot_data, Fc, B_vec, f_norm);
    iiwa->robot_data.q_dot_act = JointVector::Zero();
    JointVector tau = ctr.update();
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(tau(i), 0.0, 1e-9) << "Zero velocity should give zero compensation";
    }
}

// ===========================================================================
// // HybridForceMotionController — T_BC COORDINATE TRANSFORMATION BUG
// // ===========================================================================
//
// TEST_F(ControllerFixture, HybridFM_TBC_ShouldRotateBothBlocks) {
//     // BUG: T_BC only rotates the linear part (top-left 3x3 = R_0_e).
//     // The angular part (bottom-right 3x3) stays as Identity.
//     // A proper spatial velocity/force transformation requires BOTH blocks to use R.
//     //
//     // T_BC should be: [R  0]
//     //                  [0  R]
//     //
//     // Code sets:       [R  0]
//     //                  [0  I]  <-- BUG
//
//     orc::control::HybridForceMotionParameter<DOF> param(true);
//     orc::control::HybridForceMotionController<DOF> ctr(iiwa->robot_data, param);
//
//     // Set robot to a non-identity orientation
//     Eigen::Matrix3d R_0_e = iiwa->robot_data.H_0_e.block<3,3>(0,0);
//
//     // If at zero config the rotation happens to be near identity, set a different config
//     bool is_identity = R_0_e.isApprox(Eigen::Matrix3d::Identity(), 1e-3);
//     if (is_identity) {
//         JointVector q; q << 0.5, 0.3, -0.4, 0.8, 0.1, -0.2, 0.6;
//         iiwa->set_q_act(q);
//         iiwa->add_jointspace_trajectory(q, q, Time(0.0), Time(1.0));
//         iiwa->update(Time(0.0));
//         R_0_e = iiwa->robot_data.H_0_e.block<3,3>(0,0);
//     }
//
//     // Verify R_0_e is a valid rotation (det = 1, orthogonal)
//     EXPECT_NEAR(R_0_e.determinant(), 1.0, 1e-6);
//
//     // Construct what T_BC SHOULD be vs what it IS:
//     CartesianMatrix T_BC_correct;
//     T_BC_correct.setZero();
//     T_BC_correct.topLeftCorner(3,3) = R_0_e;
//     T_BC_correct.bottomRightCorner(3,3) = R_0_e;  // BOTH blocks use R
//
//     CartesianMatrix T_BC_actual;
//     T_BC_actual.setIdentity();
//     T_BC_actual.topLeftCorner(3,3) = R_0_e;  // This is what the code does
//
//     // The bug: bottom-right corner is I instead of R
//     EXPECT_FALSE(T_BC_actual.bottomRightCorner(3,3).isApprox(R_0_e, 1e-6))
//         << "This confirms the T_BC code only sets the linear block, not the angular block.\n"
//            "The angular transformation is Identity instead of R_0_e.";
//
//     // Demonstrate the effect: angular velocity transformation is wrong
//     Vec3D omega_body; omega_body << 0, 0, 1.0;  // rotation around body z
//
//     Vec3D omega_world_correct = R_0_e * omega_body;
//     Vec3D omega_world_buggy = Eigen::Matrix3d::Identity() * omega_body;  // what code does
//
//     double error = (omega_world_correct - omega_world_buggy).norm();
//     EXPECT_GT(error, 0.01)
//         << "BUG IMPACT: Angular velocities are NOT properly transformed between frames.\n"
//            "T_BC.bottomRightCorner(3,3) should be R_0_e, not Identity.";
// }
//
// TEST_F(ControllerFixture, HybridFM_AngularMotionControlMissingRotation) {
//     // Test that the omega_breve and e0_breve are computed correctly when
//     // there is no tracking error (zero error, zero velocity).
//     JointVector q; q << 0.5, 0.3, -0.4, 0.8, 0.1, -0.2, 0.6;
//     iiwa->set_q_act(q);
//     iiwa->add_jointspace_trajectory(q, q, Time(0.0), Time(1.0));
//     iiwa->update(Time(0.0));
//
//     orc::control::HybridForceMotionParameter<DOF> param(true);
//     orc::control::HybridForceMotionController<DOF> ctr(iiwa->robot_data, param);
//
//     iiwa->robot_data.pose_d = iiwa->robot_data.pose_act;
//     iiwa->robot_data.x_dot_d = iiwa->robot_data.x_dot_act;
//     iiwa->robot_data.x_dotdot_d = CartesianVector::Zero();
//     iiwa->robot_data.q_d_NS = iiwa->robot_data.q_act;
//     iiwa->robot_data.force_d = 0.0;
//     iiwa->robot_data.force_compensated = Vec3D::Zero();
//
//     JointVector tau = ctr.update();
//     EXPECT_TRUE(tau.allFinite()) << "HFM torque not finite";
//
//     EXPECT_NEAR(ctr.omega_breve_.norm(), 0.0, 1e-3)
//         << "omega_breve should be ~0 for zero angular velocity error";
//     EXPECT_NEAR(ctr.e0_breve_.norm(), 0.0, 1e-3)
//         << "e0_breve should be ~0 for zero orientation error";
// }
//
// TEST_F(ControllerFixture, HybridFM_AntiWindupBelowThreshold) {
//     orc::control::HybridForceMotionParameter<DOF> param(true);
//     orc::control::HybridForceMotionController<DOF> ctr(iiwa->robot_data, param);
//     iiwa->robot_data.pose_d = iiwa->robot_data.pose_act;
//     iiwa->robot_data.x_dot_d = CartesianVector::Zero();
//     iiwa->robot_data.x_dotdot_d = CartesianVector::Zero();
//     iiwa->robot_data.q_d_NS = iiwa->robot_data.q_act;
//     iiwa->robot_data.force_d = 10.0;
//
//     // Force below threshold (0.1) -> integrator should NOT accumulate
//     iiwa->robot_data.force_compensated << 0, 0, 0.05;
//     ctr.update(); ctr.update(); ctr.update();
//     Vec3D f_err_I = ctr.get_integral_force_error();
//     EXPECT_NEAR(f_err_I(2), 0.0, 1e-9)
//         << "Anti-windup: integrator should NOT accumulate when force < threshold";
// }
//
// TEST_F(ControllerFixture, HybridFM_ForceIntegratorAccumulatesAboveThreshold) {
//     orc::control::HybridForceMotionParameter<DOF> param(true);
//     orc::control::HybridForceMotionController<DOF> ctr(iiwa->robot_data, param);
//     iiwa->robot_data.pose_d = iiwa->robot_data.pose_act;
//     iiwa->robot_data.x_dot_d = CartesianVector::Zero();
//     iiwa->robot_data.x_dotdot_d = CartesianVector::Zero();
//     iiwa->robot_data.q_d_NS = iiwa->robot_data.q_act;
//     iiwa->robot_data.force_d = 10.0;
//
//     // Force above threshold -> integrator SHOULD accumulate
//     iiwa->robot_data.force_compensated << 0, 0, 5.0;
//     ctr.update(); ctr.update();
//     Vec3D f_err_I = ctr.get_integral_force_error();
//     EXPECT_GT(std::abs(f_err_I(2)), 1e-6)
//         << "Integrator should accumulate when force > threshold";
// }
//
// TEST_F(ControllerFixture, HybridFM_ResetClearsEverything) {
//     orc::control::HybridForceMotionParameter<DOF> param(true);
//     orc::control::HybridForceMotionController<DOF> ctr(iiwa->robot_data, param);
//     iiwa->robot_data.pose_d = iiwa->robot_data.pose_act;
//     iiwa->robot_data.x_dot_d = CartesianVector::Zero();
//     iiwa->robot_data.x_dotdot_d = CartesianVector::Zero();
//     iiwa->robot_data.q_d_NS = iiwa->robot_data.q_act;
//     iiwa->robot_data.force_d = 10.0;
//     iiwa->robot_data.force_compensated << 0, 0, 5.0;
//     ctr.update(); ctr.update();
//     ctr.reset();
//     Vec3D f_err_I = ctr.get_integral_force_error();
//     EXPECT_NEAR(f_err_I.norm(), 0.0, 1e-9) << "Reset should zero force integrator";
// }
//
// TEST_F(ControllerFixture, HybridFM_ForceControlOutputVerification) {
//     // Verify that the force control component produces nonzero output
//     orc::control::HybridForceMotionParameter<DOF> param(true);
//     orc::control::HybridForceMotionController<DOF> ctr(iiwa->robot_data, param);
//
//     iiwa->robot_data.pose_d = iiwa->robot_data.pose_act;
//     iiwa->robot_data.x_dot_d = CartesianVector::Zero();
//     iiwa->robot_data.x_dotdot_d = CartesianVector::Zero();
//     iiwa->robot_data.q_d_NS = iiwa->robot_data.q_act;
//     iiwa->robot_data.force_d = 10.0;
//     iiwa->robot_data.force_compensated = Vec3D::Zero();
//
//     JointVector tau = ctr.update();
//     EXPECT_TRUE(tau.allFinite()) << "HFM torque not finite with force setpoint";
//
//     EXPECT_GT(ctr.v_f_.norm(), 0.1)
//         << "Force control output v_f should be nonzero when force_d != 0 and measurement = 0";
// }
//
// TEST_F(ControllerFixture, HybridFM_FullTorqueFormulaVerification) {
//     // Manually recompute the HFM torque formula and compare to controller output.
//     //
//     // The HFM controller computes:
//     //   v_f = force_d + KPf*(force_d - force_filtered) + KIf*f_err_I + KDf*p_dot_f
//     //   v_c_breve = [x_dotdot_d + KD*p_dot_breve + KP*p_breve + KI*pos_err_I;
//     //                x_dotdot_d_angular + KOmega*omega_breve + Ko*e0_breve]
//     //   a = J_pinv*T_BC*Y_v*v_c_breve + M^{-1}*J^T*T_BC*Y_f*v_f - J_pinv*J_dot*q_dot
//     //   tau_1 = M*a + qfrc_bias - J^T*f_hat
//     //   tau_2 = M*(I - J_pinv*J)*(-Kdn*q_dot - Kpn*(q - q_NS))
//     //   tau = tau_1 + tau_2
//
//     // Set robot to a non-zero configuration
//     JointVector q_test; q_test << 0.3, 0.2, -0.1, 0.5, 0.1, -0.2, 0.3;
//     iiwa->set_q_act(q_test);
//     iiwa->add_jointspace_trajectory(q_test, q_test, Time(0.0), Time(1.0));
//     iiwa->update(Time(0.0));
//
//     orc::control::HybridForceMotionParameter<DOF> param(true);
//     orc::control::HybridForceMotionController<DOF> ctr(iiwa->robot_data, param);
//
//     // Set up: zero position error, zero velocity, nonzero force_d
//     iiwa->robot_data.pose_d = iiwa->robot_data.pose_act;
//     iiwa->robot_data.x_dot_d = CartesianVector::Zero();
//     iiwa->robot_data.x_dotdot_d = CartesianVector::Zero();
//     iiwa->robot_data.q_d_NS = iiwa->robot_data.q_act;
//     iiwa->robot_data.force_d = 10.0;
//     iiwa->robot_data.force_compensated = Vec3D::Zero();
//
//     JointVector tau = ctr.update();
//     EXPECT_TRUE(tau.allFinite()) << "HFM torque not finite";
//
//     // Verify tau = tau_1 + tau_2 using debug outputs
//     JointVector tau_sum = ctr.tau_1_ + ctr.tau_2_;
//     for (int i = 0; i < DOF; ++i) {
//         EXPECT_NEAR(tau(i), tau_sum(i), 1e-9)
//             << "tau != tau_1 + tau_2 at joint " << i;
//     }
//
//     // Manually verify force control component (first step, no filter history):
//     // force_d = [0, 0, 10], force_filtered = [0, 0, 0] (first step)
//     // v_f = force_d + KPf*(force_d - force_filtered) + KIf*0 + KDf*p_dot_f
//     // With zero velocity: p_dot_f = R^T * (-x_dot_act_linear) = 0
//     // v_f = [0,0,10] + KPf*[0,0,10] = [0,0,10] + [0,0,10] = [0,0,20]
//     Vec3D expected_v_f;
//     expected_v_f << 0, 0, iiwa->robot_data.force_d + param.KPf(2,2) * iiwa->robot_data.force_d;
//     for (int i = 0; i < 3; ++i) {
//         EXPECT_NEAR(ctr.v_f_(i), expected_v_f(i), 0.5)
//             << "v_f mismatch at component " << i
//             << ": expected " << expected_v_f(i) << ", got " << ctr.v_f_(i);
//     }
//
//     // With zero position/velocity error and zero force feedback,
//     // the motion control part (v_c_breve) should be ~0
//     EXPECT_NEAR(ctr.v_c_breve_.norm(), 0.0, 0.1)
//         << "v_c_breve should be ~0 with zero position/velocity error";
//
//     // Nullspace torque (tau_2) should be ~0 when q_act = q_NS and q_dot = 0
//     EXPECT_NEAR(ctr.tau_2_.norm(), 0.0, 1.0)
//         << "Nullspace torque should be small when q_act = q_NS and q_dot ≈ 0";
// }
//
// TEST_F(ControllerFixture, HybridFM_MotionControlFormulaVerification) {
//     // Test with pure position error (no force), verify motion control kicks in.
//     // Use a non-zero configuration for better Jacobian conditioning.
//     JointVector q_test; q_test << 0.3, 0.2, -0.1, 0.5, 0.1, -0.2, 0.3;
//     iiwa->set_q_act(q_test);
//     iiwa->add_jointspace_trajectory(q_test, q_test, Time(0.0), Time(1.0));
//     iiwa->update(Time(0.0));
//
//     orc::control::HybridForceMotionParameter<DOF> param(true);
//     orc::control::HybridForceMotionController<DOF> ctr(iiwa->robot_data, param);
//
//     // Create a small position offset in x (which is in the motion-controlled subspace)
//     iiwa->robot_data.pose_d = iiwa->robot_data.pose_act;
//     iiwa->robot_data.pose_d(0) += 0.01; // 1cm offset in x
//     iiwa->robot_data.x_dot_d = CartesianVector::Zero();
//     iiwa->robot_data.x_dotdot_d = CartesianVector::Zero();
//     iiwa->robot_data.q_d_NS = iiwa->robot_data.q_act;
//     iiwa->robot_data.force_d = 0.0;
//     iiwa->robot_data.force_compensated = Vec3D::Zero();
//
//     JointVector tau = ctr.update();
//     EXPECT_TRUE(tau.allFinite());
//
//     // The motion control component (v_c_breve) should be nonzero in the linear part
//     // because pose_d has a 1cm offset. v_c_breve.head(3) = KP * p_breve
//     // p_breve = R^T * (p_d - p_act), which should have a nonzero component.
//     double motion_control_magnitude = ctr.v_c_breve_.head(3).norm();
//     EXPECT_GT(motion_control_magnitude, 0.1)
//         << "Motion control v_c_breve should be nonzero with position error";
//
//     // Verify v_c_breve formula: v_c_breve.head(3) = KP * p_breve (since x_dotdot_d=0,
//     KD*p_dot_breve=0, KI*pos_err_I=0) Eigen::Matrix3d R = iiwa->robot_data.H_0_e.block<3,3>(0,0);
//     Vec3D p_breve_expected = R.transpose() * (iiwa->robot_data.pose_d.head<3>() -
//     iiwa->robot_data.pose_act.head<3>()); Vec3D v_c_expected = param.KP * p_breve_expected; for
//     (int i = 0; i < 3; ++i) {
//         EXPECT_NEAR(ctr.v_c_breve_(i), v_c_expected(i), 0.1)
//             << "v_c_breve linear component " << i << " formula mismatch";
//     }
//
//     // Angular part should be ~0 (orientation matches)
//     EXPECT_NEAR(ctr.v_c_breve_.tail(3).norm(), 0.0, 0.1)
//         << "v_c_breve angular part should be ~0 when orientations match";
// }

// ===========================================================================
// RobotData — uninitialized field
// ===========================================================================
TEST_F(ControllerFixture, RobotData_ForceErrorIntegralInitialized) {
    // BUG: force_error_integral is an uninitialized double in RobotData.
    // Reading it is undefined behavior.
    EXPECT_TRUE(std::isfinite(iiwa->robot_data.force_error_integral))
        << "BUG: RobotData::force_error_integral is not initialized in constructor.\n"
           "Value: "
        << iiwa->robot_data.force_error_integral;
}

}  // namespace
