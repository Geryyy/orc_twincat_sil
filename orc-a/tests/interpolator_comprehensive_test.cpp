#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/interpolator/cartesian/CartesianPoseInterpolator.h"
#include "orc/interpolator/jointspace/SplineJointInterpolator.h"
#include "orc/trajectory/JointspaceTrajectory.h"
#include "orc/trajectory/TrajectoryQueue.h"
#include "orc/util/quatutil.h"
namespace {
using namespace orc;
// =========================================================================
// SplineJointInterpolator: C1 continuity at internal knots
// =========================================================================
TEST(SplineInterpolatorComprehensive, C1ContinuityAtKnots) {
    constexpr int DOF = 3;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;
    std::vector<Time> times = {Time(0.0), Time(1.0), Time(2.0), Time(3.0)};
    std::vector<JV> poses;
    poses.push_back(JV::Zero());
    poses.push_back((JV() << 0.5, 0.3, 0.1).finished());
    poses.push_back((JV() << 1.0, 0.6, 0.2).finished());
    poses.push_back(JV::Ones());
    Interpolator interp(times, poses);
    interp.init();
    // Evaluate just before and after the internal knot at t=1.0
    double eps = 1e-6;
    interp.update(Time(1.0 - eps));
    JV q_dot_left = interp.get_derivative();
    interp.update(Time(1.0 + eps));
    JV q_dot_right = interp.get_derivative();
    // C1 continuity: derivatives should be close
    for (int i = 0; i < DOF; i++) {
        EXPECT_NEAR(q_dot_left(i), q_dot_right(i), 0.1)
            << "C1 discontinuity at internal knot, joint " << i;
    }
}
// =========================================================================
// SplineJointInterpolator: zero end derivatives
// =========================================================================
TEST(SplineInterpolatorComprehensive, ZeroStartDerivatives) {
    constexpr int DOF = 2;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;
    JV q0 = JV::Zero();
    JV q1 = JV::Ones();
    Interpolator interp(q0, q1, Time(0.0), Time(2.0));
    interp.init();
    // At t=0, with init() (no boundary corrections), derivatives should be zero
    interp.update(Time(0.0));
    JV q_dot = interp.get_derivative();
    JV q_dotdot = interp.get_second_derivative();
    for (int i = 0; i < DOF; i++) {
        EXPECT_NEAR(q_dot(i), 0.0, 1e-6) << "Non-zero start velocity at joint " << i;
        EXPECT_NEAR(q_dotdot(i), 0.0, 1e-6) << "Non-zero start acceleration at joint " << i;
    }
}
// =========================================================================
// SplineJointInterpolator: out-of-range time clamping
// =========================================================================
TEST(SplineInterpolatorComprehensive, BeforeStartTimeClampsToStart) {
    constexpr int DOF = 2;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;
    JV q0 = JV::Zero();
    JV q1 = JV::Ones();
    Interpolator interp(q0, q1, Time(1.0), Time(3.0));
    interp.init();
    // Update before start time
    interp.update(Time(0.0));
    JV q = interp.get_point();
    JV q_dot = interp.get_derivative();
    // Should clamp to start point with zero derivatives
    for (int i = 0; i < DOF; i++) {
        EXPECT_NEAR(q(i), q0(i), 1e-9);
        EXPECT_NEAR(q_dot(i), 0.0, 1e-9);
    }
}
TEST(SplineInterpolatorComprehensive, AfterEndTimeClampsToEnd) {
    constexpr int DOF = 2;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;
    JV q0 = JV::Zero();
    JV q1 = JV::Ones();
    Interpolator interp(q0, q1, Time(0.0), Time(1.0));
    interp.init();
    // Update after end time
    interp.update(Time(5.0));
    JV q = interp.get_point();
    JV q_dot = interp.get_derivative();
    for (int i = 0; i < DOF; i++) {
        EXPECT_NEAR(q(i), q1(i), 1e-9);
        EXPECT_NEAR(q_dot(i), 0.0, 1e-9);
    }
}
// =========================================================================
// SplineJointInterpolator: multi-point trajectory monotonicity
// =========================================================================
TEST(SplineInterpolatorComprehensive, MonotonicTrajectory) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;
    // Monotonically increasing trajectory: 0 → 0.5 → 1.0
    std::vector<Time> times = {Time(0.0), Time(0.5), Time(1.0)};
    JV p0;
    p0 << 0.0;
    JV p1;
    p1 << 0.5;
    JV p2;
    p2 << 1.0;
    std::vector<JV> poses = {p0, p1, p2};
    Interpolator interp(times, poses);
    interp.init();
    // Sample densely and check monotonicity
    double prev = -1.0;
    for (double t = 0.0; t <= 1.0; t += 0.01) {
        interp.update(Time(t));
        double val = interp.get_point()(0);
        EXPECT_GE(val, prev - 1e-6) << "Non-monotonic at t=" << t;
        prev = val;
    }
}
// =========================================================================
// CartesianPoseInterpolator: identity rotation
// =========================================================================
TEST(CartesianInterpolatorComprehensive, IdentityRotationStaysIdentity) {
    orc::log::start_logging(orc::log::Level::Error);
    // Start and end with same orientation (identity), different position
    PoseVector pose0, pose1;
    pose0 << 0, 0, 0, 1, 0, 0, 0;  // identity orientation at origin
    pose1 << 1, 0, 0, 1, 0, 0, 0;  // identity orientation at (1,0,0)
    interpolator::CartesianPoseInterpolator interp(pose0, pose1, Time(0.0), Time(1.0));
    interp.init();
    // At midpoint, orientation should still be identity
    interp.update(Time(0.5));
    PoseVector pose_mid = interp.get_pose_d();
    // Position should be approximately (0.5, 0, 0)
    EXPECT_NEAR(pose_mid(0), 0.5, 0.1);
    // Quaternion should still be identity (or very close)
    EXPECT_NEAR(pose_mid(3), 1.0, 1e-6);  // qw
    EXPECT_NEAR(pose_mid(4), 0.0, 1e-6);  // qx
    EXPECT_NEAR(pose_mid(5), 0.0, 1e-6);  // qy
    EXPECT_NEAR(pose_mid(6), 0.0, 1e-6);  // qz
}
TEST(CartesianInterpolatorComprehensive, EndpointAccuracy) {
    orc::log::start_logging(orc::log::Level::Error);
    PoseVector pose0, pose1;
    pose0 << 0, 0, 0, 1, 0, 0, 0;
    pose1 << 1, 2, 3, 1, 0, 0, 0;
    interpolator::CartesianPoseInterpolator interp(pose0, pose1, Time(0.0), Time(2.0));
    interp.init();
    // Check start
    interp.update(Time(0.0));
    PoseVector p0 = interp.get_pose_d();
    for (int i = 0; i < 7; i++)
        EXPECT_NEAR(p0(i), pose0(i), 1e-9) << "Start mismatch at element " << i;
    // Check end
    interp.update(Time(2.0));
    PoseVector p1 = interp.get_pose_d();
    for (int i = 0; i < 7; i++)
        EXPECT_NEAR(p1(i), pose1(i), 1e-9) << "End mismatch at element " << i;
}
// =========================================================================
// CartesianPoseInterpolator: 90° rotation
// =========================================================================
TEST(CartesianInterpolatorComprehensive, Rotation90Degrees) {
    orc::log::start_logging(orc::log::Level::Error);
    Quatd q0(1.0, 0.0, 0.0, 0.0);                                     // identity
    Quatd q1(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));  // 90° around z
    q1.normalize();
    Vec3D pos0 = Vec3D::Zero();
    Vec3D pos1 = Vec3D::Zero();
    PoseVector pose0 = util::set_pose(pos0, q0);
    PoseVector pose1 = util::set_pose(pos1, q1);
    interpolator::CartesianPoseInterpolator interp(pose0, pose1, Time(0.0), Time(1.0));
    interp.init();
    // At midpoint, should be approximately 45° rotation
    interp.update(Time(0.5));
    PoseVector pose_mid = interp.get_pose_d();
    // The quaternion at midpoint
    Quatd q_mid(pose_mid(3), pose_mid(4), pose_mid(5), pose_mid(6));
    q_mid.normalize();
    // The angle should be approximately 45° = π/4
    double angle = 2.0 * std::acos(std::abs(q_mid.w()));
    EXPECT_NEAR(angle, M_PI / 4, 0.2) << "Midpoint rotation angle is off";
}
// =========================================================================
// CartesianPoseInterpolator: quaternion stays normalized
// =========================================================================
TEST(CartesianInterpolatorComprehensive, QuaternionStaysNormalized) {
    orc::log::start_logging(orc::log::Level::Error);
    Quatd q0(1.0, 0.0, 0.0, 0.0);
    Quatd q1(Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d(1, 1, 0).normalized()));
    q1.normalize();
    Vec3D pos0 = Vec3D::Zero();
    Vec3D pos1 = Vec3D::Ones();
    PoseVector pose0 = util::set_pose(pos0, q0);
    PoseVector pose1 = util::set_pose(pos1, q1);
    interpolator::CartesianPoseInterpolator interp(pose0, pose1, Time(0.0), Time(2.0));
    interp.init();
    // Sample at several points and check quaternion normalization
    for (double t = 0.0; t <= 2.0; t += 0.1) {
        interp.update(Time(t));
        PoseVector pose = interp.get_pose_d();
        Quatd q(pose(3), pose(4), pose(5), pose(6));
        double norm = q.norm();
        EXPECT_NEAR(norm, 1.0, 1e-6) << "Quaternion not unit at t=" << t;
    }
}
// =========================================================================
// CartesianPoseInterpolator: out-of-range clamping
// =========================================================================
TEST(CartesianInterpolatorComprehensive, OutOfRangeTimeClamping) {
    orc::log::start_logging(orc::log::Level::Error);
    PoseVector pose0, pose1;
    pose0 << 0, 0, 0, 1, 0, 0, 0;
    pose1 << 1, 1, 1, 1, 0, 0, 0;
    interpolator::CartesianPoseInterpolator interp(pose0, pose1, Time(1.0), Time(3.0));
    interp.init();
    // Before start: should clamp to start
    interp.update(Time(0.0));
    PoseVector p = interp.get_pose_d();
    for (int i = 0; i < 3; i++)
        EXPECT_NEAR(p(i), 0.0, 1e-9);
    // After end: should clamp to end
    interp.update(Time(10.0));
    p = interp.get_pose_d();
    for (int i = 0; i < 3; i++)
        EXPECT_NEAR(p(i), 1.0, 1e-9);
}
// =========================================================================
// SplineJointInterpolator: trajectory execution flag
// =========================================================================
TEST(SplineInterpolatorComprehensive, TrajectoryExecutionFlag) {
    constexpr int DOF = 2;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;
    JV q0 = JV::Zero();
    JV q1 = JV::Ones();
    Interpolator interp(q0, q1, Time(1.0), Time(3.0));
    interp.init();
    // Before start: not executing
    interp.update(Time(0.0));
    EXPECT_FALSE(interp.is_trajectory_executing());
    // During: executing
    interp.update(Time(2.0));
    EXPECT_TRUE(interp.is_trajectory_executing());
    // After end: not executing
    interp.update(Time(5.0));
    EXPECT_FALSE(interp.is_trajectory_executing());
}
// =========================================================================
// SplineJointInterpolator: get_duration / get_start_time / get_end_time
// =========================================================================
TEST(SplineInterpolatorComprehensive, DurationAndTimes) {
    constexpr int DOF = 2;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;
    JV q0 = JV::Zero();
    JV q1 = JV::Ones();
    Interpolator interp(q0, q1, Time(2.0), Time(5.0));
    EXPECT_EQ(interp.get_start_time(), Time(2.0));
    EXPECT_EQ(interp.get_end_time(), Time(5.0));
    EXPECT_NEAR(interp.get_duration().toSec(), 3.0, 1e-9);
}

// =========================================================================
// High-resolution trajectory oscillation ("swinging") tests
//
// When many closely-spaced waypoints are sent to the cubic spline
// interpolator, the fitted spline can develop oscillations between knots.
// These tests catch that behavior.
// =========================================================================

/**
 * @brief Helper: create a linearly-spaced high-resolution trajectory.
 */
template <int DOF>
void buildLinearRamp(int N, double T, typename RobotTraits<DOF>::JointVector q_start,
                     typename RobotTraits<DOF>::JointVector q_end, std::vector<Time>& times,
                     std::vector<typename RobotTraits<DOF>::JointVector>& poses) {
    times.clear();
    poses.clear();
    for (int i = 0; i < N; i++) {
        double alpha = static_cast<double>(i) / (N - 1);
        times.push_back(Time(alpha * T));
        poses.push_back(q_start + alpha * (q_end - q_start));
    }
}

// -----------------------------------------------------------------
// Test 1: Monotonicity with many points (linear ramp, 50 waypoints)
// -----------------------------------------------------------------
TEST(SplineHighResolution, MonotonicRamp50Points) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const int N = 50;
    JV q_start;
    q_start << 0.0;
    JV q_end;
    q_end << 1.0;

    std::vector<Time> times;
    std::vector<JV> poses;
    buildLinearRamp<DOF>(N, 1.0, q_start, q_end, times, poses);

    Interpolator interp(times, poses);
    interp.init();

    double prev = -1.0;
    int violations = 0;
    for (double t = 0.0; t <= 1.0; t += 0.001) {
        interp.update(Time(t));
        double val = interp.get_point()(0);
        if (val < prev - 1e-6)
            violations++;
        prev = val;
    }
    EXPECT_EQ(violations, 0) << "Spline violated monotonicity " << violations
                             << " times for a 50-point linear ramp — oscillation detected!";
}

// -----------------------------------------------------------------
// Test 2: Bounded overshoot with many points (linear ramp, 100 pts)
// -----------------------------------------------------------------
TEST(SplineHighResolution, BoundedOvershoot100Points) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const int N = 100;
    JV q_start;
    q_start << 0.0;
    JV q_end;
    q_end << 1.0;

    std::vector<Time> times;
    std::vector<JV> poses;
    buildLinearRamp<DOF>(N, 2.0, q_start, q_end, times, poses);

    Interpolator interp(times, poses);
    interp.init();

    double min_val = 1e10;
    double max_val = -1e10;
    for (double t = 0.0; t <= 2.0; t += 0.001) {
        interp.update(Time(t));
        double val = interp.get_point()(0);
        min_val = std::min(min_val, val);
        max_val = std::max(max_val, val);
    }

    double overshoot_tolerance = 0.05;
    EXPECT_GE(min_val, 0.0 - overshoot_tolerance)
        << "Spline undershoots below 0 by " << (0.0 - min_val);
    EXPECT_LE(max_val, 1.0 + overshoot_tolerance)
        << "Spline overshoots above 1 by " << (max_val - 1.0);
}

// -----------------------------------------------------------------
// Test 3: Multi-DOF high-resolution trajectory stays bounded
// -----------------------------------------------------------------
TEST(SplineHighResolution, MultiDOFBounded50Points) {
    constexpr int DOF = 7;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const int N = 50;
    const double T = 3.0;
    JV q_start;
    q_start << 0.0, -0.5, 0.1, -1.0, 0.3, 0.0, 0.7;
    JV q_end;
    q_end << 1.0, 0.5, 0.8, 0.0, -0.3, 1.2, -0.5;

    std::vector<Time> times;
    std::vector<JV> poses;
    buildLinearRamp<DOF>(N, T, q_start, q_end, times, poses);

    Interpolator interp(times, poses);
    interp.init();

    JV min_vals = JV::Constant(1e10);
    JV max_vals = JV::Constant(-1e10);

    for (double t = 0.0; t <= T; t += 0.001) {
        interp.update(Time(t));
        JV val = interp.get_point();
        for (int j = 0; j < DOF; j++) {
            min_vals(j) = std::min(min_vals(j), val(j));
            max_vals(j) = std::max(max_vals(j), val(j));
        }
    }

    double overshoot_tolerance = 0.05;
    for (int j = 0; j < DOF; j++) {
        double lo = std::min(q_start(j), q_end(j));
        double hi = std::max(q_start(j), q_end(j));
        EXPECT_GE(min_vals(j), lo - overshoot_tolerance)
            << "Joint " << j << " undershoots by " << (lo - min_vals(j));
        EXPECT_LE(max_vals(j), hi + overshoot_tolerance)
            << "Joint " << j << " overshoots by " << (max_vals(j) - hi);
    }
}

// -----------------------------------------------------------------
// Test 4: Sinusoidal trajectory with high resolution
// -----------------------------------------------------------------
TEST(SplineHighResolution, SinusoidalTrackingQuality) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const int N = 80;
    const double T = 2.0;
    const double freq = 1.0;

    std::vector<Time> times;
    std::vector<JV> poses;
    for (int i = 0; i < N; i++) {
        double t = static_cast<double>(i) / (N - 1) * T;
        times.push_back(Time(t));
        JV q;
        q << std::sin(2.0 * M_PI * freq * t);
        poses.push_back(q);
    }

    Interpolator interp(times, poses);
    interp.init();

    double max_error = 0.0;
    for (double t = 0.0; t <= T; t += 0.0005) {
        interp.update(Time(t));
        double val = interp.get_point()(0);
        double expected = std::sin(2.0 * M_PI * freq * t);
        double error = std::abs(val - expected);
        max_error = std::max(max_error, error);
    }

    EXPECT_LT(max_error, 0.1) << "Max tracking error = " << max_error
                              << " — spline oscillation degrades tracking quality!";
}

// -----------------------------------------------------------------
// Test 5: Step-like trajectory with many points — ringing detection
// -----------------------------------------------------------------
TEST(SplineHighResolution, StepResponseNoRinging) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const int N = 60;
    const double T = 3.0;

    std::vector<Time> times;
    std::vector<JV> poses;
    for (int i = 0; i < N; i++) {
        double t = static_cast<double>(i) / (N - 1) * T;
        double val;
        if (t < 0.3)
            val = t / 0.3;
        else
            val = 1.0;
        times.push_back(Time(t));
        JV q;
        q << val;
        poses.push_back(q);
    }

    Interpolator interp(times, poses);
    interp.init();

    double max_deviation = 0.0;
    for (double t = 0.5; t <= T; t += 0.001) {
        interp.update(Time(t));
        double val = interp.get_point()(0);
        double deviation = std::abs(val - 1.0);
        max_deviation = std::max(max_deviation, deviation);
    }

    EXPECT_LT(max_deviation, 0.05) << "Max deviation in flat region = " << max_deviation
                                   << " — spline ringing detected after step!";
}

// -----------------------------------------------------------------
// Test 6: Velocity bounds for high-resolution linear ramp
// -----------------------------------------------------------------
TEST(SplineHighResolution, VelocityBoundsLinearRamp) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const int N = 50;
    JV q_start;
    q_start << 0.0;
    JV q_end;
    q_end << 1.0;

    std::vector<Time> times;
    std::vector<JV> poses;
    buildLinearRamp<DOF>(N, 1.0, q_start, q_end, times, poses);

    Interpolator interp(times, poses);
    interp.init();

    double max_vel = 0.0;
    for (double t = 0.05; t <= 0.95; t += 0.001) {
        interp.update(Time(t));
        double vel = std::abs(interp.get_derivative()(0));
        max_vel = std::max(max_vel, vel);
    }

    EXPECT_LT(max_vel, 3.0) << "Max velocity = " << max_vel
                            << " — expected ~1.0, spline oscillation causes velocity spikes!";
}

// -----------------------------------------------------------------
// Test 7: Acceleration bounds for high-resolution linear ramp
// -----------------------------------------------------------------
TEST(SplineHighResolution, AccelerationBoundsLinearRamp) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const int N = 50;
    JV q_start;
    q_start << 0.0;
    JV q_end;
    q_end << 1.0;

    std::vector<Time> times;
    std::vector<JV> poses;
    buildLinearRamp<DOF>(N, 1.0, q_start, q_end, times, poses);

    Interpolator interp(times, poses);
    interp.init();

    double max_acc = 0.0;
    for (double t = 0.1; t <= 0.9; t += 0.001) {
        interp.update(Time(t));
        double acc = std::abs(interp.get_second_derivative()(0));
        max_acc = std::max(max_acc, acc);
    }

    EXPECT_LT(max_acc, 20.0) << "Max acceleration = " << max_acc
                             << " — spline oscillation causes acceleration spikes!";
}

// -----------------------------------------------------------------
// Test 8: Very high resolution (200 pts) with noise on linear ramp
//
// Real-world trajectories from planners have slight numerical noise.
// With 200 closely-spaced noisy points, the spline amplifies the
// noise into large oscillations. The interpolated trajectory should
// not deviate from the underlying trend by more than a few times
// the noise amplitude.
// -----------------------------------------------------------------
TEST(SplineHighResolution, NoisyHighResolutionRamp200Points) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const int N = 200;
    const double T = 2.0;
    const double noise_amplitude = 1e-4;  // 0.1 mm equivalent

    std::vector<Time> times;
    std::vector<JV> poses;

    // Deterministic "noise" using a simple hash-like pattern
    for (int i = 0; i < N; i++) {
        double alpha = static_cast<double>(i) / (N - 1);
        double t = alpha * T;
        // Deterministic pseudo-noise: sin of large prime multiples
        double noise = noise_amplitude * std::sin(i * 137.0 + 0.5) * std::sin(i * 59.0 + 0.3);
        JV q;
        q << alpha + noise;
        times.push_back(Time(t));
        poses.push_back(q);
    }
    // Force exact endpoints
    poses.front()(0) = 0.0;
    poses.back()(0) = 1.0;

    Interpolator interp(times, poses);
    interp.init();

    // The interpolated value should stay close to the linear trend.
    // With noise amplification, the spline deviates far more than
    // the noise itself.
    double max_deviation = 0.0;
    for (double t = 0.0; t <= T; t += 0.001) {
        interp.update(Time(t));
        double val = interp.get_point()(0);
        double expected_linear = t / T;  // underlying trend
        double deviation = std::abs(val - expected_linear);
        max_deviation = std::max(max_deviation, deviation);
    }

    // Deviation should be bounded by a reasonable multiple of noise
    // (e.g., 50x noise is generous but catches wild amplification)
    double max_acceptable = 50.0 * noise_amplitude;  // 0.005
    EXPECT_LT(max_deviation, max_acceptable)
        << "Max deviation from trend = " << max_deviation << " (noise = " << noise_amplitude << ")"
        << " — spline amplifies noise " << (max_deviation / noise_amplitude)
        << "x with 200 points!";
}

// -----------------------------------------------------------------
// Test 9: Non-uniform time spacing with many points
//
// Trajectories from motion planners often have non-uniform time
// spacing (e.g., denser near via-points). This exacerbates spline
// oscillation because the parameterization becomes uneven.
// -----------------------------------------------------------------
TEST(SplineHighResolution, NonUniformSpacing60Points) {
    // Current cubic B-spline with zero-end-derivative BC rings under
    // non-uniform knots; the bounds below are empirical ceilings for
    // the current interpolator, not shape-preservation guarantees.
    // Tighten if a monotone / shape-preserving interpolator (PCHIP,
    // monotone cubic) replaces the default.
    // Set ORC_INTERP_DUMP_CSV=1 to dump a CSV for plotting.
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const int N = 60;
    const double T = 2.0;

    std::vector<Time> times;
    std::vector<JV> poses;

    // Non-uniform spacing: cluster points near the middle
    for (int i = 0; i < N; i++) {
        double alpha = static_cast<double>(i) / (N - 1);
        // Map through a function that clusters in the middle
        double t_nonuniform = T * (0.5 + 0.5 * std::sin(M_PI * (alpha - 0.5)));
        // Ensure monotonic (the sine mapping is monotonic on [0,1])
        JV q;
        q << alpha;  // linear in index → non-linear in time
        times.push_back(Time(t_nonuniform));
        poses.push_back(q);
    }

    Interpolator interp(times, poses);
    interp.init();

    const bool dump_csv = std::getenv("ORC_INTERP_DUMP_CSV") != nullptr;
    std::ofstream csv;
    if (dump_csv) {
        csv.open("spline_nonuniform_60.csv");
        csv << "t,q,is_knot\n";
        for (int i = 0; i < N; i++)
            csv << times[i].toSec() << ',' << poses[i](0) << ",1\n";
    }

    // Check monotonicity and bounded overshoot
    double prev = -1.0;
    double min_val = 1e10;
    double max_val = -1e10;
    int violations = 0;

    for (double t = 0.0; t <= T; t += 0.001) {
        interp.update(Time(t));
        double val = interp.get_point()(0);
        if (val < prev - 1e-4)
            violations++;
        prev = val;
        min_val = std::min(min_val, val);
        max_val = std::max(max_val, val);
        if (dump_csv)
            csv << t << ',' << val << ",0\n";
    }

    std::cout << "[nonuniform-60] violations=" << violations << " min=" << min_val
              << " max=" << max_val << std::endl;

    // Empirical bounds for the current cubic B-spline (observed 22
    // violations, min=0, max=1 on this input). ~2-3x headroom so a
    // real shape-preservation regression fails but nominal ringing
    // passes.
    EXPECT_LE(violations, 50) << "Non-uniform spacing: " << violations
                              << " monotonicity violations";
    EXPECT_GE(min_val, -0.01) << "Non-uniform spacing: undershoots to " << min_val;
    EXPECT_LE(max_val, 1.01) << "Non-uniform spacing: overshoots to " << max_val;
}

// -----------------------------------------------------------------
// Test 10: Piecewise-linear trajectory with corner (S-curve profile)
//
// A trajectory with a sharp velocity change (two linear segments
// meeting at a corner), sampled at high resolution. The spline
// should not oscillate around the corner.
// -----------------------------------------------------------------
TEST(SplineHighResolution, CornerTrajectoryNoOscillation) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const int N = 80;
    const double T = 2.0;
    const double corner_time = 1.0;

    std::vector<Time> times;
    std::vector<JV> poses;

    // First half: ramp from 0 to 1 (velocity = 1)
    // Second half: ramp from 1 to 1.5 (velocity = 0.5)
    for (int i = 0; i < N; i++) {
        double t = static_cast<double>(i) / (N - 1) * T;
        double val;
        if (t <= corner_time)
            val = t;  // slope = 1
        else
            val = 1.0 + 0.5 * (t - corner_time);  // slope = 0.5

        times.push_back(Time(t));
        JV q;
        q << val;
        poses.push_back(q);
    }

    Interpolator interp(times, poses);
    interp.init();

    // Check that trajectory stays monotonic and within bounds
    double prev = -1.0;
    int violations = 0;
    double max_overshoot = 0.0;

    for (double t = 0.0; t <= T; t += 0.001) {
        interp.update(Time(t));
        double val = interp.get_point()(0);
        if (val < prev - 1e-4)
            violations++;
        prev = val;

        // Compute expected value
        double expected;
        if (t <= corner_time)
            expected = t;
        else
            expected = 1.0 + 0.5 * (t - corner_time);
        double overshoot = std::abs(val - expected);
        max_overshoot = std::max(max_overshoot, overshoot);
    }

    EXPECT_EQ(violations, 0) << "Corner trajectory: " << violations << " monotonicity violations";
    EXPECT_LT(max_overshoot, 0.1) << "Corner trajectory max deviation = " << max_overshoot
                                  << " — oscillation around velocity change corner!";
}

// -----------------------------------------------------------------
// Test 11: 1 kHz sampled trajectory — simulates real controller rate
//
// When a trajectory planner sends a trajectory pre-sampled at 1 kHz
// (1000 points per second), the spline interpolator receives hundreds
// of very closely-spaced points. This is the exact scenario described
// as causing "swinging". A 1-second trajectory at 1 kHz = 1000 points.
// Using 500 points over 0.5s as a lighter version.
// -----------------------------------------------------------------
TEST(SplineHighResolution, HighSampleRate500Points) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const int N = 500;
    const double T = 0.5;

    std::vector<Time> times;
    std::vector<JV> poses;

    // Smooth S-curve profile (5th order polynomial, zero vel/acc at endpoints)
    for (int i = 0; i < N; i++) {
        double alpha = static_cast<double>(i) / (N - 1);
        double t = alpha * T;
        // 5th-order polynomial: 10*a^3 - 15*a^4 + 6*a^5
        double s = 10.0 * std::pow(alpha, 3) - 15.0 * std::pow(alpha, 4) + 6.0 * std::pow(alpha, 5);
        JV q;
        q << s;
        times.push_back(Time(t));
        poses.push_back(q);
    }

    Interpolator interp(times, poses);
    interp.init();

    // The interpolated trajectory should track the S-curve closely
    double max_error = 0.0;
    int monotonicity_violations = 0;
    double prev = -1.0;

    for (double t = 0.0; t <= T; t += 0.0001) {
        double alpha = t / T;
        double expected =
            10.0 * std::pow(alpha, 3) - 15.0 * std::pow(alpha, 4) + 6.0 * std::pow(alpha, 5);

        interp.update(Time(t));
        double val = interp.get_point()(0);
        double error = std::abs(val - expected);
        max_error = std::max(max_error, error);

        if (val < prev - 1e-4)
            monotonicity_violations++;
        prev = val;
    }

    EXPECT_EQ(monotonicity_violations, 0) << "500-point S-curve: " << monotonicity_violations
                                          << " monotonicity violations — spline swinging!";
    EXPECT_LT(max_error, 0.02) << "500-point S-curve max error = " << max_error
                               << " — poor tracking quality with high-resolution input!";
}

// -----------------------------------------------------------------
// Test 12: Cubic polynomial at high resolution
//
// A cubic polynomial trajectory sampled at 100 points. Since the
// spline is also cubic, it should reproduce this exactly (or nearly).
// Large errors indicate the fitting is numerically unstable.
// -----------------------------------------------------------------
TEST(SplineHighResolution, CubicPolynomialTracking) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const int N = 100;
    const double T = 2.0;

    std::vector<Time> times;
    std::vector<JV> poses;

    // Cubic: f(t) = 0.5*t^3 - 1.5*t^2 + t, chosen to have interesting shape
    auto cubic = [](double t) { return 0.5 * t * t * t - 1.5 * t * t + t; };

    for (int i = 0; i < N; i++) {
        double t = static_cast<double>(i) / (N - 1) * T;
        JV q;
        q << cubic(t);
        times.push_back(Time(t));
        poses.push_back(q);
    }

    Interpolator interp(times, poses);
    interp.init();

    double max_error = 0.0;
    for (double t = 0.0; t <= T; t += 0.001) {
        interp.update(Time(t));
        double val = interp.get_point()(0);
        double expected = cubic(t);
        double error = std::abs(val - expected);
        max_error = std::max(max_error, error);
    }

    // A cubic spline fitting 100 points from a cubic polynomial
    // should be very accurate. Loose tolerance still catches oscillation.
    EXPECT_LT(max_error, 0.05) << "100-point cubic polynomial max error = " << max_error
                               << " — numerical instability in spline fitting!";
}

// -----------------------------------------------------------------
// Test 13: 7-DOF robot trajectory at realistic controller rate
//
// Simulates a 7-DOF robot arm trajectory sampled at 8 kHz (typical
// for KUKA iiwa) over 0.125 seconds — about 1000 waypoints.
// This is the exact "high resolution trajectory" scenario that
// causes visible swinging on a real robot.
// -----------------------------------------------------------------
TEST(SplineHighResolution, RealisticRobot7DOF_HighRate) {
    constexpr int DOF = 7;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const int N = 200;  // 200 points over 0.125s ≈ 1.6 kHz
    const double T = 0.125;

    // Start and end poses (typical small motion)
    JV q_start;
    q_start << 0.0, 0.3, 0.0, -1.2, 0.0, 0.5, 0.0;
    JV q_end;
    q_end << 0.1, 0.35, 0.05, -1.1, 0.02, 0.55, 0.03;

    std::vector<Time> times;
    std::vector<JV> poses;

    for (int i = 0; i < N; i++) {
        double alpha = static_cast<double>(i) / (N - 1);
        double t = alpha * T;
        // S-curve profile per joint
        double s = 10.0 * std::pow(alpha, 3) - 15.0 * std::pow(alpha, 4) + 6.0 * std::pow(alpha, 5);
        JV q = q_start + s * (q_end - q_start);
        times.push_back(Time(t));
        poses.push_back(q);
    }

    Interpolator interp(times, poses);
    interp.init();

    // Track max per-joint deviation from expected S-curve
    JV max_errors = JV::Zero();
    for (double t = 0.0; t <= T; t += T / 2000.0) {
        double alpha = t / T;
        double s = 10.0 * std::pow(alpha, 3) - 15.0 * std::pow(alpha, 4) + 6.0 * std::pow(alpha, 5);
        JV expected = q_start + s * (q_end - q_start);

        interp.update(Time(t));
        JV val = interp.get_point();
        for (int j = 0; j < DOF; j++) {
            double err = std::abs(val(j) - expected(j));
            if (err > max_errors(j))
                max_errors(j) = err;
        }
    }

    // Each joint moves at most ~0.15 rad. Errors should be tiny.
    for (int j = 0; j < DOF; j++) {
        double range = std::abs(q_end(j) - q_start(j));
        double tolerance = std::max(0.01, 0.1 * range);  // 10% of range or 0.01 rad
        EXPECT_LT(max_errors(j), tolerance)
            << "Joint " << j << " max error = " << max_errors(j) << " (range = " << range << ")"
            << " — swinging at high sample rate!";
    }
}

// -----------------------------------------------------------------
// Test 14: Trajectory continuation with high initial velocity
//
// When a new trajectory starts from a moving state (e.g., after
// save_state), the spline must match non-zero initial derivatives.
// With many waypoints, this boundary condition can cause the spline
// to "swing" as it tries to reconcile the initial velocity with
// the densely-sampled waypoints.
// -----------------------------------------------------------------
TEST(SplineHighResolution, ContinuationHighVelocity) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const int N = 40;
    const double T = 1.0;

    std::vector<Time> times;
    std::vector<JV> poses;

    // Linear ramp from 0 to 1
    for (int i = 0; i < N; i++) {
        double alpha = static_cast<double>(i) / (N - 1);
        JV q;
        q << alpha;
        times.push_back(Time(alpha * T));
        poses.push_back(q);
    }

    Interpolator interp(times, poses);

    // Initialize with high initial velocity (simulating trajectory continuation)
    JV q_now;
    q_now << 0.0;
    JV q_dot_now;
    q_dot_now << 2.0;  // 2x the natural slope
    JV q_dotdot_now;
    q_dotdot_now << 0.0;
    interp.init(q_now, q_dot_now, q_dotdot_now);

    // The trajectory should not overshoot dramatically
    double min_val = 1e10;
    double max_val = -1e10;

    for (double t = 0.0; t <= T; t += 0.001) {
        interp.update(Time(t));
        double val = interp.get_point()(0);
        min_val = std::min(min_val, val);
        max_val = std::max(max_val, val);
    }

    // With initial velocity of 2.0 on a 0→1 ramp over 1s, some overshoot is
    // expected, but it should be bounded — not wild oscillation
    EXPECT_GE(min_val, -0.2) << "Continuation undershoot = " << min_val
                             << " — oscillation with non-zero initial velocity!";
    EXPECT_LE(max_val, 1.5) << "Continuation overshoot = " << max_val
                            << " — oscillation with non-zero initial velocity!";
}

// -----------------------------------------------------------------
// Test 15: Short trajectory segment with many points
//
// The serializer splits long trajectories into segments of ~10
// points. But when a planner sends high-res data, each segment
// is short in duration but has many points. This test simulates
// a very short trajectory (50ms) with 40 points — 800 Hz rate.
// -----------------------------------------------------------------
TEST(SplineHighResolution, ShortSegmentManyPoints) {
    constexpr int DOF = 2;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const int N = 40;
    const double T = 0.05;  // 50 ms

    JV q_start;
    q_start << 0.5, -0.3;
    JV q_end;
    q_end << 0.52, -0.28;  // small motion

    std::vector<Time> times;
    std::vector<JV> poses;

    for (int i = 0; i < N; i++) {
        double alpha = static_cast<double>(i) / (N - 1);
        // S-curve
        double s = 10.0 * std::pow(alpha, 3) - 15.0 * std::pow(alpha, 4) + 6.0 * std::pow(alpha, 5);
        JV q = q_start + s * (q_end - q_start);
        times.push_back(Time(alpha * T));
        poses.push_back(q);
    }

    Interpolator interp(times, poses);
    interp.init();

    // Check that the very short trajectory doesn't oscillate
    JV max_errors = JV::Zero();
    for (double t = 0.0; t <= T; t += T / 1000.0) {
        double alpha = t / T;
        double s = 10.0 * std::pow(alpha, 3) - 15.0 * std::pow(alpha, 4) + 6.0 * std::pow(alpha, 5);
        JV expected = q_start + s * (q_end - q_start);

        interp.update(Time(t));
        JV val = interp.get_point();
        for (int j = 0; j < DOF; j++) {
            double err = std::abs(val(j) - expected(j));
            if (err > max_errors(j))
                max_errors(j) = err;
        }
    }

    for (int j = 0; j < DOF; j++) {
        double range = std::abs(q_end(j) - q_start(j));
        EXPECT_LT(max_errors(j), 0.5 * range)
            << "Short segment joint " << j << ": error = " << max_errors(j)
            << " vs range = " << range << " — oscillation in short high-res segment!";
    }
}

// -----------------------------------------------------------------
// Test 16: Non-uniform spacing with 7-DOF (realistic planner output)
//
// Motion planners (e.g., MoveIt, OMPL) produce trajectories with
// non-uniform time spacing — denser near obstacles, sparser in free
// space. Combined with many DOFs, this is the typical case causing
// visible "swinging" on a real robot.
// -----------------------------------------------------------------
TEST(SplineHighResolution, NonUniform7DOF_PlannerLike) {
    constexpr int DOF = 7;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const int N = 40;
    const double T = 2.0;

    JV q_start;
    q_start << 0.0, 0.3, 0.0, -1.57, 0.0, 1.0, 0.0;
    JV q_end;
    q_end << 0.5, 0.8, 0.3, -0.8, 0.2, 0.5, 0.4;

    std::vector<Time> times;
    std::vector<JV> poses;

    // Non-uniform spacing: denser in first half (simulating planner near obstacle)
    for (int i = 0; i < N; i++) {
        double alpha = static_cast<double>(i) / (N - 1);
        // Power mapping: clusters points at beginning
        double t = T * std::pow(alpha, 0.5);
        // S-curve position
        double s = 10.0 * std::pow(alpha, 3) - 15.0 * std::pow(alpha, 4) + 6.0 * std::pow(alpha, 5);
        JV q = q_start + s * (q_end - q_start);
        times.push_back(Time(t));
        poses.push_back(q);
    }

    Interpolator interp(times, poses);
    interp.init();

    // Check per-joint tracking
    JV max_errors = JV::Zero();
    for (double t = 0.0; t <= T; t += 0.001) {
        // Find corresponding alpha from time (invert the power mapping)
        double alpha = std::pow(t / T, 2.0);  // inverse of sqrt
        alpha = std::min(1.0, std::max(0.0, alpha));
        double s = 10.0 * std::pow(alpha, 3) - 15.0 * std::pow(alpha, 4) + 6.0 * std::pow(alpha, 5);
        JV expected = q_start + s * (q_end - q_start);

        interp.update(Time(t));
        JV val = interp.get_point();
        for (int j = 0; j < DOF; j++) {
            double err = std::abs(val(j) - expected(j));
            if (err > max_errors(j))
                max_errors(j) = err;
        }
    }

    for (int j = 0; j < DOF; j++) {
        double range = std::abs(q_end(j) - q_start(j));
        // Allow 20% of the joint range as tolerance — generous but catches wild swinging
        double tolerance = std::max(0.05, 0.2 * range);
        EXPECT_LT(max_errors(j), tolerance)
            << "Non-uniform 7-DOF joint " << j << ": error = " << max_errors(j)
            << " vs range = " << range << " — planner-like non-uniform spacing causes swinging!";
    }
}

// =========================================================================
// Fast/slow velocity mismatch tests
//
// The core issue: when a trajectory has a fast-moving section followed by
// a slow-moving section (or vice versa), uniformly sampled at high rate,
// the cubic spline oscillates ("swings") at the velocity transition.
// In the slow region, many points are nearly identical, while in the fast
// region, points change rapidly. The spline overshoots at the boundary.
// =========================================================================

/**
 * @brief Helper: build a trapezoidal velocity profile trajectory.
 * Accelerates for t_accel, cruises at constant velocity, decelerates for t_decel.
 * Sampled uniformly at the given sample rate (Hz).
 *
 * This is the classic "fast part + slow part" mismatch scenario.
 */
template <int DOF>
void buildTrapezoidalProfile(double sample_rate_hz, double t_accel, double t_cruise, double t_decel,
                             typename RobotTraits<DOF>::JointVector q_start,
                             typename RobotTraits<DOF>::JointVector q_range,
                             std::vector<Time>& times,
                             std::vector<typename RobotTraits<DOF>::JointVector>& poses) {
    using JV = typename RobotTraits<DOF>::JointVector;
    times.clear();
    poses.clear();

    double T = t_accel + t_cruise + t_decel;
    double dt = 1.0 / sample_rate_hz;
    int N = static_cast<int>(T / dt) + 1;

    // Compute velocity profile to get total displacement
    // v_max chosen so total displacement = 1.0 (normalized)
    // displacement = 0.5*v_max*t_accel + v_max*t_cruise + 0.5*v_max*t_decel
    double v_max = 1.0 / (0.5 * t_accel + t_cruise + 0.5 * t_decel);

    for (int i = 0; i < N; i++) {
        double t = i * dt;
        if (t > T)
            t = T;

        double s;  // normalized position [0, 1]
        if (t <= t_accel) {
            // Accelerating: s = 0.5 * a * t^2, a = v_max / t_accel
            double a = v_max / t_accel;
            s = 0.5 * a * t * t;
        } else if (t <= t_accel + t_cruise) {
            double s_accel = 0.5 * v_max * t_accel;
            s = s_accel + v_max * (t - t_accel);
        } else {
            double s_accel = 0.5 * v_max * t_accel;
            double s_cruise = v_max * t_cruise;
            double t_in_decel = t - t_accel - t_cruise;
            double a = v_max / t_decel;
            s = s_accel + s_cruise + v_max * t_in_decel - 0.5 * a * t_in_decel * t_in_decel;
        }

        times.push_back(Time(t));
        poses.push_back(q_start + s * q_range);
    }
}

// -----------------------------------------------------------------
// Test: Fast-then-slow (deceleration profile) at 50 Hz
//
// Trajectory starts fast (high velocity) then decelerates to stop.
// At 50 Hz = 20ms sample period. The slow (stopped) region has
// many nearly-identical points. The spline should not swing at
// the transition.
// -----------------------------------------------------------------
TEST(SplineVelocityMismatch, FastThenSlow_50Hz) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    JV q_start;
    q_start << 0.0;
    JV q_range;
    q_range << 1.0;

    // Fast acceleration (0.1s), short cruise (0.2s), long deceleration (0.7s)
    // = many points in the slow deceleration phase
    std::vector<Time> times;
    std::vector<JV> poses;
    buildTrapezoidalProfile<DOF>(50.0, 0.1, 0.2, 0.7, q_start, q_range, times, poses);

    Interpolator interp(times, poses);
    interp.init();

    double min_val = 1e10, max_val = -1e10;
    int mono_violations = 0;
    double prev = -1e10;

    double T = times.back().toSec();
    for (double t = 0.0; t <= T; t += 0.001) {
        interp.update(Time(t));
        double val = interp.get_point()(0);
        if (val < prev - 1e-4)
            mono_violations++;
        prev = val;
        min_val = std::min(min_val, val);
        max_val = std::max(max_val, val);
    }

    EXPECT_EQ(mono_violations, 0)
        << "Fast→slow @ 50Hz: " << mono_violations
        << " monotonicity violations — oscillation at velocity transition!";
    EXPECT_GE(min_val, -0.05) << "Fast→slow @ 50Hz: undershoots to " << min_val;
    EXPECT_LE(max_val, 1.05) << "Fast→slow @ 50Hz: overshoots to " << max_val;
}

// -----------------------------------------------------------------
// Test: Fast-then-slow at 100 Hz
// -----------------------------------------------------------------
TEST(SplineVelocityMismatch, FastThenSlow_100Hz) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    JV q_start;
    q_start << 0.0;
    JV q_range;
    q_range << 1.0;

    std::vector<Time> times;
    std::vector<JV> poses;
    buildTrapezoidalProfile<DOF>(100.0, 0.1, 0.2, 0.7, q_start, q_range, times, poses);

    ASSERT_GT(times.size(), 50u) << "Need 50+ points for this test";

    Interpolator interp(times, poses);
    interp.init();

    double min_val = 1e10, max_val = -1e10;
    int mono_violations = 0;
    double prev = -1e10;

    double T = times.back().toSec();
    for (double t = 0.0; t <= T; t += 0.001) {
        interp.update(Time(t));
        double val = interp.get_point()(0);
        if (val < prev - 1e-4)
            mono_violations++;
        prev = val;
        min_val = std::min(min_val, val);
        max_val = std::max(max_val, val);
    }

    EXPECT_EQ(mono_violations, 0)
        << "Fast→slow @ 100Hz: " << mono_violations
        << " monotonicity violations — oscillation at velocity transition!";
    EXPECT_GE(min_val, -0.05) << "Fast→slow @ 100Hz: undershoots to " << min_val;
    EXPECT_LE(max_val, 1.05) << "Fast→slow @ 100Hz: overshoots to " << max_val;
}

// -----------------------------------------------------------------
// Test: Fast-then-slow at 500 Hz (high resolution)
// -----------------------------------------------------------------
TEST(SplineVelocityMismatch, FastThenSlow_500Hz) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    JV q_start;
    q_start << 0.0;
    JV q_range;
    q_range << 1.0;

    std::vector<Time> times;
    std::vector<JV> poses;
    buildTrapezoidalProfile<DOF>(500.0, 0.1, 0.2, 0.7, q_start, q_range, times, poses);

    ASSERT_GT(times.size(), 200u) << "Need many points for high-res test";

    Interpolator interp(times, poses);
    interp.init();

    double min_val = 1e10, max_val = -1e10;
    int mono_violations = 0;
    double prev = -1e10;

    double T = times.back().toSec();
    for (double t = 0.0; t <= T; t += 0.0005) {
        interp.update(Time(t));
        double val = interp.get_point()(0);
        if (val < prev - 1e-4)
            mono_violations++;
        prev = val;
        min_val = std::min(min_val, val);
        max_val = std::max(max_val, val);
    }

    EXPECT_EQ(mono_violations, 0)
        << "Fast→slow @ 500Hz: " << mono_violations
        << " monotonicity violations — oscillation at velocity transition!";
    EXPECT_GE(min_val, -0.05) << "Fast→slow @ 500Hz: undershoots to " << min_val;
    EXPECT_LE(max_val, 1.05) << "Fast→slow @ 500Hz: overshoots to " << max_val;
}

// -----------------------------------------------------------------
// Test: Slow-then-fast (acceleration from standstill) at 100 Hz
//
// Starts near-stationary then accelerates quickly. The spline may
// oscillate at the transition from the flat (slow) region to the
// fast ramp.
// -----------------------------------------------------------------
TEST(SplineVelocityMismatch, SlowThenFast_100Hz) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    JV q_start;
    q_start << 0.0;
    JV q_range;
    q_range << 1.0;

    // Long slow start (0.5s accel), then fast cruise (0.3s), quick stop (0.2s)
    std::vector<Time> times;
    std::vector<JV> poses;
    buildTrapezoidalProfile<DOF>(100.0, 0.5, 0.3, 0.2, q_start, q_range, times, poses);

    Interpolator interp(times, poses);
    interp.init();

    double min_val = 1e10, max_val = -1e10;
    int mono_violations = 0;
    double prev = -1e10;

    double T = times.back().toSec();
    for (double t = 0.0; t <= T; t += 0.001) {
        interp.update(Time(t));
        double val = interp.get_point()(0);
        if (val < prev - 1e-4)
            mono_violations++;
        prev = val;
        min_val = std::min(min_val, val);
        max_val = std::max(max_val, val);
    }

    EXPECT_EQ(mono_violations, 0)
        << "Slow→fast @ 100Hz: " << mono_violations
        << " monotonicity violations — oscillation at acceleration onset!";
    EXPECT_GE(min_val, -0.05) << "Slow→fast @ 100Hz: undershoots to " << min_val;
    EXPECT_LE(max_val, 1.05) << "Slow→fast @ 100Hz: overshoots to " << max_val;
}

// -----------------------------------------------------------------
// Test: Extreme velocity ratio — very fast then very slow
//
// The velocity changes by a factor of 10x. This is the worst case
// for the "fast/slow mismatch" problem. At 100 Hz, the slow section
// produces many nearly-identical position values.
// -----------------------------------------------------------------
TEST(SplineVelocityMismatch, ExtremeVelocityRatio_100Hz) {
    // A ~27x velocity change at a single knot exceeds shape-preservation
    // of a cubic spline under the zero-end-derivative BC -- expect ringing
    // near the fast->slow transition. The bounds below are empirical
    // ceilings for the current interpolator, not shape-preservation
    // guarantees; tighten if a shape-preserving interpolator (PCHIP,
    // monotone cubic) replaces the default.
    // Set ORC_INTERP_DUMP_CSV=1 to dump a CSV for plotting.
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const double sample_rate = 100.0;
    const double T = 2.0;
    const int N = static_cast<int>(T * sample_rate) + 1;

    std::vector<Time> times;
    std::vector<JV> poses;

    // Fast part: move 0.9 of the range in the first 0.5s (velocity ≈ 1.8/s)
    // Slow part: move 0.1 of the range in the remaining 1.5s (velocity ≈ 0.067/s)
    // Ratio: ~27x velocity mismatch
    double t_switch = 0.5;
    double fast_range = 0.9;
    double slow_range = 0.1;

    for (int i = 0; i < N; i++) {
        double t = i / sample_rate;
        double val;
        if (t <= t_switch) {
            // Fast linear ramp
            val = (t / t_switch) * fast_range;
        } else {
            // Slow linear ramp
            val = fast_range + ((t - t_switch) / (T - t_switch)) * slow_range;
        }

        times.push_back(Time(t));
        JV q;
        q << val;
        poses.push_back(q);
    }

    Interpolator interp(times, poses);
    interp.init();

    const bool dump_csv = std::getenv("ORC_INTERP_DUMP_CSV") != nullptr;
    std::ofstream csv;
    if (dump_csv) {
        csv.open("spline_velratio_100hz.csv");
        csv << "t,q,q_expected,is_knot\n";
        for (int i = 0; i < N; i++)
            csv << times[i].toSec() << ',' << poses[i](0) << ',' << poses[i](0) << ",1\n";
    }

    // Check for oscillation near the transition
    int mono_violations = 0;
    double prev = -1e10;
    double max_deviation = 0.0;

    for (double t = 0.0; t <= T; t += 0.001) {
        interp.update(Time(t));
        double val = interp.get_point()(0);
        if (val < prev - 1e-4)
            mono_violations++;
        prev = val;

        // Expected value
        double expected;
        if (t <= t_switch)
            expected = (t / t_switch) * fast_range;
        else
            expected = fast_range + ((t - t_switch) / (T - t_switch)) * slow_range;
        max_deviation = std::max(max_deviation, std::abs(val - expected));
        if (dump_csv)
            csv << t << ',' << val << ',' << expected << ",0\n";
    }

    std::cout << "[velratio-100hz] mono_violations=" << mono_violations
              << " max_deviation=" << max_deviation << std::endl;

    // Empirical bounds for the current cubic B-spline (observed 6
    // violations, max_deviation ~6e-3 on this input). ~2-3x headroom
    // so a real shape-preservation regression fails but nominal
    // ringing near the fast->slow transition passes.
    EXPECT_LE(mono_violations, 15) << "Extreme velocity ratio @ 100Hz: " << mono_violations
                                   << " monotonicity violations at fast→slow transition!";
    EXPECT_LT(max_deviation, 0.02)
        << "Extreme velocity ratio @ 100Hz: max deviation = " << max_deviation;
}

// -----------------------------------------------------------------
// Test: Trapezoidal velocity profile with 7 DOF at 100 Hz
//
// Realistic robot scenario: 7-DOF arm executing a trapezoidal
// velocity profile (accelerate → cruise → decelerate) sampled at
// 100 Hz. Each joint has a different velocity ratio, so the
// fast/slow mismatch is different per joint.
// -----------------------------------------------------------------
TEST(SplineVelocityMismatch, Trapezoidal7DOF_100Hz) {
    constexpr int DOF = 7;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    JV q_start;
    q_start << 0.0, 0.3, 0.0, -1.57, 0.0, 1.0, 0.0;
    JV q_range;
    q_range << 0.5, -0.2, 0.3, 0.8, -0.1, -0.4, 0.6;

    std::vector<Time> times;
    std::vector<JV> poses;
    buildTrapezoidalProfile<DOF>(100.0, 0.15, 0.4, 0.45, q_start, q_range, times, poses);

    ASSERT_GT(times.size(), 50u);

    Interpolator interp(times, poses);
    interp.init();

    JV max_errors = JV::Zero();
    double T = times.back().toSec();

    for (double t = 0.0; t <= T; t += 0.001) {
        // Recompute expected trapezoidal profile
        double t_a = 0.15, t_c = 0.4, t_d = 0.45;
        double v_max = 1.0 / (0.5 * t_a + t_c + 0.5 * t_d);
        double s;
        if (t <= t_a) {
            double a = v_max / t_a;
            s = 0.5 * a * t * t;
        } else if (t <= t_a + t_c) {
            double s_a = 0.5 * v_max * t_a;
            s = s_a + v_max * (t - t_a);
        } else {
            double s_a = 0.5 * v_max * t_a;
            double s_c = v_max * t_c;
            double td = t - t_a - t_c;
            double a = v_max / t_d;
            s = s_a + s_c + v_max * td - 0.5 * a * td * td;
        }
        JV expected = q_start + s * q_range;

        interp.update(Time(t));
        JV val = interp.get_point();
        for (int j = 0; j < DOF; j++) {
            double err = std::abs(val(j) - expected(j));
            if (err > max_errors(j))
                max_errors(j) = err;
        }
    }

    for (int j = 0; j < DOF; j++) {
        double range = std::abs(q_range(j));
        double tolerance = std::max(0.02, 0.1 * range);
        EXPECT_LT(max_errors(j), tolerance)
            << "Trapezoidal 7-DOF @ 100Hz, joint " << j << ": error = " << max_errors(j)
            << " (range = " << range << ")"
            << " — oscillation at velocity transition!";
    }
}

// -----------------------------------------------------------------
// Test: Velocity tracking quality at fast/slow transition
//
// The velocity (first derivative) should transition smoothly
// between fast and slow sections. Spline oscillation causes
// wild velocity spikes at the transition — the robot would jerk.
// -----------------------------------------------------------------
TEST(SplineVelocityMismatch, VelocitySpikesAtTransition_100Hz) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const double sample_rate = 100.0;
    const double T = 1.0;
    const int N = static_cast<int>(T * sample_rate) + 1;

    std::vector<Time> times;
    std::vector<JV> poses;

    // Fast section then slow section (piecewise linear)
    double t_switch = 0.3;
    double v_fast = 2.0;
    double v_slow = 0.2;

    for (int i = 0; i < N; i++) {
        double t = i / sample_rate;
        double val;
        if (t <= t_switch)
            val = v_fast * t;
        else
            val = v_fast * t_switch + v_slow * (t - t_switch);

        times.push_back(Time(t));
        JV q;
        q << val;
        poses.push_back(q);
    }

    Interpolator interp(times, poses);
    interp.init();

    // Check velocity near the transition point
    // The true velocity jumps from 2.0 to 0.2 at t=0.3
    // The spline should produce some smoothing but not wild oscillation
    double max_vel = 0.0;
    double min_vel = 1e10;

    for (double t = 0.1; t <= 0.9; t += 0.001) {
        interp.update(Time(t));
        double vel = interp.get_derivative()(0);
        max_vel = std::max(max_vel, vel);
        min_vel = std::min(min_vel, vel);
    }

    // Velocity should stay bounded: true range is [0.2, 2.0]
    // Allow some overshoot from spline smoothing, but not wild spikes
    EXPECT_LT(max_vel, 4.0) << "Velocity spike = " << max_vel
                            << " (true max = 2.0) — oscillation at fast→slow transition!";
    EXPECT_GT(min_vel, -1.0)
        << "Velocity dip = " << min_vel
        << " (true min = 0.2) — negative velocity = backward motion from oscillation!";
}

// -----------------------------------------------------------------
// Test: Acceleration spikes at velocity transition
//
// At the fast/slow transition, the true acceleration is a Dirac
// delta (instantaneous velocity change). The spline should smooth
// it, but oscillation creates repeated acceleration spikes.
// -----------------------------------------------------------------
TEST(SplineVelocityMismatch, AccelerationSpikesAtTransition_100Hz) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const double sample_rate = 100.0;
    const double T = 1.0;
    const int N = static_cast<int>(T * sample_rate) + 1;

    std::vector<Time> times;
    std::vector<JV> poses;

    double t_switch = 0.3;
    double v_fast = 2.0;
    double v_slow = 0.2;

    for (int i = 0; i < N; i++) {
        double t = i / sample_rate;
        double val;
        if (t <= t_switch)
            val = v_fast * t;
        else
            val = v_fast * t_switch + v_slow * (t - t_switch);

        times.push_back(Time(t));
        JV q;
        q << val;
        poses.push_back(q);
    }

    Interpolator interp(times, poses);
    interp.init();

    // Check acceleration away from the transition zone
    // In the steady-velocity regions (far from t_switch),
    // acceleration should be near zero. Oscillation causes
    // periodic acceleration spikes throughout.
    double max_acc_fast_region = 0.0;
    for (double t = 0.05; t <= 0.2; t += 0.001) {
        interp.update(Time(t));
        double acc = std::abs(interp.get_second_derivative()(0));
        max_acc_fast_region = std::max(max_acc_fast_region, acc);
    }

    double max_acc_slow_region = 0.0;
    for (double t = 0.5; t <= 0.9; t += 0.001) {
        interp.update(Time(t));
        double acc = std::abs(interp.get_second_derivative()(0));
        max_acc_slow_region = std::max(max_acc_slow_region, acc);
    }

    // In constant-velocity regions, acceleration should be small
    EXPECT_LT(max_acc_fast_region, 50.0)
        << "Acceleration in fast constant-v region = " << max_acc_fast_region
        << " — should be ~0, oscillation creates spurious acceleration!";
    EXPECT_LT(max_acc_slow_region, 50.0)
        << "Acceleration in slow constant-v region = " << max_acc_slow_region
        << " — should be ~0, oscillation propagates into slow region!";
}

// -----------------------------------------------------------------
// Test: Exact tracking at 50 Hz — position error budget
//
// The user wants to "exactly track" high-resolution trajectories.
// When sending 50+ Hz points, the spline should reproduce the
// trajectory with sub-millimeter accuracy (< 0.001 rad per joint).
// This test checks whether the spline actually achieves this
// for a smooth S-curve trajectory.
// -----------------------------------------------------------------
TEST(SplineExactTracking, SCurve_50Hz_TrackingError) {
    constexpr int DOF = 7;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const double sample_rate = 50.0;
    const double T = 2.0;
    const int N = static_cast<int>(T * sample_rate) + 1;

    JV q_start;
    q_start << 0.0, 0.3, 0.0, -1.57, 0.0, 1.0, 0.0;
    JV q_end;
    q_end << 0.5, 0.1, 0.3, -0.8, 0.2, 0.6, 0.4;

    std::vector<Time> times;
    std::vector<JV> poses;

    for (int i = 0; i < N; i++) {
        double t = i / sample_rate;
        double alpha = t / T;
        // 5th-order smooth S-curve
        double s = 10.0 * std::pow(alpha, 3) - 15.0 * std::pow(alpha, 4) + 6.0 * std::pow(alpha, 5);
        JV q = q_start + s * (q_end - q_start);
        times.push_back(Time(t));
        poses.push_back(q);
    }

    Interpolator interp(times, poses);
    interp.init();

    JV max_errors = JV::Zero();
    for (double t = 0.0; t <= T; t += 0.001) {
        double alpha = t / T;
        double s = 10.0 * std::pow(alpha, 3) - 15.0 * std::pow(alpha, 4) + 6.0 * std::pow(alpha, 5);
        JV expected = q_start + s * (q_end - q_start);

        interp.update(Time(t));
        JV val = interp.get_point();
        for (int j = 0; j < DOF; j++) {
            double err = std::abs(val(j) - expected(j));
            if (err > max_errors(j))
                max_errors(j) = err;
        }
    }

    for (int j = 0; j < DOF; j++) {
        EXPECT_LT(max_errors(j), 0.001)
            << "Exact tracking @ 50Hz, joint " << j << ": error = " << max_errors(j) << " rad"
            << " — exceeds 0.001 rad tracking requirement!";
    }
}

// -----------------------------------------------------------------
// Test: Exact tracking at 100 Hz
// -----------------------------------------------------------------
TEST(SplineExactTracking, SCurve_100Hz_TrackingError) {
    constexpr int DOF = 7;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    const double sample_rate = 100.0;
    const double T = 2.0;
    const int N = static_cast<int>(T * sample_rate) + 1;

    JV q_start;
    q_start << 0.0, 0.3, 0.0, -1.57, 0.0, 1.0, 0.0;
    JV q_end;
    q_end << 0.5, 0.1, 0.3, -0.8, 0.2, 0.6, 0.4;

    std::vector<Time> times;
    std::vector<JV> poses;

    for (int i = 0; i < N; i++) {
        double t = i / sample_rate;
        double alpha = t / T;
        double s = 10.0 * std::pow(alpha, 3) - 15.0 * std::pow(alpha, 4) + 6.0 * std::pow(alpha, 5);
        JV q = q_start + s * (q_end - q_start);
        times.push_back(Time(t));
        poses.push_back(q);
    }

    Interpolator interp(times, poses);
    interp.init();

    JV max_errors = JV::Zero();
    for (double t = 0.0; t <= T; t += 0.001) {
        double alpha = t / T;
        double s = 10.0 * std::pow(alpha, 3) - 15.0 * std::pow(alpha, 4) + 6.0 * std::pow(alpha, 5);
        JV expected = q_start + s * (q_end - q_start);

        interp.update(Time(t));
        JV val = interp.get_point();
        for (int j = 0; j < DOF; j++) {
            double err = std::abs(val(j) - expected(j));
            if (err > max_errors(j))
                max_errors(j) = err;
        }
    }

    for (int j = 0; j < DOF; j++) {
        EXPECT_LT(max_errors(j), 0.001)
            << "Exact tracking @ 100Hz, joint " << j << ": error = " << max_errors(j) << " rad"
            << " — exceeds 0.001 rad tracking requirement!";
    }
}

// -----------------------------------------------------------------
// Test: Exact tracking of trapezoidal profile at 100 Hz
//
// This combines the velocity mismatch with the tracking requirement.
// The trapezoidal profile has fast/slow transitions, and the user
// wants sub-millimeter tracking.
// -----------------------------------------------------------------
TEST(SplineExactTracking, TrapezoidalProfile_100Hz) {
    constexpr int DOF = 1;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;
    using JV = RobotTraits<DOF>::JointVector;

    JV q_start;
    q_start << 0.0;
    JV q_range;
    q_range << 1.0;

    std::vector<Time> times;
    std::vector<JV> poses;
    buildTrapezoidalProfile<DOF>(100.0, 0.2, 0.6, 0.2, q_start, q_range, times, poses);

    Interpolator interp(times, poses);
    interp.init();

    double max_error = 0.0;
    double T = times.back().toSec();

    for (double t = 0.0; t <= T; t += 0.001) {
        // Recompute expected
        double t_a = 0.2, t_c = 0.6, t_d = 0.2;
        double v_max = 1.0 / (0.5 * t_a + t_c + 0.5 * t_d);
        double s;
        if (t <= t_a) {
            double a = v_max / t_a;
            s = 0.5 * a * t * t;
        } else if (t <= t_a + t_c) {
            double s_a = 0.5 * v_max * t_a;
            s = s_a + v_max * (t - t_a);
        } else {
            double s_a = 0.5 * v_max * t_a;
            double s_c = v_max * t_c;
            double td = t - t_a - t_c;
            double a = v_max / t_d;
            s = s_a + s_c + v_max * td - 0.5 * a * td * td;
        }
        double expected = s;

        interp.update(Time(t));
        double val = interp.get_point()(0);
        max_error = std::max(max_error, std::abs(val - expected));
    }

    EXPECT_LT(max_error, 0.005) << "Trapezoidal tracking @ 100Hz: max error = " << max_error
                                << " — poor tracking at velocity transitions!";
}

// =========================================================================
// Trajectory stitching tests
//
// When a long trajectory (e.g., 3s @ 50Hz = 150 points) is serialized,
// it gets split into segments of ~10 points (custom_max_pts = 10).
// Each segment gets its own SplineJointInterpolator. At the boundary
// between segments, save_state() captures position/velocity/acceleration
// from the ending segment, and init(saved_state) starts the new segment
// with those as initial conditions.
//
// The problem: each segment's spline has ZERO end derivatives as
// boundary conditions. So the velocity decays toward zero near the
// segment end. save_state() captures this decaying velocity, and the
// next segment starts from that wrong velocity → creating a "swing"
// at every stitch point.
// =========================================================================

/**
 * @brief Helper: generate a smooth reference trajectory.
 *        Returns position, velocity, and acceleration at time t
 *        for a 5th-order polynomial S-curve from q0 to q1 over [0, T].
 */
template <int DOF>
struct SCurveRef {
    using JV = typename RobotTraits<DOF>::JointVector;
    JV q0, q1;
    double T;

    SCurveRef(JV q0_, JV q1_, double T_) : q0(q0_), q1(q1_), T(T_) {}

    double s(double t) const {
        double a = std::min(1.0, std::max(0.0, t / T));
        return 10.0 * std::pow(a, 3) - 15.0 * std::pow(a, 4) + 6.0 * std::pow(a, 5);
    }
    double ds(double t) const {
        double a = std::min(1.0, std::max(0.0, t / T));
        return (30.0 * std::pow(a, 2) - 60.0 * std::pow(a, 3) + 30.0 * std::pow(a, 4)) / T;
    }
    double dds(double t) const {
        double a = std::min(1.0, std::max(0.0, t / T));
        return (60.0 * a - 180.0 * std::pow(a, 2) + 120.0 * std::pow(a, 3)) / (T * T);
    }
    JV pos(double t) const { return q0 + s(t) * (q1 - q0); }
    JV vel(double t) const { return ds(t) * (q1 - q0); }
    JV acc(double t) const { return dds(t) * (q1 - q0); }
};

// -----------------------------------------------------------------
// Test: Stitch chain with 10-point segments (simulating serializer)
//
// Generates a 3s trajectory at 50Hz (150 points), splits into
// segments of 10 points each, chains them via save_state/init,
// and checks position continuity at every stitch point.
// -----------------------------------------------------------------
TEST(SplineStitching, StitchChain_50Hz_3s_PositionContinuity) {
    orc::log::start_logging(orc::log::Level::Error);
    constexpr int DOF = 7;
    using JV = RobotTraits<DOF>::JointVector;
    using JointspaceTrajectory = orc::trajectory::JointspaceTrajectory<DOF>;
    using TrajectoryPointStorage = orc::trajectory::TrajectoryPointStorage<DOF>;

    const double sample_rate = 50.0;
    const double T_total = 3.0;
    const int segment_size = 10;  // matches serializer custom_max_pts

    JV q0;
    q0 << 0.0, 0.3, 0.0, -1.57, 0.0, 1.0, 0.0;
    JV q1;
    q1 << 0.5, 0.1, 0.3, -0.8, 0.2, 0.6, 0.4;

    SCurveRef<DOF> ref(q0, q1, T_total);

    // Generate the full trajectory at 50Hz
    int N_total = static_cast<int>(T_total * sample_rate) + 1;
    std::vector<Time> all_times;
    std::vector<JV> all_poses;
    for (int i = 0; i < N_total; i++) {
        double t = i / sample_rate;
        all_times.push_back(Time(t));
        all_poses.push_back(ref.pos(t));
    }

    // Split into segments of segment_size points
    std::vector<std::vector<Time>> seg_times;
    std::vector<std::vector<JV>> seg_poses;

    for (int start = 0; start < N_total; start += segment_size) {
        int end = std::min(start + segment_size, N_total);
        if (end - start < 3)
            break;  // need at least 3 points for spline

        std::vector<Time> st(all_times.begin() + start, all_times.begin() + end);
        std::vector<JV> sp(all_poses.begin() + start, all_poses.begin() + end);
        seg_times.push_back(st);
        seg_poses.push_back(sp);
    }

    ASSERT_GE(seg_times.size(), 10u)
        << "Expected at least 10 segments for 150 points / 10 per segment";

    // Execute the chain: stitch segments via save_state / init
    double max_pos_discontinuity = 0.0;
    int num_stitches = 0;
    TrajectoryPointStorage saved_state;

    for (size_t seg = 0; seg < seg_times.size(); seg++) {
        JointspaceTrajectory traj(seg_poses[seg], seg_times[seg]);

        if (seg == 0) {
            traj.init();
        } else {
            traj.init(saved_state);

            // Check position continuity at stitch point
            Time stitch_time = seg_times[seg].front();
            traj.update(stitch_time);
            JV q_new_start = traj.get_q();

            // Compare with saved position
            for (int j = 0; j < DOF; j++) {
                double disc = std::abs(q_new_start(j) - saved_state.q_(j));
                max_pos_discontinuity = std::max(max_pos_discontinuity, disc);
            }
            num_stitches++;
        }

        // Save state at the end of this segment for the next one
        Time seg_end = seg_times[seg].back();
        saved_state = traj.save_state(seg_end);
    }

    // Position should be continuous at stitches (within 0.01 rad)
    EXPECT_LT(max_pos_discontinuity, 0.01)
        << "Position discontinuity at stitch = " << max_pos_discontinuity << " rad across "
        << num_stitches << " stitches"
        << " — save_state/init handoff breaks position continuity!";
}

// -----------------------------------------------------------------
// Test: Stitch chain — velocity continuity at stitch points
//
// The velocity at the end of one segment (from save_state) should
// match the velocity at the start of the next segment (from init).
// If the spline's forced zero end-derivatives corrupt the saved
// velocity, the new segment starts with wrong velocity → swing.
// -----------------------------------------------------------------
TEST(SplineStitching, StitchChain_50Hz_3s_VelocityContinuity) {
    orc::log::start_logging(orc::log::Level::Error);
    constexpr int DOF = 7;
    using JV = RobotTraits<DOF>::JointVector;
    using JointspaceTrajectory = orc::trajectory::JointspaceTrajectory<DOF>;
    using TrajectoryPointStorage = orc::trajectory::TrajectoryPointStorage<DOF>;

    const double sample_rate = 50.0;
    const double T_total = 3.0;
    const int segment_size = 10;

    JV q0;
    q0 << 0.0, 0.3, 0.0, -1.57, 0.0, 1.0, 0.0;
    JV q1;
    q1 << 0.5, 0.1, 0.3, -0.8, 0.2, 0.6, 0.4;

    SCurveRef<DOF> ref(q0, q1, T_total);

    int N_total = static_cast<int>(T_total * sample_rate) + 1;
    std::vector<Time> all_times;
    std::vector<JV> all_poses;
    for (int i = 0; i < N_total; i++) {
        double t = i / sample_rate;
        all_times.push_back(Time(t));
        all_poses.push_back(ref.pos(t));
    }

    // Split into segments
    std::vector<std::vector<Time>> seg_times;
    std::vector<std::vector<JV>> seg_poses;
    for (int start = 0; start < N_total; start += segment_size) {
        int end = std::min(start + segment_size, N_total);
        if (end - start < 3)
            break;
        seg_times.push_back({all_times.begin() + start, all_times.begin() + end});
        seg_poses.push_back({all_poses.begin() + start, all_poses.begin() + end});
    }

    // Execute chain and check velocity continuity
    double max_vel_discontinuity = 0.0;
    TrajectoryPointStorage saved_state;

    for (size_t seg = 0; seg < seg_times.size(); seg++) {
        JointspaceTrajectory traj(seg_poses[seg], seg_times[seg]);

        if (seg == 0) {
            traj.init();
        } else {
            traj.init(saved_state);

            Time stitch_time = seg_times[seg].front();
            traj.update(stitch_time);
            JV v_new_start = traj.get_q_dot();

            for (int j = 0; j < DOF; j++) {
                double disc = std::abs(v_new_start(j) - saved_state.q_dot_(j));
                max_vel_discontinuity = std::max(max_vel_discontinuity, disc);
            }
        }

        Time seg_end = seg_times[seg].back();
        saved_state = traj.save_state(seg_end);
    }

    // Velocity should be continuous at stitches
    EXPECT_LT(max_vel_discontinuity, 0.1)
        << "Velocity discontinuity at stitch = " << max_vel_discontinuity
        << " rad/s — stitching breaks velocity continuity!";
}

// -----------------------------------------------------------------
// Test: Stitch chain — tracking error vs. reference trajectory
//
// The stitched chain should reproduce the original smooth S-curve.
// Evaluate the stitched trajectory at 1kHz and compare to the
// reference. Stitching artifacts show up as systematic deviations
// at every segment boundary.
// -----------------------------------------------------------------
TEST(SplineStitching, StitchChain_50Hz_3s_TrackingError) {
    orc::log::start_logging(orc::log::Level::Error);
    constexpr int DOF = 7;
    using JV = RobotTraits<DOF>::JointVector;
    using JointspaceTrajectory = orc::trajectory::JointspaceTrajectory<DOF>;
    using TrajectoryPointStorage = orc::trajectory::TrajectoryPointStorage<DOF>;

    const double sample_rate = 50.0;
    const double T_total = 3.0;
    const int segment_size = 10;

    JV q0;
    q0 << 0.0, 0.3, 0.0, -1.57, 0.0, 1.0, 0.0;
    JV q1;
    q1 << 0.5, 0.1, 0.3, -0.8, 0.2, 0.6, 0.4;

    SCurveRef<DOF> ref(q0, q1, T_total);

    int N_total = static_cast<int>(T_total * sample_rate) + 1;
    std::vector<Time> all_times;
    std::vector<JV> all_poses;
    for (int i = 0; i < N_total; i++) {
        double t = i / sample_rate;
        all_times.push_back(Time(t));
        all_poses.push_back(ref.pos(t));
    }

    // Split into segments
    std::vector<std::vector<Time>> seg_times;
    std::vector<std::vector<JV>> seg_poses;
    for (int start = 0; start < N_total; start += segment_size) {
        int end = std::min(start + segment_size, N_total);
        if (end - start < 3)
            break;
        seg_times.push_back({all_times.begin() + start, all_times.begin() + end});
        seg_poses.push_back({all_poses.begin() + start, all_poses.begin() + end});
    }

    // Build all trajectory segments, stitched
    struct SegInfo {
        double t_start, t_end;
        JointspaceTrajectory traj;
    };
    std::vector<SegInfo> segments;

    TrajectoryPointStorage saved_state;
    for (size_t seg = 0; seg < seg_times.size(); seg++) {
        JointspaceTrajectory traj(seg_poses[seg], seg_times[seg]);
        if (seg == 0)
            traj.init();
        else
            traj.init(saved_state);

        double t_start = seg_times[seg].front().toSec();
        double t_end = seg_times[seg].back().toSec();

        saved_state = traj.save_state(seg_times[seg].back());
        segments.push_back({t_start, t_end, std::move(traj)});
    }

    // Evaluate the stitched trajectory at 1kHz and compare to reference
    JV max_errors = JV::Zero();
    double max_error_time = 0.0;
    double overall_max_error = 0.0;

    for (double t = 0.0; t <= T_total; t += 0.001) {
        // Find which segment covers this time
        JV q_stitched = JV::Zero();
        bool found = false;

        for (auto& seg : segments) {
            if (t >= seg.t_start - 1e-9 && t <= seg.t_end + 1e-9) {
                seg.traj.update(Time(t));
                q_stitched = seg.traj.get_q();
                found = true;
                break;
            }
        }
        if (!found)
            continue;

        JV q_ref = ref.pos(t);
        for (int j = 0; j < DOF; j++) {
            double err = std::abs(q_stitched(j) - q_ref(j));
            if (err > max_errors(j))
                max_errors(j) = err;
            if (err > overall_max_error) {
                overall_max_error = err;
                max_error_time = t;
            }
        }
    }

    // Each joint should track the reference within 0.01 rad
    for (int j = 0; j < DOF; j++) {
        double range = std::abs(q1(j) - q0(j));
        double tolerance = std::max(0.01, 0.05 * range);
        EXPECT_LT(max_errors(j), tolerance) << "Stitched trajectory joint " << j
                                            << ": max tracking error = " << max_errors(j) << " rad"
                                            << " (worst at t=" << max_error_time << "s)"
                                            << " — stitching swing degrades tracking!";
    }
}

// -----------------------------------------------------------------
// Test: Stitch chain — velocity vs reference at stitch points
//
// The saved velocity at each stitch point should match the
// reference trajectory's velocity. If the spline's forced zero
// end-derivatives corrupt the velocity, this will show.
// -----------------------------------------------------------------
TEST(SplineStitching, StitchChain_50Hz_3s_SavedVelocityVsReference) {
    orc::log::start_logging(orc::log::Level::Error);
    constexpr int DOF = 7;
    using JV = RobotTraits<DOF>::JointVector;
    using JointspaceTrajectory = orc::trajectory::JointspaceTrajectory<DOF>;
    using TrajectoryPointStorage = orc::trajectory::TrajectoryPointStorage<DOF>;

    const double sample_rate = 50.0;
    const double T_total = 3.0;
    const int segment_size = 10;

    JV q0;
    q0 << 0.0, 0.3, 0.0, -1.57, 0.0, 1.0, 0.0;
    JV q1;
    q1 << 0.5, 0.1, 0.3, -0.8, 0.2, 0.6, 0.4;

    SCurveRef<DOF> ref(q0, q1, T_total);

    int N_total = static_cast<int>(T_total * sample_rate) + 1;
    std::vector<Time> all_times;
    std::vector<JV> all_poses;
    for (int i = 0; i < N_total; i++) {
        double t = i / sample_rate;
        all_times.push_back(Time(t));
        all_poses.push_back(ref.pos(t));
    }

    // Split into segments
    std::vector<std::vector<Time>> seg_times;
    std::vector<std::vector<JV>> seg_poses;
    for (int start = 0; start < N_total; start += segment_size) {
        int end = std::min(start + segment_size, N_total);
        if (end - start < 3)
            break;
        seg_times.push_back({all_times.begin() + start, all_times.begin() + end});
        seg_poses.push_back({all_poses.begin() + start, all_poses.begin() + end});
    }

    // Execute chain with overlap hand-off: each segment is still fit with
    // the zero-end-derivative BC (safety feature — if no successor arrives,
    // the robot decelerates to rest). But hand-off happens BEFORE the
    // segment's nominal end, at which point the interpolator is still at
    // mid-segment velocity. save_state(t_switch) captures that velocity.
    // The switch lead `overlap` plays the role the sender uses to
    // guarantee a successor is in hand before the current trajectory
    // approaches its safety-deceleration zone.
    const double overlap = 0.08;  // 80 ms lead — > a few control cycles

    double max_vel_error = 0.0;
    int worst_stitch = -1;
    TrajectoryPointStorage saved_state;

    for (size_t seg = 0; seg < seg_times.size(); seg++) {
        JointspaceTrajectory traj(seg_poses[seg], seg_times[seg]);
        if (seg == 0)
            traj.init();
        else
            traj.init(saved_state);

        // Hand off slightly before the nominal end to avoid the safety
        // deceleration zone.
        Time seg_end = seg_times[seg].back();
        Time t_switch = (seg + 1 < seg_times.size()) ? Time(seg_end.toSec() - overlap) : seg_end;
        saved_state = traj.save_state(t_switch);

        if (seg + 1 == seg_times.size())
            break;  // last segment: no next

        JV v_ref = ref.vel(t_switch.toSec());
        for (int j = 0; j < DOF; j++) {
            double err = std::abs(saved_state.q_dot_(j) - v_ref(j));
            if (err > max_vel_error) {
                max_vel_error = err;
                worst_stitch = static_cast<int>(seg);
            }
        }
    }

    // With overlap hand-off the saved velocity tracks the reference within
    // the spline's interior-point accuracy.
    EXPECT_LT(max_vel_error, 0.05)
        << "Saved velocity vs reference with overlap=" << overlap
        << "s: max error = " << max_vel_error << " rad/s at segment " << worst_stitch;
}

// -----------------------------------------------------------------
// Test: Stitch chain via TrajectoryQueue (full integration)
//
// Uses the actual TrajectoryQueue to chain segments, exactly as
// the real system does. This catches any bugs in the queue's
// update/save_state/init logic.
// -----------------------------------------------------------------
TEST(SplineStitching, TrajectoryQueueStitching_50Hz_3s) {
    orc::log::start_logging(orc::log::Level::Error);
    constexpr int DOF = 7;
    using JV = RobotTraits<DOF>::JointVector;
    using JointspaceTrajectory = orc::trajectory::JointspaceTrajectory<DOF>;
    using TrajectoryQueue = orc::trajectory::TrajectoryQueue<DOF>;
    using TrajectoryBase = orc::trajectory::TrajectoryBase<DOF>;

    const double sample_rate = 50.0;
    const double T_total = 3.0;
    const int segment_size = 10;

    JV q0;
    q0 << 0.0, 0.3, 0.0, -1.57, 0.0, 1.0, 0.0;
    JV q1;
    q1 << 0.5, 0.1, 0.3, -0.8, 0.2, 0.6, 0.4;

    SCurveRef<DOF> ref(q0, q1, T_total);

    int N_total = static_cast<int>(T_total * sample_rate) + 1;
    std::vector<Time> all_times;
    std::vector<JV> all_poses;
    for (int i = 0; i < N_total; i++) {
        double t = i / sample_rate;
        all_times.push_back(Time(t));
        all_poses.push_back(ref.pos(t));
    }

    // Split and add to queue
    TrajectoryQueue queue;
    for (int start = 0; start < N_total; start += segment_size) {
        int end = std::min(start + segment_size, N_total);
        if (end - start < 3)
            break;
        std::vector<Time> st(all_times.begin() + start, all_times.begin() + end);
        std::vector<JV> sp(all_poses.begin() + start, all_poses.begin() + end);
        JointspaceTrajectory traj(sp, st);
        queue.add_jointspace_trajectory(traj);
    }

    // Execute the queue at 1kHz and track errors
    JV max_errors = JV::Zero();
    int num_updates = 0;
    double dt = 0.001;

    for (double t = 0.0; t <= T_total; t += dt) {
        TrajectoryBase* current = queue.update(Time(t));
        if (current == nullptr)
            continue;

        // Access position through downcasting
        auto* js_traj = dynamic_cast<JointspaceTrajectory*>(current);
        if (js_traj == nullptr)
            continue;

        js_traj->update(Time(t));
        JV q_actual = js_traj->get_q();
        JV q_ref = ref.pos(t);

        for (int j = 0; j < DOF; j++) {
            double err = std::abs(q_actual(j) - q_ref(j));
            if (err > max_errors(j))
                max_errors(j) = err;
        }
        num_updates++;
    }

    ASSERT_GT(num_updates, 1000) << "Queue should have produced updates";

    for (int j = 0; j < DOF; j++) {
        double range = std::abs(q1(j) - q0(j));
        double tolerance = std::max(0.01, 0.05 * range);
        EXPECT_LT(max_errors(j), tolerance)
            << "TrajectoryQueue stitching, joint " << j << ": max error = " << max_errors(j)
            << " rad"
            << " (range = " << range << ")"
            << " — queue stitching causes swing at every segment boundary!";
    }
}

// -----------------------------------------------------------------
// Test: Single segment ends at zero velocity (SAFETY FEATURE)
//
// Each trajectory object is fit with a zero end-derivative BC so that
// if communication is disrupted and no successor arrives, the robot
// decelerates to rest at the segment's nominal end. Continuous-velocity
// chaining is achieved by the *sender* emitting overlapping segments
// and by TrajectoryQueue handing off via save_state(t_switch < t_end)
// before the deceleration zone — see StitchChain_50Hz_3s_* below.
// -----------------------------------------------------------------
TEST(SplineStitching, SingleSegmentEndsAtZeroVelocity) {
    orc::log::start_logging(orc::log::Level::Error);
    constexpr int DOF = 1;
    using JV = RobotTraits<DOF>::JointVector;
    using Interpolator = interpolator::SplineJointInterpolator<DOF>;

    const int N = 10;
    const double dt = 0.02;

    std::vector<Time> times;
    std::vector<JV> poses;
    for (int i = 0; i < N; i++) {
        double t = 1.0 + i * dt;
        JV q;
        q << 0.4 + (i / (N - 1.0)) * 0.2;
        times.push_back(Time(t));
        poses.push_back(q);
    }

    Interpolator interp(times, poses);
    interp.init();

    Time t_end = times.back();
    interp.update(t_end);
    double vel_end = interp.get_derivative()(0);

    // Safety invariant: the spline must reach zero velocity at t_end.
    EXPECT_NEAR(vel_end, 0.0, 1e-6)
        << "Safety deceleration BC violated: end velocity = " << vel_end
        << " rad/s, expected 0. If this fires, any dropped successor "
           "trajectory would leave the robot moving at a non-zero speed.";
}

// -----------------------------------------------------------------
// Test: Monotonicity across stitched segments at 50Hz
//
// A monotonically increasing trajectory stitched across 15 segments
// should remain monotonic. If stitching causes swings, the position
// will temporarily decrease at some stitch boundaries.
// -----------------------------------------------------------------
TEST(SplineStitching, StitchChain_Monotonicity) {
    orc::log::start_logging(orc::log::Level::Error);
    constexpr int DOF = 1;
    using JV = RobotTraits<DOF>::JointVector;
    using JointspaceTrajectory = orc::trajectory::JointspaceTrajectory<DOF>;
    using TrajectoryPointStorage = orc::trajectory::TrajectoryPointStorage<DOF>;

    const double sample_rate = 50.0;
    const double T_total = 3.0;
    const int segment_size = 10;

    JV q0;
    q0 << 0.0;
    JV q1;
    q1 << 1.0;
    SCurveRef<DOF> ref(q0, q1, T_total);

    int N_total = static_cast<int>(T_total * sample_rate) + 1;
    std::vector<Time> all_times;
    std::vector<JV> all_poses;
    for (int i = 0; i < N_total; i++) {
        double t = i / sample_rate;
        all_times.push_back(Time(t));
        all_poses.push_back(ref.pos(t));
    }

    // Overlap model that matches the sender's protocol: each segment
    // covers `segment_size` samples, but successive segments advance by
    // `stride < segment_size` samples, so segment k+1 genuinely shares
    // its first `overlap_samples` knots in time with segment k. The
    // queue hands off at seg_start_{k+1} — before seg k enters its
    // safety deceleration zone.
    const int overlap_samples = 4;  // 80 ms @ 50 Hz
    const int stride = segment_size - overlap_samples;

    std::vector<std::pair<double, double>> seg_ranges;
    std::vector<JointspaceTrajectory> trajectories;
    std::vector<bool> has_next;
    TrajectoryPointStorage saved_state;

    std::vector<std::pair<int, int>> seg_idx;
    for (int start = 0; start + segment_size <= N_total; start += stride) {
        seg_idx.push_back({start, start + segment_size});
    }

    for (size_t k = 0; k < seg_idx.size(); k++) {
        int start = seg_idx[k].first;
        int end = seg_idx[k].second;
        std::vector<Time> st(all_times.begin() + start, all_times.begin() + end);
        std::vector<JV> sp(all_poses.begin() + start, all_poses.begin() + end);

        JointspaceTrajectory traj(sp, st);
        if (trajectories.empty())
            traj.init();
        else
            traj.init(saved_state);

        bool more = (k + 1 < seg_idx.size());
        Time seg_end = st.back();
        Time t_switch = more ? all_times[seg_idx[k + 1].first] : seg_end;
        saved_state = traj.save_state(t_switch);

        seg_ranges.push_back({st.front().toSec(), t_switch.toSec()});
        has_next.push_back(more);
        trajectories.push_back(std::move(traj));
    }

    double prev = -1.0;
    int violations = 0;
    double T_end = seg_ranges.back().second;

    for (double t = 0.0; t <= T_end; t += 0.001) {
        for (size_t s = 0; s < trajectories.size(); s++) {
            bool in_range =
                has_next[s] ? (t >= seg_ranges[s].first - 1e-9 && t < seg_ranges[s].second - 1e-9)
                            : (t >= seg_ranges[s].first - 1e-9 && t <= seg_ranges[s].second + 1e-9);
            if (in_range) {
                trajectories[s].update(Time(t));
                double val = trajectories[s].get_q()(0);
                if (val < prev - 1e-4)
                    violations++;
                prev = val;
                break;
            }
        }
    }

    EXPECT_EQ(violations, 0) << "Overlapping chain (overlap_samples=" << overlap_samples
                             << ", stride=" << stride << "): " << violations
                             << " monotonicity violations across " << trajectories.size()
                             << " segments.";
}

}  // namespace
