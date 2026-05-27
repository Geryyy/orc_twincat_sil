#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include <orc/OrcTypes.h>
#include <orc/trajectory/DenseJointspaceTrajectory.h>

namespace {
constexpr int DOF = 7;
using JointVector = typename orc::RobotTraits<DOF>::JointVector;
using DensePoint = orc::trajectory::DenseJointPoint<DOF>;
using DenseTraj = orc::trajectory::DenseJointspaceTrajectory<DOF>;

// Build a uniform 1kHz trajectory of N samples with q/q_dot/q_dotdot/tau_ff
// set independently by the caller via functor fns taking (double t, int joint).
template <class FnQ, class FnQd, class FnQdd, class FnTau>
DenseTraj make_traj(int N, double dt, double t0, FnQ fq, FnQd fqd, FnQdd fqdd, FnTau ftau) {
    std::vector<orc::Time> times;
    std::vector<DensePoint> pts;
    times.reserve(N);
    pts.reserve(N);
    for (int i = 0; i < N; ++i) {
        double t = t0 + i * dt;
        times.emplace_back(t);
        JointVector q, qd, qdd, tau;
        for (int j = 0; j < DOF; ++j) {
            q[j] = fq(t, j);
            qd[j] = fqd(t, j);
            qdd[j] = fqdd(t, j);
            tau[j] = ftau(t, j);
        }
        pts.emplace_back(q, qd, qdd, tau);
    }
    return DenseTraj(times, pts);
}

// Linear-in-t source: linear interp reproduces it exactly.
TEST(DenseJointspaceInterp, KnotsReturnedExactly) {
    constexpr int N = 100;
    constexpr double dt = 1e-3;  // 1 kHz
    constexpr double t0 = 0.0;

    auto fq = [](double t, int j) { return 0.1 * (j + 1) + 0.5 * (j + 1) * t; };
    auto zero = [](double, int) { return 0.0; };

    auto traj = make_traj(N, dt, t0, fq, zero, zero, zero);

    for (int i = 0; i < N; ++i) {
        double t = t0 + i * dt;
        traj.update(orc::Time(t));
        JointVector q = traj.get_q();
        for (int j = 0; j < DOF; ++j) {
            EXPECT_NEAR(q[j], fq(t, j), 1e-12) << "knot i=" << i << " joint=" << j;
        }
    }
}

// 1kHz → 8kHz linear upsample of a linear-in-t signal: exact to fp tolerance.
TEST(DenseJointspaceInterp, MidpointsMatchClosedForm) {
    constexpr int N = 50;
    constexpr double dt = 1e-3;
    constexpr double t0 = 0.0;

    auto fq = [](double t, int j) { return -0.2 + 0.3 * j + 1.7 * t; };
    auto zero = [](double, int) { return 0.0; };

    auto traj = make_traj(N, dt, t0, fq, zero, zero, zero);

    const double dt_rt = 1.0 / 8000.0;  // 8 kHz consumer
    const double t_end = t0 + (N - 1) * dt;

    for (double t = t0; t <= t_end - 1e-9; t += dt_rt) {
        traj.update(orc::Time(t));
        JointVector q = traj.get_q();
        for (int j = 0; j < DOF; ++j) {
            EXPECT_NEAR(q[j], fq(t, j), 1e-12) << "t=" << t << " joint=" << j;
        }
    }
}

// Below-start and above-end both clamp to the nearest knot.
TEST(DenseJointspaceInterp, ClampsBelowStartAndAboveEnd) {
    constexpr int N = 10;
    constexpr double dt = 1e-3;
    constexpr double t0 = 1.0;  // positive to avoid negative-Time constructor

    auto fq = [](double t, int j) { return 0.5 * (j + 1) + 2.0 * t; };
    auto zero = [](double, int) { return 0.0; };

    auto traj = make_traj(N, dt, t0, fq, zero, zero, zero);

    const double t_end = t0 + (N - 1) * dt;

    traj.update(orc::Time(t0 - 5e-3));
    JointVector q_below = traj.get_q();
    for (int j = 0; j < DOF; ++j)
        EXPECT_NEAR(q_below[j], fq(t0, j), 1e-12);

    traj.update(orc::Time(t_end + 5e-3));
    JointVector q_above = traj.get_q();
    for (int j = 0; j < DOF; ++j)
        EXPECT_NEAR(q_above[j], fq(t_end, j), 1e-12);
}

// q, q_dot, q_dotdot, tau_ff are each interpolated from their own input ramp.
TEST(DenseJointspaceInterp, AllFieldsInterpolated) {
    constexpr int N = 50;
    constexpr double dt = 1e-3;
    constexpr double t0 = 0.0;

    auto fq = [](double t, int j) { return 1.0 + 0.1 * j + 2.0 * t; };
    auto fqd = [](double t, int j) { return 3.0 + 0.2 * j + 4.0 * t; };
    auto fqdd = [](double t, int j) { return 5.0 + 0.3 * j - 6.0 * t; };
    auto ftau = [](double t, int j) { return -0.5 + 0.4 * j + 7.0 * t; };

    auto traj = make_traj(N, dt, t0, fq, fqd, fqdd, ftau);

    // Sample between knots (not on a knot) to force linear interpolation.
    const double t_sample = t0 + 3 * dt + 0.5 * dt;
    traj.update(orc::Time(t_sample));

    JointVector q = traj.get_q();
    JointVector qd = traj.get_q_dot();
    JointVector qdd = traj.get_q_dotdot();
    JointVector tau = traj.get_tau_ff();

    for (int j = 0; j < DOF; ++j) {
        EXPECT_NEAR(q[j], fq(t_sample, j), 1e-12) << "q joint=" << j;
        EXPECT_NEAR(qd[j], fqd(t_sample, j), 1e-12) << "qd joint=" << j;
        EXPECT_NEAR(qdd[j], fqdd(t_sample, j), 1e-12) << "qdd joint=" << j;
        EXPECT_NEAR(tau[j], ftau(t_sample, j), 1e-12) << "tau joint=" << j;
    }
}

// Nonlinear source: linear interp error is bounded by max|f''|·dt²/8.
// Trip-wire that fires if someone swaps the RT upsample for a different
// scheme with different error characteristics.
TEST(DenseJointspaceInterp, NonlinearInputBoundedError) {
    constexpr int N = 2001;  // 2s at 1 kHz
    constexpr double dt = 1e-3;
    constexpr double t0 = 0.0;
    const double omega = 2.0 * M_PI;  // 1 Hz sine; |f''|_max = omega^2

    auto fq = [omega](double t, int j) { return std::sin(omega * t + 0.1 * j); };
    auto zero = [](double, int) { return 0.0; };

    auto traj = make_traj(N, dt, t0, fq, zero, zero, zero);

    const double theoretical_bound = (omega * omega) * dt * dt / 8.0;
    const double tol = 1.5 * theoretical_bound;

    const double dt_rt = 1.0 / 8000.0;
    const double t_end = t0 + (N - 1) * dt;

    double worst = 0.0;
    for (double t = t0; t <= t_end - 1e-9; t += dt_rt) {
        traj.update(orc::Time(t));
        JointVector q = traj.get_q();
        for (int j = 0; j < DOF; ++j) {
            double err = std::abs(q[j] - fq(t, j));
            if (err > worst)
                worst = err;
            ASSERT_LE(err, tol) << "t=" << t << " joint=" << j << " bound=" << theoretical_bound;
        }
    }
    // Sanity: the interp must actually produce some error for a sine;
    // if this hits zero, the test has been neutered.
    EXPECT_GT(worst, 0.0);
}

}  // namespace
