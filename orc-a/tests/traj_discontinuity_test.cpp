/*
 * Test: the sender-side trajectory-splitting machinery
 * (`FlatBufferSerializer::serialize_joint_trajectory_split`) must be
 * TRANSPARENT. Feeding a long smooth trajectory through the splitter,
 * round-tripping each slice over FlatBuffers, and dispatching through
 * TrajectoryQueue must yield a setpoint stream indistinguishable (to
 * within spline-fit noise) from one that was never split -- no position
 * jump, no velocity jump, no acceleration jump at any seam.
 *
 * Reference function (unit interval, scaled per-segment):
 *     q_ref(u)   = 10 u^3 - 15 u^4 + 6 u^5
 *     qd_ref(u)  = (30 u^2 - 60 u^3 + 30 u^4) / T
 *     qdd_ref(u) = (60 u   - 180 u^2 + 120 u^3) / T^2
 * with u = (t - t0) / (t1 - t0), T = t1 - t0.
 *
 * Why this function: SplineJointInterpolator hard-codes zero velocity and
 * zero acceleration at each segment's start/end. The quintic smoothstep
 * also has q' = q'' = 0 at the boundaries, so a dense knot sampling is
 * exactly representable by the spline -- letting us pose a meaningful
 * fidelity bound on position, velocity, and acceleration simultaneously.
 *
 * The splitting transparency check relies on the TrajectoryQueue
 * save_state -> init(saved_state) stitch (TrajectoryQueue.h:73-106):
 * when the queue swaps to the next slice the JointspaceTrajectory re-fits
 * its spline with the previous slice's (q, q_dot, q_dotdot) as boundary
 * conditions, so q/qd/qdd should carry across the seam smoothly.
 *
 * Coverage split:
 *   - Algorithmic / MTU coverage lives in serialization_splitting_test.cpp.
 *   - This file exercises the 1 kHz dispatch + queue stitch end-to-end.
 *
 * CSV dump is gated by ORC_TRAJ_DISCONT_DUMP_CSV for PlotJuggler
 * inspection (setpoint vs. analytic reference overlay).
 */
#include <cstdlib>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

#include <gtest/gtest.h>

#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/util/Time.h"

#include "orc/trajectory/JointspaceTrajectory.h"
#include "orc/trajectory/TrajectoryBase.h"
#include "orc/trajectory/TrajectoryQueue.h"
#include "orc/trajectory/TrajectoryType.h"

#include "orc/com/flatbuffers/FlatBufferDeserializer.h"
#include "orc/com/flatbuffers/FlatBufferSerializer.h"

namespace {
constexpr int DOF = 2;

using JointVector = typename orc::RobotTraits<DOF>::JointVector;
using JointspaceTrajectory = orc::trajectory::JointspaceTrajectory<DOF>;
using TrajectoryQueue = orc::trajectory::TrajectoryQueue<DOF>;
using TrajectoryBase = orc::trajectory::TrajectoryBase<DOF>;
using TrajectoryType = orc::trajectory::TrajectoryType;
using Serializer = orc::com::fb::FlatBufferSerializer<DOF>;
using Deserializer = orc::com::fb::FlatBufferDeserializer<DOF>;
using Time = orc::Time;

// Quintic smoothstep on [t0, t1], clamped outside.
double q_ref(double t, double t0, double t1) {
    const double u = std::clamp((t - t0) / (t1 - t0), 0.0, 1.0);
    return 10.0 * u * u * u - 15.0 * u * u * u * u + 6.0 * u * u * u * u * u;
}
double qd_ref(double t, double t0, double t1) {
    const double T = t1 - t0;
    const double u_raw = (t - t0) / T;
    if (u_raw <= 0.0 || u_raw >= 1.0)
        return 0.0;
    const double u = u_raw;
    return (30.0 * u * u - 60.0 * u * u * u + 30.0 * u * u * u * u) / T;
}
double qdd_ref(double t, double t0, double t1) {
    const double T = t1 - t0;
    const double u_raw = (t - t0) / T;
    if (u_raw <= 0.0 || u_raw >= 1.0)
        return 0.0;
    const double u = u_raw;
    return (60.0 * u - 180.0 * u * u + 120.0 * u * u * u) / (T * T);
}

TEST(TrajSplittingIsTransparent, Smoothstep_1kHz) {
    // --- 1) sample the smooth reference q0(t) = smoothstep(t) on [0, 1],
    //        q1 held at 0 (DoF=2 is the minimum FlatBufferSerializer supports).
    constexpr int N = 100;
    const double t0 = 0.0;
    const double t1 = 1.0;

    std::vector<JointVector> q_pts;
    std::vector<Time> t_pts;
    q_pts.reserve(N);
    t_pts.reserve(N);
    for (int i = 0; i < N; ++i) {
        const double a = static_cast<double>(i) / static_cast<double>(N - 1);
        const double ti = t0 + a * (t1 - t0);
        JointVector q;
        q << q_ref(ti, t0, t1), 0.0;
        q_pts.push_back(q);
        t_pts.push_back(Time(ti));
    }

    // --- 2) serialize via the production split API. user_cap=20 forces
    //        multiple splits for N=100 knots so seam transparency is
    //        actually exercised (a 1-split run would trivially pass).
    Serializer ser;
    Deserializer deser;

    auto split_buffers = ser.serialize_joint_trajectory_split(t_pts, q_pts, /*user_cap=*/20);
    ASSERT_GT(split_buffers.size(), 1u)
        << "splitter produced no seam -- test no longer exercises transparency";

    TrajectoryQueue queue;
    for (const auto& buf : split_buffers) {
        ASSERT_FALSE(buf.empty());
        auto traj = deser.deserialize(buf.data(), buf.size());
        ASSERT_NE(traj, nullptr);
        ASSERT_EQ(traj->get_trajectory_type(), TrajectoryType::JOINTSPACE);
        queue.add_trajectory(std::move(traj));
    }

    // --- 3) dispatch at 1 kHz, record setpoint vs. analytic reference ---
    const Time Ts(1e-3);
    const Time T_end(t1 + 0.05);  // run slightly past trajectory end

    struct Sample {
        double t;
        double q, qd, qdd;
        double q_ref, qd_ref, qdd_ref;
        int traj_type;
        int seg_id;  // increments whenever the queue swaps to a new trajectory object
    };
    std::vector<Sample> log;
    log.reserve(static_cast<size_t>((T_end - Time(0.0)).toSec() / Ts.toSec()) + 1);

    TrajectoryBase* prev_ptr = nullptr;
    int seg_id = -1;

    for (Time t(0.0); t < T_end; t += Ts) {
        TrajectoryBase* curr = queue.update(t);
        if (curr != prev_ptr) {
            ++seg_id;
            prev_ptr = curr;
        }
        Sample s{t.toSec(),
                 0.0,
                 0.0,
                 0.0,
                 q_ref(t.toSec(), t0, t1),
                 qd_ref(t.toSec(), t0, t1),
                 qdd_ref(t.toSec(), t0, t1),
                 static_cast<int>(curr ? curr->get_trajectory_type() : TrajectoryType::INVALID),
                 seg_id};
        if (curr && curr->get_trajectory_type() == TrajectoryType::JOINTSPACE) {
            auto* js = static_cast<JointspaceTrajectory*>(curr);
            js->update(t);
            s.q = js->get_q()(0);
            s.qd = js->get_q_dot()(0);
            s.qdd = js->get_q_dotdot()(0);
        }
        log.push_back(s);
    }

    if (std::getenv("ORC_TRAJ_DISCONT_DUMP_CSV") != nullptr) {
        std::ofstream os("traj_discontinuity_setpoints.csv");
        os << "t,q,qd,qdd,q_ref,qd_ref,qdd_ref,traj_type,seg_id\n";
        for (const auto& s : log)
            os << s.t << ',' << s.q << ',' << s.qd << ',' << s.qdd << ',' << s.q_ref << ','
               << s.qd_ref << ',' << s.qdd_ref << ',' << s.traj_type << ',' << s.seg_id << '\n';
    }

    // --- 4) analysis ---
    //
    // Transparency = at every seam (seg_id transition between two JOINTSPACE
    // samples), the split is indistinguishable from a single-buffer dispatch:
    //   (a) |q[i] - q[i-1] - qd_ref * Ts|     small -> position continuity
    //   (b) |qd[i] - qd[i-1] - qdd_ref * Ts|  small -> velocity continuity (no clamp to 0)
    //   (c) |qdd[i] - qdd[i-1]|               bounded -> acceleration not reset
    //
    // Plus one global check: max |Δqdd| across ALL samples bounded, so a
    // seam-jump the per-seam filter misses still trips the assertion.
    //
    // Also keep absolute fidelity vs. analytic reference as a second-layer
    // regression guard on interpolator accuracy.
    // Thresholds sized ~5-30x over empirically-observed maxima (see console
    // dump). Tighter than the analytical upper bound so a regression that
    // degrades transparency by even a modest factor fails the test.
    const double thresh_q_seam = 5e-5;    // observed ~3e-6
    const double thresh_qd_seam = 5e-4;   // observed ~2e-5
    const double thresh_qdd_seam = 0.15;  // observed ~0.03
    const double thresh_qdd_glob = 0.15;  // observed ~0.06 (qdd changes fastest at t=0,1)

    int n_q_violations = 0, n_qd_violations = 0, n_qdd_violations = 0;
    double max_q_seam = 0.0, max_qd_seam = 0.0, max_qdd_seam = 0.0;
    double max_q_seam_t = 0.0, max_qd_seam_t = 0.0, max_qdd_seam_t = 0.0;

    double max_qdd_step = 0.0, max_qdd_step_t = 0.0;
    double max_q_err = 0.0, max_qd_err = 0.0, max_qdd_err = 0.0;
    double max_q_err_t = 0.0, max_qd_err_t = 0.0, max_qdd_err_t = 0.0;

    for (size_t i = 0; i < log.size(); ++i) {
        if (log[i].traj_type != static_cast<int>(TrajectoryType::JOINTSPACE))
            continue;

        const double eq = std::abs(log[i].q - log[i].q_ref);
        const double eqd = std::abs(log[i].qd - log[i].qd_ref);
        const double eqdd = std::abs(log[i].qdd - log[i].qdd_ref);
        if (eq > max_q_err) {
            max_q_err = eq;
            max_q_err_t = log[i].t;
        }
        if (eqd > max_qd_err) {
            max_qd_err = eqd;
            max_qd_err_t = log[i].t;
        }
        if (eqdd > max_qdd_err) {
            max_qdd_err = eqdd;
            max_qdd_err_t = log[i].t;
        }

        if (i == 0)
            continue;
        if (log[i - 1].traj_type != static_cast<int>(TrajectoryType::JOINTSPACE))
            continue;

        // Global smoothness of qdd (not seam-only): any seam-induced jump
        // the seg_id filter below misses still shows up here.
        const double dqdd_glob = std::abs(log[i].qdd - log[i - 1].qdd);
        if (dqdd_glob > max_qdd_step) {
            max_qdd_step = dqdd_glob;
            max_qdd_step_t = log[i].t;
        }

        if (log[i].seg_id == log[i - 1].seg_id)
            continue;

        const double r_q = std::abs((log[i].q - log[i - 1].q) - log[i].qd_ref * Ts.toSec());
        const double r_qd = std::abs((log[i].qd - log[i - 1].qd) - log[i].qdd_ref * Ts.toSec());
        const double r_qdd = std::abs(log[i].qdd - log[i - 1].qdd);

        if (r_q > max_q_seam) {
            max_q_seam = r_q;
            max_q_seam_t = log[i].t;
        }
        if (r_qd > max_qd_seam) {
            max_qd_seam = r_qd;
            max_qd_seam_t = log[i].t;
        }
        if (r_qdd > max_qdd_seam) {
            max_qdd_seam = r_qdd;
            max_qdd_seam_t = log[i].t;
        }
        if (r_q > thresh_q_seam)
            ++n_q_violations;
        if (r_qd > thresh_qd_seam)
            ++n_qd_violations;
        if (r_qdd > thresh_qdd_seam)
            ++n_qdd_violations;
    }

    std::cout << "[traj-transparency] segments dispatched: " << (seg_id + 1) << " (expected "
              << split_buffers.size() << ")\n"
              << "[traj-transparency] max seam |Δq - qd_ref*Ts|    = " << max_q_seam
              << " @ t=" << max_q_seam_t << "  (thresh " << thresh_q_seam << ")\n"
              << "[traj-transparency] max seam |Δqd - qdd_ref*Ts| = " << max_qd_seam
              << " @ t=" << max_qd_seam_t << "  (thresh " << thresh_qd_seam << ")\n"
              << "[traj-transparency] max seam |Δqdd|             = " << max_qdd_seam
              << " @ t=" << max_qdd_seam_t << "  (thresh " << thresh_qdd_seam << ")\n"
              << "[traj-transparency] max global |Δqdd|           = " << max_qdd_step
              << " @ t=" << max_qdd_step_t << "  (thresh " << thresh_qdd_glob << ")\n"
              << "[traj-transparency] max |q - q_ref|             = " << max_q_err
              << " @ t=" << max_q_err_t << "\n"
              << "[traj-transparency] max |qd - qd_ref|           = " << max_qd_err
              << " @ t=" << max_qd_err_t << "\n"
              << "[traj-transparency] max |qdd - qdd_ref|         = " << max_qdd_err
              << " @ t=" << max_qdd_err_t << "\n";

    // Seam transparency.
    EXPECT_LE(max_q_seam, thresh_q_seam) << "position jump at seam: splitting is not transparent";
    EXPECT_LE(max_qd_seam, thresh_qd_seam) << "velocity jump at seam: splitting is not transparent";
    EXPECT_LE(max_qdd_seam, thresh_qdd_seam)
        << "acceleration jump at seam: splitting is not transparent";

    // Global qdd smoothness (safety net for seam jumps the per-seam filter misses).
    EXPECT_LE(max_qdd_step, thresh_qdd_glob)
        << "unsmooth acceleration step: possible seam jump outside per-seam filter";

    // Absolute fidelity vs. analytic reference (second-layer regression guard
    // on interpolator accuracy).
    EXPECT_LE(max_q_err, 1e-5) << "pose drift vs. analytic smoothstep exceeds 1e-5";
    EXPECT_LE(max_qd_err, 5e-3) << "velocity drift vs. analytic smoothstep exceeds 5e-3";
    EXPECT_LE(max_qdd_err, 0.2) << "acceleration drift vs. analytic smoothstep exceeds 0.2";
}

}  // namespace
