// Tests for the FlatBuffer overlap-splitting machinery:
//   (1) compute_overlap_splits slicing algorithm edge cases.
//   (2) Wire-size roundtrip: every split produced by the six
//       FlatBufferSerializer::serialize_*_trajectory_split entry
//       points must fit under MAX_UDP_PAYLOAD (1472 B) and deserialize
//       to a trajectory of the expected type, for both the wire-cap
//       default (user_cap == 0) and an over-sized user cap that has
//       to be silently clamped to the wire cap.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "orc/OrcTypes.h"
#include "orc/com/TrajectoryServer.h"
#include "orc/com/flatbuffers/FlatBufferDeserializer.h"
#include "orc/com/flatbuffers/FlatBufferSerializer.h"
#include "orc/com/flatbuffers/FlatBufferSplitting.h"
#include "orc/trajectory/TrajectoryBase.h"
#include "orc/trajectory/TrajectoryType.h"
#include "orc/util/Time.h"

namespace {

constexpr int DoF = 7;
constexpr size_t kMtu = orc::com::MAX_UDP_PAYLOAD;  // 1472
constexpr size_t kOversizeCap = 100000;             // must clamp to wire_cap

using Serializer = orc::com::fb::FlatBufferSerializer<DoF>;
using Deserializer = orc::com::fb::FlatBufferDeserializer<DoF>;
using TrajectoryType = orc::trajectory::TrajectoryType;
using JointVector = orc::RobotTraits<DoF>::JointVector;
using PoseVector = orc::PoseVector;
using CartesianVec = orc::CartesianVector;
using Time = orc::Time;

JointVector make_joint_knot(int k) {
    return JointVector::LinSpaced(DoF, 0.01 * k, 0.01 * k + 0.5);
}

PoseVector make_pose_knot(int k) {
    PoseVector p = PoseVector::Zero();
    p(0) = 0.01 * k;
    p(1) = 0.02 * k;
    p(2) = 0.03 * k;
    p(3) = 1.0;  // quaternion w
    return p;
}

CartesianVec make_cart_vel_knot(int k) {
    CartesianVec v = CartesianVec::Zero();
    v(0) = 0.001 * k;
    v(1) = 0.002 * k;
    v(2) = 0.003 * k;
    return v;
}

std::vector<Time> make_time_grid(int n, double dt = 0.01) {
    std::vector<Time> t;
    t.reserve(n);
    for (int k = 0; k < n; ++k)
        t.emplace_back(dt * k);
    return t;
}

}  // namespace

// =============================================================================
// (1) Slicing algorithm edge cases.
// =============================================================================

TEST(SplitSliceAlgorithm, EmptyWhenFewerThanThreePoints) {
    EXPECT_TRUE(orc::com::fb::compute_overlap_splits(0, 5).empty());
    EXPECT_TRUE(orc::com::fb::compute_overlap_splits(1, 5).empty());
    EXPECT_TRUE(orc::com::fb::compute_overlap_splits(2, 5).empty());
}

TEST(SplitSliceAlgorithm, EmptyWhenCapBelowThree) {
    EXPECT_TRUE(orc::com::fb::compute_overlap_splits(20, 0).empty());
    EXPECT_TRUE(orc::com::fb::compute_overlap_splits(20, 1).empty());
    EXPECT_TRUE(orc::com::fb::compute_overlap_splits(20, 2).empty());
}

TEST(SplitSliceAlgorithm, SinglePrimaryWhenFitsInOneSplit) {
    {
        auto s = orc::com::fb::compute_overlap_splits(5, 5);
        ASSERT_EQ(s.size(), 1u);
        EXPECT_EQ(s[0].start_idx, 0u);
        EXPECT_EQ(s[0].end_idx, 5u);
    }
    {
        auto s = orc::com::fb::compute_overlap_splits(3, 5);
        ASSERT_EQ(s.size(), 1u);
        EXPECT_EQ(s[0].start_idx, 0u);
        EXPECT_EQ(s[0].end_idx, 3u);
    }
}

TEST(SplitSliceAlgorithm, OneOverCapYieldsPrimaryPlusInterm) {
    // n = cap + 1 = 6, cap = 5, half = 3.
    // primary [0, 5); interm [5-3, min(5-3+5, 6)) = [2, 6) length 4 (>= 3, kept).
    // tail primary prim_end=min(10,6)=6, prim_end-cursor=1 (< 3, dropped).
    auto s = orc::com::fb::compute_overlap_splits(6, 5);
    ASSERT_EQ(s.size(), 2u);
    EXPECT_EQ(s[0].start_idx, 0u);
    EXPECT_EQ(s[0].end_idx, 5u);
    EXPECT_EQ(s[1].start_idx, 2u);
    EXPECT_EQ(s[1].end_idx, 6u);
}

TEST(SplitSliceAlgorithm, OneOverCapDropsIntermWhenShorterThanThree) {
    // n = cap + 1 = 4, cap = 3, half = 2.
    // primary [0, 3); interm [3-2, min(3-2+3, 4)) = [1, 4) length 3 (kept).
    // primary tail 1 point dropped.
    auto s = orc::com::fb::compute_overlap_splits(4, 3);
    ASSERT_EQ(s.size(), 2u);
    EXPECT_EQ(s[0].start_idx, 0u);
    EXPECT_EQ(s[0].end_idx, 3u);
    EXPECT_EQ(s[1].start_idx, 1u);
    EXPECT_EQ(s[1].end_idx, 4u);
}

TEST(SplitSliceAlgorithm, ThreeCapSlicesAndInterms) {
    // n = 3 * cap = 15, cap = 5, half = 3. Expected:
    //   primary [0, 5); interm [2, 7); primary [5, 10); interm [7, 12); primary [10, 15).
    auto s = orc::com::fb::compute_overlap_splits(15, 5);
    ASSERT_EQ(s.size(), 5u);
    EXPECT_EQ(s[0].start_idx, 0u);
    EXPECT_EQ(s[0].end_idx, 5u);
    EXPECT_EQ(s[1].start_idx, 2u);
    EXPECT_EQ(s[1].end_idx, 7u);
    EXPECT_EQ(s[2].start_idx, 5u);
    EXPECT_EQ(s[2].end_idx, 10u);
    EXPECT_EQ(s[3].start_idx, 7u);
    EXPECT_EQ(s[3].end_idx, 12u);
    EXPECT_EQ(s[4].start_idx, 10u);
    EXPECT_EQ(s[4].end_idx, 15u);
}

TEST(SplitSliceAlgorithm, IntermStraddlesEverySeam) {
    // For every (primary_i, interm_i, primary_{i+1}) triple, the interm must
    // start before the seam (= primary_i.end_idx) and end strictly past it.
    auto s = orc::com::fb::compute_overlap_splits(30, 5);
    ASSERT_GE(s.size(), 3u);
    // Slices alternate primary, interm, primary, interm, ... starting with primary.
    for (size_t i = 1; i + 1 < s.size(); i += 2) {
        const auto& prev_prim = s[i - 1];
        const auto& interm = s[i];
        const auto& next_prim = s[i + 1];
        EXPECT_LT(interm.start_idx, prev_prim.end_idx)
            << "interm at index " << i << " does not start before seam";
        EXPECT_GT(interm.end_idx, next_prim.start_idx)
            << "interm at index " << i << " does not reach past seam";
    }
}

// =============================================================================
// (2) Wire-size roundtrip per entry point.
//
// Each test verifies, for both user_cap=0 (wire cap only) and user_cap=100000
// (oversize, must clamp to wire cap via effective_max_pts_per_split):
//   - serializer produces at least two splits for the 100-knot input,
//   - every split fits under MAX_UDP_PAYLOAD,
//   - every split deserializes to a non-null trajectory of the expected type.
// =============================================================================

namespace {

// Run a roundtrip check with the given user cap and assert MTU + type on every split.
template <typename Lambda>
void verify_split(const Lambda& split_fn, TrajectoryType expected_type) {
    Deserializer deser;
    auto splits = split_fn();
    ASSERT_GE(splits.size(), 2u) << "default cap should split 100-knot input";
    for (size_t i = 0; i < splits.size(); ++i) {
        const auto& buf = splits[i];
        EXPECT_LE(buf.size(), kMtu)
            << "split " << i << " (" << buf.size() << " B) exceeds MTU " << kMtu;
        auto traj = deser.deserialize(buf.data(), buf.size());
        ASSERT_NE(traj, nullptr) << "split " << i << " failed to deserialize";
        EXPECT_EQ(traj->get_trajectory_type(), expected_type)
            << "split " << i << " has wrong trajectory type";
    }
}

}  // namespace

TEST(FlatBufferSplitRoundtrip, JointTrajectory) {
    Serializer ser;
    constexpr int N = 100;
    auto t = make_time_grid(N);
    std::vector<JointVector> q;
    q.reserve(N);
    for (int k = 0; k < N; ++k)
        q.push_back(make_joint_knot(k));

    verify_split([&] { return ser.serialize_joint_trajectory_split(t, q, /*user_cap=*/0); },
                 TrajectoryType::JOINTSPACE);
    verify_split(
        [&] { return ser.serialize_joint_trajectory_split(t, q, /*user_cap=*/kOversizeCap); },
        TrajectoryType::JOINTSPACE);
}

TEST(FlatBufferSplitRoundtrip, DenseJointTrajectory) {
    Serializer ser;
    constexpr int N = 100;
    auto t = make_time_grid(N);
    std::vector<JointVector> q, qd, qdd, tau;
    q.reserve(N);
    qd.reserve(N);
    qdd.reserve(N);
    tau.reserve(N);
    for (int k = 0; k < N; ++k) {
        q.push_back(make_joint_knot(k));
        qd.push_back(make_joint_knot(k) * 0.5);
        qdd.push_back(make_joint_knot(k) * 0.25);
        tau.push_back(make_joint_knot(k) * 0.125);
    }

    verify_split([&] { return ser.serialize_dense_joint_trajectory_split(t, q, qd, qdd, tau, 0); },
                 TrajectoryType::DENSE_JOINTSPACE);
    verify_split(
        [&] {
            return ser.serialize_dense_joint_trajectory_split(t, q, qd, qdd, tau, kOversizeCap);
        },
        TrajectoryType::DENSE_JOINTSPACE);
}

TEST(FlatBufferSplitRoundtrip, CartesianTrajectory) {
    Serializer ser;
    constexpr int N = 100;
    auto t = make_time_grid(N);
    std::vector<PoseVector> p;
    p.reserve(N);
    for (int k = 0; k < N; ++k)
        p.push_back(make_pose_knot(k));

    verify_split([&] { return ser.serialize_cartesian_trajectory_split(t, p, 0); },
                 TrajectoryType::TASKSPACE);
    verify_split([&] { return ser.serialize_cartesian_trajectory_split(t, p, kOversizeCap); },
                 TrajectoryType::TASKSPACE);
}

TEST(FlatBufferSplitRoundtrip, JointspaceVelocityTrajectory) {
    Serializer ser;
    constexpr int N = 100;
    auto t = make_time_grid(N);
    std::vector<JointVector> v;
    v.reserve(N);
    for (int k = 0; k < N; ++k)
        v.push_back(make_joint_knot(k) * 0.5);

    verify_split([&] { return ser.serialize_jointspace_velocity_trajectory_split(t, v, 0); },
                 TrajectoryType::JOINTSPACE_VELOCITY);
    verify_split(
        [&] { return ser.serialize_jointspace_velocity_trajectory_split(t, v, kOversizeCap); },
        TrajectoryType::JOINTSPACE_VELOCITY);
}

TEST(FlatBufferSplitRoundtrip, CartesianVelocityTrajectory) {
    Serializer ser;
    constexpr int N = 100;
    auto t = make_time_grid(N);
    std::vector<CartesianVec> v;
    v.reserve(N);
    for (int k = 0; k < N; ++k)
        v.push_back(make_cart_vel_knot(k));

    verify_split([&] { return ser.serialize_cartesian_velocity_trajectory_split(t, v, 0); },
                 TrajectoryType::CARTESIAN_VELOCITY);
    verify_split(
        [&] { return ser.serialize_cartesian_velocity_trajectory_split(t, v, kOversizeCap); },
        TrajectoryType::CARTESIAN_VELOCITY);
}

TEST(FlatBufferSplitRoundtrip, HybridForceMotionTrajectory) {
    Serializer ser;
    constexpr int N = 100;
    auto t = make_time_grid(N);
    std::vector<PoseVector> p;
    std::vector<double> f;
    p.reserve(N);
    f.reserve(N);
    for (int k = 0; k < N; ++k) {
        p.push_back(make_pose_knot(k));
        f.push_back(0.1 * k);
    }

    verify_split([&] { return ser.serialize_hybrid_force_motion_trajectory_split(t, p, f, 0); },
                 TrajectoryType::HYBRID_FORCE_MOTION);
    verify_split(
        [&] { return ser.serialize_hybrid_force_motion_trajectory_split(t, p, f, kOversizeCap); },
        TrajectoryType::HYBRID_FORCE_MOTION);
}
