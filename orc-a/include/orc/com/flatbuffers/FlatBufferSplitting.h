#pragma once

// FlatBufferSplitting.h
//
// Pre-FlatBuffer parity: split long trajectories into overlap-split
// FlatBuffer payloads before UDP send. Two bounds drive the cap:
//
//   1. Wire size. UDP datagrams go to a TwinCAT-RT receiver that does
//      not reassemble IP fragments. Single-frame payload limit is
//      MAX_UDP_PAYLOAD (1472 B). Larger payloads are dropped on the
//      wire.
//
//   2. Receiver spline-init budget. Each enqueued split drives a
//      spline re-fit on the RT side via JointspaceTrajectory::init
//      (cost scales with knot count); a long split can blow the
//      control-loop period.
//
// Per-split cap = min(user_cap, wire_cap) (user_cap == 0 means
// "no user limit, use wire_cap"). An intermediate (overlap) slice
// straddles every seam between primaries so the receiver is never
// interpolating exactly at a boundary where the spline's forced
// q_dot = q_dotdot = 0 would flatten the setpoint stream; the
// TrajectoryQueue swaps to the overlap slice partway through the
// preceding primary via get_start_time() comparisons, with
// continuity carried by save_state -> init(saved_state) at the swap.

#include <algorithm>
#include <cstddef>
#include <vector>

namespace orc::com::fb {

// Wire budget carved from the Ethernet UDP payload limit. Not pulled
// from orc/com/TrajectoryServer.h because that header transitively
// includes FlatBufferSerializer.h via RobotState, and this header is
// itself included from FlatBufferSerializer.h -- the resulting cycle
// forward-references helpers declared in FlatBufferSerializer.h (see
// FlatBufferRobotState.h uses of to_fb_robot_status / detail::pose_at).
// The value must match orc::com::MAX_UDP_PAYLOAD (1472); a
// static_assert in FlatBufferSerializer.cpp would be ideal but the
// serializer is header-only, so keep these two constants in lockstep
// by convention. The 128 B overhead is a conservative static
// reservation for the FlatBuffer root offset, vtable, file identifier,
// and envelope table fields; validated against GetSize() in the
// wire-size roundtrip tests.
constexpr size_t kUdpPayloadBudget = 1472;
constexpr size_t kFbHeaderOverhead = 128;

// Pre-FlatBuffer parity value for the user cap: bounds receiver-side
// spline-init cost at ~10 knots worst-case. Pass this at call sites
// that want RT-safe bounds; pass 0 for wire-cap-only.
constexpr size_t kDefaultUserMaxPtsPerSplit = 10;

// Per-point conservative upper bounds on FlatBuffer serialized size.
// Each term: 24 B = time (8) + per-point table/vtable/offset (16);
// 16 B per [double] vector = length prefix (4) + offset (4) +
// alignment slack (8); the struct forms (PoseVector, CartesianVector)
// are inline with no per-vector overhead.
template <int DOF>
constexpr size_t bytes_per_joint_point() {
    return 24 + 16 + DOF * 8;  // time + [double] q (length DOF)
}

template <int DOF>
constexpr size_t bytes_per_dense_point() {
    return 24 + 4 * (16 + DOF * 8);  // time + 4 x [double] (q, q_dot, q_dotdot, tau)
}

template <int DOF>
constexpr size_t bytes_per_velocity_point() {
    return bytes_per_joint_point<DOF>();
}

constexpr size_t bytes_per_pose_point() {
    return 24 + 7 * 8;  // time + inline PoseVector (7 doubles)
}

constexpr size_t bytes_per_cartesian_velocity_point() {
    return 24 + 6 * 8;  // time + inline CartesianVector (6 doubles)
}

constexpr size_t bytes_per_hybrid_force_motion_point() {
    return 24 + 7 * 8 + 8;  // time + inline PoseVector + force scalar
}

// Wire-derived cap: how many points fit under one UDP frame.
constexpr size_t default_max_pts_per_split(size_t bytes_per_pt) {
    if (bytes_per_pt == 0)
        return 1;
    const size_t budget =
        (kUdpPayloadBudget > kFbHeaderOverhead) ? (kUdpPayloadBudget - kFbHeaderOverhead) : 0;
    const size_t n = budget / bytes_per_pt;
    return n == 0 ? 1 : n;
}

// Effective cap: min(user_cap, wire_cap). user_cap == 0 -> no user
// limit (wire_cap alone). Mirrors the pre-FB std::min guard.
constexpr size_t effective_max_pts_per_split(size_t user_cap, size_t wire_cap) {
    if (user_cap == 0)
        return wire_cap;
    return (user_cap < wire_cap) ? user_cap : wire_cap;
}

// Slice over the original time/point arrays. Pre-FB wire treats interm
// and primary slices equivalently; the TrajectoryQueue continuity is a
// queue-level property, not a wire-level one.
struct SplitSlice {
    size_t start_idx;  // inclusive
    size_t end_idx;    // exclusive
};

// Overlap slicer. For n_points > cap, emits:
//     primary_0  [0, cap)
//     interm_0   [cap - ceil(cap/2), cap - ceil(cap/2) + cap)   -- straddles seam at cap
//     primary_1  [cap, 2*cap)
//     interm_1   [2*cap - ceil(cap/2), 2*cap - ceil(cap/2) + cap)
//     ...
//     primary_N  [N*cap, min((N+1)*cap, n))                      -- tail
// Every slice is at least 3 knots (spline needs >= 3); shorter
// slices are dropped. Empty return for n < 3 or cap < 3.
inline std::vector<SplitSlice> compute_overlap_splits(size_t n_points, size_t cap) {
    std::vector<SplitSlice> out;
    if (n_points == 0 || cap < 3)
        return out;
    if (n_points <= cap) {
        if (n_points >= 3)
            out.push_back({0, n_points});
        return out;
    }
    const size_t half = (cap + 1) / 2;  // ceil(cap/2), matches pre-FB std::ceil(max/2)
    size_t cursor = 0;
    // First primary has no preceding interm (nothing to stitch to).
    out.push_back({cursor, cursor + cap});
    cursor += cap;
    while (cursor < n_points) {
        // Interm straddles the previous seam: [cursor - half, cursor - half + cap).
        const size_t interm_start = cursor - half;
        const size_t interm_end = std::min(interm_start + cap, n_points);
        if (interm_end - interm_start >= 3) {
            out.push_back({interm_start, interm_end});
        }
        // Primary: next cap-sized slice (or tail).
        const size_t prim_end = std::min(cursor + cap, n_points);
        if (prim_end - cursor >= 3) {
            out.push_back({cursor, prim_end});
        }
        cursor += cap;
    }
    return out;
}

// Template entry point used by every serialize_*_split method: derive
// cap from min(user_cap, wire_cap), slice the input, and let the caller
// produce one serialized buffer per slice. The caller's lambda owns all
// per-type bits (which underlying serializer to call, which input vectors
// to slice).
template <typename PerSliceFn>
std::vector<std::vector<uint8_t>> serialize_in_splits(size_t n_pts, size_t bytes_per_pt,
                                                      size_t user_cap, PerSliceFn per_slice) {
    const size_t wire_cap = default_max_pts_per_split(bytes_per_pt);
    const size_t cap = effective_max_pts_per_split(user_cap, wire_cap);
    const auto slices = compute_overlap_splits(n_pts, cap);
    std::vector<std::vector<uint8_t>> out;
    out.reserve(slices.size());
    for (const auto& s : slices) {
        out.push_back(per_slice(s.start_idx, s.end_idx));
    }
    return out;
}

}  // namespace orc::com::fb
