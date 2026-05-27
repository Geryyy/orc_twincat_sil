#pragma once

/**
 * @file FlatBufferTwinCAT.h
 * @brief TwinCAT-specific FlatBuffers integration
 *
 * Simplified zero-copy API for TwinCAT real-time modules. Joint vectors are
 * carried as variadic [double] in the schema; each accessor enforces
 * size() == DOF on init() and rejects mismatched buffers.
 *
 * Usage in UdpInterface.cpp:
 *
 *   #include "FlatBufferTwinCAT.h"
 *   using namespace orc::com::fb::tc;
 *
 *   auto traj_type = get_trajectory_type(pData, nData);
 *   if (traj_type == TrajectoryType::JOINTSPACE) {
 *       JointTrajectoryAccessor<7> acc(pData, nData);
 *       if (acc.is_valid()) {
 *           for (size_t i = 0; i < acc.num_points(); ++i) {
 *               double t  = acc.time(i);
 *               double q0 = acc.position(i, 0);
 *           }
 *       }
 *   }
 */

#ifndef ORC_FLATBUFFER_TWINCAT_H
#define ORC_FLATBUFFER_TWINCAT_H

#include <cstddef>
#include <cstdint>

#include "orc/util/import_flatbuffers.h"
#include "orc_messages_generated.h"

namespace orc {
namespace com {
namespace fb {
namespace tc {

// =============================================================================
// Trajectory Type Detection
// =============================================================================

/**
 * @brief Trajectory types matching orc::trajectory::TrajectoryType
 */
enum class TrajectoryType : uint16_t {
    INVALID = 0,
    STOP,
    JOINTSPACE,
    TASKSPACE,
    NULLSPACE,
    TOOLPARAM,
    JOINT_CTR_PARAM,
    CARTESIAN_CTR_PARAM,
    DENSE_JOINTSPACE
};

inline TrajectoryType get_trajectory_type(const void* buffer, size_t size) {
    if (!buffer || size < 4)
        return TrajectoryType::INVALID;

    auto msg = orc::fb::GetTrajectoryMessage(buffer);
    if (!msg)
        return TrajectoryType::INVALID;

    switch (msg->data_type()) {
        case orc::fb::TrajectoryData::JointTrajectory:
            return TrajectoryType::JOINTSPACE;
        case orc::fb::TrajectoryData::DenseJointTrajectory:
            return TrajectoryType::DENSE_JOINTSPACE;
        case orc::fb::TrajectoryData::TaskspaceTrajectory:
            return TrajectoryType::TASKSPACE;
        case orc::fb::TrajectoryData::NullspaceTrajectory:
            return TrajectoryType::NULLSPACE;
        case orc::fb::TrajectoryData::StopTrajectory:
            return TrajectoryType::STOP;
        case orc::fb::TrajectoryData::JointCtrParamTrajectory:
            return TrajectoryType::JOINT_CTR_PARAM;
        case orc::fb::TrajectoryData::CartesianCtrParamTrajectory:
            return TrajectoryType::CARTESIAN_CTR_PARAM;
        default:
            return TrajectoryType::INVALID;
    }
}

inline bool verify_message(const void* buffer, size_t size) {
    if (!buffer || size == 0)
        return false;
    flatbuffers::Verifier verifier(static_cast<const uint8_t*>(buffer), size);
    return orc::fb::VerifyTrajectoryMessageBuffer(verifier);
}

// =============================================================================
// Inline pose / cartesian struct accessors (kept here so the TwinCAT header
// has no dependency on the FlatBufferAccessors translation unit).
// =============================================================================

namespace detail {
inline double pose_at(const orc::fb::PoseVector* p, int i) {
    if (!p)
        return 0.0;
    switch (i) {
        case 0:
            return p->x();
        case 1:
            return p->y();
        case 2:
            return p->z();
        case 3:
            return p->qw();
        case 4:
            return p->qx();
        case 5:
            return p->qy();
        case 6:
            return p->qz();
        default:
            return 0.0;
    }
}

inline void copy_pose_(const orc::fb::PoseVector* p, double* out) {
    if (!p || !out)
        return;
    out[0] = p->x();
    out[1] = p->y();
    out[2] = p->z();
    out[3] = p->qw();
    out[4] = p->qx();
    out[5] = p->qy();
    out[6] = p->qz();
}
}  // namespace detail

// =============================================================================
// Zero-Copy Accessors for TwinCAT
// =============================================================================

/**
 * @brief Zero-copy accessor for joint trajectory data
 */
template <int DOF>
class JointTrajectoryAccessor {
public:
    JointTrajectoryAccessor() : valid_(false), traj_(nullptr) {}

    JointTrajectoryAccessor(const void* buffer, size_t size) : valid_(false), traj_(nullptr) {
        init(buffer, size);
    }

    bool init(const void* buffer, size_t size) {
        valid_ = false;
        traj_ = nullptr;
        if (!buffer || size < 4)
            return false;

        auto msg = orc::fb::GetTrajectoryMessage(buffer);
        if (!msg || msg->data_type() != orc::fb::TrajectoryData::JointTrajectory)
            return false;

        traj_ = msg->data_as_JointTrajectory();
        if (!traj_ || !traj_->points())
            return false;

        if (traj_->points()->size() > 0) {
            auto p0 = traj_->points()->Get(0);
            if (!p0 || !p0->position() ||
                p0->position()->size() != static_cast<flatbuffers::uoffset_t>(DOF))
                return false;
        }

        valid_ = true;
        return true;
    }

    bool is_valid() const { return valid_; }
    size_t num_points() const { return traj_ ? traj_->points()->size() : 0; }

    double time(size_t idx) const {
        return traj_->points()->Get(static_cast<flatbuffers::uoffset_t>(idx))->time_ns() / 1.0e9;
    }

    double position(size_t idx, int joint) const {
        auto pos = traj_->points()->Get(static_cast<flatbuffers::uoffset_t>(idx))->position();
        return pos ? pos->Get(static_cast<flatbuffers::uoffset_t>(joint)) : 0.0;
    }

    void copy_position(size_t idx, double* out) const {
        if (!out)
            return;
        auto pos = traj_->points()->Get(static_cast<flatbuffers::uoffset_t>(idx))->position();
        if (!pos)
            return;
        for (int j = 0; j < DOF; ++j)
            out[j] = pos->Get(static_cast<flatbuffers::uoffset_t>(j));
    }

private:
    bool valid_;
    const orc::fb::JointTrajectory* traj_;
};

/**
 * @brief Zero-copy accessor for dense joint trajectory data
 */
template <int DOF>
class DenseJointTrajectoryAccessor {
public:
    DenseJointTrajectoryAccessor() : valid_(false), traj_(nullptr) {}

    DenseJointTrajectoryAccessor(const void* buffer, size_t size) : valid_(false), traj_(nullptr) {
        init(buffer, size);
    }

    bool init(const void* buffer, size_t size) {
        valid_ = false;
        traj_ = nullptr;
        if (!buffer || size < 4)
            return false;

        auto msg = orc::fb::GetTrajectoryMessage(buffer);
        if (!msg || msg->data_type() != orc::fb::TrajectoryData::DenseJointTrajectory)
            return false;

        traj_ = msg->data_as_DenseJointTrajectory();
        if (!traj_ || !traj_->points())
            return false;

        if (traj_->points()->size() > 0) {
            auto p0 = traj_->points()->Get(0);
            if (!p0 || !p0->q() || p0->q()->size() != static_cast<flatbuffers::uoffset_t>(DOF))
                return false;
        }

        valid_ = true;
        return true;
    }

    bool is_valid() const { return valid_; }
    size_t num_points() const { return traj_ ? traj_->points()->size() : 0; }

    double time(size_t idx) const {
        return traj_->points()->Get(static_cast<flatbuffers::uoffset_t>(idx))->time_ns() / 1.0e9;
    }

    double q(size_t idx, int joint) const { return at_(idx, joint, &orc::fb::DenseJointPoint::q); }
    double q_dot(size_t idx, int joint) const {
        return at_(idx, joint, &orc::fb::DenseJointPoint::q_dot);
    }
    double q_dotdot(size_t idx, int joint) const {
        return at_(idx, joint, &orc::fb::DenseJointPoint::q_dotdot);
    }
    double tau_ff(size_t idx, int joint) const {
        return at_(idx, joint, &orc::fb::DenseJointPoint::tau_ff);
    }

    void copy_point(size_t idx, double* q_out, double* q_dot_out, double* q_dotdot_out,
                    double* tau_ff_out) const {
        auto pt = traj_->points()->Get(static_cast<flatbuffers::uoffset_t>(idx));
        copy_(pt->q(), q_out);
        copy_(pt->q_dot(), q_dot_out);
        copy_(pt->q_dotdot(), q_dotdot_out);
        copy_(pt->tau_ff(), tau_ff_out);
    }

private:
    template <typename Getter>
    double at_(size_t idx, int joint, Getter getter) const {
        auto pt = traj_->points()->Get(static_cast<flatbuffers::uoffset_t>(idx));
        auto vec = (pt->*getter)();
        return vec ? vec->Get(static_cast<flatbuffers::uoffset_t>(joint)) : 0.0;
    }

    static void copy_(const flatbuffers::Vector<double>* v, double* out) {
        if (!v || !out)
            return;
        for (int j = 0; j < DOF; ++j)
            out[j] = v->Get(static_cast<flatbuffers::uoffset_t>(j));
    }

    bool valid_;
    const orc::fb::DenseJointTrajectory* traj_;
};

/**
 * @brief Zero-copy accessor for taskspace trajectory data
 */
class TaskspaceTrajectoryAccessor {
public:
    TaskspaceTrajectoryAccessor() : valid_(false), traj_(nullptr) {}

    TaskspaceTrajectoryAccessor(const void* buffer, size_t size) : valid_(false), traj_(nullptr) {
        init(buffer, size);
    }

    bool init(const void* buffer, size_t size) {
        valid_ = false;
        traj_ = nullptr;
        if (!buffer || size < 4)
            return false;

        auto msg = orc::fb::GetTrajectoryMessage(buffer);
        if (!msg || msg->data_type() != orc::fb::TrajectoryData::TaskspaceTrajectory)
            return false;

        traj_ = msg->data_as_TaskspaceTrajectory();
        if (!traj_ || !traj_->points())
            return false;

        valid_ = true;
        return true;
    }

    bool is_valid() const { return valid_; }
    size_t num_points() const { return traj_ ? traj_->points()->size() : 0; }

    double time(size_t idx) const {
        return traj_->points()->Get(static_cast<flatbuffers::uoffset_t>(idx))->time_ns() / 1.0e9;
    }

    double pose(size_t idx, int component) const {
        return detail::pose_at(
            traj_->points()->Get(static_cast<flatbuffers::uoffset_t>(idx))->pose(), component);
    }

    void copy_pose(size_t idx, double* out) const {
        detail::copy_pose_(traj_->points()->Get(static_cast<flatbuffers::uoffset_t>(idx))->pose(),
                           out);
    }

private:
    bool valid_;
    const orc::fb::TaskspaceTrajectory* traj_;
};

/**
 * @brief Zero-copy accessor for nullspace trajectory
 */
template <int DOF>
class NullspaceTrajectoryAccessor {
public:
    NullspaceTrajectoryAccessor() : valid_(false), time_(0.0), traj_(nullptr) {}

    NullspaceTrajectoryAccessor(const void* buffer, size_t size)
        : valid_(false), time_(0.0), traj_(nullptr) {
        init(buffer, size);
    }

    bool init(const void* buffer, size_t size) {
        valid_ = false;
        time_ = 0.0;
        traj_ = nullptr;
        if (!buffer || size < 4)
            return false;

        auto msg = orc::fb::GetTrajectoryMessage(buffer);
        if (!msg || msg->data_type() != orc::fb::TrajectoryData::NullspaceTrajectory)
            return false;

        traj_ = msg->data_as_NullspaceTrajectory();
        if (!traj_ || !traj_->q_nullspace())
            return false;

        if (traj_->q_nullspace()->size() != static_cast<flatbuffers::uoffset_t>(DOF))
            return false;

        time_ = traj_->time_ns() / 1.0e9;
        valid_ = true;
        return true;
    }

    bool is_valid() const { return valid_; }
    double time() const { return time_; }

    double q_nullspace(int joint) const {
        auto v = traj_->q_nullspace();
        return v ? v->Get(static_cast<flatbuffers::uoffset_t>(joint)) : 0.0;
    }

    void copy_q_nullspace(double* out) const {
        auto v = traj_->q_nullspace();
        if (!v || !out)
            return;
        for (int j = 0; j < DOF; ++j)
            out[j] = v->Get(static_cast<flatbuffers::uoffset_t>(j));
    }

private:
    bool valid_;
    double time_;
    const orc::fb::NullspaceTrajectory* traj_;
};

// =============================================================================
// Convenience Typedefs
// =============================================================================

using JointTrajectoryAccessor7 = JointTrajectoryAccessor<7>;
using JointTrajectoryAccessor2 = JointTrajectoryAccessor<2>;
using DenseJointTrajectoryAccessor7 = DenseJointTrajectoryAccessor<7>;
using DenseJointTrajectoryAccessor2 = DenseJointTrajectoryAccessor<2>;
using NullspaceTrajectoryAccessor7 = NullspaceTrajectoryAccessor<7>;
using NullspaceTrajectoryAccessor2 = NullspaceTrajectoryAccessor<2>;

}  // namespace tc
}  // namespace fb
}  // namespace com
}  // namespace orc

#endif  // ORC_FLATBUFFER_TWINCAT_H
