#pragma once

/**
 * @file FlatBufferRobotState.h
 * @brief FlatBuffers serialization for RobotState messages
 *
 * Robot state sent from TwinCAT back to the PC. The unified RobotState
 * schema is variadic in DoF; pose/cartesian/nullspace fields are present
 * for every DoF (callers that don't use them write zeros).
 */

#include <cstdint>
#include <cstring>
#include <vector>

#include "orc/util/import_flatbuffers.h"
#include "orc_messages_generated.h"

#include "orc/OrcTypes.h"
#include "orc/RobotStatus.h"
#include "orc/RobotTraits.h"
#include "orc/com/flatbuffers/FlatBufferDeserializer.h"
#include "orc/com/flatbuffers/FlatBufferEigen.h"
#include "orc/com/flatbuffers/FlatBufferSerializer.h"  // to_fb_robot_status / from_fb_robot_status

namespace orc::com::fb {

/**
 * @brief Serialize RobotState to FlatBuffer format
 *
 * @tparam RobotType Robot type (e.g., orc::robots::Iiwa, orc::robots::LinearAxis)
 */
template <typename RobotType>
class RobotStateSerializer {
public:
    static constexpr int DOF = RobotType::DOF;
    using JointVector = typename RobotType::JointVector;

    RobotStateSerializer() : builder_(1024) {}

    /**
     * @brief Serialize directly into caller-provided buffer.
     * @return Number of bytes written; 0 if buffer too small.
     */
    size_t serialize_into(uint8_t* dest, size_t max_size, orc::Time time,
                          orc::logic::RobotStatus status, uint8_t model_id,
                          const JointVector& q_act, const JointVector& q_dot_act,
                          const JointVector& q_dotdot_act, const JointVector& tau,
                          const JointVector& q_set, const JointVector& q_dot_set,
                          const JointVector& q_dotdot_set,
                          const orc::PoseVector& x_set = orc::PoseVector::Zero(),
                          const orc::CartesianVector& x_dot_set = orc::CartesianVector::Zero(),
                          const orc::CartesianVector& x_dotdot_set = orc::CartesianVector::Zero(),
                          const JointVector& q_d_ns = JointVector::Zero()) {
        build_(time, status, model_id, q_act, q_dot_act, q_dotdot_act, tau, q_set, q_dot_set,
               q_dotdot_set, x_set, x_dot_set, x_dotdot_set, q_d_ns);
        const size_t size = builder_.GetSize();
        if (size > max_size)
            return 0;
        std::memcpy(dest, builder_.GetBufferPointer(), size);
        return size;
    }

    /**
     * @brief Serialize to vector via UdpStateMessage wrapper.
     */
    std::vector<uint8_t> serialize(
        orc::Time time, orc::logic::RobotStatus status, uint8_t model_id, const JointVector& q_act,
        const JointVector& q_dot_act, const JointVector& q_dotdot_act, const JointVector& tau,
        const JointVector& q_set, const JointVector& q_dot_set, const JointVector& q_dotdot_set,
        const orc::PoseVector& x_set = orc::PoseVector::Zero(),
        const orc::CartesianVector& x_dot_set = orc::CartesianVector::Zero(),
        const orc::CartesianVector& x_dotdot_set = orc::CartesianVector::Zero(),
        const JointVector& q_d_ns = JointVector::Zero()) {
        build_(time, status, model_id, q_act, q_dot_act, q_dotdot_act, tau, q_set, q_dot_set,
               q_dotdot_set, x_set, x_dot_set, x_dotdot_set, q_d_ns);
        return std::vector<uint8_t>(builder_.GetBufferPointer(),
                                    builder_.GetBufferPointer() + builder_.GetSize());
    }

private:
    flatbuffers::FlatBufferBuilder builder_;

    void build_(orc::Time time, orc::logic::RobotStatus status, uint8_t model_id,
                const JointVector& q_act, const JointVector& q_dot_act,
                const JointVector& q_dotdot_act, const JointVector& tau, const JointVector& q_set,
                const JointVector& q_dot_set, const JointVector& q_dotdot_set,
                const orc::PoseVector& x_set, const orc::CartesianVector& x_dot_set,
                const orc::CartesianVector& x_dotdot_set, const JointVector& q_d_ns) {
        builder_.Clear();

        const orc::fb::PoseVector fb_x_set(x_set[0], x_set[1], x_set[2], x_set[3], x_set[4],
                                           x_set[5], x_set[6]);
        const orc::fb::CartesianVector fb_x_dot_set(x_dot_set[0], x_dot_set[1], x_dot_set[2],
                                                    x_dot_set[3], x_dot_set[4], x_dot_set[5]);
        const orc::fb::CartesianVector fb_x_dotdot_set(x_dotdot_set[0], x_dotdot_set[1],
                                                       x_dotdot_set[2], x_dotdot_set[3],
                                                       x_dotdot_set[4], x_dotdot_set[5]);

        auto state = orc::fb::CreateRobotState(
            builder_, time.toNSec(), to_fb_robot_status(status), model_id,
            fb_write_eigen(builder_, q_act), fb_write_eigen(builder_, q_dot_act),
            fb_write_eigen(builder_, q_dotdot_act), fb_write_eigen(builder_, tau),
            fb_write_eigen(builder_, q_set), fb_write_eigen(builder_, q_dot_set),
            fb_write_eigen(builder_, q_dotdot_set), &fb_x_set, &fb_x_dot_set, &fb_x_dotdot_set,
            fb_write_eigen(builder_, q_d_ns));

        auto msg = orc::fb::CreateUdpStateMessage(builder_, 0, 0, state);
        builder_.Finish(msg);
    }
};

/**
 * @brief Zero-copy reader for RobotState FlatBuffer messages
 *
 * @tparam DOF Degrees of freedom; reader rejects buffers with mismatched size.
 */
template <int DOF>
class RobotStateReader {
public:
    RobotStateReader() : state_msg_(nullptr), state_(nullptr) {}

    bool init(const void* buffer, size_t size) {
        state_msg_ = nullptr;
        state_ = nullptr;
        if (!buffer || size == 0)
            return false;

        flatbuffers::Verifier verifier(static_cast<const uint8_t*>(buffer), size);
        if (!verifier.VerifyBuffer<orc::fb::UdpStateMessage>(nullptr))
            return false;

        state_msg_ = flatbuffers::GetRoot<orc::fb::UdpStateMessage>(buffer);
        if (!state_msg_)
            return false;

        state_ = state_msg_->state();
        if (!state_)
            return false;

        // DoF guard: each joint vector must be DOF-wide if present.
        if (state_->q_act() && state_->q_act()->size() != static_cast<flatbuffers::uoffset_t>(DOF))
            return false;
        return true;
    }

    orc::Time time() const { return state_ ? orc::Time::fromNSec(state_->time_ns()) : orc::Time(); }

    orc::logic::RobotStatus status() const {
        return state_ ? from_fb_robot_status(state_->status()) : orc::logic::RobotStatus::OFF;
    }

    uint8_t model_id() const { return state_ ? state_->model_id() : 0; }

    double q_act(int joint) const { return joint_at_(state_ ? state_->q_act() : nullptr, joint); }
    double q_dot_act(int joint) const {
        return joint_at_(state_ ? state_->q_dot_act() : nullptr, joint);
    }
    double q_dotdot_act(int joint) const {
        return joint_at_(state_ ? state_->q_dotdot_act() : nullptr, joint);
    }
    double tau(int joint) const { return joint_at_(state_ ? state_->tau() : nullptr, joint); }
    double q_set(int joint) const { return joint_at_(state_ ? state_->q_set() : nullptr, joint); }
    double q_dot_set(int joint) const {
        return joint_at_(state_ ? state_->q_dot_set() : nullptr, joint);
    }
    double q_dotdot_set(int joint) const {
        return joint_at_(state_ ? state_->q_dotdot_set() : nullptr, joint);
    }
    double q_d_ns(int joint) const { return joint_at_(state_ ? state_->q_d_ns() : nullptr, joint); }

    double x_set(int component) const {
        return state_ ? detail::pose_at(state_->x_set(), component) : 0.0;
    }
    double x_dot_set(int component) const {
        return state_ ? detail::cartesian_at(state_->x_dot_set(), component) : 0.0;
    }
    double x_dotdot_set(int component) const {
        return state_ ? detail::cartesian_at(state_->x_dotdot_set(), component) : 0.0;
    }

private:
    static double joint_at_(const flatbuffers::Vector<double>* v, int joint) {
        if (!v)
            return 0.0;
        const auto idx = static_cast<flatbuffers::uoffset_t>(joint);
        return idx < v->size() ? v->Get(idx) : 0.0;
    }

    const orc::fb::UdpStateMessage* state_msg_;
    const orc::fb::RobotState* state_;
};

}  // namespace orc::com::fb
