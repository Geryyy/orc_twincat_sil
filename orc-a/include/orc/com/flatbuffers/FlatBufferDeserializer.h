#pragma once

// FlatBufferDeserializer.h
//
// Zero-copy FlatBuffers deserialization for ORC trajectory messages over
// UDP. Three zero-copy reader classes (joint / dense-joint / taskspace) for
// the RT (TwinCAT) side, plus a heap-allocating deserialize() that the host
// side uses to build TrajectoryBase<DOF> objects for the TrajectoryQueue.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "orc/util/import_flatbuffers.h"

#include "orc_messages_generated.h"

#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/com/flatbuffers/FlatBufferEigen.h"
#include "orc/control/controller/cartesian/CartesianCTController.h"
#include "orc/control/controller/joint/JointCTController.h"
#include "orc/trajectory/CartesianVelocityTrajectory.h"
#include "orc/trajectory/DenseJointspaceTrajectory.h"
#include "orc/trajectory/HybridForceMotionTrajectory.h"
#include "orc/trajectory/JointspaceTrajectory.h"
#include "orc/trajectory/JointspaceVelocityTrajectory.h"
#include "orc/trajectory/TaskspaceTrajectory.h"
#include "orc/trajectory/TrajectoryBase.h"
#include "orc/trajectory/TrajectoryType.h"
#include "orc/trajectory/singleevent/CartesianCtrParamTrajectory.h"
#include "orc/trajectory/singleevent/JointCtrParamTrajectory.h"
#include "orc/trajectory/singleevent/NullspaceTrajectory.h"

namespace orc::com::fb {

using orc::trajectory::TrajectoryType;

struct TrajectoryTypeResult {
    TrajectoryType type;
    bool valid;
};

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

inline double cartesian_at(const orc::fb::CartesianVector* c, int i) {
    if (!c)
        return 0.0;
    switch (i) {
        case 0:
            return c->vx();
        case 1:
            return c->vy();
        case 2:
            return c->vz();
        case 3:
            return c->wx();
        case 4:
            return c->wy();
        case 5:
            return c->wz();
        default:
            return 0.0;
    }
}

}  // namespace detail

// ============================================================================
// Zero-copy readers (RT side)
// ============================================================================

template <int DOF>
class JointTrajectoryReader {
public:
    JointTrajectoryReader() : points_(nullptr) {}

    bool init(const void* buffer, size_t size) {
        points_ = nullptr;
        if (!buffer || size == 0)
            return false;

        flatbuffers::Verifier verifier(static_cast<const uint8_t*>(buffer), size);
        if (!orc::fb::VerifyTrajectoryMessageBuffer(verifier))
            return false;

        auto msg = orc::fb::GetTrajectoryMessage(buffer);
        if (!msg || msg->data_type() != orc::fb::TrajectoryData::JointTrajectory)
            return false;

        auto traj = msg->data_as_JointTrajectory();
        if (!traj || !traj->points())
            return false;

        if (traj->points()->size() > 0) {
            auto p0 = traj->points()->Get(0);
            if (!p0 || !p0->position() ||
                p0->position()->size() != static_cast<flatbuffers::uoffset_t>(DOF))
                return false;
        }

        points_ = traj->points();
        return true;
    }

    size_t num_points() const { return points_ ? points_->size() : 0; }

    double time(size_t idx) const {
        return points_->Get(static_cast<flatbuffers::uoffset_t>(idx))->time_ns() / 1.0e9;
    }

    double position(size_t idx, int joint) const {
        auto pos = points_->Get(static_cast<flatbuffers::uoffset_t>(idx))->position();
        return pos ? pos->Get(static_cast<flatbuffers::uoffset_t>(joint)) : 0.0;
    }

private:
    const flatbuffers::Vector<flatbuffers::Offset<orc::fb::JointTrajectoryPoint>>* points_;
};

template <int DOF>
class DenseJointTrajectoryReader {
public:
    DenseJointTrajectoryReader() : points_(nullptr) {}

    bool init(const void* buffer, size_t size) {
        points_ = nullptr;
        if (!buffer || size == 0)
            return false;

        flatbuffers::Verifier verifier(static_cast<const uint8_t*>(buffer), size);
        if (!orc::fb::VerifyTrajectoryMessageBuffer(verifier))
            return false;

        auto msg = orc::fb::GetTrajectoryMessage(buffer);
        if (!msg || msg->data_type() != orc::fb::TrajectoryData::DenseJointTrajectory)
            return false;

        auto traj = msg->data_as_DenseJointTrajectory();
        if (!traj || !traj->points())
            return false;

        if (traj->points()->size() > 0) {
            auto p0 = traj->points()->Get(0);
            if (!p0 || !p0->q() || p0->q()->size() != static_cast<flatbuffers::uoffset_t>(DOF))
                return false;
        }

        points_ = traj->points();
        return true;
    }

    size_t num_points() const { return points_ ? points_->size() : 0; }

    double time(size_t idx) const {
        return points_->Get(static_cast<flatbuffers::uoffset_t>(idx))->time_ns() / 1.0e9;
    }

    double q(size_t idx, int joint) const {
        return field_(idx, joint, &orc::fb::DenseJointPoint::q);
    }
    double q_dot(size_t idx, int joint) const {
        return field_(idx, joint, &orc::fb::DenseJointPoint::q_dot);
    }
    double q_dotdot(size_t idx, int joint) const {
        return field_(idx, joint, &orc::fb::DenseJointPoint::q_dotdot);
    }
    double tau_ff(size_t idx, int joint) const {
        return field_(idx, joint, &orc::fb::DenseJointPoint::tau_ff);
    }

private:
    template <typename Getter>
    double field_(size_t idx, int joint, Getter getter) const {
        auto pt = points_->Get(static_cast<flatbuffers::uoffset_t>(idx));
        auto vec = (pt->*getter)();
        return vec ? vec->Get(static_cast<flatbuffers::uoffset_t>(joint)) : 0.0;
    }

    const flatbuffers::Vector<flatbuffers::Offset<orc::fb::DenseJointPoint>>* points_;
};

class TaskspaceTrajectoryReader {
public:
    TaskspaceTrajectoryReader() : points_(nullptr) {}

    bool init(const void* buffer, size_t size) {
        points_ = nullptr;
        if (!buffer || size == 0)
            return false;

        flatbuffers::Verifier verifier(static_cast<const uint8_t*>(buffer), size);
        if (!orc::fb::VerifyTrajectoryMessageBuffer(verifier))
            return false;

        auto msg = orc::fb::GetTrajectoryMessage(buffer);
        if (!msg || msg->data_type() != orc::fb::TrajectoryData::TaskspaceTrajectory)
            return false;

        auto traj = msg->data_as_TaskspaceTrajectory();
        if (!traj || !traj->points())
            return false;

        points_ = traj->points();
        return true;
    }

    size_t num_points() const { return points_ ? points_->size() : 0; }

    double time(size_t idx) const {
        return points_->Get(static_cast<flatbuffers::uoffset_t>(idx))->time_ns() / 1.0e9;
    }

    double pose(size_t idx, int component) const {
        return detail::pose_at(points_->Get(static_cast<flatbuffers::uoffset_t>(idx))->pose(),
                               component);
    }

private:
    const flatbuffers::Vector<flatbuffers::Offset<orc::fb::TaskspaceTrajectoryPoint>>* points_;
};

// ============================================================================
// Heap deserializer (host side)
// ============================================================================

namespace detail {

// Map FlatBuffer union tag -> ORC TrajectoryType.
inline TrajectoryType tag_to_type(orc::fb::TrajectoryData tag) {
    switch (tag) {
        case orc::fb::TrajectoryData::JointTrajectory:
            return TrajectoryType::JOINTSPACE;
        case orc::fb::TrajectoryData::DenseJointTrajectory:
            return TrajectoryType::DENSE_JOINTSPACE;
        case orc::fb::TrajectoryData::TaskspaceTrajectory:
            return TrajectoryType::TASKSPACE;
        case orc::fb::TrajectoryData::NullspaceTrajectory:
            return TrajectoryType::NULLSPACE;
        case orc::fb::TrajectoryData::JointCtrParamTrajectory:
            return TrajectoryType::JOINT_CTR_PARAM;
        case orc::fb::TrajectoryData::CartesianCtrParamTrajectory:
            return TrajectoryType::CART_CTR_PARAM;
        case orc::fb::TrajectoryData::StopTrajectory:
            return TrajectoryType::STOP;
        case orc::fb::TrajectoryData::CartesianVelocityTrajectory:
            return TrajectoryType::CARTESIAN_VELOCITY;
        case orc::fb::TrajectoryData::JointVelocityTrajectory:
            return TrajectoryType::JOINTSPACE_VELOCITY;
        case orc::fb::TrajectoryData::HybridForceMotionTrajectory:
            return TrajectoryType::HYBRID_FORCE_MOTION;
        default:
            return TrajectoryType::INVALID;
    }
}

}  // namespace detail

template <int DOF>
class FlatBufferDeserializer {
public:
    using JointVector = typename orc::RobotTraits<DOF>::JointVector;

    FlatBufferDeserializer() = default;

    TrajectoryTypeResult get_trajectory_type(const void* buffer, size_t size) const {
        if (!verify_buffer(buffer, size))
            return {TrajectoryType::INVALID, false};
        auto msg = orc::fb::GetTrajectoryMessage(buffer);
        return {msg ? detail::tag_to_type(msg->data_type()) : TrajectoryType::INVALID, true};
    }

    JointTrajectoryReader<DOF> get_joint_trajectory_reader(const void* buffer, size_t size) const {
        JointTrajectoryReader<DOF> r;
        r.init(buffer, size);
        return r;
    }
    DenseJointTrajectoryReader<DOF> get_dense_joint_trajectory_reader(const void* buffer,
                                                                      size_t size) const {
        DenseJointTrajectoryReader<DOF> r;
        r.init(buffer, size);
        return r;
    }
    TaskspaceTrajectoryReader get_taskspace_trajectory_reader(const void* buffer,
                                                              size_t size) const {
        TaskspaceTrajectoryReader r;
        r.init(buffer, size);
        return r;
    }

    bool verify_buffer(const void* buffer, size_t size) const {
        if (!buffer || size == 0)
            return false;
        flatbuffers::Verifier v(static_cast<const uint8_t*>(buffer), size);
        return orc::fb::VerifyTrajectoryMessageBuffer(v);
    }

    // Deserialize a single buffer into a TrajectoryBase<DOF> ready to
    // enqueue. Returns nullptr on verifier failure, DoF mismatch, too few
    // points, or STOP (the caller clears the queue for STOP).
    std::unique_ptr<orc::trajectory::TrajectoryBase<DOF>> deserialize(const uint8_t* buffer,
                                                                      size_t size) const {
        if (!verify_buffer(buffer, size))
            return nullptr;
        auto msg = orc::fb::GetTrajectoryMessage(buffer);
        if (!msg)
            return nullptr;

        switch (msg->data_type()) {
            case orc::fb::TrajectoryData::JointTrajectory: {
                auto traj = msg->data_as_JointTrajectory();
                if (!traj || !traj->points())
                    return nullptr;
                const size_t n = traj->points()->size();
                if (n < 2)
                    return nullptr;

                std::vector<orc::Time> times;
                times.reserve(n);
                std::vector<JointVector> q_pts;
                q_pts.reserve(n);
                for (size_t i = 0; i < n; ++i) {
                    auto pt = traj->points()->Get(static_cast<flatbuffers::uoffset_t>(i));
                    JointVector q;
                    if (!fb_read_eigen<DOF>(pt->position(), q))
                        return nullptr;
                    times.push_back(orc::Time::fromNSec(pt->time_ns()));
                    q_pts.push_back(std::move(q));
                }
                return std::make_unique<orc::trajectory::JointspaceTrajectory<DOF>>(q_pts, times);
            }

            case orc::fb::TrajectoryData::DenseJointTrajectory: {
                auto traj = msg->data_as_DenseJointTrajectory();
                if (!traj || !traj->points())
                    return nullptr;
                const size_t n = traj->points()->size();
                if (n < 1)
                    return nullptr;

                std::vector<orc::Time> times;
                times.reserve(n);
                std::vector<JointVector> q, qd, qdd, tau;
                q.reserve(n);
                qd.reserve(n);
                qdd.reserve(n);
                tau.reserve(n);
                for (size_t i = 0; i < n; ++i) {
                    auto pt = traj->points()->Get(static_cast<flatbuffers::uoffset_t>(i));
                    JointVector a, b, c, d;
                    if (!fb_read_eigen<DOF>(pt->q(), a))
                        return nullptr;
                    if (!fb_read_eigen<DOF>(pt->q_dot(), b))
                        return nullptr;
                    if (!fb_read_eigen<DOF>(pt->q_dotdot(), c))
                        return nullptr;
                    if (!fb_read_eigen<DOF>(pt->tau_ff(), d))
                        return nullptr;
                    times.push_back(orc::Time::fromNSec(pt->time_ns()));
                    q.push_back(std::move(a));
                    qd.push_back(std::move(b));
                    qdd.push_back(std::move(c));
                    tau.push_back(std::move(d));
                }
                return std::make_unique<orc::trajectory::DenseJointspaceTrajectory<DOF>>(
                    times, q, qd, qdd, tau);
            }

            case orc::fb::TrajectoryData::TaskspaceTrajectory: {
                auto traj = msg->data_as_TaskspaceTrajectory();
                if (!traj || !traj->points())
                    return nullptr;
                const size_t n = traj->points()->size();
                if (n < 2)
                    return nullptr;

                std::vector<orc::Time> times;
                times.reserve(n);
                std::vector<orc::PoseVector> poses;
                poses.reserve(n);
                for (size_t i = 0; i < n; ++i) {
                    auto pt = traj->points()->Get(static_cast<flatbuffers::uoffset_t>(i));
                    if (!pt || !pt->pose())
                        return nullptr;
                    orc::PoseVector p;
                    for (int j = 0; j < 7; ++j)
                        p[j] = detail::pose_at(pt->pose(), j);
                    times.push_back(orc::Time::fromNSec(pt->time_ns()));
                    poses.push_back(std::move(p));
                }
                return std::make_unique<orc::trajectory::TaskspaceTrajectory<DOF>>(poses, times);
            }

            case orc::fb::TrajectoryData::NullspaceTrajectory: {
                auto traj = msg->data_as_NullspaceTrajectory();
                if (!traj)
                    return nullptr;
                JointVector q_ns;
                if (!fb_read_eigen<DOF>(traj->q_nullspace(), q_ns))
                    return nullptr;
                return std::make_unique<orc::trajectory::NullspaceTrajectory<DOF>>(
                    orc::Time::fromNSec(traj->time_ns()), q_ns);
            }

            case orc::fb::TrajectoryData::JointCtrParamTrajectory: {
                using JointMatrix = typename orc::RobotTraits<DOF>::JointMatrix;
                auto traj = msg->data_as_JointCtrParamTrajectory();
                if (!traj || !traj->param())
                    return nullptr;

                JointVector kp, kd, ki;
                auto p = traj->param();
                if (!fb_read_eigen<DOF>(p->kp(), kp))
                    return nullptr;
                if (!fb_read_eigen<DOF>(p->kd(), kd))
                    return nullptr;
                if (!fb_read_eigen<DOF>(p->ki(), ki))
                    return nullptr;

                JointMatrix K0 = JointMatrix::Zero(), K1 = JointMatrix::Zero(),
                            KI = JointMatrix::Zero();
                K0.diagonal() = kp;
                K1.diagonal() = kd;
                KI.diagonal() = ki;
                return std::make_unique<orc::trajectory::JointCtrParamTrajectory<DOF>>(
                    orc::Time::fromNSec(traj->time_ns()),
                    orc::control::JointCTParameter<DOF>(K0, K1, KI));
            }

            case orc::fb::TrajectoryData::CartesianCtrParamTrajectory: {
                using JointMatrix = typename orc::RobotTraits<DOF>::JointMatrix;
                auto traj = msg->data_as_CartesianCtrParamTrajectory();
                if (!traj || !traj->param())
                    return nullptr;

                auto p = traj->param();
                orc::CartesianVector kp, kd;
                kp << p->kp_trans_x(), p->kp_trans_y(), p->kp_trans_z(), p->kp_rot_x(),
                    p->kp_rot_y(), p->kp_rot_z();
                kd << p->kd_trans_x(), p->kd_trans_y(), p->kd_trans_z(), p->kd_rot_x(),
                    p->kd_rot_y(), p->kd_rot_z();

                orc::CartesianMatrix K0 = orc::CartesianMatrix::Zero();
                orc::CartesianMatrix K1 = orc::CartesianMatrix::Zero();
                K0.diagonal() = kp;
                K1.diagonal() = kd;
                return std::make_unique<orc::trajectory::CartesianCtrParamTrajectory<DOF>>(
                    orc::Time::fromNSec(traj->time_ns()),
                    orc::control::CartesianCTParameter<DOF>(K0, K1, JointMatrix::Zero(),
                                                            JointMatrix::Zero()));
            }

            case orc::fb::TrajectoryData::StopTrajectory:
                // STOP is handled by the caller (clears the queue); no trajectory object.
                return nullptr;

            case orc::fb::TrajectoryData::CartesianVelocityTrajectory: {
                auto traj = msg->data_as_CartesianVelocityTrajectory();
                if (!traj || !traj->points())
                    return nullptr;
                const size_t n = traj->points()->size();
                if (n < 2)
                    return nullptr;

                std::vector<orc::Time> times;
                times.reserve(n);
                std::vector<orc::CartesianVector> velocities;
                velocities.reserve(n);
                for (size_t i = 0; i < n; ++i) {
                    auto pt = traj->points()->Get(static_cast<flatbuffers::uoffset_t>(i));
                    if (!pt || !pt->velocity())
                        return nullptr;
                    orc::CartesianVector v;
                    v << pt->velocity()->vx(), pt->velocity()->vy(), pt->velocity()->vz(),
                        pt->velocity()->wx(), pt->velocity()->wy(), pt->velocity()->wz();
                    times.push_back(orc::Time::fromNSec(pt->time_ns()));
                    velocities.push_back(std::move(v));
                }
                return std::make_unique<orc::trajectory::CartesianVelocityTrajectory<DOF>>(
                    velocities, times);
            }

            case orc::fb::TrajectoryData::JointVelocityTrajectory: {
                auto traj = msg->data_as_JointVelocityTrajectory();
                if (!traj || !traj->points())
                    return nullptr;
                const size_t n = traj->points()->size();
                if (n < 2)
                    return nullptr;

                std::vector<orc::Time> times;
                times.reserve(n);
                std::vector<JointVector> velocities;
                velocities.reserve(n);
                for (size_t i = 0; i < n; ++i) {
                    auto pt = traj->points()->Get(static_cast<flatbuffers::uoffset_t>(i));
                    JointVector v;
                    if (!fb_read_eigen<DOF>(pt->velocity(), v))
                        return nullptr;
                    times.push_back(orc::Time::fromNSec(pt->time_ns()));
                    velocities.push_back(std::move(v));
                }
                return std::make_unique<orc::trajectory::JointspaceVelocityTrajectory<DOF>>(
                    velocities, times);
            }

            case orc::fb::TrajectoryData::HybridForceMotionTrajectory: {
                auto traj = msg->data_as_HybridForceMotionTrajectory();
                if (!traj || !traj->points())
                    return nullptr;
                const size_t n = traj->points()->size();
                if (n < 2)
                    return nullptr;

                std::vector<orc::Time> times;
                times.reserve(n);
                std::vector<orc::PoseVector> poses;
                poses.reserve(n);
                std::vector<double> forces;
                forces.reserve(n);
                for (size_t i = 0; i < n; ++i) {
                    auto pt = traj->points()->Get(static_cast<flatbuffers::uoffset_t>(i));
                    if (!pt || !pt->pose())
                        return nullptr;
                    orc::PoseVector p;
                    for (int j = 0; j < 7; ++j)
                        p[j] = detail::pose_at(pt->pose(), j);
                    times.push_back(orc::Time::fromNSec(pt->time_ns()));
                    poses.push_back(std::move(p));
                    forces.push_back(pt->force());
                }
                return std::make_unique<orc::trajectory::HybridForceMotionTrajectory<DOF>>(
                    poses, forces, times);
            }

            default:
                return nullptr;
        }
    }
};

}  // namespace orc::com::fb
