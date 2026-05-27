#pragma once

// FlatBufferSerializer.h
//
// FlatBuffers-based serialization for ORC trajectory and robot-state messages
// over UDP (zero-copy on TwinCAT receive).
//
// Shape: every public `serialize_*` method builds a FlatBufferBuilder, emits
// a single TrajectoryMessage, and returns its byte span. Two helpers
// (`finalize`, `build_point_message`) factor the common tail so each public
// method carries only its per-type payload (which CreateX to call, which
// inputs to pack).
//
// Split variants (`serialize_*_split`) ride the `serialize_in_splits`
// template in FlatBufferSplitting.h: split the input on min(user_cap,
// wire_cap) knots with overlap, and invoke the single-buffer twin per slice.

#include <cstdint>
#include <cstring>
#include <vector>

#include "orc/util/import_flatbuffers.h"

#include "orc_messages_generated.h"

#include "orc/OrcTypes.h"
#include "orc/RobotStatus.h"
#include "orc/RobotTraits.h"
#include "orc/com/flatbuffers/FlatBufferEigen.h"
#include "orc/com/flatbuffers/FlatBufferSplitting.h"
#include "orc/control/controller/cartesian/CartesianCTController.h"
#include "orc/control/controller/joint/JointCTController.h"
#include "orc/trajectory/TrajectoryType.h"

namespace orc::com::fb {

using orc::trajectory::TrajectoryType;

constexpr const char* TRAJECTORY_FILE_IDENTIFIER = "TRJ2";

// ============================================================================
// Enum conversions
// ============================================================================

inline orc::fb::TrajectoryType to_fb_trajectory_type(TrajectoryType id) {
    switch (id) {
        case TrajectoryType::INVALID:
            return orc::fb::TrajectoryType::INVALID;
        case TrajectoryType::STOP:
            return orc::fb::TrajectoryType::STOP;
        case TrajectoryType::JOINTSPACE:
            return orc::fb::TrajectoryType::JOINTSPACE;
        case TrajectoryType::TASKSPACE:
            return orc::fb::TrajectoryType::TASKSPACE;
        case TrajectoryType::NULLSPACE:
            return orc::fb::TrajectoryType::NULLSPACE;
        case TrajectoryType::TOOLPARAM:
            return orc::fb::TrajectoryType::TOOLPARAM;
        case TrajectoryType::JOINT_CTR_PARAM:
            return orc::fb::TrajectoryType::JOINT_CTR_PARAM;
        case TrajectoryType::CART_CTR_PARAM:
            return orc::fb::TrajectoryType::CARTESIAN_CTR_PARAM;
        case TrajectoryType::DENSE_JOINTSPACE:
            return orc::fb::TrajectoryType::DENSE_JOINTSPACE;
        default:
            return orc::fb::TrajectoryType::INVALID;
    }
}

inline TrajectoryType from_fb_trajectory_type(orc::fb::TrajectoryType type) {
    switch (type) {
        case orc::fb::TrajectoryType::INVALID:
            return TrajectoryType::INVALID;
        case orc::fb::TrajectoryType::STOP:
            return TrajectoryType::STOP;
        case orc::fb::TrajectoryType::JOINTSPACE:
            return TrajectoryType::JOINTSPACE;
        case orc::fb::TrajectoryType::TASKSPACE:
            return TrajectoryType::TASKSPACE;
        case orc::fb::TrajectoryType::NULLSPACE:
            return TrajectoryType::NULLSPACE;
        case orc::fb::TrajectoryType::TOOLPARAM:
            return TrajectoryType::TOOLPARAM;
        case orc::fb::TrajectoryType::JOINT_CTR_PARAM:
            return TrajectoryType::JOINT_CTR_PARAM;
        case orc::fb::TrajectoryType::CARTESIAN_CTR_PARAM:
            return TrajectoryType::CART_CTR_PARAM;
        case orc::fb::TrajectoryType::DENSE_JOINTSPACE:
            return TrajectoryType::DENSE_JOINTSPACE;
        default:
            return TrajectoryType::INVALID;
    }
}

inline orc::fb::RobotStatus to_fb_robot_status(orc::logic::RobotStatus status) {
    switch (status) {
        case orc::logic::RobotStatus::OFF:
            return orc::fb::RobotStatus::DISABLED;
        case orc::logic::RobotStatus::ENABLE:
            return orc::fb::RobotStatus::ENABLE;
        case orc::logic::RobotStatus::GRAVCOMP:
            return orc::fb::RobotStatus::ENABLED;
        default:
            return orc::fb::RobotStatus::DISABLED;
    }
}

inline orc::logic::RobotStatus from_fb_robot_status(orc::fb::RobotStatus status) {
    switch (status) {
        case orc::fb::RobotStatus::DISABLED:
        case orc::fb::RobotStatus::DISABLE:
            return orc::logic::RobotStatus::OFF;
        case orc::fb::RobotStatus::ENABLE:
            return orc::logic::RobotStatus::ENABLE;
        case orc::fb::RobotStatus::ENABLED:
            return orc::logic::RobotStatus::GRAVCOMP;
        default:
            return orc::logic::RobotStatus::OFF;
    }
}

// ============================================================================
// Shared tails
// ============================================================================

// Finish the builder and copy its bytes out.
inline std::vector<uint8_t> finalize(flatbuffers::FlatBufferBuilder& b,
                                     flatbuffers::Offset<orc::fb::TrajectoryMessage> msg) {
    b.Finish(msg, TRAJECTORY_FILE_IDENTIFIER);
    return {b.GetBufferPointer(), b.GetBufferPointer() + b.GetSize()};
}

// Build a trajectory-with-points message: allocate the point offsets,
// delegate per-point construction to `build_point` (so each serializer
// controls its own `fb_write_eigen` / struct-inline layout), wrap the
// point vector into a trajectory via `build_traj`, then tag + finalize.
template <typename PointFn, typename TrajFn>
std::vector<uint8_t> build_point_message(size_t n, orc::fb::TrajectoryData tag, PointFn build_point,
                                         TrajFn build_traj) {
    flatbuffers::FlatBufferBuilder b(4096);
    using PointOff = decltype(build_point(b, size_t{0}));
    std::vector<PointOff> pts;
    pts.reserve(n);
    for (size_t i = 0; i < n; ++i)
        pts.push_back(build_point(b, i));
    auto traj = build_traj(b, b.CreateVector(pts));
    return finalize(b, orc::fb::CreateTrajectoryMessage(b, tag, traj.Union()));
}

// ============================================================================
// Serializer
// ============================================================================

template <int DOF>
class FlatBufferSerializer {
    DECLARE_ROBOT_TRAITS_USINGS(DOF)

public:
    FlatBufferSerializer() = default;

    // ---- Trajectories with a vector of points ---------------------------

    std::vector<uint8_t> serialize_joint_trajectory(const std::vector<orc::Time>& time_pts,
                                                    const std::vector<JointVector>& joint_pts) {
        return build_point_message(
            time_pts.size(), orc::fb::TrajectoryData::JointTrajectory,
            [&](flatbuffers::FlatBufferBuilder& b, size_t i) {
                return orc::fb::CreateJointTrajectoryPoint(b, time_pts[i].toNSec(),
                                                           fb_write_eigen(b, joint_pts[i]));
            },
            [](flatbuffers::FlatBufferBuilder& b, auto pts_vec) {
                return orc::fb::CreateJointTrajectory(b, orc::fb::TrajectoryType::JOINTSPACE,
                                                      pts_vec);
            });
    }

    std::vector<uint8_t> serialize_dense_joint_trajectory(
        const std::vector<orc::Time>& time_pts, const std::vector<JointVector>& q_pts,
        const std::vector<JointVector>& q_dot_pts, const std::vector<JointVector>& q_dotdot_pts,
        const std::vector<JointVector>& tau_ff_pts) {
        return build_point_message(
            time_pts.size(), orc::fb::TrajectoryData::DenseJointTrajectory,
            [&](flatbuffers::FlatBufferBuilder& b, size_t i) {
                return orc::fb::CreateDenseJointPoint(
                    b, time_pts[i].toNSec(), fb_write_eigen(b, q_pts[i]),
                    fb_write_eigen(b, q_dot_pts[i]), fb_write_eigen(b, q_dotdot_pts[i]),
                    fb_write_eigen(b, tau_ff_pts[i]));
            },
            [](flatbuffers::FlatBufferBuilder& b, auto pts_vec) {
                return orc::fb::CreateDenseJointTrajectory(
                    b, orc::fb::TrajectoryType::DENSE_JOINTSPACE, pts_vec);
            });
    }

    std::vector<uint8_t> serialize_cartesian_trajectory(const std::vector<orc::Time>& time_pts,
                                                        const std::vector<PoseVector>& pose_pts) {
        return build_point_message(
            time_pts.size(), orc::fb::TrajectoryData::TaskspaceTrajectory,
            [&](flatbuffers::FlatBufferBuilder& b, size_t i) {
                const auto& p = pose_pts[i];
                orc::fb::PoseVector fbp(p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
                return orc::fb::CreateTaskspaceTrajectoryPoint(b, time_pts[i].toNSec(), &fbp);
            },
            [](flatbuffers::FlatBufferBuilder& b, auto pts_vec) {
                return orc::fb::CreateTaskspaceTrajectory(b, orc::fb::TrajectoryType::TASKSPACE,
                                                          pts_vec);
            });
    }

    std::vector<uint8_t> serialize_jointspace_velocity_trajectory(
        const std::vector<orc::Time>& time_pts, const std::vector<JointVector>& velocity_pts) {
        return build_point_message(
            time_pts.size(), orc::fb::TrajectoryData::JointVelocityTrajectory,
            [&](flatbuffers::FlatBufferBuilder& b, size_t i) {
                return orc::fb::CreateJointVelocityPoint(b, time_pts[i].toNSec(),
                                                         fb_write_eigen(b, velocity_pts[i]));
            },
            [](flatbuffers::FlatBufferBuilder& b, auto pts_vec) {
                return orc::fb::CreateJointVelocityTrajectory(
                    b, orc::fb::TrajectoryType::JOINTSPACE_VELOCITY, pts_vec);
            });
    }

    std::vector<uint8_t> serialize_cartesian_velocity_trajectory(
        const std::vector<orc::Time>& time_pts,
        const std::vector<orc::CartesianVector>& velocity_pts) {
        return build_point_message(
            time_pts.size(), orc::fb::TrajectoryData::CartesianVelocityTrajectory,
            [&](flatbuffers::FlatBufferBuilder& b, size_t i) {
                const auto& v = velocity_pts[i];
                orc::fb::CartesianVector fbv(v[0], v[1], v[2], v[3], v[4], v[5]);
                return orc::fb::CreateCartesianVelocityPoint(b, time_pts[i].toNSec(), &fbv);
            },
            [](flatbuffers::FlatBufferBuilder& b, auto pts_vec) {
                return orc::fb::CreateCartesianVelocityTrajectory(
                    b, orc::fb::TrajectoryType::CARTESIAN_VELOCITY, pts_vec);
            });
    }

    // Hybrid force/motion: each point carries pose + scalar force on the
    // end-effector z-axis (see HybridForceMotionController).
    std::vector<uint8_t> serialize_hybrid_force_motion_trajectory(
        const std::vector<orc::Time>& time_pts, const std::vector<orc::PoseVector>& pose_pts,
        const std::vector<double>& force_pts) {
        return build_point_message(
            time_pts.size(), orc::fb::TrajectoryData::HybridForceMotionTrajectory,
            [&](flatbuffers::FlatBufferBuilder& b, size_t i) {
                const auto& p = pose_pts[i];
                orc::fb::PoseVector fbp(p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
                return orc::fb::CreateHybridForceMotionPoint(b, time_pts[i].toNSec(), &fbp,
                                                             force_pts[i]);
            },
            [](flatbuffers::FlatBufferBuilder& b, auto pts_vec) {
                return orc::fb::CreateHybridForceMotionTrajectory(
                    b, orc::fb::TrajectoryType::HYBRID_FORCE_MOTION, pts_vec);
            });
    }

    // ---- One-shot trajectories (no per-point vector) ---------------------

    std::vector<uint8_t> serialize_nullspace_trajectory(orc::Time time,
                                                        const JointVector& q_nullspace) {
        flatbuffers::FlatBufferBuilder b(256);
        auto traj = orc::fb::CreateNullspaceTrajectory(
            b, orc::fb::TrajectoryType::NULLSPACE, time.toNSec(), fb_write_eigen(b, q_nullspace));
        return finalize(b, orc::fb::CreateTrajectoryMessage(
                               b, orc::fb::TrajectoryData::NullspaceTrajectory, traj.Union()));
    }

    // Only the diagonals of the JointMatrix gains survive the wire format
    // (schema carries per-joint kp/kd/ki vectors); off-diagonals are
    // discarded here and the receiver rebuilds diagonal matrices.
    std::vector<uint8_t> serialize_jointctrparam_trajectory(
        orc::Time time, const orc::control::JointCTParameter<DOF>& param) {
        flatbuffers::FlatBufferBuilder b(256);
        const JointVector kp = param.K0.diagonal();
        const JointVector kd = param.K1.diagonal();
        const JointVector ki = param.KI.diagonal();
        auto fb_param = orc::fb::CreateJointCTParameter(
            b, fb_write_eigen(b, kp), fb_write_eigen(b, kd), fb_write_eigen(b, ki));
        auto traj = orc::fb::CreateJointCtrParamTrajectory(
            b, orc::fb::TrajectoryType::JOINT_CTR_PARAM, time.toNSec(), fb_param);
        return finalize(b, orc::fb::CreateTrajectoryMessage(
                               b, orc::fb::TrajectoryData::JointCtrParamTrajectory, traj.Union()));
    }

    // Nullspace gains param.K0N / K1N are not carried by the schema and
    // default to zero on the receive side.
    std::vector<uint8_t> serialize_cartesianctrparam_trajectory(
        orc::Time time, const orc::control::CartesianCTParameter<DOF>& param) {
        flatbuffers::FlatBufferBuilder b(256);
        const orc::CartesianVector kp = param.K0.diagonal();
        const orc::CartesianVector kd = param.K1.diagonal();
        auto fb_param = orc::fb::CreateCartesianCTParameter(
            b, kp[0], kp[1], kp[2], kd[0], kd[1], kd[2], kp[3], kp[4], kp[5], kd[3], kd[4], kd[5]);
        auto traj = orc::fb::CreateCartesianCtrParamTrajectory(
            b, orc::fb::TrajectoryType::CARTESIAN_CTR_PARAM, time.toNSec(), fb_param);
        return finalize(b,
                        orc::fb::CreateTrajectoryMessage(
                            b, orc::fb::TrajectoryData::CartesianCtrParamTrajectory, traj.Union()));
    }

    std::vector<uint8_t> serialize_stop() {
        flatbuffers::FlatBufferBuilder b(64);
        auto traj = orc::fb::CreateStopTrajectory(b, orc::fb::TrajectoryType::STOP);
        return finalize(b, orc::fb::CreateTrajectoryMessage(
                               b, orc::fb::TrajectoryData::StopTrajectory, traj.Union()));
    }

    // ---- Split variants -------------------------------------------------
    //
    // Split long trajectories into overlap-split FlatBuffer payloads
    // (one UDP datagram each) so they fit the 1472 B UDP limit and bound
    // the receiver spline-init cycle cost. user_cap = 0 means "no user
    // limit" -- the wire cap alone governs; the TrajectoryQueue stitches
    // splits via the existing save_state -> init(saved_state) hand-off.
    // See FlatBufferSplitting.h.

    std::vector<std::vector<uint8_t>> serialize_joint_trajectory_split(
        const std::vector<orc::Time>& time_pts, const std::vector<JointVector>& joint_pts,
        size_t max_pts_per_split = 0) {
        return serialize_in_splits(time_pts.size(), bytes_per_joint_point<DOF>(), max_pts_per_split,
                                   [&](size_t a, size_t b) {
                                       return serialize_joint_trajectory(
                                           {time_pts.begin() + a, time_pts.begin() + b},
                                           {joint_pts.begin() + a, joint_pts.begin() + b});
                                   });
    }

    std::vector<std::vector<uint8_t>> serialize_dense_joint_trajectory_split(
        const std::vector<orc::Time>& time_pts, const std::vector<JointVector>& q_pts,
        const std::vector<JointVector>& q_dot_pts, const std::vector<JointVector>& q_dotdot_pts,
        const std::vector<JointVector>& tau_ff_pts, size_t max_pts_per_split = 0) {
        return serialize_in_splits(time_pts.size(), bytes_per_dense_point<DOF>(), max_pts_per_split,
                                   [&](size_t a, size_t b) {
                                       return serialize_dense_joint_trajectory(
                                           {time_pts.begin() + a, time_pts.begin() + b},
                                           {q_pts.begin() + a, q_pts.begin() + b},
                                           {q_dot_pts.begin() + a, q_dot_pts.begin() + b},
                                           {q_dotdot_pts.begin() + a, q_dotdot_pts.begin() + b},
                                           {tau_ff_pts.begin() + a, tau_ff_pts.begin() + b});
                                   });
    }

    std::vector<std::vector<uint8_t>> serialize_cartesian_trajectory_split(
        const std::vector<orc::Time>& time_pts, const std::vector<PoseVector>& pose_pts,
        size_t max_pts_per_split = 0) {
        return serialize_in_splits(
            time_pts.size(), bytes_per_pose_point(), max_pts_per_split, [&](size_t a, size_t b) {
                return serialize_cartesian_trajectory({time_pts.begin() + a, time_pts.begin() + b},
                                                      {pose_pts.begin() + a, pose_pts.begin() + b});
            });
    }

    std::vector<std::vector<uint8_t>> serialize_jointspace_velocity_trajectory_split(
        const std::vector<orc::Time>& time_pts, const std::vector<JointVector>& velocity_pts,
        size_t max_pts_per_split = 0) {
        return serialize_in_splits(time_pts.size(), bytes_per_velocity_point<DOF>(),
                                   max_pts_per_split, [&](size_t a, size_t b) {
                                       return serialize_jointspace_velocity_trajectory(
                                           {time_pts.begin() + a, time_pts.begin() + b},
                                           {velocity_pts.begin() + a, velocity_pts.begin() + b});
                                   });
    }

    std::vector<std::vector<uint8_t>> serialize_cartesian_velocity_trajectory_split(
        const std::vector<orc::Time>& time_pts,
        const std::vector<orc::CartesianVector>& velocity_pts, size_t max_pts_per_split = 0) {
        return serialize_in_splits(time_pts.size(), bytes_per_cartesian_velocity_point(),
                                   max_pts_per_split, [&](size_t a, size_t b) {
                                       return serialize_cartesian_velocity_trajectory(
                                           {time_pts.begin() + a, time_pts.begin() + b},
                                           {velocity_pts.begin() + a, velocity_pts.begin() + b});
                                   });
    }

    std::vector<std::vector<uint8_t>> serialize_hybrid_force_motion_trajectory_split(
        const std::vector<orc::Time>& time_pts, const std::vector<orc::PoseVector>& pose_pts,
        const std::vector<double>& force_pts, size_t max_pts_per_split = 0) {
        return serialize_in_splits(time_pts.size(), bytes_per_hybrid_force_motion_point(),
                                   max_pts_per_split, [&](size_t a, size_t b) {
                                       return serialize_hybrid_force_motion_trajectory(
                                           {time_pts.begin() + a, time_pts.begin() + b},
                                           {pose_pts.begin() + a, pose_pts.begin() + b},
                                           {force_pts.begin() + a, force_pts.begin() + b});
                                   });
    }
};

}  // namespace orc::com::fb
