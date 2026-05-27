// Tests for FlatBuffer deserialization paths added after the protobuf→FB
// migration: JointCtrParamTrajectory, CartesianCtrParamTrajectory, and STOP.
//
// These paths previously fell through the default branch of
// FlatBufferDeserializer::deserialize() and silently dropped messages. The
// tests below construct FlatBuffer payloads directly (no serializer helper
// exists for control-parameter trajectories yet) and verify the deserializer
// builds the expected trajectory objects.

#include <gtest/gtest.h>

#include "orc/util/import_flatbuffers.h"

#include "orc/com/flatbuffers/FlatBufferDeserializer.h"
#include "orc/com/flatbuffers/FlatBufferSerializer.h"
#include "orc/com/flatbuffers/orc_messages_generated.h"
#include "orc/robots/Robot.h"
#include "orc/trajectory/TrajectoryType.h"
#include "orc/trajectory/singleevent/CartesianCtrParamTrajectory.h"
#include "orc/trajectory/singleevent/JointCtrParamTrajectory.h"
#include "orc/util/Time.h"

namespace {

constexpr int DoF = 7;

using Deserializer = orc::com::fb::FlatBufferDeserializer<DoF>;
using Serializer = orc::com::fb::FlatBufferSerializer<DoF>;
using JointCtrParamTrajectory = orc::trajectory::JointCtrParamTrajectory<DoF>;
using CartesianCtrParamTrajectory = orc::trajectory::CartesianCtrParamTrajectory<DoF>;
using TrajectoryType = orc::trajectory::TrajectoryType;
using Time = orc::Time;

// Build a JointCtrParamTrajectory FlatBuffer payload with the given per-joint
// diagonal gain values. Returns the serialized buffer.
std::vector<uint8_t> build_joint_ctr_param_buffer(int64_t t_ns, const std::array<double, DoF>& kp,
                                                  const std::array<double, DoF>& kd,
                                                  const std::array<double, DoF>& ki) {
    flatbuffers::FlatBufferBuilder builder(256);

    auto fb_kp = builder.CreateVector(kp.data(), DoF);
    auto fb_kd = builder.CreateVector(kd.data(), DoF);
    auto fb_ki = builder.CreateVector(ki.data(), DoF);

    auto param = orc::fb::CreateJointCTParameter(builder, fb_kp, fb_kd, fb_ki);
    auto traj = orc::fb::CreateJointCtrParamTrajectory(
        builder, orc::fb::TrajectoryType::JOINT_CTR_PARAM, t_ns, param);
    auto msg = orc::fb::CreateTrajectoryMessage(
        builder, orc::fb::TrajectoryData::JointCtrParamTrajectory, traj.Union());
    builder.Finish(msg, "TRJ2");

    return std::vector<uint8_t>(builder.GetBufferPointer(),
                                builder.GetBufferPointer() + builder.GetSize());
}

std::vector<uint8_t> build_cart_ctr_param_buffer(int64_t t_ns, const std::array<double, 6>& kp,
                                                 const std::array<double, 6>& kd) {
    flatbuffers::FlatBufferBuilder builder(256);

    auto param =
        orc::fb::CreateCartesianCTParameter(builder, kp[0], kp[1], kp[2], kd[0], kd[1], kd[2],
                                            kp[3], kp[4], kp[5], kd[3], kd[4], kd[5]);
    auto traj = orc::fb::CreateCartesianCtrParamTrajectory(
        builder, orc::fb::TrajectoryType::CARTESIAN_CTR_PARAM, t_ns, param);
    auto msg = orc::fb::CreateTrajectoryMessage(
        builder, orc::fb::TrajectoryData::CartesianCtrParamTrajectory, traj.Union());
    builder.Finish(msg, "TRJ2");

    return std::vector<uint8_t>(builder.GetBufferPointer(),
                                builder.GetBufferPointer() + builder.GetSize());
}

std::vector<uint8_t> build_stop_buffer() {
    flatbuffers::FlatBufferBuilder builder(64);

    auto traj = orc::fb::CreateStopTrajectory(builder, orc::fb::TrajectoryType::STOP);
    auto msg = orc::fb::CreateTrajectoryMessage(builder, orc::fb::TrajectoryData::StopTrajectory,
                                                traj.Union());
    builder.Finish(msg, "TRJ2");

    return std::vector<uint8_t>(builder.GetBufferPointer(),
                                builder.GetBufferPointer() + builder.GetSize());
}

}  // namespace

// -----------------------------------------------------------------------------
// JOINT_CTR_PARAM
// -----------------------------------------------------------------------------

TEST(FlatBufferCtrParamStop, JointCtrParam_TypeDetected) {
    Deserializer deser;
    auto buf = build_joint_ctr_param_buffer(
        /*t_ns=*/1'000'000'000, {1, 2, 3, 4, 5, 6, 7}, {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7},
        {10, 20, 30, 40, 50, 60, 70});

    auto result = deser.get_trajectory_type(buf.data(), buf.size());
    ASSERT_TRUE(result.valid);
    EXPECT_EQ(result.type, TrajectoryType::JOINT_CTR_PARAM);
}

TEST(FlatBufferCtrParamStop, JointCtrParam_Deserialized) {
    Deserializer deser;

    const std::array<double, DoF> kp = {1, 2, 3, 4, 5, 6, 7};
    const std::array<double, DoF> kd = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
    const std::array<double, DoF> ki = {10, 20, 30, 40, 50, 60, 70};
    const int64_t t_ns = 2'500'000'000;  // 2.5 s

    auto buf = build_joint_ctr_param_buffer(t_ns, kp, kd, ki);

    auto traj_ptr = deser.deserialize(buf.data(), buf.size());
    ASSERT_NE(traj_ptr, nullptr);
    ASSERT_EQ(traj_ptr->get_trajectory_type(), TrajectoryType::JOINT_CTR_PARAM);

    auto* jcp = static_cast<JointCtrParamTrajectory*>(traj_ptr.get());
    EXPECT_DOUBLE_EQ(jcp->get_start_time().toSec(), 2.5);

    auto param = jcp->get_parameter();
    for (int i = 0; i < DoF; ++i) {
        EXPECT_DOUBLE_EQ(param.K0(i, i), kp[i]) << "K0 diag " << i;
        EXPECT_DOUBLE_EQ(param.K1(i, i), kd[i]) << "K1 diag " << i;
        EXPECT_DOUBLE_EQ(param.KI(i, i), ki[i]) << "KI diag " << i;
        // off-diagonals must be zero
        for (int j = 0; j < DoF; ++j) {
            if (i == j)
                continue;
            EXPECT_DOUBLE_EQ(param.K0(i, j), 0.0);
            EXPECT_DOUBLE_EQ(param.K1(i, j), 0.0);
            EXPECT_DOUBLE_EQ(param.KI(i, j), 0.0);
        }
    }
}

// -----------------------------------------------------------------------------
// CART_CTR_PARAM
// -----------------------------------------------------------------------------

TEST(FlatBufferCtrParamStop, CartCtrParam_TypeDetected) {
    Deserializer deser;
    auto buf = build_cart_ctr_param_buffer(
        /*t_ns=*/500'000'000, {10, 20, 30, 40, 50, 60}, {1, 2, 3, 4, 5, 6});

    auto result = deser.get_trajectory_type(buf.data(), buf.size());
    ASSERT_TRUE(result.valid);
    EXPECT_EQ(result.type, TrajectoryType::CART_CTR_PARAM);
}

TEST(FlatBufferCtrParamStop, CartCtrParam_Deserialized) {
    Deserializer deser;

    // kp/kd order on the wire: trans_x, trans_y, trans_z, rot_x, rot_y, rot_z
    const std::array<double, 6> kp = {100, 200, 300, 11, 22, 33};
    const std::array<double, 6> kd = {1.0, 2.0, 3.0, 0.11, 0.22, 0.33};
    const int64_t t_ns = 1'250'000'000;

    auto buf = build_cart_ctr_param_buffer(t_ns, kp, kd);

    auto traj_ptr = deser.deserialize(buf.data(), buf.size());
    ASSERT_NE(traj_ptr, nullptr);
    ASSERT_EQ(traj_ptr->get_trajectory_type(), TrajectoryType::CART_CTR_PARAM);

    auto* ccp = static_cast<CartesianCtrParamTrajectory*>(traj_ptr.get());
    EXPECT_DOUBLE_EQ(ccp->get_start_time().toSec(), 1.25);

    auto param = ccp->get_parameter();
    for (int i = 0; i < 6; ++i) {
        EXPECT_DOUBLE_EQ(param.K0(i, i), kp[i]) << "K0 cart diag " << i;
        EXPECT_DOUBLE_EQ(param.K1(i, i), kd[i]) << "K1 cart diag " << i;
        for (int j = 0; j < 6; ++j) {
            if (i == j)
                continue;
            EXPECT_DOUBLE_EQ(param.K0(i, j), 0.0);
            EXPECT_DOUBLE_EQ(param.K1(i, j), 0.0);
        }
    }

    // FB schema carries no nullspace gains; deserializer must default K0N/K1N
    // to zero so downstream controllers see a well-defined state.
    EXPECT_TRUE(param.K0N.isZero());
    EXPECT_TRUE(param.K1N.isZero());
}

// -----------------------------------------------------------------------------
// STOP
// -----------------------------------------------------------------------------

TEST(FlatBufferCtrParamStop, Stop_TypeDetected) {
    Deserializer deser;
    auto buf = build_stop_buffer();

    auto result = deser.get_trajectory_type(buf.data(), buf.size());
    ASSERT_TRUE(result.valid);
    EXPECT_EQ(result.type, TrajectoryType::STOP);
}

TEST(FlatBufferCtrParamStop, Stop_DeserializeReturnsNullptr) {
    // STOP intentionally has no trajectory object. The Robot layer handles it
    // by clearing the queue directly; the bare deserializer returns nullptr.
    Deserializer deser;
    auto buf = build_stop_buffer();

    auto traj_ptr = deser.deserialize(buf.data(), buf.size());
    EXPECT_EQ(traj_ptr, nullptr);
}

TEST(FlatBufferCtrParamStop, Stop_ClearsRobotTrajectoryQueue) {
    using orc::robots::Robot;
    using JointVector = orc::RobotTraits<DoF>::JointVector;

    // Load an actual robot model so the Robot<> instance is fully constructed.
    std::string model_path = "../models/iiwa_hanging.mjb";
    Robot<DoF> robot(model_path.c_str(), 0.001, "iiwa_link_e");

    // Seed the queue with two jointspace trajectories.
    JointVector q_start = JointVector::Zero();
    JointVector q_end = JointVector::Constant(0.5);
    robot.add_jointspace_trajectory(q_start, q_end, Time(0.0), Time(1.0));
    robot.add_jointspace_trajectory(q_end, q_start, Time(1.0), Time(2.0));
    ASSERT_EQ(robot.traj_queue.get_queue_size(), 2u);

    // STOP must drain the queue synchronously.
    auto buf = build_stop_buffer();
    const bool consumed = robot.add_trajectory_from_flatbuffer(buf.data(), buf.size());
    EXPECT_TRUE(consumed);
    EXPECT_EQ(robot.traj_queue.get_queue_size(), 0u);
}

// -----------------------------------------------------------------------------
// Negative: invalid control-parameter payloads must not crash
// -----------------------------------------------------------------------------

TEST(FlatBufferCtrParamStop, JointCtrParam_TruncatedBufferRejected) {
    Deserializer deser;
    auto buf = build_joint_ctr_param_buffer(
        /*t_ns=*/0, {1, 1, 1, 1, 1, 1, 1}, {1, 1, 1, 1, 1, 1, 1}, {1, 1, 1, 1, 1, 1, 1});
    // Drop the tail. Verifier must reject; deserializer must not dereference.
    buf.resize(buf.size() / 2);

    auto result = deser.get_trajectory_type(buf.data(), buf.size());
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(deser.deserialize(buf.data(), buf.size()), nullptr);
}

// -----------------------------------------------------------------------------
// Serializer → deserializer roundtrip
//
// These tests exercise the public FlatBufferSerializer entry points that the
// Python-side orcpy Robot class calls. Before adding them, the serializer had
// no serialize_jointctrparam / serialize_cartesianctrparam / serialize_stop
// roundtrip coverage.
// -----------------------------------------------------------------------------

TEST(FlatBufferCtrParamStop, JointCtrParam_SerializerRoundtrip) {
    using orc::control::JointCTParameter;
    using JointMatrix = orc::RobotTraits<DoF>::JointMatrix;
    using JointVector = orc::RobotTraits<DoF>::JointVector;

    Serializer ser;
    Deserializer deser;

    // Diagonal gains — only diagonals survive the wire format by design.
    JointVector kp_diag;
    kp_diag << 1, 2, 3, 4, 5, 6, 7;
    JointVector kd_diag;
    kd_diag << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
    JointVector ki_diag;
    ki_diag << 10, 20, 30, 40, 50, 60, 70;

    JointMatrix K0 = JointMatrix::Zero();
    JointMatrix K1 = JointMatrix::Zero();
    JointMatrix KI = JointMatrix::Zero();
    K0.diagonal() = kp_diag;
    K1.diagonal() = kd_diag;
    KI.diagonal() = ki_diag;
    JointCTParameter<DoF> param_in(K0, K1, KI);

    const Time t_in(1.5);
    auto buf = ser.serialize_jointctrparam_trajectory(t_in, param_in);
    ASSERT_FALSE(buf.empty());

    auto traj_ptr = deser.deserialize(buf.data(), buf.size());
    ASSERT_NE(traj_ptr, nullptr);
    ASSERT_EQ(traj_ptr->get_trajectory_type(), TrajectoryType::JOINT_CTR_PARAM);

    auto* jcp = static_cast<JointCtrParamTrajectory*>(traj_ptr.get());
    EXPECT_DOUBLE_EQ(jcp->get_start_time().toSec(), 1.5);

    auto param_out = jcp->get_parameter();
    for (int i = 0; i < DoF; ++i) {
        EXPECT_DOUBLE_EQ(param_out.K0(i, i), kp_diag[i]);
        EXPECT_DOUBLE_EQ(param_out.K1(i, i), kd_diag[i]);
        EXPECT_DOUBLE_EQ(param_out.KI(i, i), ki_diag[i]);
    }
}

TEST(FlatBufferCtrParamStop, JointCtrParam_OffDiagonalsDroppedOnWire) {
    // Document that non-diagonal entries do not survive the FB schema.
    using orc::control::JointCTParameter;
    using JointMatrix = orc::RobotTraits<DoF>::JointMatrix;

    Serializer ser;
    Deserializer deser;

    JointMatrix K0 = JointMatrix::Identity() * 2.0;
    JointMatrix K1 = JointMatrix::Identity() * 3.0;
    JointMatrix KI = JointMatrix::Identity() * 4.0;
    // Poison the off-diagonals.
    K0(0, 1) = 99.0;
    K1(1, 0) = -77.0;
    KI(2, 3) = 12345.0;
    JointCTParameter<DoF> param_in(K0, K1, KI);

    auto buf = ser.serialize_jointctrparam_trajectory(Time(0.0), param_in);
    auto traj_ptr = deser.deserialize(buf.data(), buf.size());
    ASSERT_NE(traj_ptr, nullptr);

    auto* jcp = static_cast<JointCtrParamTrajectory*>(traj_ptr.get());
    auto param_out = jcp->get_parameter();

    EXPECT_DOUBLE_EQ(param_out.K0(0, 1), 0.0) << "off-diagonal must be zeroed";
    EXPECT_DOUBLE_EQ(param_out.K1(1, 0), 0.0);
    EXPECT_DOUBLE_EQ(param_out.KI(2, 3), 0.0);
    // Diagonals still correct.
    for (int i = 0; i < DoF; ++i) {
        EXPECT_DOUBLE_EQ(param_out.K0(i, i), 2.0);
        EXPECT_DOUBLE_EQ(param_out.K1(i, i), 3.0);
        EXPECT_DOUBLE_EQ(param_out.KI(i, i), 4.0);
    }
}

TEST(FlatBufferCtrParamStop, CartCtrParam_SerializerRoundtrip) {
    using orc::control::CartesianCTParameter;
    using JointMatrix = orc::RobotTraits<DoF>::JointMatrix;

    Serializer ser;
    Deserializer deser;

    // K0 / K1 as diagonal CartesianMatrix, K0N / K1N populated but must be
    // dropped by the wire format.
    orc::CartesianVector kp_cart;
    kp_cart << 100, 200, 300, 11, 22, 33;
    orc::CartesianVector kd_cart;
    kd_cart << 1.0, 2.0, 3.0, 0.11, 0.22, 0.33;

    orc::CartesianMatrix K0 = orc::CartesianMatrix::Zero();
    orc::CartesianMatrix K1 = orc::CartesianMatrix::Zero();
    K0.diagonal() = kp_cart;
    K1.diagonal() = kd_cart;
    JointMatrix K0N = JointMatrix::Identity() * 5.0;  // poison
    JointMatrix K1N = JointMatrix::Identity() * 6.0;  // poison
    CartesianCTParameter<DoF> param_in(K0, K1, K0N, K1N);

    const Time t_in(2.75);
    auto buf = ser.serialize_cartesianctrparam_trajectory(t_in, param_in);
    ASSERT_FALSE(buf.empty());

    auto traj_ptr = deser.deserialize(buf.data(), buf.size());
    ASSERT_NE(traj_ptr, nullptr);
    ASSERT_EQ(traj_ptr->get_trajectory_type(), TrajectoryType::CART_CTR_PARAM);

    auto* ccp = static_cast<CartesianCtrParamTrajectory*>(traj_ptr.get());
    EXPECT_DOUBLE_EQ(ccp->get_start_time().toSec(), 2.75);

    auto param_out = ccp->get_parameter();
    for (int i = 0; i < 6; ++i) {
        EXPECT_DOUBLE_EQ(param_out.K0(i, i), kp_cart[i]);
        EXPECT_DOUBLE_EQ(param_out.K1(i, i), kd_cart[i]);
    }
    // Nullspace gains are NOT carried by the schema; deserializer defaults to zero.
    EXPECT_TRUE(param_out.K0N.isZero()) << "K0N must not traverse the wire";
    EXPECT_TRUE(param_out.K1N.isZero()) << "K1N must not traverse the wire";
}

TEST(FlatBufferCtrParamStop, Stop_SerializerProducesStopType) {
    Serializer ser;
    Deserializer deser;

    auto buf = ser.serialize_stop();
    ASSERT_FALSE(buf.empty());

    auto result = deser.get_trajectory_type(buf.data(), buf.size());
    ASSERT_TRUE(result.valid);
    EXPECT_EQ(result.type, TrajectoryType::STOP);
    EXPECT_EQ(deser.deserialize(buf.data(), buf.size()), nullptr);
}

TEST(FlatBufferCtrParamStop, Stop_SerializerBufferClearsRobotQueue) {
    using orc::robots::Robot;
    using JointVector = orc::RobotTraits<DoF>::JointVector;

    Serializer ser;
    std::string model_path = "../models/iiwa_hanging.mjb";
    Robot<DoF> robot(model_path.c_str(), 0.001, "iiwa_link_e");

    JointVector q_start = JointVector::Zero();
    JointVector q_end = JointVector::Constant(0.25);
    robot.add_jointspace_trajectory(q_start, q_end, Time(0.0), Time(1.0));
    ASSERT_EQ(robot.traj_queue.get_queue_size(), 1u);

    // End-to-end: serializer produces a buffer that Robot consumes as STOP.
    auto buf = ser.serialize_stop();
    ASSERT_TRUE(robot.add_trajectory_from_flatbuffer(buf.data(), buf.size()));
    EXPECT_EQ(robot.traj_queue.get_queue_size(), 0u);
}
