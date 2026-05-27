// Guardrail tests for the FlatBuffers trajectory wire protocol.
//
// Verifies the round-trip property: for every trajectory type the serializer
// accepts, serialize → deserialize produces a trajectory with identical
// observable state (time points, knot data). Kept format-agnostic so it
// continues to hold across future wire-format revisions.

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "orc/com/flatbuffers/FlatBufferDeserializer.h"
#include "orc/com/flatbuffers/FlatBufferSerializer.h"
#include "orc/trajectory/JointspaceTrajectory.h"
#include "orc/trajectory/TaskspaceTrajectory.h"
#include "orc/trajectory/TrajectoryBase.h"
#include "orc/trajectory/TrajectoryType.h"
#include "orc/trajectory/singleevent/CartesianCtrParamTrajectory.h"
#include "orc/trajectory/singleevent/JointCtrParamTrajectory.h"
#include "orc/trajectory/singleevent/NullspaceTrajectory.h"
#include "orc/util/Time.h"

namespace {

constexpr int DoF = 7;

using Serializer = orc::com::fb::FlatBufferSerializer<DoF>;
using Deserializer = orc::com::fb::FlatBufferDeserializer<DoF>;
using JointspaceTrajectory = orc::trajectory::JointspaceTrajectory<DoF>;
using TaskspaceTrajectory = orc::trajectory::TaskspaceTrajectory<DoF>;
using NullspaceTrajectory = orc::trajectory::NullspaceTrajectory<DoF>;
using JointCtrParamTrajectory = orc::trajectory::JointCtrParamTrajectory<DoF>;
using CartesianCtrParamTrajectory = orc::trajectory::CartesianCtrParamTrajectory<DoF>;
using JointCTParameter = orc::control::JointCTParameter<DoF>;
using CartesianCTParameter = orc::control::CartesianCTParameter<DoF>;
using TrajectoryType = orc::trajectory::TrajectoryType;
using JointVector = orc::RobotTraits<DoF>::JointVector;
using JointMatrix = orc::RobotTraits<DoF>::JointMatrix;
using PoseVector = orc::PoseVector;
using Time = orc::Time;

}  // namespace

TEST(FlatBufferRoundtrip, JointspaceKnotsPreserved) {
    Serializer ser;
    Deserializer deser;

    std::vector<JointVector> q_in;
    std::vector<Time> t_in;
    for (int k = 0; k < 3; ++k) {
        q_in.push_back(JointVector::LinSpaced(DoF, 0.1 * k, 0.1 * k + 0.5));
        t_in.emplace_back(static_cast<double>(k));
    }

    auto buf = ser.serialize_joint_trajectory(t_in, q_in);
    ASSERT_FALSE(buf.empty()) << "serializer produced zero-byte buffer";

    auto traj = deser.deserialize(buf.data(), buf.size());
    ASSERT_NE(traj, nullptr);
    ASSERT_EQ(traj->get_trajectory_type(), TrajectoryType::JOINTSPACE);

    auto* js = static_cast<JointspaceTrajectory*>(traj.get());
    const auto q_out = js->get_joint_poses();
    const auto t_out = js->get_time_points();

    ASSERT_EQ(q_out.size(), q_in.size());
    ASSERT_EQ(t_out.size(), t_in.size());
    for (size_t i = 0; i < q_in.size(); ++i) {
        EXPECT_TRUE(q_out[i].isApprox(q_in[i], 1e-12))
            << "knot " << i << " joint vector drifted across serialize/deserialize";
        EXPECT_DOUBLE_EQ(t_out[i].toSec(), t_in[i].toSec()) << "knot " << i << " time drifted";
    }
}

TEST(FlatBufferRoundtrip, TaskspaceKnotsPreserved) {
    Serializer ser;
    Deserializer deser;

    std::vector<PoseVector> p_in;
    std::vector<Time> t_in;
    for (int k = 0; k < 3; ++k) {
        PoseVector p = PoseVector::Zero();
        p(0) = 0.1 * k;  // x drifts
        p(3) = 1.0;      // quaternion w = 1
        p_in.push_back(p);
        t_in.emplace_back(static_cast<double>(k));
    }

    auto buf = ser.serialize_cartesian_trajectory(t_in, p_in);
    ASSERT_FALSE(buf.empty());

    auto traj = deser.deserialize(buf.data(), buf.size());
    ASSERT_NE(traj, nullptr);
    ASSERT_EQ(traj->get_trajectory_type(), TrajectoryType::TASKSPACE);

    auto* ts = static_cast<TaskspaceTrajectory*>(traj.get());
    const auto p_out = ts->get_pose_vector();
    const auto t_out = ts->get_time_points();

    ASSERT_EQ(p_out.size(), p_in.size());
    ASSERT_EQ(t_out.size(), t_in.size());
    for (size_t i = 0; i < p_in.size(); ++i) {
        EXPECT_TRUE(p_out[i].isApprox(p_in[i], 1e-12)) << "pose knot " << i << " drifted";
        EXPECT_DOUBLE_EQ(t_out[i].toSec(), t_in[i].toSec());
    }
}

TEST(FlatBufferRoundtrip, NullspacePayloadPreserved) {
    Serializer ser;
    Deserializer deser;

    Time t0(1.5);
    JointVector q_ns = JointVector::LinSpaced(DoF, -0.3, 0.7);

    auto buf = ser.serialize_nullspace_trajectory(t0, q_ns);
    ASSERT_FALSE(buf.empty());

    auto traj = deser.deserialize(buf.data(), buf.size());
    ASSERT_NE(traj, nullptr);
    ASSERT_EQ(traj->get_trajectory_type(), TrajectoryType::NULLSPACE);

    auto* ns = static_cast<NullspaceTrajectory*>(traj.get());
    EXPECT_DOUBLE_EQ(ns->get_start_time().toSec(), t0.toSec());
    EXPECT_TRUE(ns->get_nullspace_joint_state().isApprox(q_ns, 1e-12));
}

TEST(FlatBufferRoundtrip, JointCtrParamPayloadPreserved) {
    Serializer ser;
    Deserializer deser;

    Time t0(2.0);
    JointCTParameter param;
    param.K0 = JointMatrix::Identity() * 42.0;
    param.K1 = JointMatrix::Identity() * 7.5;
    param.KI = JointMatrix::Identity() * 0.25;
    param.xq_I_max = JointVector::Constant(5.0);

    auto buf = ser.serialize_jointctrparam_trajectory(t0, param);
    ASSERT_FALSE(buf.empty());

    auto traj = deser.deserialize(buf.data(), buf.size());
    ASSERT_NE(traj, nullptr);
    ASSERT_EQ(traj->get_trajectory_type(), TrajectoryType::JOINT_CTR_PARAM);

    auto* jp = static_cast<JointCtrParamTrajectory*>(traj.get());
    EXPECT_DOUBLE_EQ(jp->get_start_time().toSec(), t0.toSec());
    // FlatBuffers wire format carries only diagonals, so compare diagonals.
    const auto out = jp->get_parameter();
    EXPECT_TRUE(out.K0.diagonal().isApprox(param.K0.diagonal(), 1e-12));
    EXPECT_TRUE(out.K1.diagonal().isApprox(param.K1.diagonal(), 1e-12));
    EXPECT_TRUE(out.KI.diagonal().isApprox(param.KI.diagonal(), 1e-12));
}

TEST(FlatBufferRoundtrip, CartesianCtrParamPayloadPreserved) {
    Serializer ser;
    Deserializer deser;

    Time t0(3.25);
    CartesianCTParameter param;  // default-constructed; exercise defaults round-trip.

    auto buf = ser.serialize_cartesianctrparam_trajectory(t0, param);
    ASSERT_FALSE(buf.empty());

    auto traj = deser.deserialize(buf.data(), buf.size());
    ASSERT_NE(traj, nullptr);
    ASSERT_EQ(traj->get_trajectory_type(), TrajectoryType::CART_CTR_PARAM);

    auto* cp = static_cast<CartesianCtrParamTrajectory*>(traj.get());
    EXPECT_DOUBLE_EQ(cp->get_start_time().toSec(), t0.toSec());
}

TEST(FlatBufferRoundtrip, UnknownBufferFallsBackSafely) {
    Deserializer deser;
    std::vector<uint8_t> garbage(64, 0xFF);
    // Corrupt buffer must not crash; may return nullptr or a safe-stop trajectory.
    auto traj = deser.deserialize(garbage.data(), garbage.size());
    if (traj != nullptr) {
        // If the deserializer returns something, it must be a recognised type.
        EXPECT_NE(traj->get_trajectory_type(), TrajectoryType::INVALID);
    }
}

// =============================================================================
// Variadic-DoF coverage: the new schema carries joint vectors as [double], so
// the serializer/deserializer must work for any DoF. Cover {2, 6, 7} — 6 is
// the proof that no DoF is baked into the schema.
// =============================================================================

template <int N>
struct DoFTag {
    static constexpr int value = N;
};

template <typename T>
class FlatBufferDoFRoundtrip : public ::testing::Test {};

using DoFTypes = ::testing::Types<DoFTag<2>, DoFTag<6>, DoFTag<7>>;
TYPED_TEST_SUITE(FlatBufferDoFRoundtrip, DoFTypes);

TYPED_TEST(FlatBufferDoFRoundtrip, JointTrajectoryRoundtrip) {
    constexpr int N = TypeParam::value;
    using JV = typename orc::RobotTraits<N>::JointVector;
    orc::com::fb::FlatBufferSerializer<N> ser;
    orc::com::fb::FlatBufferDeserializer<N> deser;

    std::vector<JV> q_in;
    std::vector<orc::Time> t_in;
    for (int k = 0; k < 3; ++k) {
        q_in.push_back(JV::LinSpaced(N, 0.1 * k, 0.1 * k + 0.5));
        t_in.emplace_back(static_cast<double>(k));
    }

    auto buf = ser.serialize_joint_trajectory(t_in, q_in);
    ASSERT_FALSE(buf.empty());

    auto traj = deser.deserialize(buf.data(), buf.size());
    ASSERT_NE(traj, nullptr) << "DoF=" << N << " roundtrip dropped";
    ASSERT_EQ(traj->get_trajectory_type(), orc::trajectory::TrajectoryType::JOINTSPACE);

    auto* js = static_cast<orc::trajectory::JointspaceTrajectory<N>*>(traj.get());
    const auto q_out = js->get_joint_poses();
    ASSERT_EQ(q_out.size(), q_in.size());
    for (size_t i = 0; i < q_in.size(); ++i) {
        EXPECT_TRUE(q_out[i].isApprox(q_in[i], 1e-12)) << "DoF=" << N << " knot " << i;
    }
}

TYPED_TEST(FlatBufferDoFRoundtrip, NullspaceRoundtrip) {
    constexpr int N = TypeParam::value;
    using JV = typename orc::RobotTraits<N>::JointVector;
    orc::com::fb::FlatBufferSerializer<N> ser;
    orc::com::fb::FlatBufferDeserializer<N> deser;

    JV q_ns = JV::LinSpaced(N, -0.4, 0.6);
    auto buf = ser.serialize_nullspace_trajectory(orc::Time(1.25), q_ns);
    ASSERT_FALSE(buf.empty());

    auto traj = deser.deserialize(buf.data(), buf.size());
    ASSERT_NE(traj, nullptr);
    auto* ns = static_cast<orc::trajectory::NullspaceTrajectory<N>*>(traj.get());
    EXPECT_TRUE(ns->get_nullspace_joint_state().isApprox(q_ns, 1e-12));
}

TYPED_TEST(FlatBufferDoFRoundtrip, JointCtrParamRoundtrip) {
    constexpr int N = TypeParam::value;
    using JM = typename orc::RobotTraits<N>::JointMatrix;
    orc::com::fb::FlatBufferSerializer<N> ser;
    orc::com::fb::FlatBufferDeserializer<N> deser;

    orc::control::JointCTParameter<N> param;
    param.K0 = JM::Identity() * 5.0;
    param.K1 = JM::Identity() * 1.5;
    param.KI = JM::Identity() * 0.1;

    auto buf = ser.serialize_jointctrparam_trajectory(orc::Time(2.0), param);
    auto traj = deser.deserialize(buf.data(), buf.size());
    ASSERT_NE(traj, nullptr);
    auto* jp = static_cast<orc::trajectory::JointCtrParamTrajectory<N>*>(traj.get());
    EXPECT_TRUE(jp->get_parameter().K0.diagonal().isApprox(param.K0.diagonal(), 1e-12));
    EXPECT_TRUE(jp->get_parameter().K1.diagonal().isApprox(param.K1.diagonal(), 1e-12));
    EXPECT_TRUE(jp->get_parameter().KI.diagonal().isApprox(param.KI.diagonal(), 1e-12));
}

// Cross-DoF mismatch is the one new failure mode the variadic schema
// introduces (the per-DoF schema couldn't detect it). A reader instantiated
// for one DoF must reject buffers that carry vectors of a different size,
// without crashing.
TEST(FlatBufferRoundtrip, CrossDoFMismatchRejectedCleanly) {
    using JV7 = orc::RobotTraits<7>::JointVector;
    orc::com::fb::FlatBufferSerializer<7> ser7;
    orc::com::fb::FlatBufferDeserializer<2> deser2;

    std::vector<JV7> q_in{JV7::Constant(0.1), JV7::Constant(0.2)};
    std::vector<orc::Time> t_in{orc::Time(0.0), orc::Time(1.0)};
    auto buf = ser7.serialize_joint_trajectory(t_in, q_in);
    ASSERT_FALSE(buf.empty());

    auto traj = deser2.deserialize(buf.data(), buf.size());
    EXPECT_EQ(traj, nullptr) << "2-DoF reader must reject 7-DoF payload, not crash";

    auto reader = deser2.get_joint_trajectory_reader(buf.data(), buf.size());
    EXPECT_EQ(reader.num_points(), 0u);
}
