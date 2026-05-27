/**
 * @file robot_state_test.cpp
 * @brief Tests for RobotState serialization round-trips.
 *        Adapted to use FlatBuffer-based RobotState API.
 */
#include <gtest/gtest.h>
#include <cstring>
#include <vector>
#include "orc/Orc.h"
#include "orc/RobotStatus.h"
#include "orc/com/RobotState.h"
#include "orc/com/flatbuffers/FlatBufferRobotState.h"
#include "orc/robots/Iiwa.h"
#include "orc/util/Logger.h"
namespace {
constexpr int DOF = 7;
using Time = orc::Time;
using JointVector = orc::RobotTraits<DOF>::JointVector;
using PoseVector = orc::PoseVector;
using CartesianVector = orc::CartesianVector;
using Iiwa = orc::robots::Iiwa;
using RobotState = orc::com::RobotState<Iiwa>;
using RobotStatus = orc::logic::RobotStatus;
// ===========================================================================
// RobotState serialization/deserialization round-trip
// ===========================================================================
TEST(RobotStateTest, SerializeDeserializeRoundTrip) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    JointVector q = JointVector::Zero();
    iiwa.set_q_act(q);
    iiwa.update(Time(0.0));
    RobotState state(iiwa, Time(1.5), RobotStatus::ENABLE, 42);
    // Use new FlatBuffer-based serialize API
    auto buffer = state.serialize();
    ASSERT_GT(buffer.size(), 0u) << "Serialized buffer should not be empty";
    // Deserialize
    RobotState recovered =
        RobotState::deserialize(reinterpret_cast<const char*>(buffer.data()), buffer.size());
    EXPECT_EQ(recovered.time_, Time(1.5));
    EXPECT_EQ(recovered.status, RobotStatus::ENABLE);
    EXPECT_EQ(recovered.model_id, 42);
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(recovered.q_act(i), state.q_act(i), 1e-6);
        EXPECT_NEAR(recovered.q_dot_act(i), state.q_dot_act(i), 1e-6);
        EXPECT_NEAR(recovered.tau(i), state.tau(i), 1e-6);
    }
}
TEST(RobotStateTest, BufferConstructor) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    iiwa.update(Time(0.0));
    RobotState original(iiwa, Time(2.0), RobotStatus::GRAVCOMP, 7);
    auto buffer = original.serialize();
    std::vector<unsigned char> ubuffer(buffer.begin(), buffer.end());
    RobotState from_buffer(ubuffer);
    EXPECT_EQ(from_buffer.time_, Time(2.0));
    EXPECT_EQ(from_buffer.status, RobotStatus::GRAVCOMP);
    EXPECT_EQ(from_buffer.model_id, 7);
}
TEST(RobotStateTest, AllStatusValues) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    iiwa.update(Time(0.0));
    for (auto status : {RobotStatus::OFF, RobotStatus::ENABLE, RobotStatus::GRAVCOMP}) {
        RobotState state(iiwa, Time(0.0), status, 0);
        auto buffer = state.serialize();
        RobotState recovered =
            RobotState::deserialize(reinterpret_cast<const char*>(buffer.data()), buffer.size());
        EXPECT_EQ(recovered.status, status);
    }
}
TEST(RobotStateTest, NonZeroJointConfigRoundTrip) {
    orc::log::start_logging(orc::log::Level::Error);
    Iiwa iiwa("../models/iiwa_hanging.mjb");
    JointVector q;
    q << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7;
    iiwa.set_q_act(q);
    iiwa.update(Time(0.0));
    iiwa.add_jointspace_trajectory(q, q, Time(0.0), Time(1.0));
    iiwa.update(Time(0.0));
    RobotState state(iiwa, Time(0.0), RobotStatus::ENABLE, 1);
    auto buffer = state.serialize();
    RobotState recovered =
        RobotState::deserialize(reinterpret_cast<const char*>(buffer.data()), buffer.size());
    for (int i = 0; i < DOF; ++i) {
        EXPECT_NEAR(recovered.q_act(i), q(i), 1e-6);
    }
}

// Guardrails against the UDP segfault: malformed / truncated / oversized buffers
// must not crash and must leave the returned state at its default (zeroed).
TEST(RobotStateTest, DeserializeZeroSizeIsSafe) {
    orc::log::start_logging(orc::log::Level::Error);
    std::vector<uint8_t> empty;
    RobotState recovered = RobotState::deserialize(reinterpret_cast<const char*>(empty.data()), 0);
    EXPECT_EQ(recovered.time_, Time(0.0));
    EXPECT_EQ(recovered.model_id, 0);
}

TEST(RobotStateTest, DeserializeGarbageIsSafe) {
    orc::log::start_logging(orc::log::Level::Error);
    std::vector<uint8_t> garbage(128, 0xAB);
    RobotState recovered =
        RobotState::deserialize(reinterpret_cast<const char*>(garbage.data()), garbage.size());
    EXPECT_EQ(recovered.time_, Time(0.0));
}

// Build a valid serialized buffer without requiring a loaded Iiwa model —
// avoids mujoco-path/model-lookup preconditions in tests that only exercise
// the deserializer edge cases.
static std::vector<uint8_t> make_valid_state_buffer(Time t = Time(1.0), uint8_t model_id = 1) {
    orc::com::fb::RobotStateSerializer<Iiwa> ser;
    JointVector z = JointVector::Zero();
    return ser.serialize(t, RobotStatus::ENABLE, model_id, z, z, z, z, z, z, z);
}

TEST(RobotStateTest, DeserializeTruncatedIsSafe) {
    orc::log::start_logging(orc::log::Level::Error);
    auto buffer = make_valid_state_buffer();
    ASSERT_GT(buffer.size(), 8u);
    // Truncate by one byte — Verifier must reject, no crash.
    RobotState recovered =
        RobotState::deserialize(reinterpret_cast<const char*>(buffer.data()), buffer.size() - 1);
    EXPECT_EQ(recovered.time_, Time(0.0));
}

TEST(RobotStateTest, DeserializeOversizedBufferStillWorks) {
    // Simulates the UDP case: receive into a 2048-byte buffer, but the message
    // is shorter. Passing the actual message length must succeed; passing the
    // full buffer length (stale trailing bytes) must also succeed since the
    // verifier walks offsets from the root and ignores trailing junk.
    orc::log::start_logging(orc::log::Level::Error);
    auto buffer = make_valid_state_buffer(Time(2.5), 9);
    std::vector<uint8_t> padded(2048, 0xFF);
    std::copy(buffer.begin(), buffer.end(), padded.begin());
    RobotState recovered =
        RobotState::deserialize(reinterpret_cast<const char*>(padded.data()), buffer.size());
    EXPECT_EQ(recovered.time_, Time(2.5));
    EXPECT_EQ(recovered.model_id, 9);
}
}  // namespace
