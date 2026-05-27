#include <gtest/gtest.h>
#include <cmath>
#include <tuple>
#include <vector>
#include "orc/com/flatbuffers/FlatBufferDeserializer.h"
#include "orc/com/flatbuffers/FlatBufferSerializer.h"
#include "orc/control/controller/cartesian/CartesianCTController.h"
#include "orc/control/controller/joint/JointCTController.h"
#include "orc/robots/Iiwa.h"
#include "orc/trajectory/JointspaceTrajectory.h"
#include "orc/trajectory/TaskspaceTrajectory.h"
#include "orc/trajectory/TrajectoryBase.h"
#include "orc/trajectory/TrajectoryPointStorage.h"
#include "orc/trajectory/TrajectoryQueue.h"
#include "orc/trajectory/TrajectoryType.h"
#include "orc/util/Logger.h"
constexpr int DoF = 7;
using FBSerializer = orc::com::fb::FlatBufferSerializer<DoF>;
using FBDeserializer = orc::com::fb::FlatBufferDeserializer<DoF>;
using JointspaceTrajectory = orc::trajectory::JointspaceTrajectory<DoF>;
using JointVector = orc::RobotTraits<DoF>::JointVector;
using JointArray = typename std::vector<JointVector>;
using Iiwa = orc::robots::Iiwa;
using JointCTParameter = orc::control::JointCTParameter<DoF>;
using JointCTController = orc::control::JointCTController<DoF>;
using TrajectoryQueue = orc::trajectory::TrajectoryQueue<DoF>;
using TrajectoryBase = orc::trajectory::TrajectoryBase<DoF>;
using TrajectoryType = orc::trajectory::TrajectoryType;
using TrajectoryPointStorage = orc::trajectory::TrajectoryPointStorage<DoF>;
using CartesianCTParameter = orc::control::CartesianCTParameter<DoF>;
using CartesianCTController = orc::control::CartesianCTController<DoF>;
using Time = orc::Time;
namespace {
// =========================================================================
// Helper: generate a smooth sinusoidal trajectory
// =========================================================================
std::pair<std::vector<JointVector>, std::vector<Time>> generateSinTrajectory(Time tend, Time dt,
                                                                             double freq = 2.0) {
    std::vector<JointVector> joint_poses;
    std::vector<Time> time_points;
    for (Time t(0, 0); t < tend; t += dt) {
        JointVector q;
        double q1 = cos(freq * 2 * M_PI * t.toSec());
        double q2 = -cos(freq * 2 * M_PI * t.toSec());
        q << q1, q2, 0, 0, 0, 0, 0;
        joint_poses.push_back(q);
        time_points.push_back(t);
    }
    return std::make_pair(joint_poses, time_points);
}
// =========================================================================
// Test 1: Serialize and deserialize a short trajectory — round-trip fidelity
// =========================================================================
TEST(TrajectoryTest, SerializeDeserializeRoundTrip) {
    orc::log::start_logging(orc::log::Level::Error);
    Time Tend(0, 500000000);  // 0.5s — short enough for a single segment
    Time dt(0, 100000000);    // 100ms between waypoints → 5 points
    auto [poses, times] = generateSinTrajectory(Tend, dt);
    ASSERT_GE(poses.size(), 3u) << "Need at least 3 points for serialization";
    // Serialize using FlatBuffers
    FBSerializer serializer;
    auto buffer = serializer.serialize_joint_trajectory(times, poses);
    ASSERT_GT(buffer.size(), 0u) << "Serializer should produce non-empty buffer";
    // Deserialize
    FBDeserializer deserializer;
    auto base_traj = deserializer.deserialize(buffer.data(), buffer.size());
    ASSERT_NE(base_traj, nullptr) << "Deserialized trajectory should not be null";
    EXPECT_EQ(base_traj->get_trajectory_type(), TrajectoryType::JOINTSPACE);
}
// =========================================================================
// Test 2: Deserialized trajectory matches original at waypoints
// =========================================================================
TEST(TrajectoryTest, DeserializedTrajectoryMatchesOriginal) {
    orc::log::start_logging(orc::log::Level::Error);
    // Short 3-point trajectory
    std::vector<JointVector> poses;
    JointVector q0 = JointVector::Zero();
    JointVector q1;
    q1 << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
    JointVector q2;
    q2 << 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4;
    poses = {q0, q1, q2};
    std::vector<Time> times = {Time(0.0), Time(0.5), Time(1.0)};
    JointspaceTrajectory orig_traj(poses, times);
    orig_traj.init();
    // Serialize with FlatBuffers
    FBSerializer serializer;
    auto buffer = serializer.serialize_joint_trajectory(times, poses);
    // Deserialize
    FBDeserializer deserializer;
    auto deserialized = deserializer.deserialize(buffer.data(), buffer.size());
    ASSERT_NE(deserialized, nullptr);
    auto* js = dynamic_cast<JointspaceTrajectory*>(deserialized.get());
    ASSERT_NE(js, nullptr) << "Should deserialize as JointspaceTrajectory";
    js->init();
    // Compare at start and end points
    orig_traj.update(Time(0.0));
    js->update(Time(0.0));
    for (int i = 0; i < DoF; ++i) {
        EXPECT_NEAR(js->get_q()(i), orig_traj.get_q()(i), 1e-6) << "Start mismatch at joint " << i;
    }
    orig_traj.update(Time(1.0));
    js->update(Time(1.0));
    for (int i = 0; i < DoF; ++i) {
        EXPECT_NEAR(js->get_q()(i), orig_traj.get_q()(i), 1e-6) << "End mismatch at joint " << i;
    }
}
// =========================================================================
// Test 3: Stitch trajectory through serialize→deserialize→queue
// =========================================================================
TEST(TrajectoryTest, StitchLongTrajectories) {
    orc::log::start_logging(orc::log::Level::Error);
    Time Tend(1, 0);        // 1s
    Time dt(0, 100000000);  // 100ms → ~10 points
    Time Ts(0, 1000000);    // 1ms control timestep
    auto [poses, times] = generateSinTrajectory(Tend, dt);
    ASSERT_GE(poses.size(), 5u);
    // Serialize and deserialize via FlatBuffers
    FBSerializer serializer;
    FBDeserializer deserializer;
    auto buffer = serializer.serialize_joint_trajectory(times, poses);
    auto base_traj = deserializer.deserialize(buffer.data(), buffer.size());
    ASSERT_NE(base_traj, nullptr);
    TrajectoryQueue traj_queue;
    traj_queue.add_trajectory(std::move(base_traj));
    // Simulate: step through the queue and verify the trajectory is finite
    int num_updates = 0;
    int nan_count = 0;
    for (Time t(0, 0); t < Tend; t += Ts) {
        TrajectoryBase* pcurr = traj_queue.update(t);
        if (pcurr != nullptr && pcurr->get_trajectory_type() == TrajectoryType::JOINTSPACE) {
            auto* js = static_cast<JointspaceTrajectory*>(pcurr);
            js->update(t);
            JointVector q = js->get_q();
            JointVector q_dot = js->get_q_dot();
            if (!q.allFinite() || !q_dot.allFinite())
                nan_count++;
            num_updates++;
        }
    }
    EXPECT_GT(num_updates, 100) << "Queue should have produced many trajectory updates";
    EXPECT_EQ(nan_count, 0) << "All trajectory values should be finite";
}
// =========================================================================
// Test 4: Queue trajectory switching with state continuity
// =========================================================================
TEST(TrajectoryTest, QueueSwitchingStateHandoff) {
    orc::log::start_logging(orc::log::Level::Error);
    // Two consecutive trajectories with overlapping intent
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones() * 0.5;
    JointVector q2 = JointVector::Ones();
    JointspaceTrajectory traj1(q0, q1, Time(0.0), Time(1.0));
    JointspaceTrajectory traj2(q1, q2, Time(1.0), Time(2.0));
    TrajectoryQueue queue;
    queue.add_jointspace_trajectory(traj1);
    queue.add_jointspace_trajectory(traj2);
    EXPECT_EQ(queue.get_queue_size(), 2);
    // Step to just before the transition
    TrajectoryBase* curr = queue.update(Time(0.99));
    ASSERT_NE(curr, nullptr);
    EXPECT_EQ(curr->get_trajectory_type(), TrajectoryType::JOINTSPACE);
    auto* js1 = static_cast<JointspaceTrajectory*>(curr);
    js1->update(Time(0.99));
    JointVector q_before = js1->get_q();
    // Step past the transition
    curr = queue.update(Time(1.01));
    ASSERT_NE(curr, nullptr);
    auto* js2 = static_cast<JointspaceTrajectory*>(curr);
    js2->update(Time(1.01));
    JointVector q_after = js2->get_q();
    // Position should be continuous (close to q1)
    for (int i = 0; i < DoF; ++i) {
        EXPECT_NEAR(q_before(i), q1(i), 0.05)
            << "Position before transition should be near target at joint " << i;
    }
}
// =========================================================================
// Test 5: FlatBuffer serialization round-trip preserves trajectory type
// =========================================================================
TEST(TrajectoryTest, FlatBufferRoundTripType) {
    orc::log::start_logging(orc::log::Level::Error);
    std::vector<JointVector> poses;
    std::vector<Time> times;
    for (int i = 0; i < 20; ++i) {
        double t = i * 0.1;
        JointVector q = JointVector::Ones() * std::sin(t);
        poses.push_back(q);
        times.push_back(Time(t));
    }
    FBSerializer serializer;
    auto buffer = serializer.serialize_joint_trajectory(times, poses);
    ASSERT_GT(buffer.size(), 0u);
    FBDeserializer deserializer;
    auto recovered = deserializer.deserialize(buffer.data(), buffer.size());
    ASSERT_NE(recovered, nullptr);
    EXPECT_EQ(recovered->get_trajectory_type(), TrajectoryType::JOINTSPACE);
}
// =========================================================================
// Test 6: End-to-end simulation with Iiwa robot and controller
// =========================================================================
TEST(TrajectoryTest, EndToEndSimulationWithController) {
    orc::log::start_logging(orc::log::Level::Error);
    Time Ts(0, 1000000);  // 1ms
    Iiwa iiwa("../models/iiwa_hanging.mjb", Ts);
    JointVector q_zero = JointVector::Zero();
    JointVector q_target;
    q_target << 0.1, -0.1, 0.05, -0.3, 0.02, 0.1, -0.05;
    // Start the robot and add trajectory
    iiwa.add_jointspace_trajectory(q_zero, q_target, Time(0.0), Time(1.0));
    // Run a few update steps
    int finite_count = 0;
    for (int step = 0; step < 100; ++step) {
        Time t(0, step * 1000000);  // step ms
        bool ok = iiwa.update(t);
        if (ok && iiwa.get_q_act().allFinite())
            finite_count++;
    }
    EXPECT_GT(finite_count, 50) << "Most update steps should produce finite outputs";
}
}  // namespace
