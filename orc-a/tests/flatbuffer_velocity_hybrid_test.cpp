// FlatBuffer wire-protocol tests for the three trajectory kinds added after
// the protobuf->FB migration: jointspace velocity, Cartesian velocity, and
// hybrid force/motion. Each test serialises through
// FlatBufferSerializer<DOF>, round-trips through FlatBufferDeserializer<DOF>,
// and asserts the reconstructed trajectory object preserves the knot data.

#include <gtest/gtest.h>

#include <vector>

#include "orc/com/flatbuffers/FlatBufferDeserializer.h"
#include "orc/com/flatbuffers/FlatBufferSerializer.h"
#include "orc/robots/Robot.h"
#include "orc/trajectory/CartesianVelocityTrajectory.h"
#include "orc/trajectory/HybridForceMotionTrajectory.h"
#include "orc/trajectory/JointspaceVelocityTrajectory.h"
#include "orc/trajectory/TrajectoryType.h"
#include "orc/util/Time.h"

namespace {

constexpr int DoF = 7;

using Serializer = orc::com::fb::FlatBufferSerializer<DoF>;
using Deserializer = orc::com::fb::FlatBufferDeserializer<DoF>;
using TrajectoryType = orc::trajectory::TrajectoryType;
using JointspaceVelocityTrajectory = orc::trajectory::JointspaceVelocityTrajectory<DoF>;
using CartesianVelocityTrajectory = orc::trajectory::CartesianVelocityTrajectory<DoF>;
using HybridForceMotionTrajectory = orc::trajectory::HybridForceMotionTrajectory<DoF>;
using JointVector = orc::RobotTraits<DoF>::JointVector;
using Time = orc::Time;

}  // namespace

// -----------------------------------------------------------------------------
// Jointspace velocity
// -----------------------------------------------------------------------------

TEST(FlatBufferVelocityHybrid, JointspaceVelocity_TypeDetected) {
    Serializer ser;
    Deserializer deser;

    std::vector<JointVector> vels;
    std::vector<Time> times;
    for (int k = 0; k < 3; ++k) {
        vels.push_back(JointVector::LinSpaced(DoF, 0.1 * k, 0.1 * k + 0.5));
        times.emplace_back(static_cast<double>(k));
    }

    auto buf = ser.serialize_jointspace_velocity_trajectory(times, vels);
    ASSERT_FALSE(buf.empty());

    auto result = deser.get_trajectory_type(buf.data(), buf.size());
    ASSERT_TRUE(result.valid);
    EXPECT_EQ(result.type, TrajectoryType::JOINTSPACE_VELOCITY);
}

TEST(FlatBufferVelocityHybrid, JointspaceVelocity_Roundtrip) {
    Serializer ser;
    Deserializer deser;

    std::vector<JointVector> vels_in;
    std::vector<Time> times_in;
    for (int k = 0; k < 4; ++k) {
        JointVector v;
        for (int j = 0; j < DoF; ++j)
            v[j] = 0.01 * (k + 1) * (j + 1);
        vels_in.push_back(v);
        times_in.emplace_back(0.5 * k);
    }

    auto buf = ser.serialize_jointspace_velocity_trajectory(times_in, vels_in);
    auto traj_ptr = deser.deserialize(buf.data(), buf.size());
    ASSERT_NE(traj_ptr, nullptr);
    ASSERT_EQ(traj_ptr->get_trajectory_type(), TrajectoryType::JOINTSPACE_VELOCITY);

    auto* v_traj = static_cast<JointspaceVelocityTrajectory*>(traj_ptr.get());
    auto vels_out = v_traj->get_velocity_points();
    auto times_out = v_traj->get_time_points();

    ASSERT_EQ(vels_out.size(), vels_in.size());
    ASSERT_EQ(times_out.size(), times_in.size());
    for (size_t i = 0; i < vels_in.size(); ++i) {
        EXPECT_TRUE(vels_out[i].isApprox(vels_in[i], 1e-12)) << "velocity knot " << i << " drifted";
        EXPECT_DOUBLE_EQ(times_out[i].toSec(), times_in[i].toSec());
    }
}

TEST(FlatBufferVelocityHybrid, JointspaceVelocity_IntegrationUpdatesRobotData) {
    // Use a real Robot to get a correctly-constructed RobotData (needs model).
    using orc::robots::Robot;
    Robot<DoF> robot("../models/iiwa_hanging.mjb", 0.001, "iiwa_link_e");
    robot.robot_data.q_d.setZero();

    std::vector<JointVector> vels = {JointVector::Constant(1.0), JointVector::Constant(1.0),
                                     JointVector::Constant(1.0)};
    std::vector<Time> times = {Time(0.0), Time(0.5), Time(1.0)};
    JointspaceVelocityTrajectory traj(vels, times);
    traj.init();

    traj.update(Time(0.0), robot.robot_data);
    EXPECT_TRUE(robot.robot_data.q_dot_d.isApprox(JointVector::Constant(1.0), 1e-9));

    traj.update(Time(0.5), robot.robot_data);
    // q_d should advance by ~0.5 after dt=0.5 at v=1.0
    EXPECT_NEAR(robot.robot_data.q_d[0], 0.5, 1e-9);
}

// -----------------------------------------------------------------------------
// Cartesian velocity
// -----------------------------------------------------------------------------

TEST(FlatBufferVelocityHybrid, CartesianVelocity_TypeDetected) {
    Serializer ser;
    Deserializer deser;

    std::vector<orc::CartesianVector> vels;
    std::vector<Time> times;
    for (int k = 0; k < 3; ++k) {
        orc::CartesianVector v;
        v << 0.1 * k, 0, 0, 0, 0, 0;
        vels.push_back(v);
        times.emplace_back(static_cast<double>(k));
    }

    auto buf = ser.serialize_cartesian_velocity_trajectory(times, vels);
    ASSERT_FALSE(buf.empty());

    auto result = deser.get_trajectory_type(buf.data(), buf.size());
    ASSERT_TRUE(result.valid);
    EXPECT_EQ(result.type, TrajectoryType::CARTESIAN_VELOCITY);
}

TEST(FlatBufferVelocityHybrid, CartesianVelocity_Roundtrip) {
    Serializer ser;
    Deserializer deser;

    std::vector<orc::CartesianVector> vels_in;
    std::vector<Time> times_in;
    for (int k = 0; k < 4; ++k) {
        orc::CartesianVector v;
        v << 0.1 * k, 0.2 * k, 0.3 * k, 0.01 * k, 0.02 * k, 0.03 * k;
        vels_in.push_back(v);
        times_in.emplace_back(0.25 * k);
    }

    auto buf = ser.serialize_cartesian_velocity_trajectory(times_in, vels_in);
    auto traj_ptr = deser.deserialize(buf.data(), buf.size());
    ASSERT_NE(traj_ptr, nullptr);
    ASSERT_EQ(traj_ptr->get_trajectory_type(), TrajectoryType::CARTESIAN_VELOCITY);

    auto* v_traj = static_cast<CartesianVelocityTrajectory*>(traj_ptr.get());
    auto vels_out = v_traj->get_velocity_points();
    auto times_out = v_traj->get_time_points();

    ASSERT_EQ(vels_out.size(), vels_in.size());
    for (size_t i = 0; i < vels_in.size(); ++i) {
        EXPECT_TRUE(vels_out[i].isApprox(vels_in[i], 1e-12));
        EXPECT_DOUBLE_EQ(times_out[i].toSec(), times_in[i].toSec());
    }
}

// -----------------------------------------------------------------------------
// Hybrid force / motion
// -----------------------------------------------------------------------------

TEST(FlatBufferVelocityHybrid, HybridForceMotion_TypeDetected) {
    Serializer ser;
    Deserializer deser;

    std::vector<orc::PoseVector> poses;
    std::vector<double> forces;
    std::vector<Time> times;
    for (int k = 0; k < 3; ++k) {
        orc::PoseVector p = orc::PoseVector::Zero();
        p[0] = 0.1 * k;
        p[3] = 1.0;  // qw = 1 → identity rotation
        poses.push_back(p);
        forces.push_back(5.0 * k);
        times.emplace_back(static_cast<double>(k));
    }

    auto buf = ser.serialize_hybrid_force_motion_trajectory(times, poses, forces);
    ASSERT_FALSE(buf.empty());

    auto result = deser.get_trajectory_type(buf.data(), buf.size());
    ASSERT_TRUE(result.valid);
    EXPECT_EQ(result.type, TrajectoryType::HYBRID_FORCE_MOTION);
}

TEST(FlatBufferVelocityHybrid, HybridForceMotion_Roundtrip) {
    Serializer ser;
    Deserializer deser;

    std::vector<orc::PoseVector> poses_in;
    std::vector<double> forces_in;
    std::vector<Time> times_in;
    for (int k = 0; k < 4; ++k) {
        orc::PoseVector p = orc::PoseVector::Zero();
        p[0] = 0.1 * k;
        p[1] = -0.05 * k;
        p[2] = 0.2;
        p[3] = 1.0;
        poses_in.push_back(p);
        forces_in.push_back(1.0 + k);
        times_in.emplace_back(0.5 * k);
    }

    auto buf = ser.serialize_hybrid_force_motion_trajectory(times_in, poses_in, forces_in);
    auto traj_ptr = deser.deserialize(buf.data(), buf.size());
    ASSERT_NE(traj_ptr, nullptr);
    ASSERT_EQ(traj_ptr->get_trajectory_type(), TrajectoryType::HYBRID_FORCE_MOTION);

    auto* h_traj = static_cast<HybridForceMotionTrajectory*>(traj_ptr.get());
    h_traj->init();

    // Step through the trajectory: at each knot time, commanded pose/force
    // must match the input. Interpolation on the knot itself returns the knot.
    for (size_t i = 0; i < times_in.size(); ++i) {
        h_traj->update(times_in[i]);
        EXPECT_NEAR(h_traj->get_force(), forces_in[i], 1e-9) << "force knot " << i << " drifted";
        // Pose linear part must match the knot exactly at the knot time.
        EXPECT_NEAR(h_traj->get_pose()[0], poses_in[i][0], 1e-6);
        EXPECT_NEAR(h_traj->get_pose()[1], poses_in[i][1], 1e-6);
        EXPECT_NEAR(h_traj->get_pose()[2], poses_in[i][2], 1e-6);
    }
}

// -----------------------------------------------------------------------------
// Negatives
// -----------------------------------------------------------------------------

TEST(FlatBufferVelocityHybrid, TruncatedVelocityBufferRejected) {
    Serializer ser;
    Deserializer deser;

    std::vector<JointVector> vels(3, JointVector::Ones());
    std::vector<Time> times = {Time(0.0), Time(0.5), Time(1.0)};
    auto buf = ser.serialize_jointspace_velocity_trajectory(times, vels);
    buf.resize(buf.size() / 2);

    auto result = deser.get_trajectory_type(buf.data(), buf.size());
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(deser.deserialize(buf.data(), buf.size()), nullptr);
}

TEST(FlatBufferVelocityHybrid, TruncatedHybridBufferRejected) {
    Serializer ser;
    Deserializer deser;

    std::vector<orc::PoseVector> poses(3, orc::PoseVector::Zero());
    for (auto& p : poses)
        p[3] = 1.0;
    std::vector<double> forces = {1.0, 2.0, 3.0};
    std::vector<Time> times = {Time(0.0), Time(0.5), Time(1.0)};

    auto buf = ser.serialize_hybrid_force_motion_trajectory(times, poses, forces);
    buf.resize(buf.size() / 3);

    auto result = deser.get_trajectory_type(buf.data(), buf.size());
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(deser.deserialize(buf.data(), buf.size()), nullptr);
}
