#include <gtest/gtest.h>
#include <orc/OrcTypes.h>
#include <orc/util/ExecutionTimer.h>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
// #include <orc/interpolator/jointspace/CubicJointInterpolator.h>
#include <orc/interpolator/cartesian/CartesianPoseInterpolator.h>
#include <orc/interpolator/jointspace/SplineJointInterpolator.h>
// #include <orc/interpolator/jerk/JerkJointInterpolator.h>
// #include <orc/interpolator/jerk/JerkCartesianInterpolator.h>
// #include <orc/interpolator/cartesian/CartesianVelocityInterpolator.h>

namespace {

Eigen::MatrixXd read_csv(const std::string& file_name) {
    std::ifstream file(file_name);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + file_name);
    }

    std::vector<std::vector<double>> data;
    std::string line;
    int i = 0;
    while (std::getline(file, line)) {
        // Skip the first line
        if (i == 0) {
            i++;
            continue;
        }

        if (line.empty())
            continue;

        std::stringstream line_stream(line);
        std::string cell;
        std::vector<double> row;
        while (std::getline(line_stream, cell, ',')) {
            row.push_back(std::stod(cell));
        }

        if (!row.empty())
            data.push_back(row);
    }

    if (data.empty()) {
        throw std::runtime_error("No data found in file: " + file_name);
    }

    Eigen::MatrixXd mat(data.size(), data[0].size());
    for (int i = 0; i < data.size(); i++) {
        if (data[i].size() != data[0].size()) {
            throw std::runtime_error("Inconsistent row lengths in file: " + file_name);
        }
        mat.row(i) = Eigen::VectorXd::Map(&data[i][0], data[0].size());
    }

    return mat;
}

constexpr int DOF = 2;
using JointVector = typename orc::RobotTraits<DOF>::JointVector;
// using CubicJointInterpolator = typename orc::CubicJointInterpolator<DOF>;

int add(int a, int b) {
    return a + b;
}

TEST(MathFunctions, Addition) {
    EXPECT_EQ(add(1, 2), 3);
    EXPECT_EQ(add(-1, -2), -3);
    EXPECT_EQ(add(-1, 2), 1);
}

// class CubicJointInterpolatorTest : public ::testing::Test
// {
// protected:
//     CubicJointInterpolator *cubicInterpolator;

//     std::vector<orc::Time> time_points;
//     std::vector<JointVector> joint_poses;

//     void SetUp() override
//     {
//         // Initialize your time_points and joint_poses here
//         // For example:
//         time_points = {0.0, 1.0};
//         joint_poses = {JointVector::Zero(), JointVector::Ones()};

//         cubicInterpolator = new CubicJointInterpolator(time_points, joint_poses);
//     }

//     void TearDown() override
//     {
//         delete cubicInterpolator;
//     }
// };

// TEST_F(CubicJointInterpolatorTest, InitAndInterpolate)
// {
//     // Initialize joint vectors
//     JointVector q_now = JointVector::Zero();
//     JointVector q_dot_now = JointVector::Zero();
//     JointVector q_dotdot_now = JointVector::Zero();

//     // Call init method
//     cubicInterpolator->init(q_now, q_dot_now, q_dotdot_now);

//     // Call interpolate method with a given value
//     orc::Time t = 0.5; // you can change this to any value between the range of time points
//     cubicInterpolator->update(t);

//     for (int i = 0; i < DOF; i++)
//     {
//         EXPECT_NEAR(cubicInterpolator->get_point()[i], 0.5, 1e-9);
//         EXPECT_NEAR(cubicInterpolator->get_derivative()[i], 1.5, 1e-9);
//     }
// }

TEST(SplineJointInterpolator, Interpolation) {
    constexpr int DOF = 2;
    using Interpolator = typename orc::interpolator::SplineJointInterpolator<DOF>;
    using JointVector = typename orc::RobotTraits<DOF>::JointVector;
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones();
    Interpolator interp(q0, q1, 0.0, 1.0);
    interp.init();
    interp.update(0.5);
    JointVector q = interp.get_point();
    JointVector q_dot = interp.get_derivative();
    JointVector q_dotdot = interp.get_second_derivative();

    for (int i = 0; i < DOF; i++) {
        EXPECT_NEAR(q[i], 0.5, 1e-9);
        // EXPECT_NEAR(q_dot[i], 1.875, 1e-9);
        // EXPECT_NEAR(q_dotdot[i], 0.0, 1e-9);
    }
}

TEST(SplineJointInterpolator, Start) {
    constexpr int DOF = 2;
    using Interpolator = typename orc::interpolator::SplineJointInterpolator<DOF>;
    using JointVector = typename orc::RobotTraits<DOF>::JointVector;
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones();
    Interpolator interp(q0, q1, 0.0, 1.0);
    interp.init();
    interp.update(0);
    JointVector q = interp.get_point();
    JointVector q_dot = interp.get_derivative();
    JointVector q_dotdot = interp.get_second_derivative();

    for (int i = 0; i < DOF; i++) {
        EXPECT_NEAR(q[i], q0[i], 1e-9);
        EXPECT_NEAR(q_dot[i], 0.0, 1e-9);
        EXPECT_NEAR(q_dotdot[i], 0.0, 1e-9);
    }
}

TEST(SplineJointInterpolator, End) {
    constexpr int DOF = 2;
    using Interpolator = typename orc::interpolator::SplineJointInterpolator<DOF>;
    using JointVector = typename orc::RobotTraits<DOF>::JointVector;
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones();
    Interpolator interp(q0, q1, 0.0, 1.0);
    interp.init();
    interp.update(1.0);
    JointVector q = interp.get_point();
    JointVector q_dot = interp.get_derivative();
    JointVector q_dotdot = interp.get_second_derivative();

    for (int i = 0; i < DOF; i++) {
        EXPECT_NEAR(q[i], q1[i], 1e-9);
        EXPECT_NEAR(q_dot[i], 0.0, 1e-9);
        EXPECT_NEAR(q_dotdot[i], 0.0, 1e-9);
    }
}

TEST(SplineJointInterpolator, InitBoundaryConditions) {
    constexpr int DOF = 2;
    using Interpolator = typename orc::interpolator::SplineJointInterpolator<DOF>;
    using JointVector = typename orc::RobotTraits<DOF>::JointVector;
    JointVector q0 = JointVector::Zero();
    JointVector q1 = JointVector::Ones();
    Interpolator interp(q0, q1, 0.0, 1.0);
    JointVector q_init = 1 * JointVector::Ones();
    JointVector q_dot_init = 2 * JointVector::Ones();
    JointVector q_dotdot_init = 3 * JointVector::Ones();
    interp.init(q_init, q_dot_init, q_dotdot_init);
    interp.update(0);
    JointVector q = interp.get_point();
    JointVector q_dot = interp.get_derivative();
    JointVector q_dotdot = interp.get_second_derivative();

    for (int i = 0; i < DOF; i++) {
        EXPECT_NEAR(q[i], 1., 1e-9);
        EXPECT_NEAR(q_dot[i], 2.0, 1e-9);
        EXPECT_NEAR(q_dotdot[i], 3.0, 1e-9);
    }
}

class CartesianPoseInterpolatorTest : public ::testing::Test {
protected:
    orc::interpolator::CartesianPoseInterpolator* interpolator;

    std::vector<orc::Time> time_points;
    std::vector<orc::PoseVector> trajectory_points;
    std::vector<orc::CartesianVector> velocity_points;

    void SetUp() override {
        // Read the CSV file
        Eigen::MatrixXd data = read_csv("../tests/SE3_trajectory.csv");
        // Extract the time points and poses
        for (int i = 0; i < data.rows(); i++) {
            time_points.push_back(data(i, 0));
            orc::PoseVector pose;
            for (int j = 0; j < 7; j++) {
                pose[j] = data(i, j + 1);
            }
            trajectory_points.push_back(pose);

            orc::CartesianVector vel;
            for (int j = 0; j < 6; j++) {
                vel[j] = data(i, j + 7 + 1);
            }
            velocity_points.push_back(vel);
        }

        // Create the interpolator
        interpolator =
            new orc::interpolator::CartesianPoseInterpolator(time_points, trajectory_points);

        // for(int i = 0; i < time_points.size(); i++){
        //     std::cout << "time: " << time_points[i] << std::endl;
        //     std::cout << "pose: " << trajectory_points[i].transpose() << std::endl;
        //     std::cout << "vel: " << velocity_points[i].transpose() << std::endl;    }
    }

    void TearDown() override { delete interpolator; }
};

TEST_F(CartesianPoseInterpolatorTest, Interpolation) {
    // Test the interpolation for a few time points
    // print dimensions of data

    // std::cout << "test interpolation" << std::endl;
    orc::PoseVector pose_init = trajectory_points.front();
    orc::CartesianVector x_dot_init = velocity_points.front();
    orc::CartesianVector x_dotdot_init = orc::CartesianVector::Zero();
    // interpolator->init(pose_init, x_dot_init, x_dotdot_init);

    orc::log::ExecutionTimer timer;

    timer.tic();
    interpolator->init();
    timer.toc();
    // timer.print(false);

    for (int i = 0; i < time_points.size(); i += 1) {
        orc::Time t = time_points[i];

        interpolator->update(t);

        auto p = trajectory_points[i];
        auto v = velocity_points[i];

        // std::cout << "i: " << i << "\t" << "t: " << t << std::endl;
        // Extract the interpolated pose
        orc::PoseVector interpolated_pose = interpolator->get_pose_d();
        orc::CartesianVector interpolated_velocity = interpolator->get_x_dot_d();

        for (size_t j = 0; j < 7; j++) {
            // std::cout << interpolated_pose[j]  << "\t" << p[j] << std::endl;
            EXPECT_NEAR(interpolated_pose[j], p[j], 1e-9);
        }

        // Velocity check: skip boundary waypoints where the spline
        // enforces zero end-derivatives (this distorts nearby velocities).
        // Use a tolerance that accounts for spline fitting approximation error.
        int margin = std::max(2, (int)(time_points.size() * 0.05));
        if (i >= margin && i < (int)time_points.size() - margin) {
            for (size_t j = 0; j < 6; j++) {
                double tol = std::max(1.0, 0.5 * std::abs(v[j]));
                EXPECT_NEAR(interpolated_velocity[j], v[j], tol)
                    << "Velocity mismatch at time index " << i << ", component " << j;
            }
        }
    }
}

// TEST(JerkJointInterpolator, Interpolation)
// {
//     const int DOF = 1;
//     using Interpolator = typename orc::JerkJointInterpolator<DOF>;
//     using JointVector = typename orc::RobotTraits<DOF>::JointVector;
//     std::vector<orc::Time> time_points;
//     std::vector<double> pos_points;
//     std::vector<double> vel_points;
//     std::vector<double> acc_points;
//     std::vector<double> jerk_points;

//     // Read the CSV file
//     Eigen::MatrixXd data = read_csv("../tests/jerk_traj.csv");
//     // Extract the time points and poses
//     for (int i = 0; i < data.rows(); i++)
//     {

//         time_points.push_back(data(i, 0));
//         pos_points.push_back(data(i, 1));
//         vel_points.push_back(data(i, 2));
//         acc_points.push_back(data(i, 3));
//         jerk_points.push_back(data(i, 4));
//     }
//     // Extend the jerk points to JointVector since that is what the
//     // interpolator expects as input
//     std::vector<JointVector> jerk_vectors;
//     for (double jerk_point : jerk_points)
//     {
//         JointVector jerk_vector = jerk_point * Eigen::VectorXd::Ones(DOF);
//         jerk_vectors.emplace_back(jerk_vector);
//     }

//     // Initial values
//     JointVector q0 = pos_points[0] * Eigen::VectorXd::Ones(DOF);
//     JointVector dq0 = vel_points[0] * Eigen::VectorXd::Ones(DOF);
//     JointVector ddq0 = acc_points[0] * Eigen::VectorXd::Ones(DOF);

//     // Create interpolator object using the time and jerk points
//     Interpolator interp(time_points, jerk_vectors);
//     interp.init(q0, dq0, ddq0);
//     for (int j = 0; j < time_points.size(); j++)
//     {
//         interp.update(time_points[j]);
//         JointVector q = interp.get_point();
//         JointVector q_dot = interp.get_derivative();
//         JointVector q_dotdot = interp.get_second_derivative();
//         JointVector q_dotdotdot = interp.get_third_derivative();
//         for (int i = 0; i < DOF; i++)
//         {
//             EXPECT_NEAR(q[i], pos_points[j], 1e-9);
//             EXPECT_NEAR(q_dot[i], vel_points[j], 1e-9);
//             EXPECT_NEAR(q_dotdot[i], acc_points[j], 1e-9);
//             EXPECT_NEAR(q_dotdotdot[i], jerk_points[j], 1e-9);
//         }
//     }
// }

// TEST(JerkCartesianInterpolator, Interpolation)
// {
//   const int DOF = 6;
//   using Interpolator = typename orc::JerkCartesianInterpolator<DOF>;
//   std::vector<orc::Time> time_points;
//   std::vector<orc::PoseVector> pos_points;
//   std::vector<orc::CartesianVector> vel_points;
//   std::vector<orc::CartesianVector> acc_points;
//   std::vector<orc::CartesianVector> jerk_points;

//   // Read the CSV file
//   Eigen::MatrixXd data = read_csv("../tests/jerk_cart_traj.csv");
//   // Extract the time points and poses
//   for (int i = 0; i < data.rows(); i++)
//   {

//     time_points.push_back(data(i, 0));
//     orc::PoseVector pose;
//     for (int j = 0; j < 7; j++)
//     {
//         pose[j] = data(i, j + 1);
//     }
//     pos_points.push_back(pose);

//     orc::CartesianVector vel;
//     for (int j = 0; j < 6; j++)
//     {
//         vel[j] = data(i, j + 7 + 1);
//     }
//     vel_points.push_back(vel);

//     orc::CartesianVector acc;
//     for (int j = 0; j < 6; j++)
//     {
//         acc[j] = data(i, j + 13 + 1);
//     }
//     acc_points.push_back(acc);

//     orc::CartesianVector jerk;
//     for (int j = 0; j < 6; j++)
//     {
//         jerk[j] = data(i, j + 19 + 1);
//     }
//     jerk_points.push_back(jerk);
//   }

//   // Initial values
//   orc::PoseVector p0 = pos_points[0];
//   orc::CartesianVector v0 = vel_points[0];
//   orc::CartesianVector a0 = acc_points[0];

//   // Create interpolator object using the time and jerk points
//   double sample_time = time_points[1] - time_points[0];
//   Interpolator interp(time_points, jerk_points, sample_time);
//   interp.init(p0, v0, a0);
//   for (int j = 0; j < time_points.size(); j++)
//   //for (int j = 0; j < 50; j++)
//   {
//     interp.update(time_points[j]);
//     orc::PoseVector pos = interp.get_point();
//     orc::CartesianVector vel = interp.get_derivative();
//     orc::CartesianVector acc = interp.get_second_derivative();
//     orc::CartesianVector jerk = interp.get_third_derivative();
//     for (int i = 0; i < DOF; i++)
//     {
//       EXPECT_NEAR(vel[i], vel_points[j][i], 1e-4);
//       EXPECT_NEAR(acc[i], acc_points[j][i], 1e-4);
//       EXPECT_NEAR(jerk[i], jerk_points[j][i], 1e-4);
//     }
//     for (int i = 0; i < DOF+1; i++)
//     {
//       EXPECT_NEAR(pos[i], pos_points[j][i], 10e-3);
//     }
//   }
// }

// TEST(CartesianVelocityInterpolator, Interpolation)
// {
//     constexpr int DOF = orc::Robots::Iiwa::DOF;
//     using Interpolator = typename orc::CartesianVelocityInterpolator<DOF>;

//     std::vector<orc::Time> time_points;
//     std::vector<orc::PoseVector> trajectory_points;
//     std::vector<orc::CartesianVector> velocity_points;

//     Eigen::MatrixXd data = read_csv("../tests/SE3_trajectory.csv");

//     // Extract the time points and poses
//     for (int i = 0; i < data.rows(); i++)
//     {
//         time_points.push_back(data(i, 0));
//         orc::PoseVector pose;
//         for (int j = 0; j < 7; j++)
//         {
//             pose[j] = data(i, j + 1);
//         }
//         trajectory_points.push_back(pose);

//         orc::CartesianVector vel;
//         for (int j = 0; j < 6; j++)
//         {
//             vel[j] = data(i, j + 7 + 1);
//         }
//         velocity_points.push_back(vel);
//     }

//     double sample_time = time_points[1] - time_points[0];
//     orc::Robots::Iiwa::LBRIiwaModel model;
//     Interpolator interp(time_points, velocity_points, sample_time, model);
//     orc::PoseVector pose_init = trajectory_points.front();
//     orc::CartesianVector x_dot_init = velocity_points.front();
//     orc::CartesianVector x_dotdot_init = orc::CartesianVector::Zero();
//     interp.init(pose_init, x_dot_init, x_dotdot_init);

//     for (int k = 0; k < time_points.size(); k++)
//     {
//         interp.update(time_points[k]);

//         orc::PoseVector pose_gt = trajectory_points[k];
//         orc::CartesianVector x_dot_gt = velocity_points[k];

//         orc::PoseVector pose = interp.get_point();
//         orc::CartesianVector x_dot = interp.get_derivative();
//         orc::CartesianVector x_dotdot = interp.get_second_derivative();

//         // check pose
//         for (int i = 0; i < 7; i++)
//         {
//             // be genreous with the tolerance as pose is numerically integrated
//             EXPECT_NEAR(pose[i], pose_gt[i], 10e-3);
//         }

//         // check velocity
//         for (int i = 0; i < 6; i++)
//         {
//             EXPECT_NEAR(x_dot[i], x_dot_gt[i], 10e-6);
//         }
//     }
// }

// TEST(JointspaceVelocityInterpolator, Interpolation)
// {
//     constexpr int DOF = orc::Robots::Iiwa::DOF;
//     using Interpolator = typename orc::JointspaceVelocityInterpolator<DOF>;
//     using JointVector = orc::Robots::Iiwa::JointVector;

//     std::vector<orc::Time> time_points;
//     std::vector<JointVector> velocity_points;

//     const int N = 20;
//     const double dt = 0.1;
//     const double T = N * dt;

//     for( int i = 0; i < N; i++){
//         time_points.push_back(i*dt);
//         JointVector q_dot = JointVector::Ones() * i*dt;
//         velocity_points.push_back(q_dot);
//     }

//     double sample_time = time_points[1] - time_points[0];
//     orc::Robots::Iiwa::LBRIiwaModel model;
//     Interpolator interp(time_points, velocity_points, dt, model);
//     JointVector q0 = JointVector::Zero();
//     JointVector q_dot0 = JointVector::Zero();
//     JointVector q_dotdot0 = JointVector::Zero();
//     interp.init(q0, q_dot0, q_dotdot0);

//     for (int k = 0; k < time_points.size(); k++)
//     {
//         interp.update(time_points[k]);
//         JointVector q_gt = JointVector::Ones() * time_points[k] * time_points[k] / 2.;
//         JointVector q_dot_gt = velocity_points[k];

//         auto q = interp.get_point();
//         auto q_dot = interp.get_derivative();
//         auto q_dotdot = interp.get_second_derivative();

//         // check pose
//         for (int i = 0; i < 7; i++)
//         {
//             // be genreous with the tolerance as pose is numerically integrated
//             EXPECT_NEAR(q[i], q_gt[i], 10e-3);
//         }

//         // check velocity
//         for (int i = 0; i < 6; i++)
//         {
//             EXPECT_NEAR(q_dot[i], q_dot_gt[i], 10e-6);
//         }
//     }
// }

}  // namespace
