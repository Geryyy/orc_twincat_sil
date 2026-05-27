/**
 * @file interpolator_example.cpp
 * @author anonymous
 * @brief This example shows how to use the SplineJointInterpolator. This approach can
 * easily be extended to other interpolator classes.
 * @date 2025-08-04
 *
 * @copyright Copyright (c) 2025
 *
 */
#include <iostream>
#include <tuple>
#include <vector>

#include <matplot/matplot.h>
#include <Eigen/Dense>
#include "orc/util/import_mujoco.h"

#include <orc/Orc.h>

constexpr int DoF = 7;
using Iiwa = orc::robots::Iiwa;
using Time = orc::Time;

int main() {
    // Timings
    Time Tend = Time(1, 0);
    Time dt(0, 500000);  // sample time
    Time Tstart = Time(0, 0);

    // Start and end points of trajectory
    Iiwa::JointVector q0;
    q0.setZero();
    Iiwa::JointVector q1;
    q1.setOnes();

    std::vector<double> q_vec;
    std::vector<double> qdot_vec;
    std::vector<double> qdotdot_vec;
    std::vector<double> time_vec;

    // Initialize interpolator
    orc::interpolator::SplineJointInterpolator<Iiwa::DOF> interpolator(q0, q1, Tstart, Tend);
    interpolator.init();

    for (Time t = 0; t <= Tend; t = t + dt) {
        // Get current data from interpolator and save to vectors for representation
        interpolator.update(t);
        Iiwa::JointVector q = interpolator.get_point();
        Iiwa::JointVector q_dot = interpolator.get_derivative();
        Iiwa::JointVector q_dotdot = interpolator.get_second_derivative();

        q_vec.push_back(q(0));
        qdot_vec.push_back(q_dot(0));
        qdotdot_vec.push_back(q_dotdot(0));
        time_vec.push_back(t.toSec());
    }

    // Plot the results
    using namespace matplot;
    subplot(3, 1, 0);
    plot(time_vec, q_vec);
    ylabel("q");
    subplot(3, 1, 1);
    plot(time_vec, qdot_vec);
    ylabel("qdot");
    subplot(3, 1, 2);
    plot(time_vec, qdotdot_vec);
    ylabel("qdotdot");
    xlabel("t in sec");
    sgtitle("Interpolated trajectory");
    show();

    return 0;
}
