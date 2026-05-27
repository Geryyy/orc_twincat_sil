#pragma once

#include <cmath>
#include <vector>
#ifndef TC_VER
#include <stdexcept>
#endif

#include "orc/RobotTraits.h"
#include "orc/Splines"
#include "orc/interpolator/ManifoldInterpolatorBase.h"
#include "orc/util/Time.h"

namespace orc::interpolator {

template <int DOF>
class SplineJointInterpolator
    : public ManifoldInterpolatorBase<Time, typename RobotTraits<DOF>::JointVector,
                                      typename RobotTraits<DOF>::JointVector> {
    using BaseType = ManifoldInterpolatorBase<Time, typename RobotTraits<DOF>::JointVector,
                                              typename RobotTraits<DOF>::JointVector>;
    using JointVector = typename RobotTraits<DOF>::JointVector;

    static const int spline_deg = 3;
    typedef Eigen::Spline<double, DOF, spline_deg> SplineJoint_t;
    typedef Eigen::SplineFitting<SplineJoint_t> SplineJointFitting_t;

    SplineJoint_t spline_interp_;

public:
    // RT note: constructors cannot throw (TwinCAT/Codesys RT C++ forbids exceptions).
    // Callers must guarantee time_points.size() >= 2 and t0 != t1. On violation we
    // default-construct the spline; a subsequent init() will simply produce a zero
    // trajectory rather than crash the control loop.
    SplineJointInterpolator(std::vector<Time>& time_points, std::vector<JointVector>& joint_poses)
        : BaseType(time_points, joint_poses), spline_interp_() {}

    SplineJointInterpolator(JointVector q0, JointVector q1, Time t0, Time t1)
        : BaseType(q0, q1, t0, t1), spline_interp_() {}

    void init() {
        JointVector zero = JointVector::Zero();
        init(zero, zero, zero, false);
    }

    void init(JointVector& q_now, JointVector& q_dot_now, JointVector& q_dotdot_now) {
        init(q_now, q_dot_now, q_dotdot_now, true);
    }

    /**
     * @brief Same as init(q_now, q_dot_now, q_dotdot_now) but rebases the
     *        interpolator's start time to @p handoff_time so the fitted
     *        spline places @p q_now exactly at absolute time
     *        @p handoff_time (not at the segment's original first knot).
     *
     *        Knots whose original time is <= handoff_time are dropped
     *        (they are already in the past); a new first knot is
     *        prepended at (handoff_time, q_now).
     *
     *        This prevents a ~O(Ts * q_dot) discontinuity at segment
     *        hand-offs when the dispatch grid is not aligned with the
     *        segment start times (e.g. overlapping TrajectoryObjects).
     */
    void init(JointVector& q_now, JointVector& q_dot_now, JointVector& q_dotdot_now,
              Time handoff_time) {
        std::vector<Time> t_vec = this->get_time_points();
        std::vector<JointVector> knot_vec = this->get_trajectory_points();

        if (t_vec.size() != knot_vec.size() || t_vec.size() < 2) {
            // malformed trajectory — fall back to unshifted path
            init(q_now, q_dot_now, q_dotdot_now, true);
            return;
        }

        // If the requested handoff is before this segment's nominal
        // start, or after its end, fall back to the legacy behaviour.
        if (handoff_time <= t_vec.front() || handoff_time > t_vec.back()) {
            init(q_now, q_dot_now, q_dotdot_now, true);
            return;
        }

        // Drop knots strictly before handoff_time (they are in the past).
        size_t first_future = 0;
        while (first_future < t_vec.size() && t_vec[first_future] <= handoff_time)
            ++first_future;

        // Build rebased knot set: (handoff_time, q_now) + remaining future knots.
        std::vector<Time> t_new;
        std::vector<JointVector> k_new;
        t_new.reserve(t_vec.size() - first_future + 1);
        k_new.reserve(knot_vec.size() - first_future + 1);

        t_new.push_back(handoff_time);
        k_new.push_back(q_now);
        for (size_t i = first_future; i < t_vec.size(); ++i) {
            t_new.push_back(t_vec[i]);
            k_new.push_back(knot_vec[i]);
        }

        // Need at least 2 knots to fit a cubic spline meaningfully.
        if (t_new.size() < 2) {
            init(q_now, q_dot_now, q_dotdot_now, true);
            return;
        }

        // Push the rebased knots back into the base so the usual
        // get_time_points()/get_trajectory_points()/get_duration()
        // paths work consistently with this segment from now on.
        this->set_time_points(t_new);
        this->set_trajectory_points(k_new);

        // Fit the spline in the rebased frame. correct_start=true ensures
        // q_dot / q_dotdot continuity at u = 0 (= handoff_time).
        init(q_now, q_dot_now, q_dotdot_now, true);
    }

private:
    void init(JointVector& q_now, JointVector& q_dot_now, JointVector& q_dotdot_now,
              bool correct_start) {
        Time T_traj = this->get_duration();

        std::vector<Time> t_vec = this->get_time_points();
        auto joint_pose_vec = this->get_trajectory_points();

        // number of knots
        const int N = static_cast<int>(t_vec.size());

        // Zero-duration segment → singular spline fit. On Linux/host we fail
        // loudly; on RT targets (TC_VER) the constructor contract requires no
        // exceptions, so we leave the spline default-constructed and return.
        if (T_traj.toNSec() == 0 || N < 2) {
#ifndef TC_VER
            throw std::invalid_argument(
                "SplineJointInterpolator::init: zero-duration segment or <2 knots");
#else
            return;
#endif
        }

        /* prepare data for spline interpolation */
        // joint pose knots: copy std::vector to Eigen::Matrix
        Eigen::MatrixXd joint_knots(DOF, N);

        int i_start;

        if (correct_start) {
            // replace first pose with current pose for smooth transition
            joint_knots.template block<DOF, 1>(0, 0) = q_now;
            i_start = 1;
        } else {
            i_start = 0;
        }

        for (size_t i = i_start; i < joint_pose_vec.size(); i++) {
            joint_knots.template block<DOF, 1>(0, i) = joint_pose_vec[i];
        }

        // scaled time knots: u_knots..[0,1]
        Eigen::VectorXd u_knots(N, 1);

        for (int i = 0; i < N; i++) {
            u_knots[i] = static_cast<double>((t_vec[i] - this->get_start_time()) / T_traj);
        }

        /* prepare end derivatives for spline fitting:
             Derivatives must be scaled with trajectory duration due to
             u \in [0,1] and t \in [t0,t1].
             */

        // start derivatives
        Eigen::MatrixXd joints_start_derivs = Eigen::MatrixXd::Zero(DOF, 2);
        if (correct_start) {
            joints_start_derivs.col(0) = q_dot_now * T_traj.toSec();  // first derivative
            joints_start_derivs.col(1) =
                q_dotdot_now * (T_traj * T_traj).toSec();  // second derivative
        }

        // end derivatives = 0
        Eigen::MatrixXd joints_end_derivs = Eigen::MatrixXd::Zero(DOF, 2);

        // orc::log::write_debug("SplineJointInterpolator.init(): InterpolateWithEndDerivatives()");
        int64_t start_time2;
        // orc::log::tic(&start_time2);

        const auto fit_deriv =
            SplineJointFitting_t::template InterpolateWithEndDerivatives<Eigen::MatrixXd>(
                joint_knots, joints_start_derivs, joints_end_derivs, spline_deg, u_knots);

        // orc::log::toc_write(start_time2);

        SplineJoint_t fitted_spline(fit_deriv);
        spline_interp_ = fitted_spline;
    }

private:
    void interpolate(Time t_) {
        const Time T_traj = this->get_duration();
        // scale spline parameter, u..[0,1]
        double u = (t_ / T_traj).toSec();

        // call spline interpolation
        auto deriv = spline_interp_.derivatives(u, 2);
        this->set_point(deriv.template block<DOF, 1>(
            0, 0));  // col(0)..0-th derivative
                     // scale derivative due to scaled spline parameter u
        this->set_derivative(deriv.template block<DOF, 1>(0, 1) *
                             (1.0 / T_traj.toSec()));  // col(1)..1-th derivative
        this->set_second_derivative(deriv.template block<DOF, 1>(0, 2) *
                                    (1.0 / (T_traj * T_traj).toSec()));  // col(2)..2-th derivative

        // std::cout << "t: " << t_ << "T_traj: " << T_traj << ", u: " << u << std::endl;
    }
};

}  // namespace orc::interpolator
