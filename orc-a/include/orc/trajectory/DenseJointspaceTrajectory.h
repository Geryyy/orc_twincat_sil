#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "orc/RobotTraits.h"
#include "orc/control/Controller.h"
#include "orc/robots/RobotData.h"
#include "orc/trajectory/TrajectoryBase.h"
#include "orc/trajectory/TrajectoryPointStorage.h"
#include "orc/trajectory/TrajectoryType.h"
#include "orc/util/Logger.h"

namespace orc::trajectory {
/**
 * @brief Dense trajectory point containing position, velocity, acceleration and feedforward torque
 *
 * @tparam DOF Degrees of freedom
 */
template <int DOF>
struct DenseJointPoint {
    using JointVector = typename RobotTraits<DOF>::JointVector;

    JointVector q;
    JointVector q_dot;
    JointVector q_dotdot;
    JointVector tau_ff;  // Feedforward torque

    DenseJointPoint()
        : q(JointVector::Zero()),
          q_dot(JointVector::Zero()),
          q_dotdot(JointVector::Zero()),
          tau_ff(JointVector::Zero()) {}

    DenseJointPoint(const JointVector& q_, const JointVector& q_dot_, const JointVector& q_dotdot_,
                    const JointVector& tau_ff_)
        : q(q_), q_dot(q_dot_), q_dotdot(q_dotdot_), tau_ff(tau_ff_) {}

    // Constructor without feedforward (backwards compatible)
    DenseJointPoint(const JointVector& q_, const JointVector& q_dot_, const JointVector& q_dotdot_)
        : q(q_), q_dot(q_dot_), q_dotdot(q_dotdot_), tau_ff(JointVector::Zero()) {}
};

/**
 * @brief Dense jointspace trajectory without interpolation
 *
 * This trajectory type stores pre-sampled points at a fixed sample rate.
 * At each update, it simply looks up the nearest point (or interpolates linearly
 * between two adjacent points). No spline fitting is performed.
 *
 * Use this for high-frequency trajectories (e.g., 8kHz) where you want to
 * execute the exact trajectory computed offline, including feedforward torques.
 *
 * @tparam DOF Degrees of freedom
 */
// TODO: Zero-copy optimization — for trajectories received via FlatBuffer,
//       consider a FlatBuffer-backed variant that holds a reference to the
//       received buffer and reads DenseJointPoint data directly from it
//       during update(). The variadic schema stores each joint vector as
//       a contiguous `[double]` block, so `Eigen::Map<const JointVector>(
//       pt->q()->data())` is a zero-copy view.
template <int DOF>
class DenseJointspaceTrajectory : public orc::trajectory::TrajectoryBase<DOF> {
    using JointVector = typename RobotTraits<DOF>::JointVector;
    using RobotData = orc::robots::RobotData<DOF>;
    using TrajectoryBase = typename orc::trajectory::TrajectoryBase<DOF>;
    using TrajectoryPointStorage = typename orc::trajectory::TrajectoryPointStorage<DOF>;
    using DensePoint = DenseJointPoint<DOF>;

    std::vector<Time> time_points_;
    std::vector<DensePoint> traj_points_;

    Time t_start_;
    Time t_end_;
    Time dt_;  // Sample time (assumed uniform)

    size_t current_index_ = 0;

    // Current state
    JointVector q_, q_dot_, q_dotdot_, tau_ff_;

public:
    /**
     * @brief Construct from time points and dense trajectory points
     *
     * @param time_points Vector of time stamps
     * @param traj_points Vector of dense trajectory points (q, q_dot, q_dotdot, tau_ff)
     */
    DenseJointspaceTrajectory(const std::vector<Time>& time_points,
                              const std::vector<DensePoint>& traj_points)
        : TrajectoryBase(), time_points_(time_points), traj_points_(traj_points) {
        this->traj_type = TrajectoryType::DENSE_JOINTSPACE;

        if (time_points_.size() >= 2) {
            t_start_ = time_points_.front();
            t_end_ = time_points_.back();
            dt_ = time_points_[1] - time_points_[0];
        } else {
            t_start_ = 0.0;
            t_end_ = 0.0;
            dt_ = 0.001;  // Default 1ms
        }
    }

    /**
     * @brief Construct from separate vectors (for easier serialization)
     *
     * @param time_points Time stamps
     * @param q_points Position points
     * @param q_dot_points Velocity points
     * @param q_dotdot_points Acceleration points
     * @param tau_ff_points Feedforward torque points
     */
    DenseJointspaceTrajectory(const std::vector<Time>& time_points,
                              const std::vector<JointVector>& q_points,
                              const std::vector<JointVector>& q_dot_points,
                              const std::vector<JointVector>& q_dotdot_points,
                              const std::vector<JointVector>& tau_ff_points)
        : TrajectoryBase(), time_points_(time_points) {
        this->traj_type = TrajectoryType::DENSE_JOINTSPACE;

        size_t n = time_points.size();
        traj_points_.reserve(n);

        for (size_t i = 0; i < n; ++i) {
            DensePoint pt;
            pt.q = (i < q_points.size()) ? q_points[i] : JointVector::Zero();
            pt.q_dot = (i < q_dot_points.size()) ? q_dot_points[i] : JointVector::Zero();
            pt.q_dotdot = (i < q_dotdot_points.size()) ? q_dotdot_points[i] : JointVector::Zero();
            pt.tau_ff = (i < tau_ff_points.size()) ? tau_ff_points[i] : JointVector::Zero();
            traj_points_.push_back(pt);
        }

        if (time_points_.size() >= 2) {
            t_start_ = time_points_.front();
            t_end_ = time_points_.back();
            dt_ = time_points_[1] - time_points_[0];
        } else {
            t_start_ = 0.0;
            t_end_ = 0.0;
            dt_ = 0.001;
        }
    }

    /**
     * @brief Update trajectory state at given time using index lookup
     *
     * Uses fast index computation for uniformly sampled trajectories.
     * Falls back to linear interpolation between nearest points.
     *
     * @param t Current time
     */
    void update(Time t) {
        if (traj_points_.empty()) {
            q_ = JointVector::Zero();
            q_dot_ = JointVector::Zero();
            q_dotdot_ = JointVector::Zero();
            tau_ff_ = JointVector::Zero();
            return;
        }

        // Clamp time to trajectory bounds
        Time t_local = t;
        if (t_local < t_start_)
            t_local = t_start_;
        if (t_local > t_end_)
            t_local = t_end_;

        // Fast index computation for uniform sampling
        double idx_float = ((t_local - t_start_) / dt_).toSec();
        size_t idx = static_cast<size_t>(idx_float);

        // Clamp index
        if (idx >= traj_points_.size() - 1) {
            idx = traj_points_.size() - 1;
            // Use last point directly
            const DensePoint& pt = traj_points_[idx];
            q_ = pt.q;
            q_dot_ = pt.q_dot;
            q_dotdot_ = pt.q_dotdot;
            tau_ff_ = pt.tau_ff;
            return;
        }

        // Linear interpolation between adjacent points
        double alpha = idx_float - static_cast<double>(idx);
        const DensePoint& pt0 = traj_points_[idx];
        const DensePoint& pt1 = traj_points_[idx + 1];

        q_ = pt0.q + alpha * (pt1.q - pt0.q);
        q_dot_ = pt0.q_dot + alpha * (pt1.q_dot - pt0.q_dot);
        q_dotdot_ = pt0.q_dotdot + alpha * (pt1.q_dotdot - pt0.q_dotdot);
        tau_ff_ = pt0.tau_ff + alpha * (pt1.tau_ff - pt0.tau_ff);

        current_index_ = idx;
    }

    /**
     * @brief Update trajectory and write to robot_data
     *
     * @param t Current time
     * @param robot_data Robot data structure to update
     */
    void update(Time t, RobotData& robot_data) {
        update(t);
        robot_data.q_d = q_;
        robot_data.q_dot_d = q_dot_;
        robot_data.q_dotdot_d = q_dotdot_;
        robot_data.tau_ff = tau_ff_;
    }

    // Getters
    JointVector get_q() const { return q_; }
    JointVector get_q_dot() const { return q_dot_; }
    JointVector get_q_dotdot() const { return q_dotdot_; }
    JointVector get_tau_ff() const { return tau_ff_; }

    Time get_start_time() override { return t_start_; }
    Time get_end_time() const { return t_end_; }
    Time get_sample_time() const { return dt_; }
    size_t get_num_points() const { return traj_points_.size(); }

    const std::vector<Time>& get_time_points() const { return time_points_; }
    const std::vector<DensePoint>& get_trajectory_points() const { return traj_points_; }

    void init(TrajectoryPointStorage saved_state) override {
        if (saved_state.previous_type == TrajectoryType::JOINTSPACE ||
            saved_state.previous_type == TrajectoryType::DENSE_JOINTSPACE) {
            // Could adjust first point to match saved state for smooth transition
            // For dense trajectories, we typically want exact execution
            q_ = saved_state.q_;
            q_dot_ = saved_state.q_dot_;
            q_dotdot_ = saved_state.q_dotdot_;
        }
        current_index_ = 0;
    }

    void init() override {
        current_index_ = 0;
        if (!traj_points_.empty()) {
            q_ = traj_points_[0].q;
            q_dot_ = traj_points_[0].q_dot;
            q_dotdot_ = traj_points_[0].q_dotdot;
            tau_ff_ = traj_points_[0].tau_ff;
        }
    }

    TrajectoryPointStorage save_state(Time time) override {
        update(time);
        TrajectoryPointStorage state(q_, q_dot_, q_dotdot_);
        state.previous_type = TrajectoryType::DENSE_JOINTSPACE;
        return state;
    }
};

}  // namespace orc::trajectory
