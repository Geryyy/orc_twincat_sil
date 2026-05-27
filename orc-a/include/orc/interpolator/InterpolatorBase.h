#pragma once

#include <cmath>
#include <string>
#include <vector>

namespace orc::interpolator {

// Interface class
template <typename Time, typename M, typename Tx>
class InterpolatorBase {
protected:
    std::vector<Time> time_points_;

    Time t0, t1;
    M x0, x1;
    M x;
    Tx x_dot, x_dotdot, x_dotdotdot;
    bool trajectory_execution = false;
    bool debug_out1_flag = false;
    bool debug_out2_flag = false;

public:
    explicit InterpolatorBase(std::vector<Time>& time_points) : time_points_(time_points) {
        t0 = time_points_.front();
        t1 = time_points_.back();
    }

    InterpolatorBase(Time t0, Time t1) : time_points_(), t0(t0), t1(t1) {
        // add 3. point to create spline
        Time ti = (t1 - t0) / 2.0 + t0;

        time_points_.push_back(t0);
        time_points_.push_back(ti);
        time_points_.push_back(t1);
    }

    InterpolatorBase() : time_points_() {}

    virtual ~InterpolatorBase() = default;

    virtual void init() = 0;

    void update(Time t) {
        // orc::log::write_debug("InterpolatorBase current t in update: " + std::to_string(t) + ",
        // with traj t0 = " + std::to_string(t0) + ", t0 - t = " + std::to_string(t0 - t));
        if (t0 > t) {
            if (!debug_out1_flag) {
                orc::log::write_debug("InterpolatorBase t < t0");
                debug_out1_flag = true;
            }
            // trajectory has not started yet
            // typically not used
            x = this->get_start_point();
            x_dot = Tx::Zero();
            x_dotdot = Tx::Zero();

            trajectory_execution = false;
        } else if (t > t1) {
            if (!debug_out2_flag) {
                orc::log::write_debug("InterpolatorBase t > t1");
                debug_out2_flag = true;
            }
            // trajectory has finished
            x = this->get_end_point();
            x_dot = Tx::Zero();
            x_dotdot = Tx::Zero();

            trajectory_execution = false;
        } else {
            Time t_ = t - t0;

            dispatch(t_);

            trajectory_execution = true;
            debug_out1_flag = false;
            debug_out2_flag = false;
        }
    }

    bool is_trajectory_executing() { return trajectory_execution; }

    Time get_start_time() { return t0; }

    Time get_end_time() { return t1; }

    Time get_duration() { return t1 - t0; }

    virtual M get_start_point() = 0;

    virtual M get_end_point() = 0;

    std::vector<Time> get_time_points() { return time_points_; }

protected:
    // t_ .. time since trajectory start
    virtual void dispatch(Time t_) = 0;

    virtual void interpolate(Time t_) = 0;

    void set_time_points(std::vector<Time>& time_points) {
        time_points_ = time_points;
        t0 = time_points_.front();
        t1 = time_points_.back();
    }
};
}  // namespace orc::interpolator
