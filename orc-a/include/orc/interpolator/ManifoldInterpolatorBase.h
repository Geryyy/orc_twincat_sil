
#pragma once

#include <cmath>
#include <vector>

#include "orc/interpolator/InterpolatorBase.h"

namespace orc::interpolator {

// Interface class
template <typename Time, typename M, typename Tx>
class ManifoldInterpolatorBase : public InterpolatorBase<Time, M, Tx> {
    using BaseType = InterpolatorBase<Time, M, Tx>;

protected:
    std::vector<M> x_points_;

public:
    ManifoldInterpolatorBase(std::vector<Time>& time_points, std::vector<M>& x_points)
        : BaseType(time_points), x_points_(x_points) {
        this->x0 = x_points_.front();
        this->x1 = x_points_.back();
    }

    ManifoldInterpolatorBase(M x0, M x1, Time t0, Time t1) : BaseType(t0, t1), x_points_() {
        this->x0 = x0;
        this->x1 = x1;

        // add 3. point to create spline
        M xi = (this->x1 - this->x0) / 2 + this->x0;

        x_points_.push_back(x0);
        x_points_.push_back(xi);
        x_points_.push_back(x1);
    }

    ManifoldInterpolatorBase() : BaseType(), x_points_() {}

    // This is not a pure virtual function because it does not make sense for the
    // jerk interpolation to implement this
    virtual void init(M& x_init, Tx& x_dot_init, Tx& x_dotdot_init) = 0;

    virtual void init() = 0;

    M get_start_point() { return this->x0; }

    M get_end_point() { return this->x1; }

    M get_point() { return this->x; }

    Tx get_derivative() { return this->x_dot; }

    Tx get_second_derivative() { return this->x_dotdot; }

    Tx get_third_derivative() { return this->x_dotdotdot; }

    std::vector<M> get_trajectory_points() { return x_points_; }

protected:
    void set_point(M x_) { this->x = x_; }
    void set_derivative(Tx x_dot_) { this->x_dot = x_dot_; }
    void set_second_derivative(Tx x_dotdot_) { this->x_dotdot = x_dotdot_; }
    void set_third_derivative(Tx x_dotdotdot_) { this->x_dotdotdot = x_dotdotdot_; }

    void set_trajectory_points(std::vector<M>& x_points) {
        x_points_ = x_points;
        this->x0 = x_points_.front();
        this->x1 = x_points_.back();
    }

    void dispatch(Time t_) { this->interpolate(t_); }
};
}  // namespace orc::interpolator
