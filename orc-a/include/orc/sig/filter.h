#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include "orc/util/Time.h"
#include "orc/util/import_eigen.h"

/* NOTE: Filter implementation relies on Eigen Array types for elementwise operations. */

namespace orc::sig {

template <typename T>
class FirstOrderIIR {
protected:
    Time Ta_;
    T y_m;  // output y[k-1]
    T u_m;  // input u[k-1]
public:
    T a;     // output coeff. a1, with a0 = 1
    T b[2];  // input coeff.

public:
    FirstOrderIIR(T u_init, T y_init, Time Ta) : Ta_(Ta) {
        a = T::Ones();
        b[0] = T::Ones();
        b[1] = T::Ones();
        u_m = u_init;
        y_m = y_init;
    }

    T update(T u) {
        T y;

        y = (b[0] * u + b[1] * u_m - a * y_m);

        u_m = u;
        y_m = y;
        return y;
    }

    void reset(T u_reset, T y_reset) {
        u_m = u_reset;
        y_m = y_reset;
    }

    T get_time_constant(T f_c_norm) {
        T f_min_N =
            1e-6 *
            T::Ones();  // set lower threshold for normalized frequency to avoid division by 0
        auto f_c_N = T::Ones().min(f_min_N.max(f_c_norm));  // limit to 0..1
        // guard against zero/negative sampling period (division by zero)
        double Ta_sec = FirstOrderIIR<T>::Ta_.toSec();
        if (!(Ta_sec > 1e-12))
            Ta_sec = 1e-12;
        double f_max = 1.0 / (2.0 * Ta_sec);
        T T_1 = 1.0 / (2.0 * M_PI * f_c_N * f_max);
        return T_1;
    }
};

template <typename T>
class DT1 : public FirstOrderIIR<T> {
    T V_D;
    T T_R;

public:
    explicit DT1(Time Ta) : DT1(T::Zero(), T::Zero(), Ta) {}

    DT1(double f_c_norm, Time Ta)
        : DT1(T::Zero(), T::Zero(), T::Ones(), f_c_norm * T::Ones(), Ta) {}

    DT1(T f_c_norm, Time Ta) : DT1(T::Zero(), T::Zero(), T::Ones(), f_c_norm, Ta) {}

    DT1(T u_init, T y_init, Time Ta) : DT1(u_init, y_init, T::Ones(), 0.5 * T::Ones(), Ta) {}

    DT1(T u_init, T y_init, T gain, T f_c_norm, Time Ta) : FirstOrderIIR<T>(u_init, y_init, Ta) {
        T_R = FirstOrderIIR<T>::get_time_constant(f_c_norm);  // set T_R
        // clamp T_R away from zero to guard the reciprocal below
        T T_R_safe = T_R.max(1e-12 * T::Ones());
        double Ta_sec = Ta.toSec();
        if (!(Ta_sec > 1e-12))
            Ta_sec = 1e-12;
        // clamp exponent argument to avoid overflow/underflow
        T arg = (Ta_sec * T_R_safe.inverse()).min(700.0 * T::Ones()).max(-700.0 * T::Ones());
        V_D = gain * T_R_safe / Ta_sec * (T::Ones() - exp(-arg));

        // access to template class coeff via template proxy.
        // For more details see:
        // https://isocpp.org/wiki/faq/templates#nondependent-name-lookup-members
        FirstOrderIIR<T>::b[0] = V_D * T_R_safe.inverse();
        FirstOrderIIR<T>::b[1] = -1 * FirstOrderIIR<T>::b[0];
        FirstOrderIIR<T>::a = -1 * exp(-arg);
    }
};

template <typename T>
class PT1 : public FirstOrderIIR<T> {
    T V;
    T T_1;

public:
    PT1(double f_c_norm, Time Ta) : PT1(T::Zero(), T::Zero(), f_c_norm, Ta) {}

    PT1(T f_c_norm, Time Ta) : PT1(T::Zero(), T::Zero(), T::Ones(), f_c_norm, Ta) {}

    // PT1(T u_init, T y_init, double f_c_norm, Time Ta) : PT1(u_init, y_init, T::Ones(), f_c_norm*
    // T::Ones(), Ta) {}

    PT1(T u_init, T y_init, T gain, T f_c_norm, Time Ta) : FirstOrderIIR<T>(u_init, y_init, Ta) {
        V = gain;
        T_1 = FirstOrderIIR<T>::get_time_constant(f_c_norm);  // set T_1
        // clamp T_1 away from zero to guard the reciprocal
        T T_1_safe = T_1.max(1e-12 * T::Ones());
        double Ta_sec = Ta.toSec();
        if (!(Ta_sec > 1e-12))
            Ta_sec = 1e-12;
        // clamp exponent argument to avoid overflow
        T arg = (Ta_sec * T_1_safe.inverse()).min(700.0 * T::Ones()).max(-700.0 * T::Ones());
        T tmp = exp(arg);
        // access to template class coeff via template proxy.
        // For more details see:
        // https://isocpp.org/wiki/faq/templates#nondependent-name-lookup-members
        FirstOrderIIR<T>::b[0] = T::Zero();
        FirstOrderIIR<T>::b[1] = (tmp - T::Ones()) * V * tmp.inverse();
        FirstOrderIIR<T>::a = -1 * tmp.inverse();
    }
};

/**
 * @brief Causal Savitzky-Golay differentiator / smoother.
 *
 * Fits a polynomial of order @p poly_order to the last @p window samples in a
 * least-squares sense and evaluates the @p deriv_order -th derivative of that
 * polynomial at the most recent sample. Compared to cascaded first-order
 * differentiators (DT1), this estimates higher-order derivatives (e.g. joint
 * acceleration) directly from position with far less noise amplification.
 *
 * The fit weights are precomputed once in the constructor and applied as a fixed
 * FIR kernel in update(), so the per-cycle cost is a single dot product per
 * element and the real-time path performs no dynamic allocation.
 *
 * Operates elementwise on Eigen Array types, like the other filters in this file.
 */
template <typename T>
class SavitzkyGolay {
    Time Ta_;
    int N_;                      // window length [samples]
    int head_ = 0;               // ring-buffer index of the newest sample
    bool initialized_ = false;   // whether the window has been primed
    std::vector<double> coeff_;  // FIR weights; coeff_[j] multiplies the sample at lag j
    std::vector<T> buffer_;      // ring buffer of past inputs

    static double factorial(int n) {
        double f = 1.0;
        for (int i = 2; i <= n; ++i)
            f *= static_cast<double>(i);
        return f;
    }

public:
    /**
     * @param window      Number of samples in the sliding window (>= poly_order + 1)
     * @param poly_order  Order of the fitted polynomial (>= deriv_order)
     * @param deriv_order Derivative to estimate: 0 = smoothing, 1 = velocity, 2 = acceleration
     * @param Ta          Sample time in seconds
     */
    SavitzkyGolay(int window, int poly_order, int deriv_order, Time Ta) : Ta_(Ta) {
        // sanitize parameters so the least-squares problem stays well posed
        if (deriv_order < 0)
            deriv_order = 0;
        if (poly_order < deriv_order)
            poly_order = deriv_order;
        if (window < poly_order + 1)
            window = poly_order + 1;
        N_ = window;

        // guard against a zero/negative sampling period (division by zero)
        double Ta_sec = Ta_.toSec();
        if (!(Ta_sec > 1e-12))
            Ta_sec = 1e-12;

        // Build the design matrix in normalized sample units (x = -lag, newest at x = 0)
        // to keep the system well conditioned; the derivative is rescaled to physical
        // units afterwards via 1 / Ta^deriv_order.
        Eigen::MatrixXd A(N_, poly_order + 1);
        for (int j = 0; j < N_; ++j) {
            double x = -static_cast<double>(j);
            double p = 1.0;
            for (int i = 0; i <= poly_order; ++i) {
                A(j, i) = p;
                p *= x;
            }
        }

        // Pseudo-inverse G = (A^T A)^-1 A^T (shape (poly_order+1) x N) via least-squares QR.
        Eigen::MatrixXd G =
            A.colPivHouseholderQr().solve(Eigen::MatrixXd::Identity(N_, N_));

        // The deriv_order-th derivative of the fit at x = 0 is deriv_order! * c_deriv_order,
        // i.e. row deriv_order of G; convert from per-sample to per-second.
        double scale = factorial(deriv_order) / std::pow(Ta_sec, deriv_order);
        coeff_.resize(N_);
        for (int j = 0; j < N_; ++j)
            coeff_[j] = scale * G(deriv_order, j);

        T zero = T::Zero();
        buffer_.assign(N_, zero);
    }

    T update(const T& u) {
        if (!initialized_) {
            // prime the whole window with the first sample to avoid a startup spike
            buffer_.assign(N_, u);
            head_ = 0;
            initialized_ = true;
        } else {
            head_ = (head_ + 1) % N_;
            buffer_[head_] = u;
        }

        T y = T::Zero();
        for (int j = 0; j < N_; ++j) {
            int idx = (head_ - j + N_) % N_;  // lag j; j = 0 is the newest sample
            y += coeff_[j] * buffer_[idx];
        }
        return y;
    }

    void reset(const T& value) {
        buffer_.assign(N_, value);
        head_ = 0;
        initialized_ = true;
    }
};

}  // namespace orc::sig
