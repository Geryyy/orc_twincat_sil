#include "orc/sig/filter.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
namespace {
using namespace orc;
using Arr2 = Eigen::Array<double, 2, 1>;
using Arr1 = Eigen::Array<double, 1, 1>;
// =========================================================================
// PT1 filter tests
// =========================================================================
TEST(PT1FilterTest, StepResponseConvergesToInput) {
    // A PT1 with unity gain should converge to the step input value
    double f_c_norm = 0.1;  // 10% of Nyquist
    Time Ta(0, 1'000'000);  // 1ms sample time
    sig::PT1<Arr2> filter(Arr2::Ones() * f_c_norm, Ta);
    Arr2 u_step = Arr2::Ones() * 5.0;
    Arr2 y;
    // Run for 1000 steps (1 second) — should converge
    for (int i = 0; i < 1000; i++) {
        y = filter.update(u_step);
    }
    // After many steps, output should be close to input * gain (gain=1)
    EXPECT_NEAR(y(0), 5.0, 0.01);
    EXPECT_NEAR(y(1), 5.0, 0.01);
}
TEST(PT1FilterTest, InitialOutputIsZero) {
    double f_c_norm = 0.1;
    Time Ta(0, 1'000'000);
    sig::PT1<Arr2> filter(Arr2::Ones() * f_c_norm, Ta);
    // First update with unit step — output should be near zero
    // (PT1 b[0]=0, so first output = b[1]*u_m - a*y_m with u_m=0, y_m=0)
    Arr2 y = filter.update(Arr2::Ones());
    // Output should be very small (near zero for first sample)
    EXPECT_LT(std::abs(y(0)), 1.0);
    EXPECT_LT(std::abs(y(1)), 1.0);
}
TEST(PT1FilterTest, ResetBehavior) {
    double f_c_norm = 0.1;
    Time Ta(0, 1'000'000);
    sig::PT1<Arr2> filter(Arr2::Ones() * f_c_norm, Ta);
    // Run some steps
    for (int i = 0; i < 100; i++)
        filter.update(Arr2::Ones() * 5.0);
    // Reset to zero state
    filter.reset(Arr2::Zero(), Arr2::Zero());
    // Next update with zero input should give near-zero output
    Arr2 y = filter.update(Arr2::Zero());
    EXPECT_NEAR(y(0), 0.0, 1e-12);
    EXPECT_NEAR(y(1), 0.0, 1e-12);
}
TEST(PT1FilterTest, ElementWiseIndependence) {
    // Use different cutoff frequencies per element
    Arr2 f_c_norm;
    f_c_norm << 0.05, 0.5;  // slow and fast cutoff
    Time Ta(0, 1'000'000);
    sig::PT1<Arr2> filter(Arr2::Ones() * f_c_norm, Ta);
    Arr2 u = Arr2::Ones();
    Arr2 y;
    // After 50 steps, the fast-cutoff element should have converged more
    for (int i = 0; i < 50; i++)
        y = filter.update(u);
    // Element 1 (fast cutoff=0.5) should be closer to 1.0 than element 0 (slow cutoff=0.05)
    EXPECT_GT(y(1), y(0));
}
TEST(PT1FilterTest, CoefficientAIsNegative) {
    // The feedback coefficient 'a' should be negative for a stable PT1
    // a = -1 * exp(-Ta/T1) → always negative
    double f_c_norm = 0.1;
    Time Ta(0, 1'000'000);
    sig::PT1<Arr2> filter(Arr2::Ones() * f_c_norm, Ta);
    // a is a public member
    EXPECT_LT(filter.a(0), 0.0) << "PT1 feedback coefficient 'a' should be negative (stable pole)";
    EXPECT_LT(filter.a(1), 0.0);
}
// =========================================================================
// DT1 filter tests
// =========================================================================
TEST(DT1FilterTest, ImpulseResponseDecays) {
    double f_c_norm = 0.1;
    Time Ta(0, 1'000'000);
    sig::DT1<Arr1> filter(Arr1::Ones() * f_c_norm, Ta);
    // Single impulse
    Arr1 u_impulse;
    u_impulse << 1.0;
    Arr1 y = filter.update(u_impulse);
    double peak = std::abs(y(0));
    // Follow with zeros
    Arr1 u_zero = Arr1::Zero();
    for (int i = 0; i < 100; i++) {
        y = filter.update(u_zero);
    }
    // After many zero-input steps, output should be near zero
    EXPECT_NEAR(y(0), 0.0, 1e-6);
}
TEST(DT1FilterTest, ConstantInputGivesZero) {
    // A derivative filter with constant input should converge to zero
    double f_c_norm = 0.1;
    Time Ta(0, 1'000'000);
    sig::DT1<Arr2> filter(Arr2::Ones() * f_c_norm, Ta);
    Arr2 u = Arr2::Ones() * 3.0;
    Arr2 y;
    for (int i = 0; i < 500; i++) {
        y = filter.update(u);
    }
    // Steady-state output of DT1 for constant input should be zero
    EXPECT_NEAR(y(0), 0.0, 1e-4);
    EXPECT_NEAR(y(1), 0.0, 1e-4);
}
TEST(DT1FilterTest, RampInputGivesConstant) {
    // DT1 of a ramp should converge to a constant (the slope)
    double f_c_norm = 0.1;
    Time Ta(0, 1'000'000);
    sig::DT1<Arr1> filter(Arr1::Ones() * f_c_norm, Ta);
    Arr1 y;
    double slope = 1000.0;  // ramp slope in units per second (Ta = 1ms)
    // Feed a ramp: u[k] = slope * k * Ta
    for (int k = 0; k < 2000; k++) {
        Arr1 u;
        u << slope * k * Ta.toSec();
        y = filter.update(u);
    }
    // In steady state, DT1 of ramp ≈ gain * slope
    // The gain depends on filter parameters, so just check it's roughly constant
    Arr1 y_prev = y;
    for (int k = 2000; k < 2100; k++) {
        Arr1 u;
        u << slope * k * Ta.toSec();
        y = filter.update(u);
    }
    // Output should not have changed much
    EXPECT_NEAR(y(0), y_prev(0), 1.0);
}
TEST(DT1FilterTest, ResetBehavior) {
    double f_c_norm = 0.1;
    Time Ta(0, 1'000'000);
    sig::DT1<Arr2> filter(Arr2::Ones() * f_c_norm, Ta);
    // Run some steps
    for (int i = 0; i < 50; i++)
        filter.update(Arr2::Ones() * static_cast<double>(i));
    // Reset
    filter.reset(Arr2::Zero(), Arr2::Zero());
    // Next update with zero should give zero
    Arr2 y = filter.update(Arr2::Zero());
    EXPECT_NEAR(y(0), 0.0, 1e-12);
}
TEST(DT1FilterTest, CoefficientAIsNegative) {
    double f_c_norm = 0.1;
    Time Ta(0, 1'000'000);
    sig::DT1<Arr2> filter(Arr2::Ones() * f_c_norm, Ta);
    EXPECT_LT(filter.a(0), 0.0) << "DT1 feedback coefficient 'a' should be negative (stable pole)";
}
TEST(DT1FilterTest, DT1CoefficientsSymmetric) {
    // b[0] should equal -b[1] for DT1 (derivative element)
    double f_c_norm = 0.1;
    Time Ta(0, 1'000'000);
    sig::DT1<Arr1> filter(Arr1::Ones() * f_c_norm, Ta);
    EXPECT_NEAR(filter.b[0](0), -filter.b[1](0), 1e-12)
        << "DT1 feedforward coefficients should be symmetric (b[0] = -b[1])";
}
// =========================================================================
// Filter with edge-case sample times
// =========================================================================
TEST(FilterEdgeCase, PT1WithVerySmallSampleTime) {
    // 1 microsecond sample time
    Time Ta(0, 1'000);
    double f_c_norm = 0.1;
    sig::PT1<Arr1> filter(Arr1::Ones() * f_c_norm, Ta);
    // Should not produce NaN or Inf
    Arr1 y = filter.update(Arr1::Ones());
    EXPECT_FALSE(std::isnan(y(0)));
    EXPECT_FALSE(std::isinf(y(0)));
}
TEST(FilterEdgeCase, DT1WithVerySmallSampleTime) {
    Time Ta(0, 1'000);
    double f_c_norm = 0.1;
    sig::DT1<Arr1> filter(Arr1::Ones() * f_c_norm, Ta);
    Arr1 y = filter.update(Arr1::Ones());
    EXPECT_FALSE(std::isnan(y(0)));
    EXPECT_FALSE(std::isinf(y(0)));
}
}  // namespace
