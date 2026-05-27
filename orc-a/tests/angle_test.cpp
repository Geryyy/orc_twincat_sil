#include "orc/util/Angle.h"
#include <gtest/gtest.h>
#include <cmath>

namespace {
using orc::util::wrap_to_pi;

// =========================================================================
// Basic wrapping
// =========================================================================
TEST(WrapToPiTest, Zero) {
    EXPECT_DOUBLE_EQ(wrap_to_pi(0.0), 0.0);
}

TEST(WrapToPiTest, SmallPositive) {
    EXPECT_NEAR(wrap_to_pi(0.5), 0.5, 1e-12);
}

TEST(WrapToPiTest, SmallNegative) {
    EXPECT_NEAR(wrap_to_pi(-0.5), -0.5, 1e-12);
}

TEST(WrapToPiTest, PiHalf) {
    EXPECT_NEAR(wrap_to_pi(M_PI / 2), M_PI / 2, 1e-12);
}

TEST(WrapToPiTest, NegativePiHalf) {
    EXPECT_NEAR(wrap_to_pi(-M_PI / 2), -M_PI / 2, 1e-12);
}

// =========================================================================
// Boundary at ±π — this is the tricky edge case
// =========================================================================
TEST(WrapToPiTest, ExactlyPi) {
    // fmod(π + π, 2π) = fmod(2π, 2π) = 0
    // result = 0 - π = -π
    // Mathematically we'd prefer +π, but -π is often accepted.
    // The key point: the result must be in [-π, π].
    double result = wrap_to_pi(M_PI);
    EXPECT_GE(result, -M_PI);
    EXPECT_LE(result, M_PI);
    // More specifically, fmod-based implementations typically return -π here
    EXPECT_NEAR(std::abs(result), M_PI, 1e-12);
}

TEST(WrapToPiTest, ExactlyNegativePi) {
    double result = wrap_to_pi(-M_PI);
    EXPECT_GE(result, -M_PI);
    EXPECT_LE(result, M_PI);
    EXPECT_NEAR(std::abs(result), M_PI, 1e-12);
}

// =========================================================================
// Full rotations
// =========================================================================
TEST(WrapToPiTest, TwoPi) {
    // 2π should wrap to 0
    EXPECT_NEAR(wrap_to_pi(2 * M_PI), 0.0, 1e-12);
}

TEST(WrapToPiTest, NegativeTwoPi) {
    // -2π should wrap to 0
    EXPECT_NEAR(wrap_to_pi(-2 * M_PI), 0.0, 1e-12);
}

TEST(WrapToPiTest, ThreePi) {
    // 3π = π + 2π → should wrap to π (or -π)
    double result = wrap_to_pi(3 * M_PI);
    EXPECT_NEAR(std::abs(result), M_PI, 1e-12);
}

TEST(WrapToPiTest, FourPi) {
    // 4π = 0 + 2*2π → should wrap to 0
    EXPECT_NEAR(wrap_to_pi(4 * M_PI), 0.0, 1e-12);
}

TEST(WrapToPiTest, NegativeThreePi) {
    double result = wrap_to_pi(-3 * M_PI);
    EXPECT_NEAR(std::abs(result), M_PI, 1e-12);
}

// =========================================================================
// Large angles
// =========================================================================
TEST(WrapToPiTest, LargePositiveAngle) {
    double angle = 1000.0 * 2 * M_PI + 0.1;
    double result = wrap_to_pi(angle);
    EXPECT_NEAR(result, 0.1, 1e-6);  // relaxed tolerance for large angle
    EXPECT_GE(result, -M_PI);
    EXPECT_LE(result, M_PI);
}

TEST(WrapToPiTest, LargeNegativeAngle) {
    double angle = -1000.0 * 2 * M_PI - 0.1;
    double result = wrap_to_pi(angle);
    EXPECT_NEAR(result, -0.1, 1e-6);
    EXPECT_GE(result, -M_PI);
    EXPECT_LE(result, M_PI);
}

// =========================================================================
// Angles just past boundaries
// =========================================================================
TEST(WrapToPiTest, JustOverPi) {
    double angle = M_PI + 1e-10;
    double result = wrap_to_pi(angle);
    // Should wrap to approximately -π + 1e-10
    EXPECT_GE(result, -M_PI - 1e-9);
    EXPECT_LE(result, M_PI + 1e-9);
}

TEST(WrapToPiTest, JustUnderNegativePi) {
    double angle = -M_PI - 1e-10;
    double result = wrap_to_pi(angle);
    EXPECT_GE(result, -M_PI - 1e-9);
    EXPECT_LE(result, M_PI + 1e-9);
}

// =========================================================================
// Values that should stay unchanged
// =========================================================================
TEST(WrapToPiTest, ValueInRange) {
    // Values in (-π, π) should remain unchanged
    double angles[] = {-3.0, -2.0, -1.0, -0.01, 0.01, 1.0, 2.0, 3.0};
    for (double a : angles) {
        double result = wrap_to_pi(a);
        EXPECT_NEAR(result, a, 1e-12) << "Failed for angle: " << a;
    }
}

// =========================================================================
// Symmetry test: wrap_to_pi(x) and wrap_to_pi(-x)
// =========================================================================
TEST(WrapToPiTest, AntiSymmetry) {
    // For most angles: wrap_to_pi(-x) = -wrap_to_pi(x)
    double angles[] = {0.5, 1.0, 2.0, 4.0, 7.0};
    for (double a : angles) {
        double pos = wrap_to_pi(a);
        double neg = wrap_to_pi(-a);
        EXPECT_NEAR(pos + neg, 0.0, 1e-12) << "Anti-symmetry failed for: " << a;
    }
}

// =========================================================================
// Wrapping used in controller error computation
// =========================================================================
TEST(WrapToPiTest, JointErrorWrapping) {
    // In JointCTController: e = wrap_to_pi(q_act - q_d)
    // If q_act = 3.0 and q_d = -3.0, error = 6.0 which wraps to ~6-2π ≈ -0.283
    double error = wrap_to_pi(3.0 - (-3.0));
    EXPECT_NEAR(error, 6.0 - 2 * M_PI, 1e-12);
    EXPECT_GE(error, -M_PI);
    EXPECT_LE(error, M_PI);
}

// =========================================================================
// Idempotency — wrap(wrap(x)) == wrap(x) for the full ±3π range
// =========================================================================
TEST(WrapToPiTest, WrapIsIdempotent) {
    for (double x = -3.0 * M_PI; x < 3.0 * M_PI; x += 0.37) {
        double once = wrap_to_pi(x);
        double twice = wrap_to_pi(once);
        EXPECT_NEAR(once, twice, 1e-12) << "wrap not idempotent at x=" << x;
        EXPECT_GE(once, -M_PI - 1e-12);
        EXPECT_LE(once, M_PI + 1e-12);
    }
}

}  // namespace
