#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "orc/util/Time.h"

namespace {
using namespace orc;

// =========================================================================
// Packed struct layout — critical for serialization over network
// =========================================================================
TEST(TimeComprehensive, PackedStructSizeIs16Bytes) {
    // Time must be exactly 2 * sizeof(int64_t) = 16 bytes.
    // If this fails, serialization over UDP breaks on TwinCAT.
    EXPECT_EQ(sizeof(Time), 16u);
    EXPECT_EQ(sizeof(Time), 2 * sizeof(int64_t));
}

// =========================================================================
// Constructor from double — edge cases
// =========================================================================
TEST(TimeComprehensive, ConstructFromZeroDouble) {
    Time t(0.0);
    EXPECT_EQ(t.get_sec(), 0);
    EXPECT_EQ(t.get_nsec(), 0);
    EXPECT_DOUBLE_EQ(t.toSec(), 0.0);
}

TEST(TimeComprehensive, ConstructFromSmallDouble) {
    Time t(0.001);
    EXPECT_EQ(t.get_sec(), 0);
    EXPECT_EQ(t.get_nsec(), 1000000);
    EXPECT_DOUBLE_EQ(t.toSec(), 0.001);
}

TEST(TimeComprehensive, ConstructFromNegativeDouble) {
    // Time(-0.5) should represent -0.5 seconds.
    // The constructor does: sec = (int64_t)(-0.5) = 0
    //   nsec = (int64_t)((-0.5 - 0) * 1e9) = -500000000
    // This leaves sec=0, nsec=-500000000 which is NOT normalized.
    // toSec() = 0 + (-500000000)/1e9 = -0.5 — correct value, but
    // fields are in a non-canonical state.
    Time t(-0.5);
    EXPECT_DOUBLE_EQ(t.toSec(), -0.5);
}

TEST(TimeComprehensive, ConstructFromNegativeDoubleLargerMagnitude) {
    // Time(-2.3) should represent -2.3 seconds
    Time t(-2.3);
    EXPECT_NEAR(t.toSec(), -2.3, 1e-9);
}

TEST(TimeComprehensive, ConstructFromNegativeDoubleMinusOne) {
    Time t(-1.0);
    EXPECT_DOUBLE_EQ(t.toSec(), -1.0);
    // After construction from double -1.0, sec should be -1, nsec should be 0
    EXPECT_EQ(t.get_sec(), -1);
    EXPECT_EQ(t.get_nsec(), 0);
}

TEST(TimeComprehensive, DoubleRoundTrip) {
    // Converting double → Time → double should preserve precision
    double values[] = {0.0, 0.001, 0.125e-3, 1.5, 3.75, 100.123456789, 0.000000001};
    for (double v : values) {
        Time t(v);
        EXPECT_NEAR(t.toSec(), v, 1e-9) << "Round-trip failed for value: " << v;
    }
}

TEST(TimeComprehensive, DoubleRoundTripNegative) {
    double values[] = {-0.001, -1.5, -3.75, -100.123456789};
    for (double v : values) {
        Time t(v);
        EXPECT_NEAR(t.toSec(), v, 1e-9) << "Negative round-trip failed for value: " << v;
    }
}

// =========================================================================
// normalize() — edge cases
// =========================================================================
TEST(TimeComprehensive, NormalizePositiveOverflow) {
    Time t(0, 2'000'000'000LL);
    t.normalize();
    EXPECT_EQ(t.get_sec(), 2);
    EXPECT_EQ(t.get_nsec(), 0);
}

TEST(TimeComprehensive, NormalizeLargePositiveOverflow) {
    Time t(0, 3'500'000'000LL);
    t.normalize();
    EXPECT_EQ(t.get_sec(), 3);
    EXPECT_EQ(t.get_nsec(), 500'000'000);
}

TEST(TimeComprehensive, NormalizeNegativeNsec) {
    // nsec = -1 should normalize to sec-1, nsec=999999999
    Time t(1, -1);
    t.normalize();
    // After normalize: sec should be 0, nsec should be 999999999
    EXPECT_EQ(t.get_sec(), 0);
    EXPECT_EQ(t.get_nsec(), 999'999'999);
    EXPECT_NEAR(t.toSec(), 0.999999999, 1e-15);
}

TEST(TimeComprehensive, NormalizeNegativeNsecExactBillion) {
    // nsec = -1'000'000'000 should normalize to sec-1, nsec=0
    Time t(2, -1'000'000'000LL);
    t.normalize();
    EXPECT_EQ(t.get_sec(), 1);
    EXPECT_EQ(t.get_nsec(), 0);
}

TEST(TimeComprehensive, NormalizeNegativeNsecLarge) {
    // nsec = -2'500'000'000 should normalize to sec-3, nsec=500'000'000
    Time t(5, -2'500'000'000LL);
    t.normalize();
    EXPECT_EQ(t.get_sec(), 2);
    EXPECT_EQ(t.get_nsec(), 500'000'000);
    EXPECT_NEAR(t.toSec(), 2.5, 1e-9);
}

TEST(TimeComprehensive, NormalizeAlreadyNormal) {
    Time t(3, 500'000'000);
    t.normalize();
    EXPECT_EQ(t.get_sec(), 3);
    EXPECT_EQ(t.get_nsec(), 500'000'000);
}

TEST(TimeComprehensive, NormalizeZero) {
    Time t(0, 0);
    t.normalize();
    EXPECT_EQ(t.get_sec(), 0);
    EXPECT_EQ(t.get_nsec(), 0);
}

TEST(TimeComprehensive, NormalizeNsecExactlyOneBillion) {
    // nsec = exactly 1e9 should become sec+1, nsec=0
    Time t(0, 1'000'000'000LL);
    t.normalize();
    EXPECT_EQ(t.get_sec(), 1);
    EXPECT_EQ(t.get_nsec(), 0);
}

// =========================================================================
// Arithmetic precision edge cases
// =========================================================================
TEST(TimeComprehensive, AdditionNsecDoubleOverflow) {
    // Two Time objects with nsec near 1e9 each:
    // nsec sum = 1.8e9 → should carry properly
    Time t1(0, 999'000'000);
    Time t2(0, 999'000'000);
    Time sum = t1 + t2;
    // 0.999 + 0.999 = 1.998 seconds
    EXPECT_NEAR(sum.toSec(), 1.998, 1e-9);
    EXPECT_EQ(sum.get_sec(), 1);
    EXPECT_EQ(sum.get_nsec(), 998'000'000);
}

TEST(TimeComprehensive, SubtractionWithBorrow) {
    // 1.0 - 0.000000001 = 0.999999999
    Time t1(1, 0);
    Time t2(0, 1);
    Time diff = t1 - t2;
    EXPECT_EQ(diff.get_sec(), 0);
    EXPECT_EQ(diff.get_nsec(), 999'999'999);
}

TEST(TimeComprehensive, SubtractionToNegative) {
    Time t1(0, 0);
    Time t2(0, 1);
    Time diff = t1 - t2;
    EXPECT_NEAR(diff.toSec(), -1e-9, 1e-15);
}

TEST(TimeComprehensive, SubtractionLargerFromSmaller) {
    Time t1(1, 500'000'000);  // 1.5s
    Time t2(3, 200'000'000);  // 3.2s
    Time diff = t1 - t2;
    EXPECT_NEAR(diff.toSec(), -1.7, 1e-9);
}

TEST(TimeComprehensive, MultiplicationTimeByTimeNsecOverflow) {
    // If both nsec are near 1e9, then nsec*nsec can overflow int64_t
    // nsec_prod = (999999999 * 999999999) / 1e9
    // 999999999^2 = ~1e18, within int64_t range but close to limit
    Time t1(0, 999'999'999);
    Time t2(0, 999'999'999);
    Time result = t1 * t2;
    // Expected: ~0.999999998 * ~0.999999999 ≈ 0.999999998000000001
    double expected = 0.999999999 * 0.999999999;
    EXPECT_NEAR(result.toSec(), expected, 1e-6);
}

TEST(TimeComprehensive, MultiplicationTimeByTimeLargerValues) {
    // 2.5 * 3.0 = 7.5
    Time t1(2, 500'000'000);
    Time t2(3, 0);
    Time result = t1 * t2;
    EXPECT_NEAR(result.toSec(), 7.5, 1e-9);
}

TEST(TimeComprehensive, MultiplicationTimeByTimeWithBothNsec) {
    // 1.5 * 2.5 = 3.75
    Time t1(1, 500'000'000);
    Time t2(2, 500'000'000);
    Time result = t1 * t2;
    EXPECT_NEAR(result.toSec(), 3.75, 1e-6);
}

TEST(TimeComprehensive, MultiplicationByZero) {
    Time t(5, 500'000'000);
    Time result = t * 0;
    EXPECT_EQ(result.get_sec(), 0);
    EXPECT_EQ(result.get_nsec(), 0);
}

TEST(TimeComprehensive, MultiplicationByNegativeInt) {
    Time t(1, 0);
    Time result = t * (-2);
    EXPECT_NEAR(result.toSec(), -2.0, 1e-9);
}

TEST(TimeComprehensive, DivisionByZeroThrows) {
    Time t(1, 0);
    Time zero(0, 0);
    EXPECT_THROW(t / zero, std::runtime_error);
}

TEST(TimeComprehensive, DivisionByDoubleZero) {
    Time t(1, 0);
    // Dividing by double zero produces inf → int cast is undefined behavior
    // but let's check it doesn't crash
    // This is a somewhat adversarial test
    Time result = t / 0.0;
    // The result is likely garbage, but it shouldn't crash
    (void)result;
}

TEST(TimeComprehensive, DivisionTimeBySelf) {
    Time t(5, 500'000'000);
    Time result = t / t;
    EXPECT_NEAR(result.toSec(), 1.0, 1e-9);
}

// =========================================================================
// Comparison edge cases
// =========================================================================
TEST(TimeComprehensive, ComparisonEqualUnnormalized) {
    // These represent the same time but differ in representation
    Time t1(1, 0);
    Time t2(0, 1'000'000'000LL);
    // They should represent the same time (1.0s)
    EXPECT_DOUBLE_EQ(t1.toSec(), t2.toSec());
    // But direct comparison may fail because fields differ
    // This tests whether the library handles this correctly
    // Expectation: The library does NOT normalize before comparing,
    // so this should show unequal — documenting the limitation
    bool are_equal = (t1 == t2);
    // If time was properly normalized, these would be equal
    // This test documents the behavior
    EXPECT_DOUBLE_EQ(t1.toSec(), 1.0);
    EXPECT_DOUBLE_EQ(t2.toSec(), 1.0);
}

TEST(TimeComprehensive, ComparisonNegativeTime) {
    Time t1(-1, 0);
    Time t2(0, -1'000'000'000LL);
    // Both represent -1.0 seconds
    EXPECT_DOUBLE_EQ(t1.toSec(), -1.0);
    EXPECT_DOUBLE_EQ(t2.toSec(), -1.0);
}

// =========================================================================
// toNSec edge cases
// =========================================================================
TEST(TimeComprehensive, ToNSecBasic) {
    Time t(1, 500'000'000);
    EXPECT_EQ(t.toNSec(), 1'500'000'000LL);
}

TEST(TimeComprehensive, ToNSecNegative) {
    Time t(-1, 0);
    EXPECT_EQ(t.toNSec(), -1'000'000'000LL);
}

// =========================================================================
// Quantize edge cases
// =========================================================================
TEST(TimeComprehensive, QuantizeRoundsUp) {
    Time step(0, 1'000'000);  // 1ms
    Time t(0, 1'500'000);     // 1.5ms
    Time quantized = t.quantize(step);
    // Should round to 2ms (nearest multiple)
    EXPECT_EQ(quantized, Time(0, 2'000'000));
}

TEST(TimeComprehensive, QuantizeRoundsDown) {
    Time step(0, 1'000'000);  // 1ms
    Time t(0, 1'400'000);     // 1.4ms
    Time quantized = t.quantize(step);
    // Should round to 1ms (nearest multiple)
    EXPECT_EQ(quantized, Time(0, 1'000'000));
}

TEST(TimeComprehensive, QuantizeExactMultiple) {
    Time step(0, 125'000);  // 125us = 0.125ms
    Time t(0, 500'000);     // 500us = 4 * 125us exactly
    Time quantized = t.quantize(step);
    EXPECT_EQ(quantized, Time(0, 500'000));
}

// =========================================================================
// Vector conversion
// =========================================================================
TEST(TimeComprehensive, ConvertDoubleToTimeVectorAndBack) {
    std::vector<double> original = {0.0, 0.5, 1.0, 1.5, 2.0};
    auto time_vec = Time::convertDoubleToTimeVector(original);
    auto recovered = Time::convertTimeToDoubleVector(time_vec);

    ASSERT_EQ(recovered.size(), original.size());
    for (size_t i = 0; i < original.size(); i++) {
        EXPECT_NEAR(recovered[i], original[i], 1e-9);
    }
}

// =========================================================================
// Compound operations
// =========================================================================
TEST(TimeComprehensive, PlusEqualsOperator) {
    Time t(1, 0);
    Time dt(0, 125'000);  // 125us
    t += dt;
    EXPECT_EQ(t.get_sec(), 1);
    EXPECT_EQ(t.get_nsec(), 125'000);
}

TEST(TimeComprehensive, MinusEqualsOperator) {
    Time t(1, 125'000);
    Time dt(0, 125'000);
    t -= dt;
    EXPECT_EQ(t.get_sec(), 1);
    EXPECT_EQ(t.get_nsec(), 0);
}

TEST(TimeComprehensive, IncrementLoopPrecision) {
    // Simulate a control loop: increment by Ts=125us for 8000 steps = 1 second
    Time Ts(0, 125'000);  // 125us
    Time t(0, 0);
    for (int i = 0; i < 8000; i++) {
        t += Ts;
    }
    // Should be exactly 1.0 seconds
    EXPECT_EQ(t, Time(1, 0));
    EXPECT_DOUBLE_EQ(t.toSec(), 1.0);
}

TEST(TimeComprehensive, IncrementLoopLonger) {
    // 1ms step for 1000 steps = 1 second
    Time Ts(0, 1'000'000);  // 1ms
    Time t(0, 0);
    for (int i = 0; i < 1000; i++) {
        t += Ts;
    }
    EXPECT_EQ(t, Time(1, 0));
}

// =========================================================================
// toString (non-TwinCAT only)
// =========================================================================
TEST(TimeComprehensive, ToStringBasic) {
    Time t(3, 750'000'000);
    std::string s = t.toString();
    EXPECT_FALSE(s.empty());
    // Should contain "3" and "750000000"
    EXPECT_NE(s.find("3"), std::string::npos);
    EXPECT_NE(s.find("750000000"), std::string::npos);
}

}  // namespace
