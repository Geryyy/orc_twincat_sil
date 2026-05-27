#include "orc/util/Time.h"  // Assuming Time class is defined in Time.h
#include "gtest/gtest.h"

namespace {
using namespace orc;

TEST(TimeTest, ConvertToSeconds) {
    Time t1(5, 500000000);  // 5.5 seconds
    EXPECT_DOUBLE_EQ(t1.toSec(), 5.5);
}

TEST(TimeTest, ExplicitCastToSeconds) {
    Time t1(5, 500000000);
    double seconds = static_cast<double>(t1);
    EXPECT_DOUBLE_EQ(seconds, 5.5);
}

TEST(TimeTest, ConvertToNanoseconds) {
    Time t2(2, 300000000);  // 2.3 seconds
    EXPECT_EQ(t2.toNSec(), 2300000000);
}

TEST(TimeTest, ExplicitCastToNanoseconds) {
    Time t2(2, 300000000);
    int64_t nseconds = static_cast<int64_t>(t2);
    EXPECT_EQ(nseconds, 2300000000);
}

TEST(TimeTest, ConstructorFromDouble) {
    Time t(3.75);  // 3.75 seconds
    EXPECT_EQ(t.toNSec(), 3750000000);
}

TEST(TimeTest, UnaryNegation) {
    Time t(3, 750000000);  // 3.75 seconds
    Time neg = -t;
    EXPECT_DOUBLE_EQ(static_cast<double>(neg), -3.75);
}

TEST(TimeTest, Quantize) {
    Time Ts(0.125e-3);
    Time t(3, 750000000);  // 3.75 seconds
    Time offset(0, 00001);
    Time quantised = (t + offset).quantize(Ts);
    EXPECT_EQ(quantised, Time(3, 750000000));  // 3.75 seconds
}

TEST(Timetest, QuantizeZero) {
    Time Ts(0.125e-3);
    Time t(0, 0);  // 0 seconds
    Time quantised = (t).quantize(Ts);
    EXPECT_EQ(quantised, Time(0, 0));  // 0 seconds
}

TEST(Timetest, QuantizeByZero) {
    Time Ts(0, 0);
    Time t(1, 1);  // 0 seconds
    Time quantised = (t).quantize(Ts);
    EXPECT_EQ(quantised, Time(1, 1));  // 0 seconds
}

TEST(TimeTest, Addition) {
    Time t1(5, 500000000);  // 5.5 seconds
    Time t2(2, 300000000);  // 2.3 seconds
    Time sum = t1 + t2;
    EXPECT_DOUBLE_EQ(static_cast<double>(sum), 7.8);
}

TEST(TimeTest, AdditionWithDouble) {
    Time Ts(0.125e-3);
    Time t1(5, 500000000);  // 5.5 seconds
    double t2 = 2.3;
    Time sum = (t1 + t2).quantize(Ts);
    EXPECT_EQ(sum, Time(7, 800000000));  // 7.8 seconds
    Time sum2 = (t2 + t1).quantize(Ts);
    EXPECT_EQ(sum2, Time(7, 800000000));  // 7.8 seconds
}

TEST(TimeTest, Subtraction) {
    Time t1(5, 500000000);  // 5.5 seconds
    Time t2(2, 300000000);  // 2.3 seconds
    Time diff = t1 - t2;
    EXPECT_DOUBLE_EQ(static_cast<double>(diff), 3.2);
}

TEST(TimeTest, SubtractionWithDouble) {
    Time Ts(0.125e-3);
    Time t1(5, 500000000);  // 5.5 seconds
    double t2 = 2.3;
    Time diff = t1 - t2;
    EXPECT_EQ(diff, Time(3, 200000000));  // 3.2 seconds
    Time diff2 = (t2 - t1).quantize(Ts);
    EXPECT_EQ(diff2, Time(-3, -200000000));  // -3.2 seconds
}

TEST(TimeTest, MultiplicationTime) {
    Time t1(5, 500000000);  // 5.5 seconds
    Time t2(2, 000000000);  // 2.3 seconds
    Time result = t1 * t2;
    EXPECT_EQ(result, Time(11, 0));  // 11 seconds
    Time result2 = t2 * t1;
    EXPECT_EQ(result2, Time(11, 0));  // 11 seconds
}

TEST(TimeTest, MultiplicationTimeNsec) {
    Time t1(0, 100000000);  // 5.5 seconds
    Time t2(0, 100000000);  // 2.3 seconds
    Time result = t1 * t2;
    EXPECT_EQ(result, Time(0, 10000000));  // 11 seconds
}

TEST(TimeTest, Multiplication) {
    Time t(5, 500000000);  // 5.5 seconds

    Time result = t * 2;
    EXPECT_EQ(result, Time(11, 0));  // 11 seconds

    result = t * 3;
    EXPECT_EQ(result, Time(16, 500000000));  // 16.5 seconds
}

TEST(TimeTest, MultiplicationwithDouble) {
    Time t(5, 500000000);  // 5.5 seconds

    Time result = t * 2.0;
    EXPECT_EQ(result, Time(11, 0));  // 11 seconds

    result = 3.0 * t;
    EXPECT_EQ(result, Time(16, 500000000));  // 16.5 seconds
}

TEST(TimeTest, Division) {
    Time t(10, 0);  // 10 seconds

    Time result = t / 2.0;
    EXPECT_EQ(result, Time(5, 0));  // 5 seconds

    result = t / 3.0;
    EXPECT_EQ(result, Time(3, 333333333));  // 3.333 seconds
}

TEST(TimeTest, DivisionWithDouble) {
    Time t(10, 0);  // 10 seconds

    Time result = t / 2.;
    EXPECT_EQ(result, Time(5, 0));  // 5 seconds

    result = 3.0 / t;
    EXPECT_EQ(result, Time(0, 3e8));  // 3e-8 seconds}
}

TEST(TimeTest, DivisionWithDoubleSmall) {
    Time t(10, 0);  // 10 seconds

    Time result = t / 0.2;
    EXPECT_EQ(result, Time(50, 0));  // 5 seconds

    result = 0.1 / t;

    EXPECT_EQ(result, Time(0.01));  // 3e-8 seconds}
}

TEST(TimeTest, DivisionTime) {
    Time t(10, 1);  // 10 seconds
    Time t2(1, 0);

    Time result = t / t2;
    EXPECT_EQ(result, Time(10, 1));
}

TEST(TimeTest, DivisionTimeDoubleResult) {
    Time t2(10, 0);  // 10 seconds
    Time t(9, 988000000);

    Time res = t / t2;
    double result = res.toSec();

    EXPECT_NEAR(result, 9.988 / 10.0, 1e-9);
}

TEST(TimeTest, DivisionTimeSmall) {
    Time t(10, 1e8);  // 10 seconds
    Time t2(0, 1e8);

    Time result = t / t2;
    EXPECT_EQ(result, Time(101, 0));
}

TEST(TimeTest, ToSecFunction) {
    Time t(3, 750000000);  // 3.75 seconds
    EXPECT_DOUBLE_EQ(t.toSec(), 3.75);
}

TEST(TimeTest, ToNSecFunction) {
    Time t(4, 120000000);  // 4.12 seconds
    EXPECT_EQ(t.toNSec(), 4120000000);
}

TEST(TimeTest, ComparisonOperators) {
    Time t1(5, 500000000);  // 5.5 seconds
    Time t2(2, 300000000);  // 2.3 seconds
    Time t3(5, 500000000);  // Same as t1

    EXPECT_TRUE(t2 < t1);
    EXPECT_TRUE(t2 <= t1);
    EXPECT_TRUE(t1 <= t3);
    EXPECT_TRUE(t1 >= t3);
    EXPECT_TRUE(t1 > t2);
    EXPECT_FALSE(t2 > t1);
}

TEST(TimeTest, ComparisonOperatorsWithDouble) {
    Time t1(5, 500000000);  // 5.5 seconds
    double t2 = 2.3;
    double t3 = 5.5;

    EXPECT_TRUE(t2 < t1);
    EXPECT_TRUE(t2 <= t1);
    EXPECT_TRUE(t1 <= t3);
    EXPECT_TRUE(t1 >= t3);
    EXPECT_TRUE(t1 > t2);
    EXPECT_FALSE(t2 > t1);
}

TEST(TimeTest, DoubleAssignment) {
    Time Ts_default;
    Ts_default = 1e-3;  // Assigning a double value

    // Check if the assignment works correctly
    EXPECT_EQ(Ts_default.toSec(), 0.001);  // Verify if the time is set to 1 millisecond
}

TEST(TimeTest, ConvertToDoubleVector) {
    Time t1(5, 500000000);  // 5.5 seconds
    Time t2(2, 300000000);  // 2.3 seconds
    Time t3(5, 500000000);  // Same as t1

    std::vector<double> t_vec = Time::convertTimeToDoubleVector({t1, t2, t3});

    EXPECT_DOUBLE_EQ(t_vec[0], 5.5);
    EXPECT_DOUBLE_EQ(t_vec[1], 2.3);
    EXPECT_DOUBLE_EQ(t_vec[2], 5.5);
}

TEST(TimeTest, ConvertTimeToDoubleVector) {
    Time t1(5, 500000000);  // 5.5 seconds
    Time t2(2, 300000000);  // 2.3 seconds
    Time t3(5, 500000000);  // Same as t1

    std::vector<double> t_vec = Time::convertTimeToDoubleVector({t1, t2, t3});

    EXPECT_DOUBLE_EQ(t_vec[0], 5.5);
    EXPECT_DOUBLE_EQ(t_vec[1], 2.3);
    EXPECT_DOUBLE_EQ(t_vec[2], 5.5);
}

}  // namespace
