#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include <orc/sig/filter.h>
#include <orc/util/Time.h>

namespace {
using Time = orc::Time;
using Arr = Eigen::Array<double, 3, 1>;

// M-15: extreme Ta / f_c must not produce NaN/Inf coefficients.
TEST(FilterEdge, PT1ExtremeTaNoNaN) {
    Arr fc = 0.5 * Arr::Ones();
    // Very tiny sampling period
    orc::sig::PT1<Arr> f(fc, Time(1e-15));
    EXPECT_TRUE(f.a.isFinite().all());
    EXPECT_TRUE(f.b[0].isFinite().all());
    EXPECT_TRUE(f.b[1].isFinite().all());
}

TEST(FilterEdge, DT1ExtremeTaNoNaN) {
    Arr fc = 0.5 * Arr::Ones();
    orc::sig::DT1<Arr> f(fc, Time(1e-15));
    EXPECT_TRUE(f.a.isFinite().all());
    EXPECT_TRUE(f.b[0].isFinite().all());
    EXPECT_TRUE(f.b[1].isFinite().all());
}

TEST(FilterEdge, PT1ZeroTaNoCrash) {
    Arr fc = 0.5 * Arr::Ones();
    EXPECT_NO_THROW({
        orc::sig::PT1<Arr> f(fc, Time(0, 0));
        EXPECT_TRUE(f.a.isFinite().all());
    });
}
}  // namespace
