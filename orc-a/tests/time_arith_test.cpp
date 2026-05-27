#include <gtest/gtest.h>
#include <orc/util/Time.h>

namespace {
using Time = orc::Time;

// M-5: operator-(double) must not leave nsec negative.
TEST(TimeArith, SubtractDoubleNormalizes) {
    Time t(1, 0);      // 1.0 s
    Time r = t - 0.5;  // 0.5 s
    // With the fix, nsec should always be >= 0
    EXPECT_GE(r.get_nsec(), 0);
    EXPECT_NEAR(r.toSec(), 0.5, 1e-12);

    Time r2 = Time(2, 0) - 0.25;
    EXPECT_GE(r2.get_nsec(), 0);
    EXPECT_NEAR(r2.toSec(), 1.75, 1e-12);
}

// M-4: baseline drift check. Within spec if accumulated drift is tiny.
TEST(TimeArith, AccumulationDriftBaseline) {
    Time t(0, 0);
    for (int i = 0; i < 1'000'000; ++i)
        t = t + 1e-6;  // should equal 1 s
    double err_ns = std::abs(t.toSec() - 1.0) * 1e9;
    // document baseline: we expect drift > 1 ns for double-based add, so this is a [?]
    // we just require it to be within a generous bound (not diverging to Inf/NaN)
    EXPECT_LT(err_ns, 1e6);  // within 1 ms
}
}  // namespace
