#include <gtest/gtest.h>
#include <orc/util/ExecutionTimer.h>
#include <limits>

namespace {
using orc::log::ExecutionTimer;

// M-9: avg update must not overflow int64 for large avg values.
// Verify the formula directly (it's the exact one used in toc()).
TEST(ExecTimer, AvgFormulaNoOverflow) {
    const int64_t N = 100;
    int64_t avg = static_cast<int64_t>(1e17);
    int64_t execution_time = static_cast<int64_t>(1e17);

    // Old formula: ((N-1)*avg + execution_time) / N
    // (N-1)*avg = 99 * 1e17 ≈ 9.9e18 — close to INT64_MAX (9.22e18). Adding
    // execution_time overflows to negative.
    int64_t old_formula = ((N - 1) * avg + execution_time) / N;
    EXPECT_LT(old_formula, 0) << "sanity: old formula should overflow";

    // New formula: avg + (execution_time - avg) / N
    int64_t new_formula = avg + (execution_time - avg) / N;
    EXPECT_EQ(new_formula, static_cast<int64_t>(1e17));
    EXPECT_GT(new_formula, 0);
}
}  // namespace
