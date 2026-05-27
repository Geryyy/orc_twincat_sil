// Shared test helpers: CSV loader + trajectory fixture template.
//
// Design goals:
//  - header-only, no dependency on robot models / MuJoCo (keeps it usable
//    from every *_test.cpp including ones that run in CI without MuJoCo),
//  - drop-in replacement for the ad-hoc read_csv() copies in
//    interpolator_test.cpp and iiwa_test.cpp.
//
// An IiwaTestFixture is intentionally NOT provided here: loading an .mjb is
// environment-dependent (model path differs between `ctest` runs and the
// packaged wheel), and not every CI lane has MuJoCo. The per-test factory
// helpers in iiwa_test.cpp already encapsulate that.
#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <orc/RobotTraits.h>
#include <orc/util/Time.h>

namespace orc::testing {

/// Load a numeric CSV into an Eigen::MatrixXd.
/// - Skips a single header line if the first line is non-numeric.
/// - Rejects empty files and inconsistent row widths.
inline Eigen::MatrixXd load_csv(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open())
        throw std::runtime_error("load_csv: cannot open '" + path + "'");

    std::vector<std::vector<double>> rows;
    std::string line;
    bool first = true;
    while (std::getline(file, line)) {
        if (line.empty())
            continue;

        // Detect header: if the first line's first token fails to parse as
        // a number, treat it as a header and skip. Otherwise use the row.
        if (first) {
            first = false;
            try {
                size_t idx = 0;
                (void)std::stod(line, &idx);
                // parsed ok -> use this row
            } catch (...) {
                continue;  // header
            }
        }

        std::stringstream ls(line);
        std::string cell;
        std::vector<double> r;
        while (std::getline(ls, cell, ','))
            r.push_back(std::stod(cell));
        if (!r.empty())
            rows.push_back(std::move(r));
    }

    if (rows.empty())
        throw std::runtime_error("load_csv: empty data in '" + path + "'");

    const auto cols = rows.front().size();
    Eigen::MatrixXd mat(rows.size(), cols);
    for (size_t i = 0; i < rows.size(); ++i) {
        if (rows[i].size() != cols)
            throw std::runtime_error("load_csv: ragged rows in '" + path + "'");
        mat.row(i) = Eigen::VectorXd::Map(rows[i].data(), cols);
    }
    return mat;
}

/// Generic trajectory / interpolator test fixture: builds an N-point linear
/// ramp from zero to one over [0, t_end] in the given vector type.
/// Works for any Eigen column vector type.
template <typename VectorT>
class TrajectoryFixture : public ::testing::Test {
protected:
    std::vector<VectorT> points;
    std::vector<orc::Time> times;

    /// Populate `points` and `times` with `n` samples along a linear ramp
    /// from VectorT::Zero() to VectorT::Ones() between t0 and t1.
    void make_linear_ramp(int n, double t0 = 0.0, double t1 = 1.0) {
        points.clear();
        times.clear();
        points.reserve(n);
        times.reserve(n);
        for (int i = 0; i < n; ++i) {
            const double a = static_cast<double>(i) / static_cast<double>(n - 1);
            points.push_back(VectorT::Zero() + a * (VectorT::Ones() - VectorT::Zero()));
            times.emplace_back(t0 + a * (t1 - t0));
        }
    }
};

}  // namespace orc::testing
