#include <gtest/gtest.h>
#include <fstream>
#include "orc/Orc.h"
#include "test_constants.h"

namespace {
using JointVector = typename orc::robots::Iiwa::JointVector;
using JointMatrix = typename orc::robots::Iiwa::JointMatrix;
using JacobianMatrix = typename orc::robots::Iiwa::JacobianMatrix;

/**
 * @brief Utility function to read matrix from .csv file
 *
 * @param file_name
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd read_csv(const std::string& file_name) {
    std::ifstream file(file_name);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + file_name);
    }

    std::vector<std::vector<double>> data;
    std::string line;
    int i = 0;
    while (std::getline(file, line)) {
        if (line.empty())
            continue;

        std::stringstream line_stream(line);
        std::string cell;
        std::vector<double> row;
        while (std::getline(line_stream, cell, ',')) {
            row.push_back(std::stod(cell));
        }

        if (!row.empty())
            data.push_back(row);
    }

    if (data.empty()) {
        throw std::runtime_error("No data found in file: " + file_name);
    }

    Eigen::MatrixXd mat(data.size(), data[0].size());
    for (int i = 0; i < data.size(); i++) {
        if (data[i].size() != data[0].size()) {
            throw std::runtime_error("Inconsistent row lengths in file: " + file_name);
        }
        mat.row(i) = Eigen::VectorXd::Map(&data[i][0], data[0].size());
    }

    return mat;
}

class IiwaTest : public testing::Test {
protected:
    orc::robots::Iiwa* iiwa;
    JointVector q_null;
    JointVector q_ones;

    void SetUp() override {
        iiwa = new orc::robots::Iiwa("../models/iiwa_hanging.mjb", 125e-6);
        q_null.setZero();
        q_ones.setOnes();
    }

    void TearDown() override { delete iiwa; }
};

/**
 * @brief Compare Mass matrix with pre-calculated one
 *
 */
TEST_F(IiwaTest, checkMassMatrixCandle) {
    iiwa->set_q_act(q_null);
    iiwa->set_q_dot_act(q_null);
    iiwa->set_q_dotdot_act(q_null);
    iiwa->update(0);

    JointMatrix M = iiwa->robot_data.M;

    // pre-calculated sizes
    Eigen::MatrixXd M_pre = read_csv("../tests/data/mass_matrix_00.csv");

    double M_norm = (M - M_pre).norm();
    EXPECT_LE(M_norm, NORM_THRESHOLD);
}

TEST_F(IiwaTest, checkBiasForceCandle) {
    iiwa->set_q_act(q_null);
    iiwa->set_q_dot_act(q_null);
    iiwa->set_q_dotdot_act(q_null);
    iiwa->update(0);

    JointVector bias_force = iiwa->robot_data.qfrc_bias;
    Eigen::MatrixXd bias_force_pre = read_csv("../tests/data/bias_force_00.csv");

    double force_norm = (bias_force - bias_force_pre).norm();
    EXPECT_LE(force_norm, NORM_THRESHOLD);
}

TEST_F(IiwaTest, CheckJacobianCandle) {
    iiwa->set_q_act(q_null);
    iiwa->set_q_dot_act(q_null);
    iiwa->set_q_dotdot_act(q_null);
    iiwa->update(0);

    JacobianMatrix J = iiwa->robot_data.J;

    Eigen::MatrixXd J_pre = read_csv("../tests/data/J_00.csv");
    double J_norm = (J - J_pre).norm();

    // TODO: Calculate J w.r.t. the same coordinate system as old model
    // EXPECT_LE(J_norm, NORM_THRESHOLD);
}

// TODO: Add tests for other configurations
}  // namespace
