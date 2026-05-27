#include "orc/util/quatutil.h"
#include <gtest/gtest.h>
#include <cmath>
#include "orc/OrcTypes.h"
namespace {
using namespace orc;
using namespace orc::util;
TEST(QuatUtilTest, LogOfIdentity) {
    Quatd q_id(1.0, 0.0, 0.0, 0.0);
    Quatd psi = quaternion_log(q_id);
    EXPECT_NEAR(psi.vec().norm(), 0.0, 1e-12);
    // log of identity should have w=0 (pure imaginary)
    EXPECT_NEAR(psi.w(), 0.0, 1e-12);
}
TEST(QuatUtilTest, ExpOfZero) {
    Quatd q_zero(0.0, 0.0, 0.0, 0.0);
    Quatd result = quaternion_exp(q_zero);
    EXPECT_NEAR(result.w(), 1.0, 1e-12);
    EXPECT_NEAR(result.vec().norm(), 0.0, 1e-12);
}
TEST(QuatUtilTest, LogExpRoundTrip90Deg) {
    Quatd q(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
    q.normalize();
    Quatd log_q = quaternion_log(q);
    Quatd recovered = quaternion_exp(log_q);
    recovered.normalize();
    double dot = std::abs(q.coeffs().dot(recovered.coeffs()));
    EXPECT_NEAR(dot, 1.0, 1e-9);
}
TEST(QuatUtilTest, LogExpRoundTrip45Deg) {
    Quatd q(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX()));
    q.normalize();
    Quatd log_q = quaternion_log(q);
    Quatd recovered = quaternion_exp(log_q);
    recovered.normalize();
    double dot = std::abs(q.coeffs().dot(recovered.coeffs()));
    EXPECT_NEAR(dot, 1.0, 1e-9);
}
TEST(QuatUtilTest, LogExpRoundTripArbitrary) {
    Eigen::Vector3d axis = Eigen::Vector3d(1, 1, 1).normalized();
    Quatd q(Eigen::AngleAxisd(2 * M_PI / 3, axis));
    q.normalize();
    Quatd log_q = quaternion_log(q);
    Quatd recovered = quaternion_exp(log_q);
    recovered.normalize();
    double dot = std::abs(q.coeffs().dot(recovered.coeffs()));
    EXPECT_NEAR(dot, 1.0, 1e-9);
}
TEST(QuatUtilTest, LogExpRoundTripIdentity) {
    // Critical edge case: exp(log(identity)) should return identity
    Quatd q_id(1.0, 0.0, 0.0, 0.0);
    Quatd log_q = quaternion_log(q_id);
    Quatd recovered = quaternion_exp(log_q);
    recovered.normalize();
    EXPECT_NEAR(recovered.w(), 1.0, 1e-9);
    EXPECT_NEAR(recovered.vec().norm(), 0.0, 1e-9);
}
TEST(QuatUtilTest, LogSmallRotation) {
    Quatd q(Eigen::AngleAxisd(0.001, Eigen::Vector3d::UnitZ()));
    q.normalize();
    Quatd psi = quaternion_log(q);
    EXPECT_NEAR(psi.w(), 0.0, 1e-6);
    EXPECT_NEAR(psi.vec().norm(), 0.0005, 1e-6);
}
TEST(QuatUtilTest, Log180DegRotation) {
    Quatd q(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    q.normalize();
    Quatd psi = quaternion_log(q);
    EXPECT_NEAR(psi.w(), 0.0, 1e-9);
    EXPECT_NEAR(psi.vec().norm(), M_PI / 2.0, 1e-6);
}
TEST(QuatUtilTest, SetPoseLayout) {
    Vec3D pos;
    pos << 1.0, 2.0, 3.0;
    Quatd quat(0.7071, 0.0, 0.7071, 0.0);
    quat.normalize();
    PoseVector pose = set_pose(pos, quat);
    EXPECT_DOUBLE_EQ(pose[0], 1.0);
    EXPECT_DOUBLE_EQ(pose[1], 2.0);
    EXPECT_DOUBLE_EQ(pose[2], 3.0);
    EXPECT_NEAR(pose[3], quat.w(), 1e-12);
    EXPECT_NEAR(pose[4], quat.x(), 1e-12);
    EXPECT_NEAR(pose[5], quat.y(), 1e-12);
    EXPECT_NEAR(pose[6], quat.z(), 1e-12);
}
TEST(QuatUtilTest, SetPoseIdentityOrientation) {
    Vec3D pos = Vec3D::Zero();
    Quatd quat(1.0, 0.0, 0.0, 0.0);
    PoseVector pose = set_pose(pos, quat);
    EXPECT_DOUBLE_EQ(pose[3], 1.0);
    EXPECT_DOUBLE_EQ(pose[4], 0.0);
    EXPECT_DOUBLE_EQ(pose[5], 0.0);
    EXPECT_DOUBLE_EQ(pose[6], 0.0);
}
TEST(QuatUtilTest, LogNormalizesInput) {
    // quaternion_log normalizes internally
    Quatd q(2.0, 0.0, 0.0, 0.0);
    Quatd psi = quaternion_log(q);
    EXPECT_NEAR(psi.vec().norm(), 0.0, 1e-12);
}
TEST(QuatUtilTest, ExpPureImaginary) {
    double half_angle = M_PI / 4;
    Quatd q_input(0.0, 0.0, 0.0, half_angle);
    Quatd result = quaternion_exp(q_input);
    result.normalize();
    Quatd expected(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
    expected.normalize();
    double dot = std::abs(result.coeffs().dot(expected.coeffs()));
    EXPECT_NEAR(dot, 1.0, 1e-6);
}
TEST(QuatUtilTest, HemisphereMagnitudeConsistency) {
    // q and -q represent the same rotation; |log(q).vec()| must match
    // |log(-q).vec()| after hemisphere alignment (sign-flip w<0).
    const double theta = M_PI / 3.0;
    Quatd q(std::cos(theta / 2.0), std::sin(theta / 2.0), 0.0, 0.0);
    Quatd qn(-q.w(), -q.x(), -q.y(), -q.z());

    // Align hemisphere: flip if w<0.
    if (qn.w() < 0.0) {
        qn.w() = -qn.w();
        qn.x() = -qn.x();
        qn.y() = -qn.y();
        qn.z() = -qn.z();
    }

    Quatd lp = quaternion_log(q);
    Quatd ln = quaternion_log(qn);
    EXPECT_NEAR(lp.vec().norm(), ln.vec().norm(), 1e-9);
}
}  // namespace
