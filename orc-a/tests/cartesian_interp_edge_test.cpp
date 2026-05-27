// Edge-case tests for quaternion/Cartesian interpolation math.
//
// These tests target the following bug items:
//   C-1  divide-by-zero when consecutive orientations are identical
//   H-1  missing hemisphere alignment before slerp/log relative to quat0
//   H-2  psi_imag_dot instability for small phi
//   M-8  stale pose0_ when flag_correct_startpose rewrites first pose
//   M-10 orientation error singular / wrong-sign near 180 degrees

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "orc/OrcTypes.h"
#include "orc/interpolator/cartesian/CartesianPoseInterpolator.h"
#include "orc/util/quatutil.h"

namespace {
orc::PoseVector make_pose(const orc::Vec3D& p, const orc::Quatd& q_in) {
    orc::Quatd q = q_in;
    q.normalize();
    orc::PoseVector pose;
    pose.template block<3, 1>(0, 0) = p;
    pose[3] = q.w();
    pose[4] = q.x();
    pose[5] = q.y();
    pose[6] = q.z();
    return pose;
}
}  // namespace

// C-1: two identical consecutive orientations must not produce NaN in the
// psi spline output at an intermediate time.
TEST(CartesianInterpEdge, IdenticalConsecutiveOrientationsFinite) {
    orc::Quatd q = orc::Quatd::Identity();
    std::vector<orc::Time> tp = {orc::Time(0.0), orc::Time(0.5), orc::Time(1.0)};
    std::vector<orc::PoseVector> pv = {make_pose(orc::Vec3D(0, 0, 0), q),
                                       make_pose(orc::Vec3D(0.5, 0, 0), q),
                                       make_pose(orc::Vec3D(1.0, 0, 0), q)};
    orc::interpolator::CartesianPoseInterpolator interp(tp, pv);
    interp.init();

    for (double t : {0.0, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0}) {
        interp.update(orc::Time(t));
        orc::PoseVector out = interp.get_pose_d();
        orc::CartesianVector v = interp.get_x_dot_d();
        for (int i = 0; i < 7; ++i)
            ASSERT_TRUE(std::isfinite(out[i])) << "pose[" << i << "] at t=" << t;
        for (int i = 0; i < 6; ++i)
            ASSERT_TRUE(std::isfinite(v[i])) << "vel[" << i << "] at t=" << t;
    }
}

// H-1: pose1 given on the opposite hemisphere from pose0 should be handled
// gracefully (short-way interpolation, finite output).
TEST(CartesianInterpEdge, HemisphereFlipHandled) {
    // q and -q represent the same rotation. Provide pose1 with negated coeffs.
    orc::Quatd q0 = orc::Quatd(Eigen::AngleAxisd(0.2, orc::Vec3D::UnitZ()));
    orc::Quatd q1_true = orc::Quatd(Eigen::AngleAxisd(0.3, orc::Vec3D::UnitZ()));
    orc::Quatd q1_flipped(-q1_true.w(), -q1_true.x(), -q1_true.y(), -q1_true.z());

    orc::PoseVector pose0 = make_pose(orc::Vec3D::Zero(), q0);
    orc::PoseVector pose1 = make_pose(orc::Vec3D(1, 0, 0), q1_flipped);

    orc::interpolator::CartesianPoseInterpolator interp(pose0, pose1, orc::Time(0.0),
                                                        orc::Time(1.0));
    interp.init();
    // Sample at many non-knot times; every interpolated orientation must be
    // on the short-way arc from q0 to q1_true (total sweep ~0.1 rad).
    for (double t : {0.0, 0.1, 0.2, 0.25, 0.3, 0.4, 0.5, 0.6, 0.7, 0.75, 0.8, 0.9, 1.0}) {
        interp.update(orc::Time(t));
        orc::PoseVector out = interp.get_pose_d();
        for (int i = 0; i < 7; ++i)
            ASSERT_TRUE(std::isfinite(out[i])) << "non-finite at t=" << t;
        orc::Quatd q_out(out[3], out[4], out[5], out[6]);
        orc::Quatd dq = q0.conjugate() * q_out;
        double ang = 2.0 * std::atan2(dq.vec().norm(), std::abs(dq.w()));
        EXPECT_LT(ang, 0.5) << "long-way interpolation detected at t=" << t << "; ang=" << ang;
    }
}

// H-2: small but nonzero psi_imag (small phi) at start should yield a finite
// psi_imag_dot. With a nonzero initial angular velocity this drives the
// phi->0 branch.
TEST(CartesianInterpEdge, SmallPhiStartFiniteDerivatives) {
    // End orientation tiny rotation away from start. psi at the END knot then
    // has a very small phi (~1e-7), which drives the END branch of
    // get_psi_imag_dot through the small-phi regime.
    double eps = 1e-7;
    orc::Quatd q0 = orc::Quatd::Identity();
    orc::Quatd q_mid = orc::Quatd(Eigen::AngleAxisd(eps / 2, orc::Vec3D::UnitZ()));
    orc::Quatd q1 = orc::Quatd(Eigen::AngleAxisd(eps, orc::Vec3D::UnitZ()));

    std::vector<orc::Time> tp = {orc::Time(0.0), orc::Time(0.5), orc::Time(1.0)};
    std::vector<orc::PoseVector> pv = {make_pose(orc::Vec3D(0, 0, 0), q0),
                                       make_pose(orc::Vec3D(0.5, 0, 0), q_mid),
                                       make_pose(orc::Vec3D(1.0, 0, 0), q1)};
    orc::interpolator::CartesianPoseInterpolator interp(tp, pv);

    orc::PoseVector pose_now = pv.front();
    orc::CartesianVector x_dot_now = orc::CartesianVector::Zero();
    orc::CartesianVector x_dotdot_now = orc::CartesianVector::Zero();
    interp.init(pose_now, x_dot_now, x_dotdot_now);

    interp.update(orc::Time(0.25));
    orc::PoseVector out = interp.get_pose_d();
    orc::CartesianVector v = interp.get_x_dot_d();
    orc::CartesianVector a = interp.get_x_dotdot_d();
    for (int i = 0; i < 7; ++i)
        ASSERT_TRUE(std::isfinite(out[i])) << "pose " << i;
    for (int i = 0; i < 6; ++i) {
        ASSERT_TRUE(std::isfinite(v[i])) << "vel " << i;
        ASSERT_TRUE(std::isfinite(a[i])) << "acc " << i;
    }
}

// M-8: When init() corrects the start pose, the first sample of the spline
// must equal the corrected pose (not the original).
TEST(CartesianInterpEdge, StartPoseCorrectionAppliedConsistently) {
    orc::Quatd q0 = orc::Quatd::Identity();
    orc::Quatd q1 = orc::Quatd(Eigen::AngleAxisd(0.4, orc::Vec3D::UnitY()));
    orc::PoseVector pose0 = make_pose(orc::Vec3D(0, 0, 0), q0);
    orc::PoseVector pose1 = make_pose(orc::Vec3D(1, 0, 0), q1);

    orc::interpolator::CartesianPoseInterpolator interp(pose0, pose1, orc::Time(0.0),
                                                        orc::Time(1.0));

    // Correct start pose to something noticeably different in orientation.
    orc::Quatd q_now = orc::Quatd(Eigen::AngleAxisd(0.2, orc::Vec3D::UnitX()));
    orc::PoseVector pose_now = make_pose(orc::Vec3D(0.05, 0.02, 0.0), q_now);
    orc::CartesianVector v0 = orc::CartesianVector::Zero();
    orc::CartesianVector a0 = orc::CartesianVector::Zero();
    interp.init(pose_now, v0, a0);

    interp.update(orc::Time(0.0));
    orc::PoseVector out = interp.get_pose_d();
    // Orientation at t=0 must match pose_now (modulo sign).
    orc::Quatd q_out(out[3], out[4], out[5], out[6]);
    double d = std::abs(q_out.dot(q_now));
    EXPECT_NEAR(d, 1.0, 1e-6) << "start pose orientation not corrected";
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out[i], pose_now[i], 1e-6);
    }

    // Before the fix the middle waypoint (produced by slerp(orig_q0,q1))
    // does not reflect the corrected start; the interpolated orientation
    // at t=0.5 is therefore not between q_now and q1 on the short arc but
    // instead swings back toward orig_q0. Check that the orientation at
    // t=0.5 stays within the angular band spanned by q_now and q1.
    interp.update(orc::Time(0.5));
    orc::PoseVector out_mid = interp.get_pose_d();
    orc::Quatd q_mid(out_mid[3], out_mid[4], out_mid[5], out_mid[6]);
    auto ang_between = [](const orc::Quatd& a, const orc::Quatd& b) {
        orc::Quatd dq = a.conjugate() * b;
        return 2.0 * std::atan2(dq.vec().norm(), std::abs(dq.w()));
    };
    double total = ang_between(q_now, q1);
    double d_mid_now = ang_between(q_now, q_mid);
    double d_mid_q1 = ang_between(q_mid, q1);
    // Triangle inequality with tolerance: going via q_mid should not be
    // much longer than the direct q_now->q1 arc. Before fix the path
    // detours through orig_q0 (identity here) so d_mid_now+d_mid_q1 >> total.
    EXPECT_LT(d_mid_now + d_mid_q1, total + 0.1)
        << "trajectory detours: total=" << total << " via mid=" << (d_mid_now + d_mid_q1);
}

// M-10: Orientation error must be continuous across the 180-degree
// boundary. Verified against the free helper
// orc::control::cartesian_ct_orientation_error, which is the identical
// expression used inside CartesianCTController::update().
#include "orc/control/controller/cartesian/CartesianCTController.h"

TEST(CartesianInterpEdge, OrientationErrorMagnitudeCorrect) {
    orc::Vec3D axis = orc::Vec3D(0.3, 0.4, 0.5).normalized();
    orc::RotationMatrix R_d = orc::RotationMatrix::Identity();

    // At small angles, a correct orientation error has magnitude ~theta
    // (the 2 in 2*sign(w)*vec cancels the 1/2 from sin(theta/2)~theta/2).
    // The raw .vec() form gives only ~theta/2 (off by factor of 2), making
    // the controller effectively halve its orientation gain.
    double th_small = 0.1;
    orc::RotationMatrix R_s = Eigen::AngleAxisd(th_small, axis).toRotationMatrix();
    orc::Vec3D es = orc::control::cartesian_ct_orientation_error(R_s, R_d);
    EXPECT_NEAR(es.norm(), th_small, 1e-3)
        << "small-angle orientation error magnitude should ~= theta; got " << es.norm();

    // Near 180 deg, the corrected error magnitude approaches 2; the raw
    // .vec() form tops out at 1.
    orc::RotationMatrix R_p = Eigen::AngleAxisd(M_PI - 1e-6, axis).toRotationMatrix();
    orc::Vec3D ep = orc::control::cartesian_ct_orientation_error(R_p, R_d);
    EXPECT_NEAR(ep.norm(), 2.0, 1e-2)
        << "orientation error magnitude at ~180 deg should approach 2; got " << ep.norm();
}
