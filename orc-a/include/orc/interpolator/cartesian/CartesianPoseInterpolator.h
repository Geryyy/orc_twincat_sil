
#pragma once

#include <cmath>
#include <vector>
#include "orc/OrcTypes.h"
#include "orc/Splines"
#include "orc/control/Controller.h"
#include "orc/interpolator/ManifoldInterpolatorBase.h"
#include "orc/util/quatutil.h"

namespace orc::interpolator {
class CartesianPoseInterpolator : public ManifoldInterpolatorBase<Time, typename orc::PoseVector,
                                                                  typename orc::CartesianVector> {
    using BaseType =
        ManifoldInterpolatorBase<Time, typename orc::PoseVector, typename orc::CartesianVector>;

    static const int spline_deg = 6;
    typedef Eigen::Spline<double, 3, spline_deg> SplineTranslation_t;
    typedef Eigen::SplineFitting<SplineTranslation_t> SplineTranslationFitting_t;
    typedef Eigen::Spline<double, 3, spline_deg> SplinePsi_t;
    typedef Eigen::SplineFitting<SplinePsi_t> SplinePsiFitting_t;

    SplineTranslation_t spline_transl_;
    SplinePsi_t spline_psi_;

public:
    CartesianPoseInterpolator(std::vector<Time>& time_points, std::vector<PoseVector>& pose_vec)
        : BaseType(time_points, pose_vec), spline_transl_(), spline_psi_() {}

    CartesianPoseInterpolator(PoseVector pose0, PoseVector pose1, Time t0, Time t1)
        : BaseType(), spline_transl_(), spline_psi_() {
        Time ti = (t1 - t0) / 2.0 + t0;
        std::vector<Time> time_points = {t0, ti, t1};

        // position
        Vec3D pos0 = pose0.template block<3, 1>(0, 0);
        Vec3D pos1 = pose1.template block<3, 1>(0, 0);
        Vec3D posi = (pos1 - pos0) / 2 + pos0;

        // orientation
        Quatd quat0(pose0[3], pose0[4], pose0[5], pose0[6]);  // w,x,y,z
        Quatd quat1(pose1[3], pose1[4], pose1[5], pose1[6]);
        // Enforce the shortest-arc hemisphere so downstream log() in
        // init() does not pick up a long-way rotation.
        if (quat0.dot(quat1) < 0.0) {
            quat1.coeffs() = -quat1.coeffs();
            pose1[3] = quat1.w();
            pose1[4] = quat1.x();
            pose1[5] = quat1.y();
            pose1[6] = quat1.z();
        }
        Quatd quati = quat0.slerp(0.5, quat1);

        PoseVector posei = orc::util::set_pose(posi, quati);
        std::vector<PoseVector> pose_vec = {pose0, posei, pose1};

        this->set_time_points(time_points);
        this->set_trajectory_points(pose_vec);
    }

    void init(PoseVector& pose_now, CartesianVector& x_dot_now, CartesianVector& x_dotdot_now) {
        /* update start of trajectory (set pose and derivative) for smooth transition, set end
         * derivative to zero */
        init(pose_now, x_dot_now, x_dotdot_now, true);
    }

    void init() {
        /* no corrections to trajectory*/
        PoseVector pose_now = PoseVector::Zero();
        CartesianVector x_dot_now = CartesianVector::Zero();
        CartesianVector x_dotdot_now = CartesianVector::Zero();
        init(pose_now, x_dot_now, x_dotdot_now, false);
    }

private:
    void init(PoseVector& pose_now, CartesianVector& x_dot_now, CartesianVector& x_dotdot_now,
              bool correct_start) {
        bool flag_correct_startpose = correct_start;
        bool flag_correct_start_derivs = correct_start;
        bool flag_correct_end_derivs = true;

        Time T_traj = this->get_duration();

        std::vector<Time> t_vec = this->get_time_points();
        auto pose_vec = this->get_trajectory_points();

        if (flag_correct_startpose) {
            // replace first pose entry with pose_now (last pose setpoint)
            pose_vec.erase(pose_vec.begin());
            pose_vec.insert(pose_vec.begin(), pose_now);

            // Re-slerp the middle waypoint for the 3-knot 2-pose
            // constructor case so the intermediate orientation is
            // consistent with the corrected start pose.
            if (pose_vec.size() == 3) {
                Quatd q_start(pose_vec.front()[3], pose_vec.front()[4], pose_vec.front()[5],
                              pose_vec.front()[6]);
                Quatd q_end(pose_vec.back()[3], pose_vec.back()[4], pose_vec.back()[5],
                            pose_vec.back()[6]);
                if (q_start.dot(q_end) < 0.0)
                    q_end.coeffs() = -q_end.coeffs();
                Quatd q_mid = q_start.slerp(0.5, q_end);
                pose_vec[1][3] = q_mid.w();
                pose_vec[1][4] = q_mid.x();
                pose_vec[1][5] = q_mid.y();
                pose_vec[1][6] = q_mid.z();
                // position midpoint
                Vec3D p_start = pose_vec.front().template block<3, 1>(0, 0);
                Vec3D p_end = pose_vec.back().template block<3, 1>(0, 0);
                pose_vec[1].template block<3, 1>(0, 0) = 0.5 * (p_start + p_end);
            }

            // Keep the base-class start/end points consistent with the
            // corrected trajectory so interpolate() reconstructs poses
            // against the corrected q_0.
            this->set_trajectory_points(pose_vec);
        }

        PoseVector pose0_ = pose_vec.front();
        Quatd quat0 = Quatd(pose0_[3], pose0_[4], pose0_[5], pose0_[6]);
        const int N = static_cast<int>(t_vec.size());

        /*** prepare translation spline ***/

        Eigen::MatrixXd transl_knots(3, N);

        for (int i = 0; i < static_cast<int>(pose_vec.size()); i++) {
            // extract x,y,z from pose
            transl_knots.template block<3, 1>(0, i) = pose_vec[i].template block<3, 1>(0, 0);
        }

        // scaled time knots: u_knots..[0,1]
        Eigen::VectorXd u_knots(N, 1);

        for (int i = 0; i < N; i++) {
            u_knots[i] = static_cast<double>((t_vec[i] - this->get_start_time()) / T_traj);
        }

        Eigen::MatrixXd transl_start_derivs = Eigen::MatrixXd::Zero(3, 2);
        Eigen::MatrixXd transl_end_derivs = Eigen::MatrixXd::Zero(3, 2);

        // x_dot_now: x_dot, y_dot, z_dot, omega_x, omega_y, omega_z

        if (flag_correct_start_derivs) {
            transl_start_derivs.template block<3, 1>(0, 0) =
                x_dot_now.template block<3, 1>(0, 0) * T_traj.toSec();
            transl_start_derivs.template block<3, 1>(0, 1) =
                x_dotdot_now.template block<3, 1>(0, 0) * (T_traj * T_traj).toSec();
        }

        const auto fit_transl_deriv =
            SplineTranslationFitting_t::InterpolateWithEndDerivatives<Eigen::MatrixXd>(
                transl_knots, transl_start_derivs, transl_end_derivs, spline_deg, u_knots);

        // create interpolation spline and store it as unique_ptr
        SplineTranslation_t fitted_transl_spline(fit_transl_deriv);
        spline_transl_ = fitted_transl_spline;

        /*** prepare spline for psi = log(quat) ***/

        Eigen::MatrixXd psi_imag(3, N);
        Eigen::VectorXd theta(N);
        Eigen::MatrixXd n(3, N);

        for (int l = 0; l < static_cast<int>(pose_vec.size()); l++) {
            // // extract quaternion from x,y,z,qw,qx,qy,qz and get axis and angle
            auto pose_l = pose_vec[l];
            Quatd quat_l = Quatd(pose_l[3], pose_l[4], pose_l[5], pose_l[6]);  // w,x,y,z
            // Align to the same hemisphere as quat0 so the log below
            // takes the short-way rotation.
            if (quat0.dot(quat_l) < 0.0) {
                quat_l.coeffs() = -quat_l.coeffs();
            }
            Quatd quat = quat0.conjugate() * quat_l;
            auto Psi = orc::util::quaternion_log(quat);
            Vec3D psi_ = Psi.vec();
            auto theta_ = 2 * psi_.norm();
            auto n_ = Psi.vec();
            auto psi_norm = Psi.norm();

            if (psi_norm > 1e-12) {
                n_ = Psi.vec() / psi_norm;
            } else {
                // arbitrary but valid fallback
                n_ = Vec3D(1, 0, 0);
            }

            theta(l) = theta_;
            n.template block<3, 1>(0, l) = n_;
            psi_imag.template block<3, 1>(0, l) = psi_;  // theta(l) / 2 * n_;
        }

        Eigen::Matrix<double, 3, 2> rot_start_derivs;
        Eigen::Matrix<double, 3, 2> rot_end_derivs;

        /* derivatives at trajectory start */
        rot_start_derivs.template block<3, 1>(0, 0) =
            x_dot_now.template block<3, 1>(3, 0) * T_traj.toSec();  // omega
        rot_start_derivs.template block<3, 1>(0, 1) =
            x_dotdot_now.template block<3, 1>(3, 0) * (T_traj * T_traj).toSec();  // omega_dot

        /* set end derivatives for spline interpolation */
        Eigen::MatrixXd psi_start_derivs(3, 2);

        Vec3D psi_imag_dot = get_psi_imag_dot(rot_start_derivs.template block<3, 1>(0, 0),
                                              get_orientation(pose_vec[0]),
                                              psi_imag.template block<3, 1>(0, 0), quat0);
        Vec3D psi_imag_dotdot = get_psi_imag_dotdot(
            rot_start_derivs.template block<3, 1>(0, 0),
            rot_start_derivs.template block<3, 1>(0, 1), get_orientation(pose_vec[0]),
            psi_imag.template block<3, 1>(0, 0), psi_imag_dot, quat0);

        psi_start_derivs.template block<3, 1>(0, 0) = psi_imag_dot;
        psi_start_derivs.template block<3, 1>(0, 1) = psi_imag_dotdot;

        /* derivatives at trajectory end */
        rot_end_derivs.template block<3, 1>(0, 0) = Eigen::Matrix<double, 3, 1>::Zero();  // omega
        rot_end_derivs.template block<3, 1>(0, 1) =
            Eigen::Matrix<double, 3, 1>::Zero();  // omega_dot

        Eigen::MatrixXd psi_end_derivs(3, 2);

        Vec3D psi_imag_dot_end =
            get_psi_imag_dot(rot_end_derivs.template block<3, 1>(0, 0),
                             get_orientation(pose_vec[pose_vec.size() - 1]),
                             psi_imag.template block<3, 1>(0, psi_imag.cols() - 1), quat0);
        Vec3D psi_imag_dotdot_end = get_psi_imag_dotdot(
            rot_end_derivs.template block<3, 1>(0, 0), rot_end_derivs.template block<3, 1>(0, 1),
            get_orientation(pose_vec[pose_vec.size() - 1]),
            psi_imag.template block<3, 1>(0, psi_imag.cols() - 1), psi_imag_dot_end, quat0);

        psi_end_derivs.template block<3, 1>(0, 0) = psi_imag_dot_end;
        psi_end_derivs.template block<3, 1>(0, 1) = psi_imag_dotdot_end;

        const auto fit_psi_deriv =
            SplinePsiFitting_t::InterpolateWithEndDerivatives<Eigen::MatrixXd>(
                psi_imag, psi_start_derivs, psi_end_derivs, spline_deg, u_knots);

        // create interpolation spline and store it as unique_ptr
        SplinePsi_t fitted_psi_spline(fit_psi_deriv);
        spline_psi_ = fitted_psi_spline;
    }

private:
    void interpolate(Time t_) {
        orc::PoseVector pose_d_;
        orc::CartesianVector x_dot_d_, x_dotdot_d_;

        const Time T_traj = this->get_duration();
        // scale spline parameter, u..[0,1]
        double u = static_cast<double>(t_ / T_traj);

        /*** translation ***/
        auto deriv = spline_transl_.derivatives(u, 2);
        auto p_d = deriv.template block<3, 1>(0, 0);         // col(0)..0-th derivative
        auto p_dot_d = deriv.template block<3, 1>(0, 1);     // col(1)..1-th derivative
        auto p_dotdot_d = deriv.template block<3, 1>(0, 2);  // col(2)..2-th derivative

        pose_d_.template block<3, 1>(0, 0) = p_d;
        x_dot_d_.template block<3, 1>(0, 0) = p_dot_d / T_traj.toSec();
        x_dotdot_d_.template block<3, 1>(0, 0) = p_dotdot_d / (T_traj * T_traj).toSec();

        /*** rotation ***/
        auto deriv_psi = spline_psi_.derivatives(u, 2);

        Vec3D psi = deriv_psi.template block<3, 1>(0, 0);         // col(0)..0-th derivative
        Vec3D psi_dot = deriv_psi.template block<3, 1>(0, 1);     // col(1)..1-th derivative
        Vec3D psi_dotdot = deriv_psi.template block<3, 1>(0, 2);  // col(2)..2-th derivative

        /* 0th deriv */
        Quatd q_psi(0, psi[0], psi[1], psi[2]);
        Quatd exp_psi = orc::util::quaternion_exp(q_psi);
        Quatd q_0 = get_orientation(this->get_start_point());
        Quatd q_sigma = q_0 * exp_psi;

        double phi = q_psi.norm();

        Quatd exp_dot_psi;
        Quatd exp_dotdot_psi;

        if (phi != 0) {
            /* 1st deriv */
            double a = sinc(phi);
            double b = (cos(phi) - sinc(phi)) / (phi * phi);
            double w1 = -a * psi.dot(psi_dot);
            Vec3D vec1 = a * psi_dot + b * psi.dot(psi_dot) * psi;
            exp_dot_psi = Quatd(w1, vec1[0], vec1[1], vec1[2]);

            /* 2nd deriv */
            double c = (-sin(phi) - 2 * (cos(phi) / phi)) / (phi * phi * phi) -
                       (cos(phi) - 3 * sinc(phi)) / (phi * phi * phi * phi);
            double psi_prod_psi_dot = psi.dot(psi_dot);
            double w2 = -(psi_dot.dot(psi_dot) + psi.dot(psi_dotdot)) -
                        psi_prod_psi_dot * b * psi_prod_psi_dot;
            Vec3D vec2 = b * psi_prod_psi_dot * psi_dot + a * psi_dotdot +
                         c * psi_prod_psi_dot * psi_prod_psi_dot * psi +
                         b * ((psi_dot.dot(psi_dot) + psi.dot(psi_dotdot)) * psi +
                              psi_prod_psi_dot * psi_dot);
            exp_dotdot_psi = Quatd(w2, vec2[0], vec2[1], vec2[2]);
        } else {
            exp_dot_psi = Quatd(0, psi_dot[0], psi_dot[1], psi_dot[2]);
            exp_dotdot_psi = Quatd(0, psi_dotdot[0], psi_dotdot[1], psi_dotdot[2]);
        }

        /* 1st deriv */
        Quatd q_dot = q_0 * exp_dot_psi;
        Quatd q_omega(2 * (q_dot * q_sigma.conjugate()).coeffs());

        /* 2nd deriv */
        Quatd q_dotdot = q_0 * exp_dotdot_psi;
        Quatd q_omega_dot(2 * (q_dotdot * q_sigma.conjugate()).coeffs() -
                          (q_omega * q_dot * q_sigma.conjugate()).coeffs());

        pose_d_[3] = q_sigma.w();
        pose_d_.template block<3, 1>(4, 0) = q_sigma.vec();  // pose0_.template block<4,1>(3,0);
        x_dot_d_.template block<3, 1>(3, 0) = q_omega.vec() / T_traj.toSec();  // Vec3D::Zero();
        x_dotdot_d_.template block<3, 1>(3, 0) =
            q_omega_dot.vec() / (T_traj * T_traj).toSec();  // Vec3D::Zero();

        // prime interpolator with current trajectory point
        this->set_point(pose_d_);
        this->set_derivative(x_dot_d_);
        this->set_second_derivative(x_dotdot_d_);
    }

    double sinc(double phi) {
        double sinc_phi = 0;

        if (phi < 1e-16) {
            /* if phi is near zero use taylor approximation */
            sinc_phi = 1 - (phi * phi) / 6 + (phi * phi * phi * phi) / 120 -
                       (phi * phi * phi * phi * phi * phi) / 5040;  // etc..
        } else {
            /* normal case */
            sinc_phi = sin(phi) / phi;
        }
        return sinc_phi;
    }

    Quatd get_orientation(PoseVector pose) { return Quatd(pose[3], pose[4], pose[5], pose[6]); }

    Vec3D get_psi_imag_dot(Vec3D omega, Quatd q_orient, Vec3D psi, Quatd q_0) {
        Quatd q_omega(0, omega[0], omega[1], omega[2]);
        Quatd q_orient_dot(0.5 * (q_omega * q_orient).coeffs());

        double phi = psi.norm();

        if (phi != 0) {
            Quatd exp_dot_psi = q_0.conjugate() * q_orient_dot;

            double a = sinc(phi);
            double b = (cos(phi) - sinc(phi)) / (phi * phi);

            double psi_prod_psi_dot = exp_dot_psi.w() / (-a);
            Vec3D psi_dot_imag = (exp_dot_psi.vec() - b * psi_prod_psi_dot * psi) / a;
            return psi_dot_imag;
        } else {
            // special case if theta == 0
            Vec3D psi_dot_imag_ = (q_0.conjugate() * q_orient_dot).vec();  // (eq. 29)
            return psi_dot_imag_;
        }
    }

    Vec3D get_psi_imag_dotdot(Vec3D omega, Vec3D omega_dot, Quatd q_orient, Vec3D psi,
                              Vec3D psi_dot, Quatd q_0) {
        double phi = psi.norm();

        Quatd q_omega(0, omega[0], omega[1], omega[2]);
        Quatd q_omega_dot(0, omega_dot[0], omega_dot[1], omega_dot[2]);
        Quatd q_orient_dot(0.5 * (q_omega * q_orient).coeffs());  // (eq. 22)
        Quatd q_orient_dotdot(
            0.5 * ((q_omega_dot * q_orient).coeffs() + (q_omega * q_orient_dot).coeffs()));

        if (phi != 0) {
            Quatd exp_dotdot_psi = q_0.conjugate() * q_orient_dotdot;

            double a = sinc(phi);
            double b = (cos(phi) - sinc(phi)) / (phi * phi);
            double c = (-sin(phi) - 2 * (cos(phi) / phi)) / (phi * phi * phi) -
                       (cos(phi) - 3 * sinc(phi)) / (phi * phi * phi * phi);

            double psi_prod_psi_dot = psi.dot(psi_dot);
            double psi_prod_psi_dotdot =
                -1 * (exp_dotdot_psi.w() + b * psi_prod_psi_dot * psi_prod_psi_dot +
                      psi_dot.dot(psi_dot));
            Vec3D psi_dotdot_imag = (exp_dotdot_psi.vec() - b * psi_prod_psi_dot * psi_dot -
                                     c * psi_prod_psi_dot * psi_prod_psi_dot * psi -
                                     b * ((psi_dot.dot(psi_dot) + psi_prod_psi_dotdot) * psi +
                                          psi_prod_psi_dot * psi_dot)) /
                                    a;

            return psi_dotdot_imag;
        } else {
            Vec3D psi_dotdot_imag = (q_0.conjugate() * q_orient_dotdot).vec();
            return psi_dotdot_imag;
        }
    }

public:
    PoseVector get_pose_d() { return this->get_point(); }

    CartesianVector get_x_dot_d() { return this->get_derivative(); }

    CartesianVector get_x_dotdot_d() { return this->get_second_derivative(); }

    CartesianVector get_x_dotdotdot_d() { return this->get_third_derivative(); }

    std::vector<PoseVector> get_pose_vector() { return this->get_trajectory_points(); }
};

}  // namespace orc::interpolator
