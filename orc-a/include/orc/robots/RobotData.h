#pragma once

#include <memory>

#include "orc/util/import_mujoco.h"

#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/util/Time.h"

namespace orc::robots {
/**
 * @brief Struct, which is managed by orc::robots::Robot and provided to
 * controllers for effort calculations.
 *
 * @tparam DOF Degrees of freedom
 */
template <int DOF>
struct RobotData {
    DECLARE_ROBOT_TRAITS_USINGS(DOF)
    using Time = orc::Time;
    using PoseVector = orc::PoseVector;

    // Set by Robot constructor
    Time t = 0;
    mjModel* model = nullptr;
    mjData* data = nullptr;
    Time Ta = 0; /**< Robot step time */

    // Set by user
    // -------------------------------------------------------------------
    Eigen::Map<JointVector> q_act;      // Maps to mjData's d->qpos
    Eigen::Map<JointVector> q_dot_act;  // Maps to mjData's d->qvel
    Eigen::Map<JointVector> qfrc_bias;  // Maps to mjData's d->qfrc_bias
    JointVector q_dotdot_act = JointVector::Zero();
    JointVector tau_motor = JointVector::Zero();
    JointVector tau_sens = JointVector::Zero();
    // -------------------------------------------------------------------

    // Set by Robot constructor
    // -------------------------------------------------------------------
    int8_t endeffector_site_id = -1;
    // -------------------------------------------------------------------

    // Set by Robot::register_SingularPeturbationController()
    // -------------------------------------------------------------------
    JointMatrix M_off = JointMatrix::Zero();
    // The singular peturbation controller modifies the systems offset mass matrix and should be
    // used in all other controllers if necessary. If there is no singular peturbation controller
    // registered, then M_off remains zero.
    // -------------------------------------------------------------------

    // Set by trajectories
    // -------------------------------------------------------------------
    JointVector q_d = JointVector::Zero();
    JointVector q_dot_d = JointVector::Zero();
    JointVector q_dotdot_d = JointVector::Zero();
    JointVector tau_ff = JointVector::Zero(); /**< Feedforward torque from trajectory */
    PoseVector pose_d = PoseVector::Zero();
    CartesianVector x_dot_d = CartesianVector::Zero();
    CartesianVector x_dotdot_d = CartesianVector::Zero();
    JointVector q_d_NS = JointVector::Zero();
    double force_d = 0.;
    // -------------------------------------------------------------------

    // Set by Robot::compute_robot_data()
    // -------------------------------------------------------------------
    JointMatrix M = JointMatrix::Zero();
    JacobianMatrix J = JacobianMatrix::Zero();
    JacobianMatrix J_dot = JacobianMatrix::Zero();
    JacobianInverseMatrix J_inv = JacobianInverseMatrix::Zero();
    HomogeneousTransformation H_0_e = HomogeneousTransformation::Identity();
    PoseVector pose_act = PoseVector::Zero();
    CartesianVector x_dot_act = CartesianVector::Zero();
    CartesianVector x_dotdot_act = CartesianVector::Zero();
    JointVector G = JointVector::Zero(); /**< Gravity vector */
    bool collision_detected = false;     /**< Collision detection flag */
    // -------------------------------------------------------------------

    // Set by Robot::update()
    // -------------------------------------------------------------------
    JointVector tau_primary =
        JointVector::Zero(); /**< Joint effort calculated by primary controller, i.e. JointTracking,
                                PoseTracking, or GravityCompensation */
    // -------------------------------------------------------------------

    // Measurement section
    // -------------------------------------------------------------------
    orc::Vec3D force_measurement = Vec3D::Zero();
    int8_t force_sensor_id = -1;
    Vec3D force_compensated = Vec3D::Zero(); /**< measured force with compensation for tool mass */
    double force_error_integral = 0.0;
    // -------------------------------------------------------------------

    RobotData(mjModel* model, mjData* data, Time Ta)
        : model(model),
          data(data),
          Ta(Ta),
          q_act(data->qpos),
          q_dot_act(data->qvel),
          qfrc_bias(data->qfrc_bias) {}
};
}  // namespace orc::robots
