#pragma once

#include <cmath>
#include <memory>
#include <vector>
// #include <stdexcept>
#include <string>

#include "orc/util/import_mujoco.h"

#include "orc/util/Angle.h"

#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/com/flatbuffers/FlatBufferDeserializer.h"
#include "orc/com/flatbuffers/FlatBufferSerializer.h"
#include "orc/control/ControllerBases.h"
#include "orc/robots/RobotData.h"
#include "orc/trajectory/Trajectories.h"
#include "orc/util/import_eigen.h"

namespace orc::robots {

/**
 * @class Robot
 * @brief Implementation of a generic robot class
 *
 * This class serves as a base for specific robot implementations, providing
 * a framework for handling robot models, trajectories, and control.
 *
 * @tparam DOF
 */
template <int _DOF>
class Robot {
public:
    static constexpr int DOF = _DOF;

    DECLARE_ROBOT_TRAITS_USINGS(_DOF)

    using TrajectoryQueue = typename orc::trajectory::TrajectoryQueue<DOF>;
    using FlatBufferSerializer = typename com::fb::FlatBufferSerializer<DOF>;
    using FlatBufferDeserializer = typename com::fb::FlatBufferDeserializer<DOF>;

    using ControllerBase = orc::control::ControllerBase<DOF>;
    using JointTrackingController = orc::control::JointTrackingController<DOF>;
    using PoseTrackingController = orc::control::PoseTrackingController<DOF>;
    using JointCTController = orc::control::JointCTController<DOF>;
    using JointCTParameter = orc::control::JointCTParameter<DOF>;
    using CartesianCTController = orc::control::CartesianCTController<DOF>;
    using CartesianCTParameter = orc::control::CartesianCTParameter<DOF>;
    using JointPDPController = orc::control::JointPDPController<DOF>;
    using JointPDPParameter = orc::control::JointPDPParameter<DOF>;
    using VelocityController = orc::control::VelocityController<DOF>;
    using GravityCompController = orc::control::GravityCompController<DOF>;
    using GravityCompParameter = orc::control::GravityCompParameter<DOF>;
    using SingularPertrubationController = orc::control::SingularPerturbationController<DOF>;
    using SingularPerturbationParameter = orc::control::SingularPerturbationParameter<DOF>;
    using FrictionCompController = orc::control::FrictionCompController<DOF>;
    using FrictionCompParameter = orc::control::FrictionCompParameter<DOF>;
    using HybridForceMotionController = orc::control::HybridForceMotionController<DOF>;

    using TrajectoryType = orc::trajectory::TrajectoryType;
    using JointspaceTrajectory = typename orc::trajectory::JointspaceTrajectory<DOF>;
    using DenseJointspaceTrajectory = typename orc::trajectory::DenseJointspaceTrajectory<DOF>;
    using NullspaceTrajectory = typename orc::trajectory::NullspaceTrajectory<DOF>;
    using JointCtrParamTrajectory = typename orc::trajectory::JointCtrParamTrajectory<DOF>;
    using CartesianCtrParamTrajectory = typename orc::trajectory::CartesianCtrParamTrajectory<DOF>;
    using TaskspaceTrajectory = typename orc::trajectory::TaskspaceTrajectory<DOF>;
    using TrajectoryBase = typename orc::trajectory::TrajectoryBase<DOF>;
    using HybridForceMotionTrajectory = typename orc::trajectory::HybridForceMotionTrajectory<DOF>;

    using RobotData = orc::robots::RobotData<DOF>;
    using Matrix3 = Eigen::Matrix<double, 3, 3>;

private:
    uint8_t model_id_; /**<  Model identifier to verify that the same robot model is being used on
                          both sides */

public:
    mjModel* model = NULL; /**< Mujoco model pointer */
    mjData* data = NULL;   /**< Mujoco data pointer */
    RobotData robot_data;

    TrajectoryQueue traj_queue;

    FlatBufferSerializer serializer_;
    FlatBufferDeserializer deserializer_;

    std::unique_ptr<JointTrackingController>
        js_controller; /**< Controller for jointspace trajectories */
    std::unique_ptr<PoseTrackingController>
        ts_controller; /**< Controller for taskspace trajectories */
    std::unique_ptr<GravityCompController>
        gc_controller;                                 /**< Controller for gravity compensation */
    std::unique_ptr<ControllerBase> custom_controller; /**< Custom controller for additional use*/
    std::vector<std::unique_ptr<ControllerBase>>
        secondary_controllers; /**< Secondary controller vector */

    JointVector tau_; /**< Joint torques computed by the controller */

    // Flags for optimizing RobotData computation
    bool is_PDPController_registered = false;
    bool is_CartesianCTController_registered = false;
    bool is_GravityCompController_registered = false;

#ifdef TC_VER
    Robot(unsigned char* model_binary, unsigned int model_size, Time Ta, CTcTrace m_Trace,
          std::string name_link_e = "", std::string name_sensor_force = "")
        : model(mj_loadModelBuffer((void*)model_binary, (unsigned int)model_size, m_Trace)),
          data(mj_makeData(model)),
          robot_data(model, data, Ta) {
        robot_data.endeffector_site_id = mj_name2id(model, mjOBJ_SITE, name_link_e.c_str());
        robot_data.force_sensor_id = mj_name2id(model, mjOBJ_SENSOR, name_sensor_force.c_str());
        model_id_ = model->names[0];
    }
#else
    /**
     * @brief Construct a new Robot object. Endeffector is determined by body name saved in
     * Robot::name_link_e.
     *
     * Initializes the robot with a MuJoCo model and a time step.
     *
     * @param mjb_path MuJoCo model path
     * @param Ta Robot step time
     * @param name_link_e Name of endeffector body. This is necessary for all robots using
     * PoseTracking.
     */
    Robot(const char* mjb_path, Time Ta, std::string name_link_e = "",
          std::string name_sensor_force = "")
        : model(mj_loadModel(mjb_path, NULL)),
          data(mj_makeData(model)),
          robot_data(model, data, Ta) {
        if (model->nq != DOF) {
            throw std::runtime_error("Robot: Mujoco model DOF (" + std::to_string(model->nq) +
                                     ") does not match Robot DOF template parameter (" +
                                     std::to_string(DOF) + ")");
        }
        if (!name_link_e.empty()) {
            robot_data.endeffector_site_id = mj_name2id(model, mjOBJ_SITE, name_link_e.c_str());
            if (robot_data.endeffector_site_id == -1) {
                throw std::runtime_error("Robot: endeffector site '" + name_link_e +
                                         "' not found in Mujoco model");
            }
        } else {
            robot_data.endeffector_site_id = -1;
        }
        if (!name_sensor_force.empty()) {
            robot_data.force_sensor_id = mj_name2id(model, mjOBJ_SENSOR, name_sensor_force.c_str());
            if (robot_data.force_sensor_id == -1) {
                throw std::runtime_error("Robot: force sensor '" + name_sensor_force +
                                         "' not found in Mujoco model");
            }
        }
        model_id_ = model->names[0];
    }
#endif

    Robot(const Robot& other)
        : model_id_(other.model_id_),
          model(mj_copyModel(nullptr, other.model)),
          data(mj_makeData(model)),
          robot_data(model, data, other.robot_data.Ta) {
        robot_data.endeffector_site_id = other.robot_data.endeffector_site_id;
        robot_data.force_sensor_id = other.robot_data.force_sensor_id;
    }

    ~Robot() {
        mj_deleteModel(model);
        mj_deleteData(data);
    }

    // ##### Controller registration functions #####

    void register_JointCTController(JointCTParameter js_param) {
        if (js_controller != nullptr) {
            orc::log::write_warning("Joint tracking controller is already set up!");
            return;
        }
        js_controller = std::make_unique<JointCTController>(robot_data, js_param);
        orc::log::write_debug("JointCTController registered!");
    }

    void register_JointPDPController(JointPDPParameter js_param) {
        if (js_controller != nullptr) {
            orc::log::write_warning("Joint tracking controller is already set up!");
            return;
        }
        js_controller = std::make_unique<JointPDPController>(robot_data, js_param);
        is_PDPController_registered = true;
        orc::log::write_debug("PDPController registered!");
    }

    void register_VelocityController(JointMatrix K0) {
        if (js_controller != nullptr) {
            orc::log::write_warning("Joint tracking controller is already set up!");
            return;
        }
        js_controller = std::make_unique<VelocityController>(robot_data, K0);
        orc::log::write_debug("VelocityController registered!");
    }

    void register_CartesianCTController(CartesianCTParameter ts_param) {
        if (ts_controller != nullptr) {
            orc::log::write_warning("Task tracking controller is already set up!");
            return;
        }
        ts_controller = std::make_unique<CartesianCTController>(robot_data, ts_param);
        is_CartesianCTController_registered = true;
        orc::log::write_debug("CartesianCTController registered!");
    }

    void register_GravityCompController(GravityCompParameter gc_param) {
        if (gc_controller != nullptr) {
            orc::log::write_warning("Gravity compensation controller is already set up!");
            return;
        }
        gc_controller = std::make_unique<GravityCompController>(robot_data, gc_param);
        is_GravityCompController_registered = true;
        orc::log::write_debug("GravityCompController registered!");
    }

    void register_SingularPertrubationController(SingularPerturbationParameter sp_param) {
        auto sp_controller = std::make_unique<SingularPertrubationController>(robot_data, sp_param);
        robot_data.M_off = sp_controller->get_M_off();  // Set offset matrix to robot data

        secondary_controllers.push_back(
            std::unique_ptr<orc::control::ControllerBase<DOF>>(std::move(sp_controller)));
        orc::log::write_debug("SingularPertrubationController registered!");
    }

    void register_FrictionCompController(FrictionCompParameter fc_param) {
        auto fc_controller = std::make_unique<FrictionCompController>(robot_data, fc_param);
        secondary_controllers.push_back(
            std::unique_ptr<orc::control::ControllerBase<DOF>>(std::move(fc_controller)));
        orc::log::write_debug("FrictionCompController registered!");
    }

    void register_HybridForceMotionController(orc::control::HybridForceMotionParameter<DOF> param) {
        if (custom_controller != nullptr) {
            orc::log::write_warning("A custom controller is already set up!");
            return;
        }
        custom_controller = std::make_unique<HybridForceMotionController>(robot_data, param);
        orc::log::write_debug("HybridForceMotionController registered!");
    }

    void register_CoulombFrictionCompController(
        orc::control::CoulombFrictionCompParameter<DOF> param) {
        auto cf_controller =
            std::make_unique<orc::control::CoulombFrictionCompController<DOF>>(robot_data, param);
        secondary_controllers.push_back(
            std::unique_ptr<orc::control::ControllerBase<DOF>>(std::move(cf_controller)));
        orc::log::write_debug("CoulombFrictionCompController registered!");
    }

    // ##### Update function #####
    /**
     * @brief Main update function
     *
     * @param t
     * @param grav_comp_only Setting this true, only GravityCompensation controller will be used, if
     * available
     * @return bool Return true if calculation was successful
     */
    bool update(Time t, bool grav_comp_only = false) {
        JointVector tau_local;

        // Compute robot data for controllers
        robot_data.t = t;
        compute_robot_data();

        // Compute primary torque either by GravityCompensation, JointTracking, or PoseTracking.
        if (grav_comp_only == true) {
            // Only run gravity compensation, ignore queue
            if (gc_controller == nullptr) {
                orc::log::write_error("No GravityCompController registered!");
                return false;
            }
            robot_data.tau_primary = gc_controller->update();
        } else {
            // Main trajectory queue algorithm
            TrajectoryBase* pcurr_traj = this->traj_queue.update(t);
            if (pcurr_traj != nullptr) {
                // process active trajectory
                TrajectoryType type = pcurr_traj->get_trajectory_type();

                // Although dynamic_cast would be safer, static_cast is faster
                // and we are sure about the type here. Also TwinCAT does not support RTTI.
                if (type == TrajectoryType::JOINTSPACE) {
                    auto js_traj = static_cast<JointspaceTrajectory*>(pcurr_traj);
                    js_traj->update(t, robot_data);

                    if (js_controller == nullptr) {
                        orc::log::write_error("No JointTrackingController registered!");
                        return false;
                    }
                    robot_data.tau_primary = js_controller->update();
                } else if (type == TrajectoryType::DENSE_JOINTSPACE) {
                    // Dense trajectory: pre-sampled points with feedforward torque
                    auto dense_traj = static_cast<DenseJointspaceTrajectory*>(pcurr_traj);
                    dense_traj->update(t, robot_data);

                    if (js_controller == nullptr) {
                        orc::log::write_error("No JointTrackingController registered!");
                        return false;
                    }
                    // Controller computes feedback, we add feedforward torque
                    robot_data.tau_primary = js_controller->update() + robot_data.tau_ff;
                } else if (type == TrajectoryType::TASKSPACE) {
                    auto ts_traj = static_cast<TaskspaceTrajectory*>(pcurr_traj);
                    ts_traj->update(t, robot_data);

                    if (ts_controller == nullptr) {
                        orc::log::write_error("No PoseTrackingController registered!");
                        return false;
                    }
                    robot_data.tau_primary = ts_controller->update();
                } else if (type == TrajectoryType::NULLSPACE) {
                    auto ns_traj = static_cast<NullspaceTrajectory*>(pcurr_traj);
                    robot_data.q_d_NS = ns_traj->get_nullspace_joint_state();
                    // Add jointspace trajectory for active control and re-run
                    add_jointspace_trajectory(robot_data.q_act, robot_data.q_act, t, t + 2.);
                    return update(t);
                } else if (type == TrajectoryType::JOINT_CTR_PARAM) {
                    auto js_ctr_param_traj = static_cast<JointCtrParamTrajectory*>(pcurr_traj);
                    if (js_controller == nullptr) {
                        orc::log::write_error("No JointTrackingController registered!");
                        return false;
                    }
                    if (js_controller->get_type() != control::ControllerType::JOINT_CT) {
                        orc::log::write_error(
                            "JointCtrParamTrajectory is only compatible with JointCTController!");
                        return false;
                    }
                    auto joint_ct_controller = static_cast<JointCTController*>(js_controller.get());
                    joint_ct_controller->set_parameter(js_ctr_param_traj->get_parameter());
                    orc::log::write_info(
                        "JointCtrParamTrajectory processed, parameters updated in controller.");
                    add_jointspace_trajectory(robot_data.q_act, robot_data.q_act, t, t + 2.);
                    return update(t);
                } else if (type == TrajectoryType::CART_CTR_PARAM) {
                    auto cart_param_traj = static_cast<CartesianCtrParamTrajectory*>(pcurr_traj);
                    if (ts_controller == nullptr) {
                        orc::log::write_error("No PoseTrackingController registered!");
                        return false;
                    }
                    if (ts_controller->get_type() != control::ControllerType::CARTESIAN_CT) {
                        orc::log::write_error(
                            "CartesianCtrParamTrajectory is only compatible with "
                            "CartesianCTController!");
                        return false;
                    }
                    auto cartesian_ct_controller =
                        static_cast<CartesianCTController*>(ts_controller.get());
                    cartesian_ct_controller->set_parameter(cart_param_traj->get_parameter());
                    orc::log::write_info(
                        "CartesianCtrParamTrajectory processed, parameters updated in controller.");
                    add_jointspace_trajectory(robot_data.q_act, robot_data.q_act, t, t + 2.);
                    return update(t);
                } else if (type == TrajectoryType::HYBRID_FORCE_MOTION) {
                    auto hybrid_traj = static_cast<HybridForceMotionTrajectory*>(pcurr_traj);
                    if (custom_controller == nullptr) {
                        orc::log::write_error("custom_controller is not registered!");
                        return false;
                    }

                    robot_data.force_error_integral =
                        static_cast<HybridForceMotionController*>(custom_controller.get())
                            ->get_integral_force_error()[2];
                    hybrid_traj->update(t, robot_data);
                    robot_data.tau_primary = custom_controller->update();
                } else if (type == TrajectoryType::JOINTSPACE_VELOCITY) {
                    auto v_traj = static_cast<orc::trajectory::JointspaceVelocityTrajectory<DOF>*>(
                        pcurr_traj);
                    v_traj->update(t, robot_data);
                    if (js_controller == nullptr) {
                        orc::log::write_error("No JointTrackingController registered!");
                        return false;
                    }
                    robot_data.tau_primary = js_controller->update();
                } else if (type == TrajectoryType::CARTESIAN_VELOCITY) {
                    auto v_traj =
                        static_cast<orc::trajectory::CartesianVelocityTrajectory<DOF>*>(pcurr_traj);
                    v_traj->update(t, robot_data);
                    if (ts_controller == nullptr) {
                        orc::log::write_error("No PoseTrackingController registered!");
                        return false;
                    }
                    robot_data.tau_primary = ts_controller->update();
                } else {
                    // This case should never happen, but just in case...
                    orc::log::write_error("NO TRAJECTORY AVAILABLE");
                    return false;
                }
                if (type != TrajectoryType::HYBRID_FORCE_MOTION && custom_controller != nullptr &&
                    custom_controller->get_type() == control::ControllerType::HYBRID_FORCE_MOTION) {
                    custom_controller->reset();
                }
            }
        }

        // Run through secondary controllers and sum.
        tau_local = robot_data.tau_primary;
        for (const auto& secondary_controller : secondary_controllers) {
            tau_local += secondary_controller->update();
        }

        tau_ = tau_local;  // Only write to tau_, if all went well and return true
        return true;
    }

    /**
     * @brief Resets the robot model to given resting joint configuration
     *
     * @param q_act Joint positions to reset to
     */
    void reset(JointVector& q_act) {
        robot_data.q_act = q_act;
        robot_data.q_dot_act = JointVector::Zero();
        robot_data.q_dotdot_act = JointVector::Zero();

        if (js_controller)
            js_controller->reset();
        if (ts_controller)
            ts_controller->reset();
        if (custom_controller)
            custom_controller->reset();
    }

    virtual void start(Time t, JointVector q_act, JointVector q_set, Time T_traj) {
        // Clear old trajectories
        traj_queue.clear();

        // reset to controllers, and robot to current position
        reset(q_act);

        // Reset joint configuration
        robot_data.q_act = q_act;
        robot_data.q_dot_act = JointVector::Zero();
        robot_data.q_dotdot_act = JointVector::Zero();

        // Move to initial position at startup
        this->add_jointspace_trajectory(q_act, q_set, t, t + T_traj);
    }

    void stop() {
        // TODO: implement
    }

    JointVector get_q_act() { return robot_data.q_act; }

    JointVector get_q_dot_act() { return robot_data.q_dot_act; }

    JointVector get_q_dotdot_act() { return robot_data.q_dotdot_act; }

    JointVector get_tau_act() { return tau_; }

    // JointVector get_tau_sp()
    // {
    // 	return model_.tau_sp;
    // }

    // JointVector get_tau_fc()
    // {
    // 	return model_.tau_fc;
    // }

    // double get_manipulability()
    // {
    // 	return model_.w_manipulability;
    // }
    // TODO rename if works
    bool add_trajectory_from_flatbuffer(const uint8_t* buffer, size_t size) {
        // STOP is a control signal, not a queued trajectory: clear the queue
        // directly before the deserializer would otherwise return nullptr.
        auto type_result = deserializer_.get_trajectory_type(buffer, size);
        if (type_result.valid && type_result.type == TrajectoryType::STOP) {
            traj_queue.clear();
            return true;
        }

        std::unique_ptr<TrajectoryBase> traj_ptr = deserializer_.deserialize(buffer, size);
        if (traj_ptr != nullptr) {
            traj_queue.add_trajectory(std::move(traj_ptr));
            return true;
        }
        return false;
    }

    void add_jointspace_trajectory(JointVector q0, JointVector q1, Time t0, Time t1) {
        JointspaceTrajectory traj(q0, q1, t0, t1);
        traj_queue.add_jointspace_trajectory(traj);
    }

    void add_jointspace_trajectory(std::vector<JointVector>& joint_poses,
                                   std::vector<Time>& time_points) {
        JointspaceTrajectory traj(joint_poses, time_points);
        traj_queue.add_jointspace_trajectory(traj);
    }

    void add_taskspace_trajectory(PoseVector pose0, PoseVector pose1, Time t0, Time t1) {
        TaskspaceTrajectory traj(pose0, pose1, t0, t1);
        traj_queue.add_taskspace_trajectory(traj);
    }

    void add_taskspace_trajectory(std::vector<PoseVector>& pose_vec,
                                  std::vector<Time>& time_points) {
        TaskspaceTrajectory traj(pose_vec, time_points);
        traj_queue.add_taskspace_trajectory(traj);
    }

    void add_hybrid_force_motion_trajectory(std::vector<PoseVector>& pose_vec,
                                            std::vector<double>& force_vec,
                                            std::vector<Time>& time_points) {
        HybridForceMotionTrajectory traj(pose_vec, force_vec, time_points);
        traj_queue.add_hybrid_force_motion_trajectory(traj);
    }

    // void add_jointspace_jerk_trajectory(std::vector<JointVector> &jerk_points, std::vector<Time>
    // &time_points)
    // {
    // 	JointspaceJerkTrajectory traj(jerk_points, time_points, model_);
    // 	traj_queue.add_jointspace_jerk_trajectory(traj);
    // }

    // void add_cartesian_velocity_trajectory(std::vector<CartesianVector> &vel_points,
    // std::vector<Time> &time_points)
    // {
    // 	CartesianVelocityTrajectory traj(vel_points, time_points, model_);
    // 	traj_queue.add_cartesian_velocity_trajectory(traj);
    // }

    // void add_jointspace_velocity_trajectory(std::vector<JointVector> &vel_points,
    // std::vector<Time> &time_points)
    // {
    // 	JointspaceVelocityTrajectory traj(vel_points, time_points, model_);
    // 	traj_queue.add_jointspace_velocity_trajectory(traj);
    // }

    // ##### Getter functions #####

    JointVector get_joint_error() {
        return (robot_data.q_act - robot_data.q_d).unaryExpr(orc::util::wrap_to_pi);
    }

    JointVector get_joint_error_dot() { return robot_data.q_dot_act - robot_data.q_dot_d; }

    CartesianVector get_cartesian_error() {
        CartesianVector e;
        CartesianPositionVector p, p_d;

        /* extract quaternion from desired and actual pose, respectively */
        Quatd quat_d(robot_data.pose_d[3], robot_data.pose_d[4], robot_data.pose_d[5],
                     robot_data.pose_d[6]);  // w,x,y,z
        RotationMatrix R_d = quat_d.toRotationMatrix();
        RotationMatrix R = robot_data.H_0_e.template block<3, 3>(0, 0);
        auto quat_e = Quatd(R * R_d.transpose());

        p = robot_data.H_0_e.template block<3, 1>(0, 3);
        p_d = robot_data.pose_d.template block<3, 1>(0, 0);
        e.template block<3, 1>(0, 0) = p - p_d;
        e.template block<3, 1>(3, 0) = quat_e.vec();

        return e;
    }

    CartesianVector get_cartesian_error_dot() { return robot_data.x_dot_act - robot_data.x_dot_d; }

    double get_force_error() { return robot_data.force_d - robot_data.force_compensated(2); }

    double get_integral_force_error() { return robot_data.force_error_integral; }

    PoseVector get_pose_set() { return robot_data.pose_d; }

    CartesianVector get_x_dot_set() { return robot_data.x_dot_d; }

    CartesianVector get_x_dotdot_set() { return robot_data.x_dotdot_d; }

    CartesianVector get_x_dotdotdot_set() {
        // TODO: What to do with 3rd derivatives
        return CartesianVector::Zero();
    }

    JointVector get_q_NS_set() { return robot_data.q_d_NS; }

    PoseVector get_pose_act() { return robot_data.pose_act; }

    CartesianVector get_x_dot_act() { return robot_data.x_dot_act; }

    CartesianVector get_x_dotdot_act() { return robot_data.x_dotdot_act; }

    CartesianVector get_x_dotdotdot_act() {
        // TODO:
        return CartesianVector::Zero();
    }

    JointVector get_q_set() { return robot_data.q_d; }

    JointVector get_q_dot_set() { return robot_data.q_dot_d; }

    JointVector get_q_dotdot_set() { return robot_data.q_dotdot_d; }

    JointVector get_q_dotdotdot_set() {
        // TODO:
        return JointVector::Zero();
    }

    bool is_taskspace_traj_active() {
        return traj_queue.get_current_trajectory_type() == TrajectoryType::TASKSPACE;
    }

    bool is_jointspace_traj_active() {
        return traj_queue.get_current_trajectory_type() == TrajectoryType::JOINTSPACE;
    }

    uint8_t get_model_id() { return model_id_; }

    int8_t get_endeffector_site_id() { return robot_data.endeffector_site_id; }

    int8_t get_force_sensor_id() { return robot_data.force_sensor_id; }

    /**
     * @brief Returns filtered force form hybrid force motion controller if registered.
     *
     * @return Vec3D
     */
    Vec3D get_force_filtered() {
        if (!custom_controller) {
            orc::log::write_error("get_force_filtered(): no custom_controller registered!");
            return Vec3D::Zero();
        }
        if (custom_controller->get_type() == orc::control::ControllerType::HYBRID_FORCE_MOTION) {
            auto hybrid_controller =
                static_cast<HybridForceMotionController*>(custom_controller.get());
            return hybrid_controller->force_filtered;
        } else {
            orc::log::write_error(
                "get_force_filtered(): custom_controller is not HybridForceMotionController!");
            return Vec3D::Zero();
        }
    }

    // ##### Setter function #####

    void set_t(Time t) { robot_data.t = t; }

    void set_q_act(JointVector& q_act) { robot_data.q_act = q_act; }

    void set_q_dot_act(JointVector& q_dot_act) { robot_data.q_dot_act = q_dot_act; }

    void set_q_dotdot_act(JointVector& q_dotdot_act) { robot_data.q_dotdot_act = q_dotdot_act; }

    void set_tau_motor(JointVector& tau_motor) { robot_data.tau_motor = tau_motor; }

    void set_tau_sens(JointVector& tau_sens) { robot_data.tau_sens = tau_sens; }

    void set_q_d_nullspace(JointVector& q_d_NS) { robot_data.q_d_NS = q_d_NS; }

    void set_force_measured(Vec3D force_measured) { robot_data.force_measurement = force_measured; }

    // ##### util functions #####

    /**
     * @brief Utility function for extracting the homogeneous transformation from Mujoco
     *
     * @param model
     * @param data
     * @return HomogeneousTransformation
     */
    HomogeneousTransformation get_current_H_0_e() {
        mjtNum xpos_e[3];
        mjtNum xmat_e[9];
        mjtNum H0e_mj[4 * 4] = {};

        if (robot_data.endeffector_site_id < 0) {
            orc::log::write_error("Invalid endeffector site ID!");
            return HomogeneousTransformation::Identity();
        }

        mju_copy(xpos_e, &data->site_xpos[3 * robot_data.endeffector_site_id], 3);
        mju_copy(xmat_e, &data->site_xmat[9 * robot_data.endeffector_site_id], 9);
        H0e_mj[15] = 1.;

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                H0e_mj[i * 4 + j] = xmat_e[i * 3 + j];
            }
            H0e_mj[i * 4 + 3] = xpos_e[i];
        }

        HomogeneousTransformation H_0_e = Eigen::Map<HomogeneousTransformation>(H0e_mj);
        return H_0_e;
    }

    /**
     * @brief Utility function for extracting the endeffector jacobian matrix from Mujoco
     *
     * @param model
     * @param data
     * @return JacobianMatrix
     */
    JacobianMatrix get_current_jacobian() {
        mjtNum jacp_arr[3 * DOF];
        mjtNum jacr_arr[3 * DOF];
        mjtNum jac_arr[6 * DOF];

        mj_jacSite(model, data, jacp_arr, jacr_arr, robot_data.endeffector_site_id);
        mju_copy(jac_arr, jacp_arr, 3 * DOF);
        mju_copy(&jac_arr[3 * DOF], jacr_arr, 3 * DOF);
        return Eigen::Map<JacobianMatrix>(jac_arr);
    }

    /**
     * @brief Calculates derivative of Jacobian matrix using finite differences
     *
     * @tparam DOF
     * @param model mjModel
     * @param data corresponding mjData
     * @param endeffector_site_id Body ID of endeffector
     * @return orc::RobotTraits<DOF>::JacobianMatrix
     */
    JacobianMatrix get_current_jacobian_dot() {
        // Uses finite differences to compute derivative of Jacobian. Approach
        // is equivalent to https://github.com/google-deepmind/mujoco/issues/411

        mjtNum h = 1e-6;
        mjtNum jac[6 * DOF];
        mjtNum jac_h[6 * DOF];
        mjtNum jac_dot[6 * DOF];
        mjtNum jac_diff[6 * DOF];
        mjtNum qpos_orig[DOF];

        // Store data->qpos for to set back to at the end of this calculation
        mju_copy(qpos_orig, data->qpos, DOF);

        mj_forward(model, data);
        mj_jacBody(model, data, jac, &jac[3 * DOF], robot_data.endeffector_site_id);
        mj_integratePos(model, data->qpos, data->qvel, h);
        mj_forward(model, data);
        mj_jacBody(model, data, jac_h, &jac_h[3 * DOF], robot_data.endeffector_site_id);

        mju_sub(jac_diff, jac_h, jac, 6 * DOF);
        mju_scl(jac_dot, jac_diff, 1 / h, 6 * DOF);

        // return to original joint position
        mju_copy(data->qpos, qpos_orig, DOF);
        mj_forward(model, data);
        return Eigen::Map<JacobianMatrix>(jac_dot);
    }

    /**
     * @brief Calculates inverse of  Jacobian damped least-squares inverse as described in [B.
     * Siciliano, Robotics - Modelling, Planning and Control, 2010]: eq (3.59)
     *
     * @tparam DOF
     * @param J Jacobian to invert
     * @return orc::RobotTraits<DOF>::JacobianInverseMatrix
     */
    JacobianInverseMatrix get_current_inverse_jacobian(JacobianMatrix J) {
        double lambda = 1e-6;  // 1e-6;
        return J.transpose() *
               (J * J.transpose() + (lambda * lambda) * Eigen::Matrix<double, 6, 6>::Identity())
                   .inverse();
    }

    /**
     * @brief Extracts mass matrix for mjModel and mjData
     *
     * @tparam DOF
     * @param model
     * @param data
     * @return orc::RobotTraits<DOF>::JointMatrix
     */
    JointMatrix get_current_mass_matrix() {
        mjtNum M_arr[DOF * DOF];
        mj_fullM(model, M_arr, data->qM);
        return Eigen::Map<JointMatrix>(M_arr);
    }

    int find_id_to_site_name(const char* name) {
        // finds the body ID to the given body name from mjModel
        for (int i = 0; i < model->nsite; i++) {
            if (!strcmp(name, mj_id2name(model, mjOBJ_SITE, i)))
                return i;
        }
        // if none found
        orc::log::write_error("Site name not found in MJB file!!");
        return -1;
    }

    // ##### Private functions #####
    const Vec3D GRAVITY_VECTOR = Vec3D(0, 0, -9.81);

private:
    void compute_robot_data() {
        if (is_GravityCompController_registered) {
            // Temporarily set qvel to zero and forward the adapted model.
            // This way the qfrc_bias force becomes identical to the gravitational vector.
            // Refer to
            // https://mujoco.readthedocs.io/en/stable/computation/index.html#general-framework for
            // more info. This is done before the main model forward, such that the unneeded model
            // data at the end of compute_robot_data is equal to the actual forwarded model data.
            JointVector q_dot_orig = robot_data.q_dot_act;
            robot_data.q_dot_act = JointVector::Zero();
            mj_forward(model, data);
            robot_data.G = robot_data.qfrc_bias;  // This works as robot_data.qfrc_bias is a
                                                  // Eigen::Map to memory inside mjData
            robot_data.q_dot_act = q_dot_orig;
        }

        // Forward model
        mj_forward(model, data);

        // Collision detection
        if (data->ncon > 0 && robot_data.collision_detected == false) {
            robot_data.collision_detected = true;
        }
        if (data->ncon == 0) {
            robot_data.collision_detected = false;
        }

        // Dynamic matrices
        robot_data.M = get_current_mass_matrix();
        robot_data.J = get_current_jacobian();
        robot_data.J_inv = get_current_inverse_jacobian(robot_data.J);
        robot_data.J_dot = get_current_jacobian_dot();
        robot_data.H_0_e = get_current_H_0_e();

        if (is_CartesianCTController_registered) {
            // Compute pose and derivatives
            CartesianPositionVector p = robot_data.H_0_e.template block<3, 1>(0, 3);
            RotationMatrix R = robot_data.H_0_e.template block<3, 3>(0, 0);
            auto quat_ = Quatd(R);
            robot_data.pose_act.template block<3, 1>(0, 0) = p;
            robot_data.pose_act[3] = quat_.w();
            robot_data.pose_act.template block<3, 1>(4, 0) = quat_.vec();
            robot_data.x_dot_act = robot_data.J * robot_data.q_dot_act;
            robot_data.x_dotdot_act = robot_data.J_dot * robot_data.q_dot_act;
        }

        // Compensate force measurement
        if (robot_data.force_sensor_id >= 0) {
            if (robot_data.model->sensor_objtype[robot_data.force_sensor_id] == mjOBJ_SITE) {
                int site_id = robot_data.model->sensor_objid[robot_data.force_sensor_id];
                int body_id = robot_data.model->site_bodyid[site_id];
                mjtNum eef_mass = robot_data.model->body_subtreemass[body_id];
                Eigen::Map<RotationMatrix> rotmat_eef(robot_data.data->site_xmat + 9 * site_id);
                robot_data.data->site_xmat[site_id];
                Vec3D gravitational_force = eef_mass * rotmat_eef * GRAVITY_VECTOR;
                // take care of the sign convention (for F/T sensors)
                robot_data.force_compensated = robot_data.force_measurement + gravitational_force;
            }
        }
    }
};
}  // namespace orc::robots
