#pragma once

#include <orc/com/com_settings.h>
#include <orc/control/Controller.h>
#include <orc/robots/Robot.h>
#include <orc/sig/filter.h>
#include <orc/util/Time.h>
#include <orc/util/import_eigen.h>

namespace orc::robots {

class Iiwa : public Robot<7> {
public:
    constexpr static int DOF = 7;

    using Robot<DOF>::FlatBufferSerializer;
    using Robot<DOF>::FlatBufferDeserializer;
    using Robot<DOF>::TrajectoryQueue;
    using Robot<DOF>::TrajectoryBase;
    using Robot<DOF>::Robot;
    using Time = orc::Time;

    DECLARE_ROBOT_TRAITS_USINGS(DOF)

    using JointCTController = orc::control::JointCTController<DOF>;
    using JointCTParameter = orc::control::JointCTParameter<DOF>;
    using JointspaceTrajectory = orc::trajectory::JointspaceTrajectory<DOF>;
    using CartesianCTController = orc::control::CartesianCTController<DOF>;
    using CartesianCTParameter = orc::control::CartesianCTParameter<DOF>;

    using SingularPerturbationController = orc::control::SingularPerturbationController<DOF>;
    using SingularPerturbationParameter = orc::control::SingularPerturbationParameter<DOF>;
    using FrictionCompController = orc::control::FrictionCompController<DOF>;
    using FrictionCompParameter = orc::control::FrictionCompParameter<DOF>;
    using GravityCompController = orc::control::GravityCompController<DOF>;
    using GravityCompParameter = orc::control::GravityCompParameter<DOF>;
    using HybridForceMotionController = orc::control::HybridForceMotionController<DOF>;
    using HybridForceMotionParameter = orc::control::HybridForceMotionParameter<DOF>;

    // Communication settings
    static constexpr int ROBOT_INDEX = 0;
    static constexpr uint16_t SERVER_PORT =
        orc::com::SERVER_PORT + orc::com::ROBOT_OFFSET * ROBOT_INDEX;
    static constexpr uint16_t CLIENT_PORT =
        orc::com::CLIENT_PORT + orc::com::ROBOT_OFFSET * ROBOT_INDEX;
    static constexpr uint16_t SIL_MODEL_PORT =
        orc::com::SIL_MODEL_PORT + orc::com::ROBOT_OFFSET * ROBOT_INDEX;
    static constexpr uint16_t SIL_CONTROLLER_PORT =
        orc::com::SIL_CONTROLLER_PORT + orc::com::ROBOT_OFFSET * ROBOT_INDEX;

    static constexpr Time Ts_default = 0.125e-3;

    inline const static std::string name_link_e = "iiwa_link_e";

private:
    // Savitzky-Golay design for the acceleration estimate (tune for your sample rate).
    static constexpr int SG_WINDOW = 15;     // sliding-window length [samples]
    static constexpr int SG_POLY_ORDER = 2;  // fitted polynomial order

    // Filters
    orc::sig::PT1<JointArray> q_filter;
    orc::sig::DT1<JointArray> q_dot_filter;
    // Second derivative from position via Savitzky-Golay: lower noise than a DT1 cascade.
    orc::sig::SavitzkyGolay<JointArray> q_dotdot_filter;

public:
#ifdef TC_VER
    Iiwa(unsigned char* model_binary, unsigned int model_size, JointCTParameter js_param,
         CartesianCTParameter ts_param, SingularPerturbationParameter sp_param,
         FrictionCompParameter fc_param, GravityCompParameter gc_param, Time Ta,
         JointArray f_cutoff_norm, CTcTrace m_Trace, std::string name_site_e = name_link_e,
         std::string name_sensor_force = "")
        : Robot(model_binary, model_size, Ta, m_Trace, name_site_e, name_sensor_force),
          q_filter(f_cutoff_norm, Ta),
          q_dot_filter(f_cutoff_norm, Ta),
          q_dotdot_filter(SG_WINDOW, SG_POLY_ORDER, 2, Ta) {
        this->register_JointCTController(js_param);
        this->register_CartesianCTController(ts_param);
        this->register_SingularPertrubationController(sp_param);
        this->register_FrictionCompController(fc_param);
        this->register_GravityCompController(gc_param);
    }

    Iiwa(unsigned char* model_binary, unsigned int model_size, JointCTParameter js_param,
         CartesianCTParameter ts_param, GravityCompParameter gc_param, Time Ta,
         JointArray f_cutoff_norm, CTcTrace m_Trace, std::string name_site_e = name_link_e,
         std::string name_sensor_force = "")
        : Robot(model_binary, model_size, Ta, m_Trace, name_site_e, name_sensor_force),
          q_filter(f_cutoff_norm, Ta),
          q_dot_filter(f_cutoff_norm, Ta),
          q_dotdot_filter(SG_WINDOW, SG_POLY_ORDER, 2, Ta) {
        this->register_JointCTController(js_param);
        this->register_CartesianCTController(ts_param);
        this->register_GravityCompController(gc_param);
    }

#else
    /**
     * @brief Construct a new Iiwa object with default sample time
     *
     * @param mjb_path Path to MJB model binary
     * @param endeffector_site_name Name of the endeffector site
     */
    Iiwa(const char* mjb_path, std::string endeffector_site_name = name_link_e,
         std::string name_sensor_force = "")
        : Iiwa(mjb_path, Ts_default, endeffector_site_name, name_sensor_force) {}

    /**
     * @brief Construct a new Iiwa object with default controller parameters
     *
     * @param mjb_path Path to MJB model binary
     * @param Ta Sample time in sec
     * @param endeffector_site_name Name of the endf(effector site
     */
    Iiwa(const char* mjb_path, Time Ta, std::string endeffector_site_name = name_link_e,
         std::string name_sensor_force = "")
        : Iiwa(mjb_path, JointCTParameter(), CartesianCTParameter(),
               SingularPerturbationParameter(), FrictionCompParameter(), GravityCompParameter(), Ta,
               200 * JointArray::Ones() * 2 * Ta.toSec(), endeffector_site_name,
               name_sensor_force) {}

    /**
     * @brief Construct a new Iiwa object. Registers all controllers necessary for operation, i.e.,
     * JointCTController, CartesianCTController, SingularPerturbationController,
     * FrictionCompController, and GravityCompController.
     *
     * @param mjb_path Path to MJB model binary
     * @param js_param JointspaceCTCcontroller parameters
     * @param ts_param CartesianCTController parameters
     * @param Ta Sample time
     * @param f_cutoff_norm Cutoff frequency for joint filters
     * @param endeffector_site_name Name of the endeffector site
     */
    Iiwa(const char* mjb_path, JointCTParameter js_param, CartesianCTParameter ts_param,
         SingularPerturbationParameter sp_param, FrictionCompParameter fc_param,
         GravityCompParameter gc_param, Time Ta, JointArray f_cutoff_norm,
         std::string endeffector_body_name = name_link_e, std::string name_sensor_force = "")
        : Robot(mjb_path, Ta, endeffector_body_name, name_sensor_force),
          q_filter(f_cutoff_norm, Ta),
          q_dot_filter(f_cutoff_norm, Ta),
          q_dotdot_filter(SG_WINDOW, SG_POLY_ORDER, 2, Ta) {
        this->register_JointCTController(js_param);
        this->register_CartesianCTController(ts_param);
        this->register_SingularPertrubationController(sp_param);
        this->register_FrictionCompController(fc_param);
        this->register_GravityCompController(gc_param);
    }

    /**
     * @brief Construct a new Iiwa object for simulation purposes. Registers all controllers
     * necessary for simulation, i.e., JointCTController, CartesianCTController, and
     * GravityCompController.
     *
     * @param mjb_path Path to MJB model binary
     * @param js_param
     * @param ts_param
     * @param gc_param
     * @param Ta Sample time
     * @param f_cutoff_norm Cutoff frequency for joint filters
     */
    Iiwa(const char* mjb_path, JointCTParameter js_param, CartesianCTParameter ts_param,
         GravityCompParameter gc_param, Time Ta, JointArray f_cutoff_norm,
         std::string endeffector_body_name = name_link_e, std::string name_sensor_force = "")
        : Robot(mjb_path, Ta, endeffector_body_name, name_sensor_force),
          q_filter(f_cutoff_norm, Ta),
          q_dot_filter(f_cutoff_norm, Ta),
          q_dotdot_filter(SG_WINDOW, SG_POLY_ORDER, 2, Ta) {
        this->register_JointCTController(js_param);
        this->register_CartesianCTController(ts_param);
        this->register_GravityCompController(gc_param);
    }
#endif

    Iiwa(const Iiwa& other) = default;

    using Robot<DOF>::update;

    JointVector update(Time t, JointVector& q_act, JointVector& tau_sens_act,
                       JointVector& tau_motor_act, bool grav_comp) {
        JointVector tau_d, tau_sp, tau_fc;

        return JointVector::Zero();
    }

    void start(Time t, JointVector q_act, JointVector q_set, Time T_traj) override {
        // Clear old trajectories
        traj_queue.clear();

        // Reset controllers, and positions
        reset(q_act);

        // Reset joint configuration
        set_q_act_filtered_derivatives(q_act);

        // Move to initial position at startup
        this->add_jointspace_trajectory(q_act, q_set, t, t + T_traj);
    }

    /**
     * @brief Set actual filtered joint configurations and derivatives using q_filter, q_dot_filter,
     * and q_dotdot_filter.
     *
     * @param q_act
     */
    void set_q_act_filtered_derivatives(JointVector& q_act) {
        robot_data.q_act = q_filter.update(q_act);
        robot_data.q_dot_act = q_dot_filter.update(q_act);
        // Savitzky-Golay estimates acceleration directly from position.
        robot_data.q_dotdot_act = q_dotdot_filter.update(q_act);
    }

    struct IiwaContrParam : public orc::control::ControllerParameter<7> {
        IiwaContrParam(bool simulation = true) {
            if (simulation) {
                K0_cart = (CartesianVector::Ones() * 550.0).asDiagonal();
                K1_cart = (CartesianVector::Ones() * 120.0).asDiagonal();
                K0_N_cart = (JointVector::Ones() * 50).asDiagonal();
                K1_N_cart = (JointVector::Ones() * 20).asDiagonal();

                K0_joint = (JointVector::Ones() * 550).asDiagonal();
                K1_joint = (JointVector::Ones() * 330).asDiagonal();
                KI_joint.diagonal() = JointVector::Zero();

                K0_cart(3, 3) = 1200;  // 1200 // 4800
                K0_cart(4, 4) = 1200;  // 1200
                K0_cart(5, 5) = 1200;  // 1200

                f_c_fast = 500 * JointArray::Ones();
                f_c_slow = 500 * JointArray::Ones();
            } else {
                K0_cart = (CartesianVector::Ones() * 900.0).asDiagonal();
                K1_cart = (CartesianVector::Ones() * 60.0).asDiagonal();
                K0_N_cart = (JointVector::Ones() * 120).asDiagonal();
                K1_N_cart = (JointVector::Ones() * 60).asDiagonal();

                K0_joint = (JointVector::Ones() * 1200).asDiagonal();
                K1_joint = (JointVector::Ones() * 60).asDiagonal();
                KI_joint = (JointVector::Ones() * 8000).asDiagonal();  // 1000

                B = get_B();

                K_sp.diagonal() << 4, 4, 4, 5, 3, 2.5, 2.5;
                D_sp.diagonal() << 0.015, 0.015, 0.015, 0.02, 0.01, 0.01, 0.01;

                L_fc << 200, 200, 300, 300, 500, 1000, 1000;

                D_gc.diagonal() = 1 * JointVector::Ones();

                f_c_fast << 400, 400, 400, 400, 500, 1000, 1000;
                f_c_slow = 200 * JointArray::Ones();
            }
        }
    };

private:
    static JointVector get_B() {
        // These parameters are copied from the original model.
        const double n__1 = 160;
        const double n__2 = 160;
        const double n__3 = 160;
        const double n__4 = 160;
        const double n__5 = 100;
        const double n__6 = 160;
        const double n__7 = 160;
        const double I__r1xx = 0.000185;
        const double I__r1yy = I__r1xx;
        const double I__r1zz = 0.000185 + 0.000238166953125;
        const double I__r2xx = 0.000185;
        const double I__r2yy = I__r2xx;
        const double I__r2zz = 0.000185 + 0.000238166953125;
        const double I__r3xx = 0.000129;
        const double I__r3yy = I__r3xx;
        const double I__r3zz = 0.000129 + 6.20384375e-005;
        const double I__r4xx = 0.000129;
        const double I__r4yy = I__r4xx;
        const double I__r4zz = 0.000129 + 6.20384375e-005;
        const double I__r5xx = 7.5e-05;
        const double I__r5yy = I__r5xx;
        const double I__r5zz = 7.5e-05 + 7.20968e-005;
        const double I__r6xx = 1.5e-05;
        const double I__r6yy = I__r6xx;
        const double I__r6zz = 1.5e-05 + 3.51125e-006;
        const double I__r7xx = 1.5e-05;
        const double I__r7yy = I__r7xx;
        const double I__r7zz = 1.5e-05 + 3.51125e-006;

        JointVector B;
        B << n__1 * n__1 * I__r1zz, n__2 * n__2 * I__r2zz, n__3 * n__3 * I__r3zz,
            n__4 * n__4 * I__r4zz, n__5 * n__5 * I__r5zz, n__6 * n__6 * I__r6zz,
            n__7 * n__7 * I__r7zz;
        return B;
    }
};
}  // namespace orc::robots
