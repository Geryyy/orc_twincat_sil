#pragma once

#include "orc/control/ControllerParameter.h"
#include "orc/control/controller/joint/JointPDPController.h"
#include "orc/robots/Robot.h"

namespace orc::robots {
class Kinova : public Robot<7> {
public:
    constexpr static int DOF = 7;
    static constexpr uint16_t SERVER_PORT = 10000;
    static constexpr uint16_t CLIENT_PORT = 11000;
    static constexpr Time Ta_default = 1e-3;
    inline const static std::string name_link_e = "kinova3/gripping_point";
    inline const static JointVector q_home = JointVector({0., 0.27, 3.14, -2.27, 0., 0.96, 1.57});

    /**
     * @brief Construct a new Kinova object with default sample time
     *
     * @param mjb_path Path to MJB model binary
     * @param endeffector_site_name Name of the endeffector site
     */
    Kinova(const char* mjb_path, std::string endeffector_site_name = name_link_e)
        : Kinova(mjb_path, JointPDPParameter(), GravityCompParameter(), Ta_default,
                 endeffector_site_name) {}

    /**
     * @brief Construct a new Kinova object for simulation purposes. Registers all controllers
     * necessary for simulation, i.e., JointPDPController, CartesianCTController, and
     * GravityCompController.
     *
     * @param mjb_path Path to MJB model binary
     * @param js_param
     * @param gc_param
     * @param Ta Sample time
     * @param f_cutoff_norm Cutoff frequency for joint filters
     */
    Kinova(const char* mjb_path, JointPDPParameter js_param, GravityCompParameter gc_param, Time Ta,
           std::string endeffector_site_name = name_link_e)
        : Robot<DOF>(mjb_path, Ta, endeffector_site_name) {
        register_JointPDPController(js_param);
        register_GravityCompController(gc_param);
    }

    /**
     * @brief Returns home joint configuration.
     *
     * @return JointVector
     */
    static JointVector get_q_home() { return q_home; }

    struct KinovaContrParam : public orc::control::ControllerParameter<7> {
        KinovaContrParam(bool simulation = true) {
            if (simulation) {
                KP_PDP = (JointVector::Ones() * 100.).asDiagonal();
                KD_PDP = (JointVector::Ones() * 1.).asDiagonal();
            } else {
                JointVector Kp_diag, Kd_diag;
                Kp_diag << 1000., 750., 750., 500., 250., 250., 250.;
                Kd_diag << 150., 100., 100., 75., 50., 50, 50.;

                KP_PDP = Kp_diag.asDiagonal();
                KD_PDP = Kd_diag.asDiagonal();
            }
        }
    };
};
}  // namespace orc::robots
