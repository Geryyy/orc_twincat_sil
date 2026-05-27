#pragma once

#include "orc/com/com_settings.h"
#include "orc/control/ControllerParameter.h"
#include "orc/robots/Robot.h"
#include "orc/util/Logger.h"
#include "orc/util/Time.h"

namespace orc::robots {
class LinearAxis : public Robot<2> {
public:
    static constexpr int DOF = 2;

    DECLARE_ROBOT_TRAITS_USINGS(DOF)

    using Time = orc::Time;
    using Robot<DOF>::FlatBufferSerializer;
    using Robot<DOF>::FlatBufferDeserializer;
    using Robot<DOF>::TrajectoryQueue;
    using Robot<DOF>::TrajectoryBase;
    using Robot<DOF>::Robot;

    // Communication settings
    static constexpr int ROBOT_INDEX = 1;
    static constexpr uint16_t SERVER_PORT =
        orc::com::SERVER_PORT + orc::com::ROBOT_OFFSET * ROBOT_INDEX;
    static constexpr uint16_t CLIENT_PORT =
        orc::com::CLIENT_PORT + orc::com::ROBOT_OFFSET * ROBOT_INDEX;
    static constexpr uint16_t SIL_MODEL_PORT =
        orc::com::SIL_MODEL_PORT + orc::com::ROBOT_OFFSET * ROBOT_INDEX;
    static constexpr uint16_t SIL_CONTROLLER_PORT =
        orc::com::SIL_CONTROLLER_PORT + orc::com::ROBOT_OFFSET * ROBOT_INDEX;

    static constexpr Time Ts_default = 1e-3;

#ifdef TC_VER
    /**
     * @brief Construct a new Linear Axis object in TwinCAT.
     *
     * @param model_binary
     * @param model_size Size of model binary
     * @param Ta Sample time
     * @param m_Trace CTcTrace object for logging
     * @param K0 Velocity controller feedback matrix
     */
    LinearAxis(unsigned char* model_binary, unsigned int model_size, Time Ta, CTcTrace m_Trace,
               JointMatrix K0 = (JointVector::Ones() * 25.).asDiagonal())
        : Robot(model_binary, model_size, Ta, m_Trace, "end_effector") {
        init_linear_axis(K0);
    }
#else
    /**
     * @brief Construct a new Linear Axis object
     *
     * @param mjb_path Path to MJB model
     * @param Ta Sample time
     * @param K0 Velocity controller feedback matrix
     */
    LinearAxis(const char* mjb_path, Time Ta,
               JointMatrix K0 = (JointVector::Ones() * 25.).asDiagonal())
        : Robot(mjb_path, Ta, "end_effector") {
        init_linear_axis(K0);
    }

    /**
     * @brief Construct a new Linear Axis object with default sample time and default velocity
     * controller feeback matrix.
     *
     * @param mjb_path
     */
    LinearAxis(const char* mjb_path) : LinearAxis(mjb_path, Ts_default) {}
#endif

private:
    void init_linear_axis(JointMatrix K0) { this->register_VelocityController(K0); }
};
}  // namespace orc::robots
