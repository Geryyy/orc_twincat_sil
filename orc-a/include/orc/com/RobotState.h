#pragma once

#include <cstdint>
#include <vector>
#include "orc/OrcTypes.h"
#include "orc/RobotStatus.h"
#include "orc/RobotTraits.h"
#include "orc/util/import_eigen.h"

// FlatBuffers support
#include "orc/com/flatbuffers/FlatBufferRobotState.h"

namespace orc::com {
/**
 * @brief Structure holding all current information of a Robot of type RobotType.
 * RobotState is used to communicate the current state of the robot (or MuJoCo simulation) to the
 * sending PC.
 *
 * @tparam RobotType
 */
template <typename RobotType>
struct RobotState {
    static constexpr int DOF = RobotType::DOF;
    using JointVector = typename RobotType::JointVector;
    using JointMatrix = typename RobotType::JointMatrix;
    using RobotStatus = orc::logic::RobotStatus;

    // Maximum expected size of serialized RobotState (for buffer allocation)
    // This is a conservative estimate for FlatBuffers serialized data
    static constexpr size_t DATA_SIZE = 2048;

    orc::Time time_ = 0;

    // robot status
    RobotStatus status;

    // joint states
    JointVector q_act;        /**< Actual joint configuration */
    JointVector q_dot_act;    /**< Actual joint velocity */
    JointVector q_dotdot_act; /**< Actual joint acceleration */

    // joint effort
    JointVector tau; /**< Actual joint torques */

    // setpoints
    JointVector q_set;            /**< Joint configuration setpoint */
    JointVector q_dot_set;        /**< Joint velocity setpoint */
    JointVector q_dotdot_set;     /**< Joint acceleration setpoint*/
    PoseVector x_set;             /**< Pose setpoint*/
    CartesianVector x_dot_set;    /**< Pose velocity setpoint */
    CartesianVector x_dotdot_set; /**< Pose acceleration setpoint*/
    JointVector q_d_NS;           /**< Nullspace pose used by CartesianCTController*/

    uint8_t model_id; /**< Model identifier to compare if both ends use the same model. Equivalent
                         to mjModel.names[0] */

    RobotState() {
        q_act.setZero();
        q_dot_act.setZero();
        q_dotdot_act.setZero();
        tau.setZero();
        q_set.setZero();
        q_dot_set.setZero();
        q_dotdot_set.setZero();
        x_set.setZero();
        x_dot_set.setZero();
        x_dotdot_set.setZero();
        q_d_NS.setZero();
        status = RobotStatus::OFF;
        model_id = 0;
    }

    /**
     * @brief Construct a new Robot State object from a robot
     */
    RobotState(RobotType& robot, Time time, RobotStatus status, uint8_t model_id = 0)
        : time_(time), status(status), model_id(model_id) {
        q_act = robot.get_q_act();
        q_dot_act = robot.get_q_dot_act();
        q_dotdot_act = robot.get_q_dotdot_act();
        tau = robot.get_tau_act();

        q_set = robot.get_q_set();
        q_dot_set = robot.get_q_dot_set();
        q_dotdot_set = robot.get_q_dotdot_set();

        // cartesian control setpoints
        x_set = robot.get_pose_set();
        x_dot_set = robot.get_x_dot_set();
        x_dotdot_set = robot.get_x_dotdot_set();
        q_d_NS = robot.get_q_NS_set();
    }

    RobotState(RobotType& robot, Time time) : RobotState(robot, time, RobotStatus::ENABLE, 0) {}

    /**
     * @brief Construct from serialized FlatBuffer data
     */
    explicit RobotState(const std::vector<uint8_t>& data) {
        deserialize_from_flatbuffer(data.data(), data.size());
    }

    explicit RobotState(std::vector<unsigned char>& data) {
        deserialize_from_flatbuffer(data.data(), data.size());
    }

    /**
     * @brief Serialize RobotState to FlatBuffer format
     */
    // TODO: Zero-copy optimization — consider keeping a persistent FlatBufferBuilder as a member
    //       to avoid reallocating the internal buffer on every serialize() call. Call
    //       builder.Reset() between serializations instead.
    std::vector<uint8_t> serialize() const {
        fb::RobotStateSerializer<RobotType> serializer;
        return serializer.serialize(time_, status, model_id, q_act, q_dot_act, q_dotdot_act, tau,
                                    q_set, q_dot_set, q_dotdot_set, x_set, x_dot_set, x_dotdot_set,
                                    q_d_NS);
    }

    /**
     * @brief Serialize to char vector (for compatibility)
     */
    // TODO: Zero-copy optimization — this creates a second copy of the serialized data.
    //       Consider removing this method and having callers use serialize() directly with
    //       uint8_t data, or cast the pointer: reinterpret_cast<const char*>(data.data()).
    std::vector<char> serialize_to_chars() const {
        auto data = serialize();
        return std::vector<char>(data.begin(), data.end());
    }

    /**
     * @brief Deserialize from FlatBuffer data
     */
    // TODO: Zero-copy optimization — instead of copying every field into a RobotState struct,
    //       callers could read directly from the FlatBuffer via RobotStateReader, avoiding the
    //       per-element copy into Eigen vectors. Consider returning a RobotStateReader or a
    //       lightweight view wrapper that lazily reads from the buffer.
    static RobotState deserialize(const char* buffer, size_t size) {
        RobotState state;
        state.deserialize_from_flatbuffer(reinterpret_cast<const uint8_t*>(buffer), size);
        return state;
    }

private:
    // TODO: Zero-copy optimization — the per-joint copy from the FlatBuffer
    //       reader into Eigen vectors could be avoided on the hot path.
    //       FlatBuffer scalar vectors (`[double]`) expose `data()` returning a
    //       `const double*` pointing into the buffer, so an `Eigen::Map<const
    //       JointVector>(reader.<field>_data())` view eliminates the copy.
    //       Requires extending RobotStateReader with a *_data() accessor.
    void deserialize_from_flatbuffer(const uint8_t* buffer, size_t size) {
        fb::RobotStateReader<DOF> reader;
        if (!reader.init(buffer, size))
            return;

        time_ = reader.time();
        status = reader.status();
        model_id = reader.model_id();

        for (int i = 0; i < DOF; ++i) {
            q_act[i] = reader.q_act(i);
            q_dot_act[i] = reader.q_dot_act(i);
            q_dotdot_act[i] = reader.q_dotdot_act(i);
            tau[i] = reader.tau(i);
            q_set[i] = reader.q_set(i);
            q_dot_set[i] = reader.q_dot_set(i);
            q_dotdot_set[i] = reader.q_dotdot_set(i);
            q_d_NS[i] = reader.q_d_ns(i);
        }

        for (int i = 0; i < 7; ++i)
            x_set[i] = reader.x_set(i);
        for (int i = 0; i < 6; ++i) {
            x_dot_set[i] = reader.x_dot_set(i);
            x_dotdot_set[i] = reader.x_dotdot_set(i);
        }
    }
};

}  // namespace orc::com
