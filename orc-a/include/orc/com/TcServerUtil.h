#pragma once

#include <cstring>
#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/com/RobotState.h"
#include "orc/com/flatbuffers/FlatBufferRobotState.h"
#include "orc/robots/Robot.h"
#include "orc/trajectory/Trajectories.h"

namespace orc::com {
using RobotStatus = orc::logic::RobotStatus;

template <typename RobotType>
uint16_t send_robot_data(RobotType& contr_, Time time, RobotStatus status, uint8_t model_id,
                         uint8_t tx_data[], uint16_t max_size) {
    // Persistent serializer (keeps FlatBufferBuilder buffer across calls).
    // Safe under TwinCAT: each task has its own template instantiation and
    // cyclic code runs in a single RT task with no concurrent re-entry.
    static orc::com::fb::RobotStateSerializer<RobotType> serializer;

    const size_t written = serializer.serialize_into(
        tx_data, max_size, time, status, model_id, contr_.get_q_act(), contr_.get_q_dot_act(),
        contr_.get_q_dotdot_act(), contr_.get_tau_act(), contr_.get_q_set(), contr_.get_q_dot_set(),
        contr_.get_q_dotdot_set(), contr_.get_pose_set(), contr_.get_x_dot_set(),
        contr_.get_x_dotdot_set(), contr_.get_q_NS_set());
    return static_cast<uint16_t>(written);
}

template <typename RobotType>
uint16_t send_robot_data(RobotType& contr_, Time time, RobotStatus status,
                         uint64_t model_hash_value, uint8_t tx_data[], uint16_t max_size) {
    // Truncate hash to uint8_t for model_id (legacy compatibility)
    uint8_t model_id = static_cast<uint8_t>(model_hash_value & 0xFF);
    return send_robot_data(contr_, time, status, model_id, tx_data, max_size);
}

template <typename RobotType>
uint16_t send_robot_data(RobotType& contr_, Time time, RobotStatus status, uint8_t tx_data[],
                         uint16_t max_size) {
    uint8_t default_model_id = 0;
    return send_robot_data(contr_, time, status, default_model_id, tx_data, max_size);
}

template <typename RobotType>
void handle_receive(RobotType& contr_, uint8_t recv_buffer_[], uint16_t msg_size) {
    contr_.add_trajectory_from_flatbuffer(recv_buffer_, static_cast<size_t>(msg_size));
}
}  // namespace orc::com
