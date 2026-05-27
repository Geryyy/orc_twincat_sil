#pragma once

#include "stdint.h"

namespace orc::com {
// Trajectory and robot state communication
constexpr uint16_t SERVER_PORT = 10000;  // i.e. twincat / gazebo
constexpr uint16_t CLIENT_PORT = 11000;  // i.e. python / ros application

// Software in the loop communication
constexpr uint16_t SIL_CONTROLLER_PORT = 10001;  // i.e. twincat
constexpr uint16_t SIL_MODEL_PORT = 11001;       // i.e. gazebo simulation

constexpr uint16_t ROBOT_OFFSET = 10;

}  // namespace orc::com
