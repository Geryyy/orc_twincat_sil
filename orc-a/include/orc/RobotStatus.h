#pragma once

#include <cstdint>

namespace orc::logic {
enum class RobotStatus : uint16_t {
    OFF = 0,
    ENABLE = 1,
    GRAVCOMP = 2,
};
}
