#pragma once

#include <cmath>

#include "orc/OrcTypes.h"

namespace orc::util {

inline auto wrap_to_pi = [](double x) {
    double y = std::fmod(x + M_PI, 2 * M_PI);
    if (y < 0)
        y += 2 * M_PI;
    return y - M_PI;
};

}  // namespace orc::util
