#include <gtest/gtest.h>
#include <stdexcept>

#include "orc/robots/Iiwa.h"
#include "orc/robots/Robot.h"

// These tests verify that Robot construction throws instead of calling
// std::exit on validation failures. Working directory is tests/ (per
// gtest_discover_tests WORKING_DIRECTORY), so models live at ../models.

TEST(RobotValidation, ThrowsOnMissingEndeffectorSite) {
    // iiwa_hanging has DOF=7 so the DOF check passes, but we ask for a
    // nonexistent endeffector site — constructor must throw.
    EXPECT_THROW(
        { orc::robots::Iiwa iiwa("../models/iiwa_hanging.mjb", 125e-6, "not_a_real_site_name"); },
        std::runtime_error);
}

TEST(RobotValidation, ThrowsOnMissingForceSensor) {
    EXPECT_THROW(
        {
            orc::robots::Iiwa iiwa("../models/iiwa_hanging.mjb", 125e-6, "",
                                   "not_a_real_force_sensor");
        },
        std::runtime_error);
}

TEST(RobotValidation, ThrowsOnDofMismatch) {
    // Loading a 7-DoF iiwa model into a 2-DoF Robot must throw on DOF check.
    EXPECT_THROW(
        { orc::robots::Robot<2> r("../models/iiwa_hanging.mjb", 125e-6); }, std::runtime_error);
}
