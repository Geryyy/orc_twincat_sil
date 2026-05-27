#pragma once

#include <orc/control/Controller.h>
#include <orc/robots/Robot.h>
#include <orc/util/Time.h>
#include <orc/util/import_eigen.h>

namespace orc::robots {

class DummyRobot : public Robot<2> {
private:
    int endeffector_site_id;  // The model's body ID for the end-effector
    int base_body_id;

public:
    constexpr static int DOF = 2;
    using JointCTController = typename orc::control::JointCTController<DOF>;
    using JointCTParameter = typename orc::control::JointCTParameter<DOF>;
    using CartesianCTParameter = typename orc::control::CartesianCTParameter<DOF>;
    using Robot<DOF>::Robot;
    using Time = orc::Time;

    DummyRobot(const char* mjb_path, Time Ta)
        : Robot(mjb_path, JointCTParameter(), CartesianCTParameter(), Ta) {}

    JointVector update(Time t, JointVector& q_act) { return JointVector::Zero(); }

    void reset(JointVector& q_act) {}
};
}  // namespace orc::robots
