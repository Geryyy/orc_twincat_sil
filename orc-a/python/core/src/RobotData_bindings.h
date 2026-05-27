#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>

#include <orc/robots/RobotData.h>

namespace nb = nanobind;
using namespace nb::literals;

template <int DOF>
void add_RobotData(nb::module_& m) {
    using RobotData = orc::robots::RobotData<DOF>;

    nb::class_<RobotData>(m, "RobotData")
        .def_rw("t", &RobotData::t)
        .def_rw("Ta", &RobotData::Ta)
        .def_rw("endeffector_site_id", &RobotData::endeffector_site_id)

        // .def_rw("q_act", &RobotData::q_act)
        // .def_rw("q_dot_act", &RobotData::q_dot_act)
        // .def_rw("qfrc_bias", &RobotData::qfrc_bias)
        // .def_rw("q_dotdot_act", &RobotData::q_dotdot_act)
        .def_rw("tau_motor", &RobotData::tau_motor)
        .def_rw("tau_sens", &RobotData::tau_sens)

        .def_rw("q_d", &RobotData::q_d)
        .def_rw("q_dot_d", &RobotData::q_dot_d)
        .def_rw("q_dotdot_d", &RobotData::q_dotdot_d)
        .def_rw("pose_d", &RobotData::pose_d)
        .def_rw("x_dot_d", &RobotData::x_dot_d)
        .def_rw("x_dotdot_d", &RobotData::x_dotdot_d)
        .def_rw("q_d_NS", &RobotData::q_d_NS)
        .def_rw("force_d", &RobotData::force_d)
        .def_rw("collision_detected", &RobotData::collision_detected)

        // .def_rw("M_off", &RobotData::M_off)
        // .def_rw("M", &RobotData::M)
        // .def_rw("J", &RobotData::J)
        // .def_rw("J_dot", &RobotData::J_dot)
        // .def_rw("J_inv", &RobotData::J_inv);
        // .def_rw("H_0_e", &RobotData::H_0_e)
        // .def_rw("pose_act", &RobotData::pose_act)
        // .def_rw("x_dot_act", &RobotData::x_dot_act)
        // .def_rw("x_dotdot_act", &RobotData::x_dotdot_act)
        // .def_rw("G", &RobotData::G)

        .def_rw("tau_primary", &RobotData::tau_primary)

        .def_rw("force_measurement", &RobotData::force_measurement)
        .def_rw("force_compensated", &RobotData::force_compensated);
}
