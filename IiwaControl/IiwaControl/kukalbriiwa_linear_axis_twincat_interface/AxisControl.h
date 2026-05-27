#pragma once

#include"MotorcontrollerModeOfOperation.h"

struct AxisControl {
  //Control data
  double motor_torque; //Nm

  double motor_force; //F
  double motor_position; //m
  double motor_velocity; //m/s

  bool motor_enable; //true/false

  MotorcontrollerModeOfOperation mode_of_operation;
};
