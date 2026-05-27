#pragma once

#include"KsoeSlaveParameters.h"
#include"FsoeSlaveParameters.h"
#include"Robot.h"
#include"MotorcontrollerParameters.h"


struct RobotParameters {
  const char *name;
  AmsNetId netid;
  KsoeSlaveParameters ksoe_slave_parameters[ROBOT_NUM_SLAVE];
  FsoeSlaveParameters fsoe_slave_parameters;
  MotorcontrollerParameters motorcontroller_parameters[ROBOT_NUM_MOTORCONTROLLER];
  unsigned char *NSR_INDICATOR;
  unsigned char *NSR_INPUT;
  Axis axis[ROBOT_NUM_AXIS];
};
