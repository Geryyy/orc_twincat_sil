#pragma once

struct AxisLimits {
  bool is9DOF; //Select the internal model: TRUE selects the 9DOF model (Robot_Frame), FALSE selects the 7DOF model (KUKA_Robot_Controller).

  double velocity_limit_fraction; //1, Fraction of the nominal axis velocity
  double motor_torque_limit_fraction; //1, Maximum torque fraction, which leads to a soft stop when exceeded.
  double motor_torque_saturation; //1, Maximum torque fraction, at which the current input saturates by software.
  double sensor_torque_limit_fraction; //1, Maximum allowed sensor value as fraction of nominal sensor torque
  double position_reserve; //rad, Additional software reserve for the position, which leads to a soft stop when exceeded. Applies only to KUKA joints
  double laxis_position_reserve; //m, Same as above for linear axes
  double tool_mass; //kg, Specifies the total mass of the tool attached to the robot flange.
  double tool_cog[3]; //m, Specifies the position of the center of gravity (cog) as [x,y,z].
  double tool_tcp[6]; //Specifies the distance (in m) and rotation (in rad) between the robot flange frame and the TCP frame as [x,y,z,yaw,pitch,roll].
  double world_param[6]; //Specifies the distance (in m) and rotation (in rad) between the robot base frame and the world frame as [x,y,z,yaw,pitch,roll].
  double workspace_limits[6]; // Specifies the area in m in which the endeffector is allowed to move

  double force_limit_fraction; //1, Maximum torque fraction, which leads to a soft stop when exceeded.
  double force_saturation_fraction; //1, Maximum torque fraction, at which the current input saturates by software.
};
