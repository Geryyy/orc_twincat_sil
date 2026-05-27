#pragma once

struct AxisSensor {
  //Sensor data

	// KUKA
  double motor_torque_feedback; //Nm
  double torque; //Nm
  double position; //rad
  double temperature; //K
  double current_rms; //A
  double i2t_percent; //I2t monitoring in % of holding current

  double position_filtered; //rad
  double velocity_filtered; //rad/s
  double torque_filtered; //Nm
  double torque_derivative_filtered; //Nm/s

  // linear axes

  double force; //Nm
  double position_linear; //m
  double velocity; //m/s

  double position_linear_filtered; //m
  double velocity_linear_filtered; //m/s


};


