#pragma once

#define AXIS_CURRENT_RESOLUTION(current_scale) ((double)((current_scale)*1e-6/32768.0))
//The "-" sign comes from the fact, that the sensors show positive values
//from the view of the link side. However, the robot model contains tau=K*(theta-q),
//which is positive from the motor side. To stick with this convention, the correct
//sign is introduced in the low level.
#define AXIS_TORQUE_RESOLUTION(torque_scale) ((double)(-(torque_scale)*1e-3/32768.0))
#define AXIS_RATIO_MOTOR_ENCODER 10
#define AXIS_POSITION_RESOLUTION(gear_ratio) ((double)(2*PI/(AXIS_RATIO_MOTOR_ENCODER*(gear_ratio)*65536.0)))
#define AXIS_CURRENT_TO_TORQUE(axis,current,T) ((double)(((axis)->a1*(current)+(axis)->a3*(current)*(current)*(current))*(1+(axis)->b1*((T)-(axis)->T_ref))))
#define AXIS_DEG2RAD(a) ((double)((a)*PI/180.0))
#define AXIS_RAD2DEG(a) ((double)((a)*180.0/PI))

#define AXIS_FORCE_TO_TORQUE(force) ((int)(force*16.7858))	 // Kraft vorgeben �ber Simulink -> mit Festo Automation Suite Drehmoment messen (0.32Nm) -> �ber Radius des Zahnriemens auf Kraft umrechnen (F=tau/r) (16.085N) -> Torque_actual_value messen (270N) -> Umrechnungsfaktor berechnen (270/16.085 = 16.7858)
#define AXIS_TORQUE_TO_FORCE(torque) ((double)(torque/16.7858))
#define AXIS_r_BELTDRIVE (0.125/(2*PI)) //Radius des Zahnriemens = Vorschubkonstante/2*PI = Wirkdurchmesser/2, gleich f�r alle Achsen
#define AXIS_LINEAR_POSITION_RESOLUTION (1.0/1000000.0)
#define AXIS_VELOCITY_RESOLUTION (1.0/1000000.0)



struct Axis {
  //Motor: KT-Characteristic
  double a1;
  double a3;
  double b1;
  double T_ref;
  double torque_upper_limit;
  double torque_lower_limit;
  double nominal_current;
  double holding_current;
  double maximum_current;
  double current_resolution; // A/Ink

  //Gear
  double gear_ratio;
  double position_resolution; // rad/Ink
  double position_limit; // rad
  double velocity_limit; // rad/s
  double velocity_controller_p;

  //Torque sensor
  double sensor_torque_resolution; // Nm/Ink
  double sensor_nominal_torque;
  double sensor_maximal_torque;
  short cross_force_compensation;

  //Linear axis
  double force_limit;
  double position_limit_lower;
  double position_limit_upper;
  double linear_velocity_limit;
  double linear_velocity_controller_p;

  //Filters
  double position_filter_f_0;
  double velocity_filter_f_0;
  double torque_filter_f_0;
  double torque_derivative_filter_f_0;

  //Axis number
  int axis_number;
};

