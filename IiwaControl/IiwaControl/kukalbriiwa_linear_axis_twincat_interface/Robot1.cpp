///////////////////////////////////////////////////////////////////////////////
// Robot1.cpp
#include "TcPch.h"
#pragma hdrstop

#include"Robot1.h"

RobotParameters robot1_parameters={
  //const char *name
  "Robot1",
  //AmsNetId netid
	//{128,130,84,36,2,1},
	//{192,168,10,20,2,1},128.130.84.36.2.1
	//{128,130,84,36,2,1}, /* Net-ID of Device 1 (EtherCAT-Master for Kuka Robot) */
	{0,0,0,0,0,0}, // AmsNetId will be set at runtime
    
  //KsoeSlaveParameters Robot1_ksoe_slave_parameters[ROBOT_NUM_SLAVE]
  {
    { NULL, //input_buffer will be set at runtime
      NULL, //output_buffer will be set at runtime
      1010,
      10,
      {
        33, { 0x02, 0x00, 0x10, 0x00, 0x1b, 0x00,
              0x01, 0x11, 0x00, 0x2d, 0x31, 0x01,
              0x00, 0x40, 0xba, 0x49, 0x0a, 0x00,
              0x10, 0x03, 0x03, 0x05, 0x03, 0x03,
              0x00, 0x40, 0x07, 0x00, 0x12, 0x00,
              0x00, 0x00, 0x00 }
      }
    },
    {
      NULL, //input_buffer will be set at runtime
      NULL, //output_buffer will be set at runtime
      1011,
      11,
      {
        48, { 0x02, 0x00, 0x10, 0x00, 0x2a, 0x00,
              0x01, 0x11, 0x00, 0x2d, 0x31, 0x01,
              0x00, 0x40, 0xba, 0x49, 0x0a, 0x00,
              0x14, 0x03, 0x03, 0x05, 0x03, 0x03,
              0x00, 0x40, 0x07, 0x00, 0x12, 0x00,
              0x00, 0x40, 0x83, 0x60, 0x05, 0x00,
              0x10, 0x03, 0x03, 0x05, 0x03, 0x03,
              0x00, 0x40, 0x07, 0x00, 0x21, 0x00 }
      }
    },
    {
      NULL, //input_buffer will be set at runtime
      NULL, //output_buffer will be set at runtime
      1012,
      12,
      {
        48, { 0x02, 0x00, 0x10, 0x00, 0x2a, 0x00,
              0x01, 0x11, 0x00, 0x2d, 0x31, 0x01,
              0x00, 0x40, 0x83, 0x60, 0x05, 0x00,
              0x12, 0x03, 0x03, 0x05, 0x03, 0x03,
              0x00, 0x40, 0x07, 0x00, 0x21, 0x00,
              0x00, 0x40, 0x81, 0x85, 0x03, 0x00,
              0x10, 0x03, 0x03, 0x05, 0x03, 0x03,
              0x00, 0x40, 0x07, 0x00, 0x4e, 0x00 }
      }
    },
    {
      NULL, //input_buffer will be set at runtime
      NULL, //output_buffer will be set at runtime
      1013,
      13,
      {
        48, { 0x02, 0x00, 0x10, 0x00, 0x2a, 0x00,
              0x01, 0x11, 0x00, 0x2d, 0x31, 0x01,
              0x00, 0x40, 0xc3, 0x55, 0x01, 0x00,
              0x30, 0x03, 0x03, 0x05, 0x03, 0x03,
              0x00, 0x40, 0x07, 0x00, 0x45, 0x00,
              0x00, 0x40, 0xc3, 0x55, 0x01, 0x00,
              0x20, 0x03, 0x03, 0x05, 0x03, 0x03,
              0x00, 0x40, 0x07, 0x00, 0x7a, 0x00 }
      }
    }
  },
  //FsoeSlaveParameters fsoe_slave_parameters;
  {
    NULL, //input_buffer will be set at runtime
    NULL, //output_buffer will be set at runtime
    9,
    6
  },

	//MotorcontrollerParameters RobotParameters.motorcontroller_parameters[ROBOT_NUM_MOTORCONTROLLER]
  {
	  { NULL, //input_buffer will be set at runtime
	  NULL, //output_buffer will be set at runtime
	  1
	  },
	  {
		  NULL, //input_buffer will be set at runtime
		  NULL, //output_buffer will be set at runtime
		  2
	  }
  },
  //unsigned char *NSR_INDICATOR;
  //will be set at runtime
  NULL,
  //unsigned char *NSR_INPUT;
  //will be sat at runtime
  NULL,
  //Axis axis[ROBOT_NUM_AXIS]
  //{ a1, a3, b1, T_ref, torque_upper_limit, torque_lower_limit,
  //  nominal_current, holding_current, maximum_current, current_resolution,
  //  gear_ratio, position_scale, position_limit, velocity_limit, velocity_controller_p,
  //  torque_resolution, nominal_torque, maximal_torque, cross_force_compensation,
  //  force_limit, position_limit_lower, position_limit_upper, velocity_limit, velocity_controller_p,
  //  position_filter_f_0, velocity_filter_f_0, torque_filter_f_0, torque_derivative_filter_f_0,
  //  axis_number
  //}
  {
	  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	  0.0, 0.0, 0.0, 0.0,
	  0.0, 0.0, 0.0, 0.0, 0.0,
	  0.0, 0.0, 0.0, 0,
	  ROBOT1_FORCE_LIMIT, 0.0, 0.0, 0.0, ROBOT1_FORCE_LIMIT*ROBOT1_VELOCITY_CONTROLLER_P_FACTOR,
      0.0, 0.0, 0.0, 0.0,
      -1
	  },

	  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		  0.0, 0.0, 0.0, 0.0,
		  0.0, 0.0, 0.0, 0.0, 0.0,
		  0.0, 0.0, 0.0, 0,
		  ROBOT1_FORCE_LIMIT, 0.0, 0.0, 0.0, ROBOT1_FORCE_LIMIT*ROBOT1_VELOCITY_CONTROLLER_P_FACTOR,
          0.0, 0.0, 0.0, 0.0,
      -2
	  },

    { 0.19798, -0.000016099, -0.00078731, 338.00, 4.02685, -4.02685,
      9.2195, 10.00, 20.00, AXIS_CURRENT_RESOLUTION(59999846L),
      -160, AXIS_POSITION_RESOLUTION(-160), AXIS_DEG2RAD(170.0), AXIS_DEG2RAD(85.0), 320.0*ROBOT1_VELOCITY_CONTROLLER_P_FACTOR,
      AXIS_TORQUE_RESOLUTION(674234), 320.00, 416.01, 8,
      0.0, 0.0, 0.0, 0.0, 0.0,
      1000, 1000, 1000, 1000,
	  1
    },
    { 0.19798, -0.000016099, -0.00078731, 338.00, 4.02685, -4.02685,
      9.2195, 10.00, 20.00, AXIS_CURRENT_RESOLUTION(59999846L),
      -160, AXIS_POSITION_RESOLUTION(-160), AXIS_DEG2RAD(120.0), AXIS_DEG2RAD(85.0), 320.0*ROBOT1_VELOCITY_CONTROLLER_P_FACTOR,
      AXIS_TORQUE_RESOLUTION(674234), 320.00, 416.01, -121,
      0.0, 0.0, 0.0, 0.0, 0.0,
      1000, 1000, 1000, 1000,
	  2
    },
    { 0.17479, -0.000043706, -0.00078731, 343.00, 3.31954, -3.31954,
      7.171, 7.80, 20.00, AXIS_CURRENT_RESOLUTION(59999846L),
      -160, AXIS_POSITION_RESOLUTION(-160), AXIS_DEG2RAD(170.0), AXIS_DEG2RAD(100.0), 176.0*ROBOT1_VELOCITY_CONTROLLER_P_FACTOR,
      AXIS_TORQUE_RESOLUTION(352387), 176.00, 228.00, -11,
      0.0, 0.0, 0.0, 0.0, 0.0,
      1000, 1000, 1000, 1000,
	  3
    },
    { 0.17479, -0.000043706, -0.00078731, 343.00, 3.31954, -3.31954,
      7.171, 7.80, 20.00, AXIS_CURRENT_RESOLUTION(59999846L),
      -160, AXIS_POSITION_RESOLUTION(-160), AXIS_DEG2RAD(120.0), AXIS_DEG2RAD(75.0), 176.0*ROBOT1_VELOCITY_CONTROLLER_P_FACTOR,
      AXIS_TORQUE_RESOLUTION(352387), 176.00, 228.00, -263,
      0.0, 0.0, 0.0, 0.0, 0.0,
      1000, 1000, 1000, 1000,
	  4
    },
    { 0.13777, -0.000085385, -0.00078731, 343.00, 2.10248, -2.10248,
      4.5637, 4.95, 18.20, AXIS_CURRENT_RESOLUTION(59999846L),
      -100, AXIS_POSITION_RESOLUTION(-100), AXIS_DEG2RAD(170.0), AXIS_DEG2RAD(130.0), 110.0*ROBOT1_VELOCITY_CONTROLLER_P_FACTOR,
      AXIS_TORQUE_RESOLUTION(230785), 110.00, 143.00, 177,
      0.0, 0.0, 0.0, 0.0, 0.0,
      1000, 1000, 1000, 1000,
	  5
    },
    { 0.074591, -0.000073865, -0.00078731, 343.00, 0.709083, -0.709083,
      2.8581, 3.5, 10.00, AXIS_CURRENT_RESOLUTION(59999846L),
      -160, AXIS_POSITION_RESOLUTION(-160), AXIS_DEG2RAD(120.0), AXIS_DEG2RAD(135.0), 40.0*ROBOT1_VELOCITY_CONTROLLER_P_FACTOR,
      AXIS_TORQUE_RESOLUTION(87491), 40.00, 46.00, -130,
      0.0, 0.0, 0.0, 0.0, 0.0,
      1000, 1000, 1000, 1000,
	  6
    },
    { 0.074591, -0.000073865, -0.00078731, 343.00, 0.709083, -0.709083,
      2.8581, 3.5, 10.00, AXIS_CURRENT_RESOLUTION(59999846L),
      -160, AXIS_POSITION_RESOLUTION(-160), AXIS_DEG2RAD(175.0), AXIS_DEG2RAD(135.0), 40.0*ROBOT1_VELOCITY_CONTROLLER_P_FACTOR,
      AXIS_TORQUE_RESOLUTION(87491), 40.00, 46.00, 384,
      0.0, 0.0, 0.0, 0.0, 0.0,
      1000, 1000, 1000, 1000,
	  7
    }
  }
};
