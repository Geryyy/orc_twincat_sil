#pragma once

#include "TcDef.h"
#include "TcMath.h"
#include "TcInterfaces.h"
#include "MotionState.h"

#define KSOESLAVE_FILTER_T_a (125e-6)

#include "KsoeState.h"
#include "KsoeSlaveType.h"
#include "EventCode.h"
#include "kuka_interfaceServices.h"
#include "PDSParameters.h"
#include "Axis.h"
#include "AxisLimits.h"
#include "AxisControl.h"
#include "AxisSensor.h"
#include "DT1.h"
#include "PT1.h"
#include "I2tMonitor.h"
#include "EventHandler.h"

#define KSOESLAVE_SEQUENCE_NUMBER_INITIAL 0x0001
#define KSOESLAVE_SEQUENCE_NUMBER_START   0x1000
#define KSOESLAVE_MOTION_WATCHDOG_TICKS (100000/125) //Ticks, equals 100 ms
#define OUTPUT_COOLDOWN_TIME 1000*8 //Ticks
#define KSOESLAVE_HANDLE_ERROR_FLAG(name,axis,timer_active,timer_cleared,ksoe_slave_error,errorflag,acknowledgeflag) \
  if((errorflag) && !(acknowledgeflag)) { \
		if(timer_active == 0) { \
			timer_active = OUTPUT_COOLDOWN_TIME; \
			if(axis==0) { \
    trace->Log(tlError, "%2d KsoeSlave::updateIOs(): Motion prevented by \"" name "\".", connection_id); \
  } \
			else { \
				trace->Log(tlError, "KsoeSlave::updateIOs(): Motion prevented by \"" name "\" of axis %d.", axis); \
			} \
		} \
  } \
  else if(!(errorflag) && (acknowledgeflag)) { \
		if (timer_cleared == 0) { \
			timer_cleared = OUTPUT_COOLDOWN_TIME; \
			if (axis==0) { \
    trace->Log(tlError, "%2d KsoeSlave::updateIOs(): \"" name "\" cleared.", connection_id); \
  } \
			else { \
				trace->Log(tlError, "KsoeSlave::updateIOs(): \"" name "\" of axis %d cleared.", axis); \
			} \
		} \
  } \
  if(errorflag) { \
    event_handler->set(axis,connection_id,ksoe_slave_error,EventType::ERROR); \
  } \
  if((errorflag) && motion_error==EventCode::NONE) { \
    motion_error=(ksoe_slave_error); \
  } \
  (acknowledgeflag)=(errorflag)
#define KSOESLAVE_HANDLE_SENSOR_WARNING_FLAG(name,axis,timer_active,timer_cleared,ksoe_slave_warning,warningflag,acknowledgeflag) \
  if((warningflag) && !(acknowledgeflag)) { \
		if(timer_active == 0) { \
			timer_active = OUTPUT_COOLDOWN_TIME; \
			trace->Log(tlWarning, "KsoeSlave::checkCrcReadSensor(): Soft stop caused by \"" name "\" of axis %d.", axis); \
		} \
  } \
  else if(!(warningflag) && (acknowledgeflag)) { \
		if (timer_cleared == 0) { \
			timer_cleared = OUTPUT_COOLDOWN_TIME; \
			trace->Log(tlWarning, "KsoeSlave::checkCrcReadSensor(): \"" name "\" of axis %d cleared.", axis); \
		} \
  } \
  if(warningflag) { \
      event_handler->set(axis, connection_id, ksoe_slave_warning, EventType::WARNING); \
  } \
  if((warningflag) && sensor_warning==EventCode::NONE) { \
    sensor_warning=(ksoe_slave_warning); \
  } \
  (acknowledgeflag)=(warningflag)
#define KSOESLAVE_HANDLE_MOTION_WARNING_FLAG(name,axis,timer_active,timer_cleared,ksoe_slave_warning,warningflag,acknowledgeflag) \
  if((warningflag) && !(acknowledgeflag)) { \
		if(timer_active == 0) { \
			timer_active = OUTPUT_COOLDOWN_TIME; \
			 trace->Log(tlWarning, "KsoeSlave::updateIOs(): Soft stop caused by \"" name "\" of axis %d.", axis); \
		} \
  } \
  else if(!(warningflag) && (acknowledgeflag)) { \
		if (timer_cleared == 0) { \
			timer_cleared = OUTPUT_COOLDOWN_TIME; \
			 trace->Log(tlWarning, "KsoeSlave::updateIOs(): \"" name "\" of axis %d cleared.", axis); \
		} \
  } \
  if(warningflag) { \
      event_handler->set(axis,connection_id,ksoe_slave_warning,EventType::WARNING); \
  } \
  if((warningflag) && motion_warning==EventCode::NONE) { \
    motion_warning=(ksoe_slave_warning); \
  } \
  (acknowledgeflag)=(warningflag)

//Once the position limit is reached, the robot may be enabled again and only
//velocity away from this limit is allowed. To account for sensor noise, a
//tolerance band is defined additionally, which allows minimal movements further
//into the limit. This tolerance band is defined by KSOESLAVE_VELOCITY_TOLERANCE.
#define KSOESLAVE_VELOCITY_TOLERANCE (0.1*PI/180.0) //rad/s
//hard software limit set x radians from the hardware limit
#define KSOESLAVE_POSITION_LIMIT_DISTANCE (3.0*PI/180) //rad
//Soft stop velocity tolerance
#define KSOESLAVE_SOFT_STOP_VELOCITY_TOLERANCE (0.1*PI/180.0) //rad/s
//I2t limit, given as factor of the holding current (see Axis.h, Robot1.cpp)
#define KSOELSAVE_I2TMONITOR_FACTOR 1
//Absolute current limit, given as factor of the maximum current (see Axis.h, Robot1.cpp)
#define KSOELSAVE_CURRENT_LIMIT_FACTOR 1

class KsoeSlave {
  private:
    unsigned char motor_control_counter;
    unsigned int state_machine_counter;
    USHORT sequence_number_master;
    USHORT sequence_number_slave;
    KsoeState state;
    MotionState motion_state1;
    MotionState motion_state2;
    MotionState motion_state_target;
    int motion_watchdog1;
    int motion_watchdog2;
    int state_variable;
    bool communication_toggle_bit;
    int connection_id;
    KsoeState previous_command_slave;
    KsoeSlaveType type;
    void *input_buffer;
    void *output_buffer;
    CTcTrace *trace;
    PDSParameters* parameters;
    const Axis *axis1;
    const Axis *axis2;
    const AxisLimits *limits;
    bool axis1_control_motor_torque_nan;
    bool axis2_control_motor_torque_nan;
    bool axis1_control_motor_torque_limit;
    bool axis2_control_motor_torque_limit;
    bool axis1_control_motor_torque_saturated;
    bool axis2_control_motor_torque_saturated;
    bool axis1_control_current_limit;
    bool axis2_control_current_limit;
    bool axis1_sensor_position_soft_limit;
    bool axis2_sensor_position_soft_limit;
    bool axis1_sensor_torque_limit;
    bool axis2_sensor_torque_limit;
    bool axis1_sensor_velocity_limit;
    bool axis2_sensor_velocity_limit;
    bool axis1_sensor_i2t_limit;
    bool axis2_sensor_i2t_limit;
    EventCode last_error;
    void setLastError(EventCode error);
    void setLastError(EventCode error, int axis_number);
    USHORT motor_temperature;
    EventCode motion_error;
    EventCode sensor_error;
    EventCode sensor_warning;
    EventCode motion_warning;
    EventHandler* event_handler;
    PT1* position1_filter;
    PT1* position2_filter;
    DT1* velocity1_filter;
    DT1* velocity2_filter;
    PT1* torque1_filter;
    PT1* torque2_filter;
    DT1* torque_derivative1_filter;
    DT1* torque_derivative2_filter;
    I2tMonitor* i2t_monitor1;
    I2tMonitor* i2t_monitor2;

    USHORT motor_status1_last;
    USHORT motor_status2_last;

	// cooldown timers for warnings and errors to debounce log
	int position_soft_limit_output_timer[2];
	int position_soft_limit_output_timer_cleared[2];
    int i2t_limit_output_timer[2];
    int i2t_limit_output_timer_cleared[2];
    int current_limit_output_timer[2];
    int current_limit_output_timer_cleared[2];
    int motor_warning_i2t_output_timer[2];
    int motor_warning_i2t_output_timer_cleared[2];
    int motor_warning_temperature_output_timer[2];
    int motor_warning_temperature_output_timer_cleared[2];
	int velocity_limit_output_timer[2];
	int velocity_limit_output_timer_cleared[2];
	int sensor_torque_limit_output_timer[2];
	int sensor_torque_limit_output_timer_cleared[2];
	int invalid_motor_torque_output_timer[2];
	int invalid_motor_torque_output_timer_cleared[2];
	int motor_torque_limit_output_timer[2];
	int motor_torque_limit_output_timer_cleared[2];
	int error_output_timer[2];
	int error_output_timer_cleared[2];
	int saturation_output_timer_active[2];
	int saturation_output_timer_cleared[2];
	int runtime_error_output_timer;
	int runtime_error_output_timer_cleared;


  private:
    static USHORT crcTable[256];
    static USHORT previousSequenceNumber(USHORT s);

    void increaseSequenceNumber();
    void statemachine(DWORD dc_time_L);
    void setState(KsoeState s);
    void setMotionState1(MotionState s);
    void setMotionState2(MotionState s);
    void setMotionStateTarget(MotionState s);

    void feedMotionWatchdog1();
    void feedMotionWatchdog2();
    bool doCrc(bool generate, KsoePacket* safe_data, USHORT sequence_number);
    void generateCrc();
    bool checkCrc();
    
    double torque2current(const Axis *axis, double torque);
    double current2torque(const Axis *axis, double current);

	void timer_function(int &timer);

  public:
    KsoeSlave(CTcTrace *trace, EventHandler* event_handler, int connection_id, void *input_buffer, void *output_buffer, PDSParameters* parameters, const AxisLimits* limits, const Axis* axis1);
    KsoeSlave(CTcTrace *trace, EventHandler* event_handler, int connection_id, void *input_buffer, void *output_buffer, PDSParameters* parameters, const AxisLimits* limits, const Axis* axis1, const Axis* axis2);
    ~KsoeSlave();
    void statemachineWriteControlGenerateCrc(AxisControl *output1, AxisControl *output2, DWORD dc_time_L);
    void checkCrcReadSensor(AxisSensor *input1, AxisSensor *input2);
    void reset();
    int getConnectionID();
    EventCode getLastError();
    EventCode getMotionError();
    EventCode getSensorError();
    EventCode getMotionWarning();
    EventCode getSensorWarning();
    KsoeState getState();
    MotionState getMotionStateTarget();
    bool canEnableMotion();
    bool isMotionEnabled();
    bool enableMotion();
    void disableMotion();
    bool hasValidInput();
    void setMotorTemperature(USHORT motor_temperature);
    void performSoftStop(AxisSensor *input1, AxisSensor *input2, AxisControl *output1, AxisControl *output2, DWORD dc_time_L);
	void cooldownTimer();
};
