#pragma once

#include "TcDef.h"
#include "TcMath.h"
#include "TcInterfaces.h"
#include "MotionState.h"
#include "Axis.h"
#include "AxisLimits.h"
#include "AxisControl.h"
#include "AxisSensor.h"
#include "DT1.h"
#include "PT1.h"
#include "EventCode.h"
#include "EventHandler.h"

#define MOTORCONTROLLER_T_a (125e-6)
#define MOTORCONTROLLER_MOTION_WATCHDOG_TICKS ((DWORD)(400e-3/MOTORCONTROLLER_T_a)) //Ticks, equals 100 ms
#define OUTPUT_COOLDOWN_TIME 1000*8 //Ticks
#define MOTORCONTROLLER_HANDLE_ERROR_FLAG(name,axis,timer_active,timer_cleared,ksoe_slave_error,errorflag,acknowledgeflag) \
  if((errorflag) && !(acknowledgeflag)) { \
		if(timer_active == 0) { \
			timer_active = OUTPUT_COOLDOWN_TIME; \
			trace->Log(tlError, "%2d Motorcontroller::updateIOs(): Motion prevented by \"" name "\".", motorcontroller_id); \
		} \
  } \
  else if(!(errorflag) && (acknowledgeflag)) { \
		if (timer_cleared == 0) { \
			timer_cleared = OUTPUT_COOLDOWN_TIME; \
			trace->Log(tlError, "%2d Motorcontroller::updateIOs(): \"" name "\" cleared.", motorcontroller_id); \
		} \
  } \
  if(errorflag) { \
    event_handler->set(axis,motorcontroller_id,ksoe_slave_error,EventType::ERROR); \
  } \
  if((errorflag) && motion_error==EventCode::NONE) { \
    motion_error=(ksoe_slave_error); \
  } \
  (acknowledgeflag)=(errorflag)
#define MOTORCONTROLLER_HANDLE_SENSOR_WARNING_FLAG(name,axis,timer_active,timer_cleared,ksoe_slave_warning,warningflag,acknowledgeflag) \
  if((warningflag) && !(acknowledgeflag)) { \
		if(timer_active == 0) { \
			timer_active = OUTPUT_COOLDOWN_TIME; \
			trace->Log(tlWarning, "%2d Motorcontroller::checkCrcReadSensor(): Soft stop caused by \"" name "\".", motorcontroller_id); \
		} \
  } \
  else if(!(warningflag) && (acknowledgeflag)) { \
		if (timer_cleared == 0) { \
			timer_cleared = OUTPUT_COOLDOWN_TIME; \
			trace->Log(tlWarning, "%2d Motorcontroller::checkCrcReadSensor(): \"" name "\" cleared.", motorcontroller_id); \
		} \
  } \
  if(warningflag) { \
      event_handler->set(axis, motorcontroller_id, ksoe_slave_warning, EventType::WARNING); \
  } \
  if((warningflag) && sensor_warning==EventCode::NONE) { \
    sensor_warning=(ksoe_slave_warning); \
  } \
  (acknowledgeflag)=(warningflag)
#define MOTORCONTROLLER_HANDLE_MOTION_WARNING_FLAG(name,axis,timer_active,timer_cleared,ksoe_slave_warning,warningflag,acknowledgeflag) \
  if((warningflag) && !(acknowledgeflag)) { \
		if(timer_active == 0) { \
			timer_active = OUTPUT_COOLDOWN_TIME; \
			trace->Log(tlWarning, "%2d Motorcontroller::updateIOs(): Soft stop caused by \"" name "\".", motorcontroller_id); \
		} \
  } \
  else if(!(warningflag) && (acknowledgeflag)) { \
		if (timer_cleared == 0) { \
			timer_cleared = OUTPUT_COOLDOWN_TIME; \
			trace->Log(tlWarning, "%2d Motorcontroller::updateIOs(): \"" name "\" cleared.", motorcontroller_id); \
		} \
  } \
  if(warningflag) { \
      event_handler->set(axis,motorcontroller_id,ksoe_slave_warning,EventType::WARNING); \
  } \
  if((warningflag) && motion_warning==EventCode::NONE) { \
    motion_warning=(ksoe_slave_warning); \
  } \
  (acknowledgeflag)=(warningflag)
#define COOLDOWN_TIMER_FUNCTION(timer) \
  if((timer)>0) (timer)--; \
  else (timer)=0;
//Once the position limit is reached, the robot may be enabled again and only
//velocity away from this limit is allowed. To account for sensor noise, a
//tolerance band is defined additionally, which allows minimal movements further
//into the limit. This tolerance band is defined by MOTORCONTROLLER_VELOCITY_TOLERANCE.
#define MOTORCONTROLLER_VELOCITY_TOLERANCE (0.0015) //m/s
//hard software limit set x m from the hardware limit
#define MOTORCONTROLLER_POSITION_LIMIT_DISTANCE 0.05 //m
//Soft stop velocity tolerance
#define MOTORCONTROLLER_SOFT_STOP_VELOCITY_TOLERANCE MOTORCONTROLLER_VELOCITY_TOLERANCE //m/s
// Maximum acceptable velocity, taken from Festo Automation Suite
#define MOTORCONTROLLER_VELOCITY_LIMIT 2.7 //m/s

class Motorcontroller {
private:
	MotionState motion_state;
	MotionState motion_state_target;
	int motion_watchdog;
	int motorcontroller_id;
	void *input_buffer;
	void *output_buffer;
	CTcTrace *trace;
	const Axis *axis;
	const AxisLimits *limits;
	bool axis_control_motor_torque_nan;
	bool axis_control_motor_torque_limit;
	bool axis_control_motor_torque_saturated;
	bool axis_control_motor_velocity_nan;
	bool axis_control_motor_velocity_limit;
	bool axis_control_motor_position_nan;
	bool axis_control_motor_position_soft_limit;
	bool axis_control_motor_position_hard_limit;
	bool axis_control_fault;
	bool axis_control_internal_limit_active;
	bool axis_control_following_error;
	bool axis_sensor_position_soft_limit;
	bool axis_sensor_torque_limit;
	bool axis_sensor_velocity_limit;
	double motor_position_softstop;
	EventCode last_error;
	void setLastError(EventCode error);
	EventCode motion_error;
	EventCode sensor_error;
	EventCode sensor_warning;
	EventCode motion_warning;
	EventHandler* event_handler;
	PT1* position_filter;
	DT1* velocity_filter;

	USHORT motor_status_last;

	// cooldown timers for warnings and errors to debounce log
	int invalid_motor_torque_output_timer;
	int invalid_motor_torque_output_timer_cleared;
	int motor_torque_limit_output_timer;
	int motor_torque_limit_output_timer_cleared;
	int invalid_motor_velocity_output_timer;
	int invalid_motor_velocity_output_timer_cleared;
	int invalid_motor_position_output_timer;
	int invalid_motor_position_output_timer_cleared;
	int motor_position_soft_limit_output_timer;
	int motor_position_soft_limit_output_timer_cleared;
	int motor_position_hard_limit_output_timer;
	int motor_position_hard_limit_output_timer_cleared;
	int motor_velocity_limit_output_timer;
	int motor_velocity_limit_output_timer_cleared;
	int axis_position_soft_limit_output_timer;
	int axis_position_soft_limit_output_timer_cleared;
	int axis_velocity_limit_output_timer;
	int axis_velocity_limit_output_timer_cleared;
	int saturation_active_output_timer;
	int saturation_cleared_output_timer;
	int fault_output_timer;
	int fault_output_timer_cleared;
	int internal_limit_active_timer;
	int internal_limit_active_timer_cleared;
	int following_error_timer;
	int following_error_timer_cleared;

private:
	void setMotionState(MotionState s);
	void setMotionStateTarget(MotionState s);

	void feedMotionWatchdog();
	bool isModeOfOperationTorque();
	bool isModeOfOperationVelocity();
	bool isModeOfOperationPosition();

public:
	Motorcontroller(CTcTrace *trace, EventHandler *event_handler, int motorcontroller_id, void *input_buffer, void *output_buffer, const AxisLimits* limits, const Axis* axis);
	~Motorcontroller();
	void statemachineWriteControl(AxisControl *output, DWORD dc_time_L);
	void readSensor(AxisSensor *input);
	void reset();
	int getMotorcontrollerID();
	MotorcontrollerModeOfOperation getModeOfOperation();
	void setModeOfOperation(MotorcontrollerModeOfOperation mode_of_operation);
	EventCode getLastError();
	EventCode getMotionError();
	EventCode getSensorError();
	EventCode getMotionWarning();
	EventCode getSensorWarning();
	MotionState getMotionStateTarget();
	MotionState getMotionState();
	bool canEnableMotion();
	bool isMotionEnabled();
	bool enableMotion();
	void disableMotion();
	void performSoftStop(AxisSensor *input, AxisControl *output, DWORD dc_time_L);
	void cooldownTimer();
};
