///////////////////////////////////////////////////////////////////////////////
// Motorcontroller.cpp
#include "TcPch.h"
#pragma hdrstop

#include "Motorcontroller.h"

Motorcontroller::Motorcontroller(CTcTrace *trace, EventHandler *event_handler, int motorcontroller_id, void *input_buffer, void *output_buffer, const AxisLimits* limits, const Axis* axis) {
	this->trace = trace;
	this->event_handler = event_handler;
	this->motorcontroller_id = motorcontroller_id;
	this->input_buffer = input_buffer;
	this->output_buffer = output_buffer;
	this->axis = axis;
	this->limits = limits;
	this->position_filter = new PT1(axis->position_filter_f_0);
	this->velocity_filter = new DT1(axis->velocity_filter_f_0);
	((MotorcontrollerOutput*)output_buffer)->Modes_of_operation = (uint8_t)MotorcontrollerModeOfOperation::NONE;
	reset();

	this->invalid_motor_torque_output_timer = 0;
	this->invalid_motor_torque_output_timer_cleared = 0;
	this->motor_torque_limit_output_timer = 0;
	this->motor_torque_limit_output_timer_cleared = 0;
	this->invalid_motor_velocity_output_timer = 0;
	this->invalid_motor_velocity_output_timer_cleared = 0;
	this->invalid_motor_position_output_timer = 0;
	this->invalid_motor_position_output_timer_cleared = 0;
	this->motor_position_soft_limit_output_timer = 0;
	this->motor_position_soft_limit_output_timer_cleared = 0;
	this->motor_position_hard_limit_output_timer = 0;
	this->motor_position_hard_limit_output_timer_cleared = 0;
	this->motor_velocity_limit_output_timer = 0;
	this->motor_velocity_limit_output_timer_cleared = 0;
	this->axis_position_soft_limit_output_timer = 0;
	this->axis_position_soft_limit_output_timer_cleared = 0;
	this->axis_velocity_limit_output_timer = 0;
	this->axis_velocity_limit_output_timer_cleared = 0;
	this->saturation_active_output_timer = 0;
	this->saturation_cleared_output_timer = 0;
	this->fault_output_timer = 0;
	this->fault_output_timer_cleared = 0;
	this->internal_limit_active_timer = 0;
	this->internal_limit_active_timer_cleared = 0;
	this->following_error_timer = 0;
	this->following_error_timer_cleared = 0;
}

Motorcontroller::~Motorcontroller() {
	delete position_filter;
	delete velocity_filter;
}

void Motorcontroller::reset() {
	trace->Log(tlVerbose, "%2d Motorcontroller::reset()", motorcontroller_id);
	//Preserve value of Modes_of_operation
	uint8_t tmp = ((MotorcontrollerOutput*)output_buffer)->Modes_of_operation;
	memset(output_buffer, 0, sizeof(MotorcontrollerOutput));
	((MotorcontrollerOutput*)output_buffer)->Modes_of_operation = tmp;
	motion_watchdog = 0;
	motion_state_target = MotionState::STOP;
	motion_state = MotionState::STOP;
	last_error = EventCode::NONE;
	motion_error = EventCode::NONE;
	sensor_error = EventCode::NONE;
	motion_warning = EventCode::NONE;
	sensor_warning = EventCode::NONE;
	axis_control_motor_torque_nan = false;
	axis_control_motor_torque_limit = false;
	axis_control_motor_torque_saturated = false;
	axis_control_motor_velocity_nan = false;
	axis_control_motor_velocity_limit = false;
	axis_control_motor_position_nan = false;
	axis_control_motor_position_soft_limit = false;
	axis_control_motor_position_hard_limit = false;
	axis_control_fault = false;
	axis_control_internal_limit_active = false;
	axis_control_following_error = false;
	axis_sensor_position_soft_limit = false;
	axis_sensor_torque_limit = false;
	axis_sensor_velocity_limit = false;
	motor_position_softstop = 0.0;
	position_filter->reset();
	velocity_filter->reset();
}

void Motorcontroller::setMotionState(MotionState s) {
	trace->Log(tlInfo, "%2d Motorcontroller::setMotionState(): %s (0x%02x) -> %s (0x%02x)", motorcontroller_id, toString(motion_state), motion_state, toString(s), s);
	motion_state = s;
}

void Motorcontroller::setMotionStateTarget(MotionState s) {
	if (s != MotionState::MOTION) s = MotionState::STOP;
	trace->Log(tlInfo, "%2d Motorcontroller::setMotionStateTarget(): 0x%02x -> 0x%02x", motorcontroller_id, motion_state_target, s);
	motion_state_target = s;
}

MotionState Motorcontroller::getMotionStateTarget() {
	return motion_state_target;
}

MotionState Motorcontroller::getMotionState() {
	return motion_state;
}

void Motorcontroller::feedMotionWatchdog() {
	motion_watchdog = 0;
}

bool Motorcontroller::isModeOfOperationTorque()
{
	MotorcontrollerOutput* output = (MotorcontrollerOutput*)output_buffer;
	return (MotorcontrollerModeOfOperation)output->Modes_of_operation == MotorcontrollerModeOfOperation::PROFILE_TORQUE ||
		(MotorcontrollerModeOfOperation)output->Modes_of_operation == MotorcontrollerModeOfOperation::CYCLIC_SYNCHRONOUS_TORQUE ||
		(MotorcontrollerModeOfOperation)output->Modes_of_operation == MotorcontrollerModeOfOperation::USER_TORQUE;
}

bool Motorcontroller::isModeOfOperationVelocity()
{
	MotorcontrollerOutput* output = (MotorcontrollerOutput*)output_buffer;
	return (MotorcontrollerModeOfOperation)output->Modes_of_operation == MotorcontrollerModeOfOperation::PROFILE_VELOCITY ||
		(MotorcontrollerModeOfOperation)output->Modes_of_operation == MotorcontrollerModeOfOperation::CYCLIC_SYNCHRONOUS_VELOCITY ||
		(MotorcontrollerModeOfOperation)output->Modes_of_operation == MotorcontrollerModeOfOperation::USER_VELOCITY;
}

bool Motorcontroller::isModeOfOperationPosition()
{
	MotorcontrollerOutput* output = (MotorcontrollerOutput*)output_buffer;
	return (MotorcontrollerModeOfOperation)output->Modes_of_operation == MotorcontrollerModeOfOperation::PROFILE_POSITION ||
		(MotorcontrollerModeOfOperation)output->Modes_of_operation == MotorcontrollerModeOfOperation::INTERPOLATED_POSITION ||
		(MotorcontrollerModeOfOperation)output->Modes_of_operation == MotorcontrollerModeOfOperation::CYCLIC_SYNCHRONOUS_POSITION ||
		(MotorcontrollerModeOfOperation)output->Modes_of_operation == MotorcontrollerModeOfOperation::USER_POSITION;
}

void Motorcontroller::readSensor(AxisSensor *sensor) {
	MotorcontrollerInput* input = (MotorcontrollerInput*)input_buffer;

	//Check in every cycle if the error still persists.
	sensor_error = EventCode::NONE;
	sensor_warning = EventCode::NONE;
	//Calculate the sensor signals. This should always be possible as soon as the slave
	//sends data. Therefore, check only for the reset state.
	//Read the measured motor torque and calculate it to the downthrust side (after the harmonic drive)
	sensor->force = AXIS_TORQUE_TO_FORCE(input->Torque_actual_value);
	sensor->position_linear = input->Position_actual_value*AXIS_LINEAR_POSITION_RESOLUTION;
	sensor->velocity = input->Velocity_actual_value*AXIS_VELOCITY_RESOLUTION;
	sensor->position_linear_filtered = position_filter->updateAndOutput(sensor->position_linear);
	sensor->velocity_linear_filtered = velocity_filter->updateAndOutput(sensor->position_linear);
	//Perform all checks on the filtered signals to prevent spontaneous measurement errors from
	//causing an emergency stop.
	//Check soft position limit
	if (sensor->position_linear>(axis->position_limit_upper - limits->laxis_position_reserve) ||
		sensor->position_linear<(axis->position_limit_lower + limits->laxis_position_reserve)) {
		//Position limit for axis was exceeded. Check if the velocity is directed further into the limit.
		double position_center = (axis->position_limit_upper + axis->position_limit_lower) / 2.0;
		MOTORCONTROLLER_HANDLE_SENSOR_WARNING_FLAG("Linear Axis Position Soft Limit", axis->axis_number, axis_position_soft_limit_output_timer, axis_position_soft_limit_output_timer_cleared, EventCode::MotorcontrollerWarning_POSITION_SOFT_LIMIT,
			sensor->position_linear>position_center && sensor->velocity_linear_filtered>MOTORCONTROLLER_VELOCITY_TOLERANCE ||
			sensor->position_linear<position_center && sensor->velocity_linear_filtered<-MOTORCONTROLLER_VELOCITY_TOLERANCE,
			axis_sensor_position_soft_limit);

	}
	//Check hard position limit
	if (sensor->position_linear>(axis->position_limit_upper - MOTORCONTROLLER_POSITION_LIMIT_DISTANCE) ||
		sensor->position_linear<(axis->position_limit_lower + MOTORCONTROLLER_POSITION_LIMIT_DISTANCE)) {
		//Position limit for axis was exceeded. Check if the velocity is directed further into the limit.
		double position_center = (axis->position_limit_upper + axis->position_limit_lower) / 2.0;
		if (sensor->position_linear>position_center && sensor->velocity_linear_filtered>MOTORCONTROLLER_VELOCITY_TOLERANCE ||
			sensor->position_linear<position_center && sensor->velocity_linear_filtered<-MOTORCONTROLLER_VELOCITY_TOLERANCE) {
			if (getMotionStateTarget() != MotionState::STOP) {
				trace->Log(tlError, "%2d Motorcontroller::readSensor(): Motion aborted due to position limit in axis (%.2f m).", motorcontroller_id, sensor->position_linear);
			}
			sensor_error = EventCode::MotorcontrollerError_POSITION_LIMIT_HARD;
		}
	}
	//Check velocity limit
	MOTORCONTROLLER_HANDLE_SENSOR_WARNING_FLAG("Axis Velocity Limit", axis->axis_number, axis_velocity_limit_output_timer, axis_velocity_limit_output_timer_cleared, EventCode::MotorcontrollerWarning_VELOCITY_LIMIT,
		fabs_(sensor->velocity_linear_filtered)>axis->linear_velocity_limit*limits->velocity_limit_fraction,
		axis_sensor_velocity_limit);

	//Check hard velocity limit
	// This limit is not parameterizable and safeguards from extremly high velocity limit parameters
	MOTORCONTROLLER_HANDLE_SENSOR_WARNING_FLAG("Axis Hard Velocity Limit", axis->axis_number, axis_velocity_limit_output_timer, axis_velocity_limit_output_timer_cleared, EventCode::MotorcontrollerWarning_VELOCITY_LIMIT,
		fabs_(sensor->velocity_linear_filtered) > MOTORCONTROLLER_VELOCITY_LIMIT,
		axis_sensor_velocity_limit);
}

MotorcontrollerModeOfOperation Motorcontroller::getModeOfOperation() {
	return (MotorcontrollerModeOfOperation)(((MotorcontrollerOutput*)output_buffer)->Modes_of_operation);
}

void Motorcontroller::setModeOfOperation(MotorcontrollerModeOfOperation mode_of_operation) {
	MotorcontrollerModeOfOperation mode_of_operation_old;
	if (!isMotionEnabled()) {
		MotorcontrollerOutput* output = (MotorcontrollerOutput*)output_buffer;
		mode_of_operation_old = (MotorcontrollerModeOfOperation)output->Modes_of_operation;
		if (mode_of_operation_old != mode_of_operation) {
			trace->Log(tlAlways, "%2d Motorcontroller::setModeOfOperation(): %s (0x%02x) -> %s (0x%02x)", motorcontroller_id, toString(mode_of_operation_old), mode_of_operation_old, toString(mode_of_operation), mode_of_operation);
		}
		output->Modes_of_operation = (uint8_t)mode_of_operation;
	}
}

void Motorcontroller::statemachineWriteControl(AxisControl *control, DWORD dc_time_L) {
	MotorcontrollerInput* input = (MotorcontrollerInput*)input_buffer;
	MotorcontrollerOutput* output = (MotorcontrollerOutput*)output_buffer;
	double motor_force;

	motion_watchdog++;
	if (motion_watchdog >= MOTORCONTROLLER_MOTION_WATCHDOG_TICKS) {
		trace->Log(tlError, "%2d Motorcontroller::updateIOs(): Motion aborted by watchdog in state %s (0x%02x), target %s (0x%02x).",
			motorcontroller_id, toString(motion_state), motion_state, toString(motion_state_target), motion_state_target);
		setLastError(EventCode::MotorcontrollerError_MOTION_WATCHDOG);
	}

	//Reset slave error states. Only the enabled motors are checked.
	motion_error = EventCode::NONE;
	motion_warning = EventCode::NONE;

	if (control->motor_enable) {
		//MOTORCONTROLLER_HANDLE_ERROR_FLAG() sets local variable "motion_error", if an error is present
		//Only check for errors if the corresponding motor is enabled.
		//Calculate the motor torque from the downthrust desired torque.
		motor_force = 0;
		MOTORCONTROLLER_HANDLE_MOTION_WARNING_FLAG("Invalid Motor Torque", axis->axis_number, invalid_motor_torque_output_timer, invalid_motor_torque_output_timer_cleared, EventCode::MotorcontrollerWarning_INVALID_MOTOR_TORQUE,
			isnan_(control->motor_force) || !isfinite_(control->motor_force),
			axis_control_motor_torque_nan);
		//current is limited by torque2current() additionally to the nominal current.
		MOTORCONTROLLER_HANDLE_MOTION_WARNING_FLAG("Motor Torque Limit", axis->axis_number, motor_torque_limit_output_timer, motor_torque_limit_output_timer_cleared, EventCode::MotorcontrollerWarning_MOTOR_TORQUE_LIMIT,
			isModeOfOperationTorque() && fabs_(control->motor_force)>axis->force_limit*limits->force_limit_fraction,
			axis_control_motor_torque_limit);
		MOTORCONTROLLER_HANDLE_MOTION_WARNING_FLAG("Invalid Motor Velocity", axis->axis_number, invalid_motor_velocity_output_timer, invalid_motor_velocity_output_timer_cleared, EventCode::MotorcontrollerWarning_INVALID_MOTOR_VELOCITY,
			isnan_(control->motor_velocity) || !isfinite_(control->motor_velocity),
			axis_control_motor_velocity_nan);
		MOTORCONTROLLER_HANDLE_ERROR_FLAG("Invalid Motor Position", axis->axis_number, invalid_motor_position_output_timer, invalid_motor_position_output_timer_cleared, EventCode::MotorcontrollerError_INVALID_MOTOR_POSITION,
			isnan_(control->motor_position) || !isfinite_(control->motor_position),
			axis_control_motor_position_nan);
		MOTORCONTROLLER_HANDLE_MOTION_WARNING_FLAG("Motor Position Soft Limit", axis->axis_number, motor_position_soft_limit_output_timer, motor_position_soft_limit_output_timer_cleared, EventCode::MotorcontrollerWarning_POSITION_SOFT_LIMIT,
			isModeOfOperationPosition() && !axis_control_motor_position_nan &&
			(control->motor_position>(axis->position_limit_upper - limits->position_reserve) ||
				control->motor_position<(axis->position_limit_lower + limits->position_reserve)),
			axis_control_motor_position_soft_limit);
		MOTORCONTROLLER_HANDLE_ERROR_FLAG("Motor Position Hard Limit", axis->axis_number, motor_position_hard_limit_output_timer, motor_position_hard_limit_output_timer_cleared, EventCode::MotorcontrollerError_POSITION_LIMIT_HARD,
			isModeOfOperationPosition() && !axis_control_motor_position_nan &&
			(control->motor_position>(axis->position_limit_upper - MOTORCONTROLLER_POSITION_LIMIT_DISTANCE) ||
				control->motor_position<(axis->position_limit_lower + MOTORCONTROLLER_POSITION_LIMIT_DISTANCE)),
			axis_control_motor_position_hard_limit);
		MOTORCONTROLLER_HANDLE_MOTION_WARNING_FLAG("Motor Velocity Limit", axis->axis_number, motor_velocity_limit_output_timer, motor_velocity_limit_output_timer_cleared, EventCode::MotorcontrollerWarning_MOTOR_VELOCITY_LIMIT,
			isModeOfOperationVelocity() && !axis_control_motor_velocity_nan &&
			fabs_(control->motor_velocity)>axis->linear_velocity_limit*limits->velocity_limit_fraction,
			axis_control_motor_velocity_limit);
		//See CMMT-AS-SW_manual_2020-11f_8146067d1.pdf, Section 11.7.2 Statusworte (Objekt 0x6041), Tab. 734 Übersicht zur Bitbelegung des Statusworts (Objekt 0x6041)
		MOTORCONTROLLER_HANDLE_MOTION_WARNING_FLAG("Fault", axis->axis_number, fault_output_timer, fault_output_timer_cleared, EventCode::MotorcontrollerWarning_FAULT,
			(input->Statusword & 0x004f) == 0x0008 || (input->Statusword & 0x004f) == 0x000f, //"Fault" or "Fault reaction active"
			axis_control_fault);
		MOTORCONTROLLER_HANDLE_MOTION_WARNING_FLAG("Internal Limit Active", axis->axis_number, internal_limit_active_timer, internal_limit_active_timer_cleared, EventCode::MotorcontrollerWarning_INTERNAL_LIMIT_ACTIVE,
			(input->Statusword & 0x0800) == 0x0800,
			axis_control_internal_limit_active);
		MOTORCONTROLLER_HANDLE_MOTION_WARNING_FLAG("Following Error", axis->axis_number, following_error_timer, following_error_timer_cleared, EventCode::MotorcontrollerWarning_FOLLOWING_ERROR,
			(input->Statusword & 0x2000) == 0x2000,
			axis_control_following_error);

		if (motion_error != EventCode::NONE) {
			output->Target_torque = 0;
			output->Target_velocity = 0;
			if (motor_position_softstop == 0.0) {
				motor_position_softstop = input->Position_actual_value*AXIS_LINEAR_POSITION_RESOLUTION;
			}
			output->Target_position = (LONG)(motor_position_softstop / AXIS_LINEAR_POSITION_RESOLUTION);
		}
		else {
			if (isModeOfOperationTorque()) {
				//Saturate the current
				if (fabs_(control->motor_force) >= (axis->force_limit*limits->force_saturation_fraction)) {
					motor_force = axis->force_limit*limits->force_saturation_fraction*sgn_(control->motor_force);
					if (!axis_control_motor_torque_saturated) {
						if (saturation_active_output_timer == 0) {
							saturation_active_output_timer = OUTPUT_COOLDOWN_TIME;
							trace->Log(tlWarning, "%2d Motorcontroller::updateIOs(): Motor 1 saturation active.", motorcontroller_id);
						}
					}
					axis_control_motor_torque_saturated = true;
				}
				else {
					motor_force = control->motor_force;
					if (axis_control_motor_torque_saturated) {
						if (saturation_cleared_output_timer == 0) {
							saturation_cleared_output_timer = OUTPUT_COOLDOWN_TIME;
							trace->Log(tlWarning, "%2d Motorcontroller::updateIOs(): Motor 1 saturation cleared.", motorcontroller_id);
						}
					}
					axis_control_motor_torque_saturated = false;
				}
				output->Target_torque = AXIS_FORCE_TO_TORQUE(motor_force);
				output->Target_velocity = 0;
				output->Target_position = 0;
			}
			else if (isModeOfOperationVelocity()) {
				output->Target_torque = 0;
				output->Target_velocity = (LONG)(control->motor_velocity / AXIS_VELOCITY_RESOLUTION);
				output->Target_position = 0;
			}
			else if (isModeOfOperationPosition()) {
				output->Target_torque = 0;
				output->Target_velocity = 0;
				output->Target_position = (LONG)(control->motor_position / AXIS_LINEAR_POSITION_RESOLUTION);
			}
			else {
				output->Target_torque = 0;
				output->Target_velocity = 0;
				output->Target_position = 0;
			}
		}

		switch (motion_state) {
		case MotionState::STOP:
			//When motor is stopped, also reset the softstop position.
			motor_position_softstop = 0.0;
			output->Controlword = 0x0000;
			if ((input->Statusword & 0x004f) == 0x0008) { //Fault
				output->Controlword = 0x0080; //fault reset
			}
			else if ((input->Statusword & 0x004f) == 0x000f) { //Fault reaction active
				//Wait for fault reaction to complete
			}
			else if (motion_state_target == MotionState::MOTION) {
				setMotionState(MotionState::MOTION_ENABLE);
			}
			feedMotionWatchdog();
			break;
		case MotionState::MOTION_ENABLE:
			if (motion_state_target == MotionState::STOP) {
				setMotionState(MotionState::STOP);
				feedMotionWatchdog();
			}
			else if ((input->Statusword & 0x004f) == 0x0040) { //Switch on disabled
				output->Controlword = 0x0006; //Shutdown
				if (isModeOfOperationPosition()) {
					output->Controlword |= 0x0010; //enable_ip_mode
				}
				setMotionState(MotionState::MOTOR_CONTROL_1);
				feedMotionWatchdog();
			}
			else if ((input->Statusword & 0x006f) == 0x0021) { //Ready to switch on
															   //Directly go to MotionState::MOTOR_CONTROL_1
				setMotionState(MotionState::MOTOR_CONTROL_1);
				feedMotionWatchdog();
			}
			break;
		case MotionState::MOTOR_CONTROL_1:
			if (motion_state_target == MotionState::STOP) {
				setMotionState(MotionState::DISENGAGING_MOTION);
				feedMotionWatchdog();
			}
			else if ((input->Statusword & 0x006f) == 0x0021) { //Ready to switch on
				output->Controlword = 0x0007; //Switch on
				if (isModeOfOperationPosition()) {
					output->Controlword |= 0x0010; //enable_ip_mode
				}
				setMotionState(MotionState::MOTOR_CONTROL_2);
				feedMotionWatchdog();
			}
			break;
		case MotionState::MOTOR_CONTROL_2:
			if (motion_state_target == MotionState::STOP) {
				setMotionState(MotionState::DISENGAGING_MOTION);
				feedMotionWatchdog();
			}
			else if ((input->Statusword & 0x006f) == 0x0023) { //Switched on
				output->Controlword = 0x000f; //Enable operation
				if (isModeOfOperationPosition()) {
					output->Controlword |= 0x0010; //enable_ip_mode
				}
				setMotionState(MotionState::ENGAGING_MOTION);
			}
			break;
		case MotionState::ENGAGING_MOTION:
			if (motion_state_target == MotionState::STOP) {
				setMotionState(MotionState::DISENGAGING_MOTION);
				feedMotionWatchdog();
			}
			else if ((input->Statusword & 0x006f) == 0x0027) { //Operation enabled
				setMotionState(MotionState::MOTION);
				feedMotionWatchdog();
			}
			break;
		case MotionState::MOTION:
			//Do nothing, wait for stop command or fault (checked via Statusword above)
			if (motion_state_target == MotionState::STOP) {
				setMotionState(MotionState::DISENGAGING_MOTION);
			}
			feedMotionWatchdog();
			break;
		case MotionState::DISENGAGING_MOTION:
			output->Controlword = 0x0002; //Quick stop
			setMotionState(MotionState::MOTOR_CONTROL_1_STOPPING);
			feedMotionWatchdog();
			break;
		case MotionState::MOTOR_CONTROL_1_STOPPING:
			if ((input->Statusword & 0x006f) == 0x0007) { //Quick stop active
				feedMotionWatchdog();
			}
			else {
				setMotionState(MotionState::MOTION_DISABLE);
			}
			break;
		case MotionState::MOTION_DISABLE:
			setMotionState(MotionState::STOP);
			feedMotionWatchdog();
			break;
		}
	}
	else {
		//Replicate the outputs of the motion state MotionState::STOP
		output->Target_torque = 0;
		output->Target_velocity = 0;
		output->Target_position = 0;
		output->Controlword = 0x0000;
		//Always stay in motion state STOP
		if (motion_state != MotionState::STOP) {
			setMotionState(MotionState::STOP);
		}
		feedMotionWatchdog();
	}
}

EventCode Motorcontroller::getLastError() {
	return last_error;
}

EventCode Motorcontroller::getMotionError() {
	return motion_error;
}

EventCode Motorcontroller::getSensorError() {
	return sensor_error;
}

EventCode Motorcontroller::getMotionWarning() {
	return motion_warning;
}

EventCode Motorcontroller::getSensorWarning() {
	return sensor_warning;
}

void Motorcontroller::setLastError(EventCode error) {
	//If there was an error, immediately proceed to the reset state. This clears the
	//output buffer and issues the reset command to the slave.
	trace->Log(tlInfo, "%2d Motorcontroller::setLastError(): %s (%d) -> %s (%d)", motorcontroller_id, toString(last_error), (int)last_error, toString(error), (int)error);
	event_handler->set(axis->axis_number, motorcontroller_id, error, EventType::ERROR);
	if (error != EventCode::NONE) {
		reset();
	}
	last_error = error;
}

bool Motorcontroller::canEnableMotion() {
	bool enable = true;
	if (last_error != EventCode::NONE) {
		trace->Log(tlError, "%2d Motorcontroller::canEnableMotion(): Cannot enable motion due to last error %s (%d)", motorcontroller_id, toString(last_error), (int)last_error);
		enable = false;
	}
	else if (motion_error != EventCode::NONE) {
		trace->Log(tlError, "%2d Motorcontroller::canEnableMotion(): Cannot enable motion due to motion error %s (%d)", motorcontroller_id, toString(motion_error), (int)motion_error);
		enable = false;
	}
	else if (motion_warning != EventCode::NONE) {
		trace->Log(tlError, "%2d Motorcontroller::canEnableMotion(): Cannot enable motion due to motion warning %s (%d)", motorcontroller_id, toString(motion_warning), (int)motion_warning);
		enable = false;
	}
	else if (sensor_error != EventCode::NONE) {
		trace->Log(tlError, "%2d Motorcontroller::canEnableMotion(): Cannot enable motion due to sensor warning %s (%d)", motorcontroller_id, toString(sensor_error), (int)sensor_error);
		enable = false;
	}
	else if (sensor_warning != EventCode::NONE) {
		trace->Log(tlError, "%2d Motorcontroller::canEnableMotion(): Cannot enable motion due to sensor warning %s (%d)", motorcontroller_id, toString(sensor_warning), (int)sensor_warning);
		enable = false;
	}
	else if (motion_state != MotionState::STOP) {
		trace->Log(tlError, "%2d Motorcontroller::canEnableMotion(): Cannot enable motion due to motion state1 %s (%d)", motorcontroller_id, toString(motion_state), (int)motion_state);
		enable = false;
	}
	return enable;
}

bool Motorcontroller::isMotionEnabled() {
	return motion_state_target == MotionState::MOTION || motion_state!=MotionState::STOP;
}

bool Motorcontroller::enableMotion() {
	bool success = false;
	if (canEnableMotion()) {
		setMotionStateTarget(MotionState::MOTION);
		success = true;
	}
	return success;
}

void Motorcontroller::disableMotion() {
	setMotionStateTarget(MotionState::STOP);
}

void Motorcontroller::performSoftStop(AxisSensor *input, AxisControl *output, DWORD dc_time_L) {
	if (output != NULL) {
		if (output->motor_enable) {
			//If the motor is enabled by the outer control, but was stopped by the soft stop
			//feature, the motor_enabled signal is overridden. This prevents, that the axis
			//can be enabled again by "motion_state_target".
			if (motion_state == MotionState::STOP) {
				output->motor_enable = false;
			}
			//Check if velocity is within the tolerance band and stop this axis
			else if (fabs_(input->velocity_filtered)<MOTORCONTROLLER_SOFT_STOP_VELOCITY_TOLERANCE) {
				if (motion_state == MotionState::MOTION) {
					setMotionState(MotionState::DISENGAGING_MOTION);
					trace->Log(tlInfo, "%2d Motorcontroller::performSoftStop(): Disabling motion on axis %d.", motorcontroller_id, axis->axis_number);
				}
			}
			else {
				//Apply a P velocity controller.
				output->motor_force = -axis->velocity_controller_p*input->velocity_filtered;
				output->motor_velocity = 0;
				if (motor_position_softstop == 0.0) {
					motor_position_softstop = input->position;
				}
				output->motor_position = motor_position_softstop;
			}
		}
		else {
			output->motor_force = 0;
			output->motor_velocity = 0;
		}
	}
	statemachineWriteControl(output, dc_time_L);
}

int Motorcontroller::getMotorcontrollerID() {
	return motorcontroller_id;
}

void Motorcontroller::cooldownTimer() {
	COOLDOWN_TIMER_FUNCTION(invalid_motor_torque_output_timer);
	COOLDOWN_TIMER_FUNCTION(invalid_motor_torque_output_timer_cleared);
	COOLDOWN_TIMER_FUNCTION(motor_torque_limit_output_timer);
	COOLDOWN_TIMER_FUNCTION(motor_torque_limit_output_timer_cleared);
	COOLDOWN_TIMER_FUNCTION(invalid_motor_velocity_output_timer);
	COOLDOWN_TIMER_FUNCTION(invalid_motor_velocity_output_timer_cleared);
	COOLDOWN_TIMER_FUNCTION(invalid_motor_position_output_timer);
	COOLDOWN_TIMER_FUNCTION(invalid_motor_position_output_timer_cleared);
	COOLDOWN_TIMER_FUNCTION(motor_position_soft_limit_output_timer);
	COOLDOWN_TIMER_FUNCTION(motor_position_soft_limit_output_timer_cleared);
	COOLDOWN_TIMER_FUNCTION(motor_position_hard_limit_output_timer);
	COOLDOWN_TIMER_FUNCTION(motor_position_hard_limit_output_timer_cleared);
	COOLDOWN_TIMER_FUNCTION(motor_velocity_limit_output_timer);
	COOLDOWN_TIMER_FUNCTION(motor_velocity_limit_output_timer_cleared);
	COOLDOWN_TIMER_FUNCTION(axis_position_soft_limit_output_timer);
	COOLDOWN_TIMER_FUNCTION(axis_position_soft_limit_output_timer_cleared);
	COOLDOWN_TIMER_FUNCTION(axis_velocity_limit_output_timer);
	COOLDOWN_TIMER_FUNCTION(axis_velocity_limit_output_timer_cleared);
	COOLDOWN_TIMER_FUNCTION(saturation_active_output_timer);
	COOLDOWN_TIMER_FUNCTION(saturation_cleared_output_timer);
	COOLDOWN_TIMER_FUNCTION(fault_output_timer);
	COOLDOWN_TIMER_FUNCTION(fault_output_timer_cleared);
	COOLDOWN_TIMER_FUNCTION(internal_limit_active_timer);
	COOLDOWN_TIMER_FUNCTION(internal_limit_active_timer_cleared);
	COOLDOWN_TIMER_FUNCTION(following_error_timer);
	COOLDOWN_TIMER_FUNCTION(following_error_timer_cleared);
}
