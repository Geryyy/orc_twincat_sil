#pragma once

enum class EventCode {
	NONE = 0,
	KsoeSlaveError_WRONG_CRC,
	KsoeSlaveError_WRONG_COMMAND,
	KsoeSlaveError_WRONG_VMT_RESPONSE,
	KsoeSlaveError_UNKNOWN_STATE,
	KsoeSlaveError_MOTION_WATCHDOG,
	KsoeSlaveError_INVALID_TEMPERATURE,
	KsoeSlaveError_SLAVE_RESET,
	KsoeSlaveError_MOTOR_SAFE_TORQUE_OFF_INTERN,
	KsoeSlaveError_STATIC_SAFE_TORQUE_OFF,
	KsoeSlaveError_SAFE_OS_RUNTIME,
	KsoeSlaveError_POSITION_LIMIT_HARD,
	KsoeSlaveError_MOTOR_WARNING_LOW_VOLTAGE,
	KsoeSlaveError_MOTOR_ERROR_OVER_VOLTAGE,
	KsoeSlaveError_HS_SBC,
	KsoeSlaveError_SBC,
	KsoeSlaveError_MOTOR_ERROR_INTERNAL,
	KsoeSlaveError_MOTOR_ERROR_I2T,
	KsoeSlaveError_MOTOR_ERROR_OVERCURRENT,
	KsoeSlaveError_MOTOR_ERROR_TOTAL_CURRENT,
	KsoeSlaveError_MOTOR_ERROR_TEMPERATURE,
	KsoeSlaveError_MOTOR_ERROR_UNKNOWN_COMMAND,
	KsoeSlaveWarning_VELOCITY_LIMIT,
	KsoeSlaveWarning_MOTOR_TORQUE_LIMIT,
	KsoeSlaveWarning_SENSOR_TORQUE_LIMIT,
	KsoeSlaveWarning_POSITION_SOFT_LIMIT,
	KsoeSlaveWarning_INVALID_MOTOR_TORQUE,
	KsoeSlaveWarning_I2T_LIMIT,
	KsoeSlaveWarning_CURRENT_LIMIT,
	KsoeSlaveWarning_MOTOR_WARNING_I2T,
	KsoeSlaveWarning_MOTOR_WARNING_TEMPERATURE,
	FsoeSlaveError_WRONG_CRC,
	FsoeSlaveError_WRONG_COMMAND,
	FsoeSlaveError_UNKNOWN_STATE,
	FsoeSlaveError_BUTTON_PRESSED,
	FsoeSlaveError_BUTTON_PANIC,
	RobotError_WORKSPACE_LIMIT,
	MotorcontrollerWarning_POSITION_SOFT_LIMIT,
	MotorcontrollerWarning_INVALID_MOTOR_TORQUE,
	MotorcontrollerWarning_MOTOR_TORQUE_LIMIT,
	MotorcontrollerWarning_INVALID_MOTOR_VELOCITY,
	MotorcontrollerWarning_VELOCITY_LIMIT,
	MotorcontrollerWarning_MOTOR_VELOCITY_LIMIT,
	MotorcontrollerError_POSITION_LIMIT_HARD,
	MotorcontrollerError_MOTION_WATCHDOG,
	MotorcontrollerError_INVALID_MOTOR_POSITION,
	MotorcontrollerWarning_FAULT,
	MotorcontrollerWarning_INTERNAL_LIMIT_ACTIVE,
	MotorcontrollerWarning_FOLLOWING_ERROR,
};

inline const char* toString(EventCode error) {
	switch (error) {
	case EventCode::NONE: return "NONE";
	case EventCode::KsoeSlaveError_WRONG_CRC: return "KsoeSlaveError_WRONG_CRC";
	case EventCode::KsoeSlaveError_WRONG_COMMAND: return "KsoeSlaveError_WRONG_COMMAND";
	case EventCode::KsoeSlaveError_WRONG_VMT_RESPONSE: return "KsoeSlaveError_WRONG_VMT_RESPONSE";
	case EventCode::KsoeSlaveError_UNKNOWN_STATE: return "KsoeSlaveError_UNKNOWN_STATE";
	case EventCode::KsoeSlaveError_MOTION_WATCHDOG: return "KsoeSlaveError_MOTION_WATCHDOG";
	case EventCode::KsoeSlaveError_INVALID_TEMPERATURE: return "KsoeSlaveError_INVALID_TEMPERATURE";
	case EventCode::KsoeSlaveError_SLAVE_RESET: return "KsoeSlaveError_SLAVE_RESET";
	case EventCode::KsoeSlaveError_MOTOR_SAFE_TORQUE_OFF_INTERN: return "KsoeSlaveError_MOTOR_SAFE_TORQUE_OFF_INTERN";
	case EventCode::KsoeSlaveError_STATIC_SAFE_TORQUE_OFF: return "KsoeSlaveError_STATIC_SAFE_TORQUE_OFF";
	case EventCode::KsoeSlaveError_SAFE_OS_RUNTIME: return "KsoeSlaveError_SAFE_OS_RUNTIME";
	case EventCode::KsoeSlaveError_POSITION_LIMIT_HARD: return "KsoeSlaveError_POSITION_LIMIT_HARD";
	case EventCode::KsoeSlaveError_MOTOR_WARNING_LOW_VOLTAGE: return "KsoeSlaveError_MOTOR_WARNING_LOW_VOLTAGE";
	case EventCode::KsoeSlaveError_MOTOR_ERROR_OVER_VOLTAGE: return "KsoeSlaveError_MOTOR_ERROR_OVER_VOLTAGE";
	case EventCode::KsoeSlaveError_HS_SBC: return "KsoeSlaveError_HS_SBC";
	case EventCode::KsoeSlaveError_SBC: return "KsoeSlaveError_SBC";
	case EventCode::KsoeSlaveError_MOTOR_ERROR_INTERNAL: return "KsoeSlaveError_MOTOR_ERROR_INTERNAL";
	case EventCode::KsoeSlaveError_MOTOR_ERROR_I2T: return "KsoeSlaveError_MOTOR_ERROR_I2T";
	case EventCode::KsoeSlaveError_MOTOR_ERROR_OVERCURRENT: return "KsoeSlaveError_MOTOR_ERROR_OVERCURRENT";
	case EventCode::KsoeSlaveError_MOTOR_ERROR_TOTAL_CURRENT: return "KsoeSlaveError_MOTOR_ERROR_TOTAL_CURRENT";
	case EventCode::KsoeSlaveError_MOTOR_ERROR_TEMPERATURE: return "KsoeSlaveError_MOTOR_ERROR_TEMPERATURE";
	case EventCode::KsoeSlaveError_MOTOR_ERROR_UNKNOWN_COMMAND: return "KsoeSlaveError_MOTOR_ERROR_UNKNOWN_COMMAND";
	case EventCode::KsoeSlaveWarning_VELOCITY_LIMIT: return "KsoeSlaveWarning_VELOCITY_LIMIT";
	case EventCode::KsoeSlaveWarning_MOTOR_TORQUE_LIMIT: return "KsoeSlaveWarning_MOTOR_TORQUE_LIMIT";
	case EventCode::KsoeSlaveWarning_SENSOR_TORQUE_LIMIT: return "KsoeSlaveWarning_SENSOR_TORQUE_LIMIT";
	case EventCode::KsoeSlaveWarning_POSITION_SOFT_LIMIT: return "KsoeSlaveWarning_POSITION_SOFT_LIMIT";
	case EventCode::KsoeSlaveWarning_INVALID_MOTOR_TORQUE: return "KsoeSlaveWarning_INVALID_MOTOR_TORQUE";
	case EventCode::KsoeSlaveWarning_I2T_LIMIT: return "KsoeSlaveWarning_I2T_LIMIT";
	case EventCode::KsoeSlaveWarning_CURRENT_LIMIT: return "KsoeSlaveWarning_CURRENT_LIMIT";
	case EventCode::KsoeSlaveWarning_MOTOR_WARNING_I2T: return "KsoeSlaveWarning_MOTOR_WARNING_I2T";
	case EventCode::KsoeSlaveWarning_MOTOR_WARNING_TEMPERATURE: return "KsoeSlaveWarning_MOTOR_WARNING_TEMPERATURE";
	case EventCode::FsoeSlaveError_WRONG_CRC: return "FsoeSlaveError_WRONG_CRC";
	case EventCode::FsoeSlaveError_WRONG_COMMAND: return "FsoeSlaveError_WRONG_COMMAND";
	case EventCode::FsoeSlaveError_UNKNOWN_STATE: return "FsoeSlaveError_UNKNOWN_STATE";
	case EventCode::FsoeSlaveError_BUTTON_PRESSED: return "FsoeSlaveError_BUTTON_PRESSED";
	case EventCode::FsoeSlaveError_BUTTON_PANIC: return "FsoeSlaveError_BUTTON_PANIC";
	case EventCode::RobotError_WORKSPACE_LIMIT: return "RobotError_WORKSPACE_LIMIT";
	case EventCode::MotorcontrollerWarning_POSITION_SOFT_LIMIT: return "MotorcontrollerWarning_POSITION_SOFT_LIMIT";
	case EventCode::MotorcontrollerWarning_INVALID_MOTOR_TORQUE: return "MotorcontrollerWarning_INVALID_MOTOR_TORQUE";
	case EventCode::MotorcontrollerWarning_MOTOR_TORQUE_LIMIT: return "MotorcontrollerWarning_MOTOR_TORQUE_LIMIT";
	case EventCode::MotorcontrollerWarning_INVALID_MOTOR_VELOCITY: return "MotorcontrollerWarning_INVALID_MOTOR_VELOCITY";
	case EventCode::MotorcontrollerWarning_VELOCITY_LIMIT: return "MotorcontrollerWarning_VELOCITY_LIMIT";
	case EventCode::MotorcontrollerWarning_MOTOR_VELOCITY_LIMIT: return "MotorcontrollerWarning_MOTOR_VELOCITY_LIMIT";
	case EventCode::MotorcontrollerError_POSITION_LIMIT_HARD: return "MotorcontrollerError_POSITION_LIMIT_HARD";
	case EventCode::MotorcontrollerError_MOTION_WATCHDOG: return "MotorcontrollerError_MOTION_WATCHDOG";
	case EventCode::MotorcontrollerError_INVALID_MOTOR_POSITION: return "MotorcontrollerError_INVALID_MOTOR_POSITION";
	case EventCode::MotorcontrollerWarning_FAULT: return "MotorcontrollerWarning_FAULT";
	case EventCode::MotorcontrollerWarning_INTERNAL_LIMIT_ACTIVE: return "MotorcontrollerWarning_INTERNAL_LIMIT_ACTIVE";
	case EventCode::MotorcontrollerWarning_FOLLOWING_ERROR: return "MotorcontrollerWarning_FOLLOWING_ERROR";
	default: return "(unknown)";
	}
}
