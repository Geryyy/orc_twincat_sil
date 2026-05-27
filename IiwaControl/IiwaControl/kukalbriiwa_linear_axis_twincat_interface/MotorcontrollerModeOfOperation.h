#pragma once

enum class MotorcontrollerModeOfOperation {
	NONE = 0,
	USER_TORQUE = 0xF2,
	USER_VELOCITY = 0xF3,
	USER_POSITION = 0xF5,
	PROFILE_POSITION = 1,
	PROFILE_VELOCITY = 3,
	PROFILE_TORQUE = 4,
	INTERPOLATED_POSITION = 7,
	CYCLIC_SYNCHRONOUS_POSITION = 8,
	CYCLIC_SYNCHRONOUS_VELOCITY = 9,
	CYCLIC_SYNCHRONOUS_TORQUE = 10,
};

inline const char* toString(MotorcontrollerModeOfOperation error) {
	switch (error) {
	case MotorcontrollerModeOfOperation::NONE: return "NONE";
	case MotorcontrollerModeOfOperation::USER_TORQUE: return "USER_TORQUE";
	case MotorcontrollerModeOfOperation::USER_VELOCITY: return "USER_VELOCITY";
	case MotorcontrollerModeOfOperation::USER_POSITION: return "USER_POSITION";
	case MotorcontrollerModeOfOperation::PROFILE_POSITION: return "PROFILE_POSITION";
	case MotorcontrollerModeOfOperation::PROFILE_VELOCITY: return "PROFILE_VELOCITY";
	case MotorcontrollerModeOfOperation::PROFILE_TORQUE: return "PROFILE_TORQUE";
	case MotorcontrollerModeOfOperation::INTERPOLATED_POSITION: return "INTERPOLATED_POSITION";
	case MotorcontrollerModeOfOperation::CYCLIC_SYNCHRONOUS_POSITION: return "CYCLIC_SYNCHRONOUS_POSITION";
	case MotorcontrollerModeOfOperation::CYCLIC_SYNCHRONOUS_VELOCITY: return "CYCLIC_SYNCHRONOUS_VELOCITY";
	case MotorcontrollerModeOfOperation::CYCLIC_SYNCHRONOUS_TORQUE: return "CYCLIC_SYNCHRONOUS_TORQUE";

	default: return "(unknown)";
	}
}
