#pragma once


#include "ArcTcControlInterfaces.h"
#include "orc/robots/Robot.h"
#include "orc/RobotStatus.h"
#include "orc/com/RobotState.h"

namespace orc::logic {
	class StateMachine {
		CTcTrace* ptr_Trace;
		Time t = 0;
		double T_cycle;
		RobotStatus state;
		const double T_button_hold=1.0; //s
		bool m_enable;
		bool m_disable;
		bool m_move_candle;
		bool m_motion_enabled;
		bool m_application_button;
		bool m_enable_transition;
		bool m_disable_transition;
		bool m_move_candle_transition;

	public:
		StateMachine(CTcTrace* trace_ptr, Time Ts) : ptr_Trace(trace_ptr)
		{
			t = 0;
			T_cycle = static_cast<double>(Ts);
			state = RobotStatus::OFF;

			m_enable = false;
			m_disable = false;
			m_move_candle = false;
			m_motion_enabled = false;
			m_application_button = false;
			m_enable_transition = false;
			m_disable_transition = false;
			m_move_candle_transition = false;
		}


		void run(bool application_button,
			bool motion_enabled,
			bool enable,
			bool disable,
			bool move_candle
		)
		{
			//Positive edge on application button
			if (!m_application_button && application_button) {
				t = 0;
			}
			else {
				t += T_cycle;
			}

			RobotStatus m_state = state;

			m_enable_transition = false;
			m_disable_transition = false;
			m_move_candle_transition = false;

			//Disable motion due to "Motion disable" command
			if (!m_disable && disable) {
				state = RobotStatus::OFF;
				m_disable_transition = true;
			}
			//Enable motion due to "Motion enable" command
			else if (state == RobotStatus::OFF && !motion_enabled && !m_enable && enable) {
				state = RobotStatus::ENABLE;
				m_enable_transition = true;
			}
			//Enable motion due to "Motion enable" from robot/kuka_interface
			else if (state == RobotStatus::OFF && !m_motion_enabled && motion_enabled) {
				state = RobotStatus::ENABLE;
				m_enable_transition = true;
			}
			//Disable motion due to "Motion disable" from robot/kuka interface
			else if (state != RobotStatus::OFF && m_motion_enabled && !motion_enabled) {
				state = RobotStatus::OFF;
				m_disable_transition = true;
			}
			//Switch the robot to gravitation compensation due to application button
			else if (state == RobotStatus::ENABLE && application_button && t > T_button_hold) {
				state = RobotStatus::GRAVCOMP;
			}
			//Move robot to candle configuration with "move_candle" input from HMI
			else if (state == RobotStatus::ENABLE && !m_move_candle && move_candle) {
				m_move_candle_transition = true;
			}
			//Switch the robot back to position control due to release of application button
			else if (state == RobotStatus::GRAVCOMP && !application_button) {
				state = RobotStatus::ENABLE;
				m_enable_transition = true;
			}

			if (m_state != state) {
				ptr_Trace->Log(tlVerbose, FLEAVEA "new state: %d\told state: %d", state, m_state);
				ptr_Trace->Log(tlVerbose, FLEAVEA "application button:  %d", application_button);
				ptr_Trace->Log(tlVerbose, FLEAVEA "motion enabled:  %d", motion_enabled);
			}

			m_enable = enable;
			m_disable = disable;
			m_move_candle = move_candle;
			m_motion_enabled = motion_enabled;
			m_application_button = application_button;
		}

		bool is_enable_transition() {
			return m_enable_transition;
		}

		bool is_disable_transition() {
			return m_disable_transition;
		}

		bool is_move_candle_transition() {
			return m_move_candle_transition;
		}

		RobotStatus get_state() {
			return state;
		}

	};

}