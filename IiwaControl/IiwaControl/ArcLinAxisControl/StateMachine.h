#pragma once


#include "ArcLinAxisControlInterfaces.h"
#include "orc/robots/Robot.h"
#include "orc/RobotStatus.h"

namespace orc::logic {
	class StateMachine {
		CTcTrace* ptr_Trace;
		RobotStatus state;
		bool m_enable;
		bool m_disable;
		bool m_motion_enabled;
		bool m_enable_transition;
		bool m_disable_transition;

	public:
		StateMachine(CTcTrace* trace_ptr) : ptr_Trace(trace_ptr)
		{
			state = RobotStatus::OFF;

			m_enable = false;
			m_disable = false;
			m_motion_enabled = false;
			m_enable_transition = false;
			m_disable_transition = false;
		}


		void run(bool motion_enabled,
			bool auto_enable,
			bool enable,
			bool disable
		)
		{

			RobotStatus m_state = state;

			m_enable_transition = false;
			m_disable_transition = false;

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
			//If allowed by auto_enable, enable motion due to "Motion enable" from robot/kuka_interface
			else if (state == RobotStatus::OFF && auto_enable && !m_motion_enabled && motion_enabled) {
				state = RobotStatus::ENABLE;
				m_enable_transition = true;
			}
			//Disable motion due to "Motion disable" from robot/kuka interface
			else if (state != RobotStatus::OFF && m_motion_enabled && !motion_enabled) {
				state = RobotStatus::OFF;
				m_disable_transition = true;
			}
			
			if (m_state != state) {
				ptr_Trace->Log(tlVerbose, FLEAVEA "new state: %d\told state: %d", state, m_state);
				ptr_Trace->Log(tlVerbose, FLEAVEA "motion enabled:  %d", motion_enabled);
			}

			m_enable = enable;
			m_disable = disable;
			m_motion_enabled = motion_enabled;
		}

		bool is_enable_transition() {
			return m_enable_transition;
		}

		bool is_disable_transition() {
			return m_disable_transition;
		}

		RobotStatus get_state() {
			return state;
		}

	};

}