///////////////////////////////////////////////////////////////////////////////
// EventHandler.cpp
#include "TcPch.h"
#pragma hdrstop

#include "EventHandler.h"

EventHandler::EventHandler() {
	EventHandler(NULL);
}

EventHandler::EventHandler(CTcTrace* trace) {
	memset(&event, 0, sizeof(Event));
	this->trace = trace;
}

void EventHandler::acknowledge() {
	if (event.code != (int)EventCode::NONE) {
		trace->Log(tlAlways, "STOP EVENT | Acknowledged");
	}
	event.code = (int)EventCode::NONE;
	event.type = (int)EventType::NONE;
	event.axis_number = 0;
	event.slave = 0;
}

void EventHandler::set(int axis_number, int slave, EventCode code, EventType type) {
	if (event.code == (int)EventCode::NONE) {
		event.axis_number = axis_number;
		event.slave = slave;
		event.code = (int)code;
		event.type = (int)type;
		if (trace != NULL) {
			if (axis_number != 0) {
				trace->Log(tlAlways, "STOP EVENT | Axis %d %s: %s", axis_number, toString(type), toString(code));
			}
			else {
				trace->Log(tlAlways, "STOP EVENT | Slave %d %s: %s", slave, toString(type), toString(code));
			}
		}
	}
}

Event* EventHandler::getEvent() {
	return &event;
}
