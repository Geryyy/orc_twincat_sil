#pragma once

#include"EventCode.h"
#include"EventType.h"
#include "kuka_interfaceServices.h"
#include "TcInterfaces.h"

class EventHandler {
private:
	Event event;
	CTcTrace* trace;

public:
	EventHandler();
	EventHandler(CTcTrace* trace);
	void acknowledge();
	void set(int axis_number, int slave, EventCode code, EventType type);
	Event* getEvent();
};
