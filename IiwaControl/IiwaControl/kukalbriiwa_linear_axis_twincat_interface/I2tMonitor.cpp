///////////////////////////////////////////////////////////////////////////////
// i2t_monitor.cpp
#include "TcPch.h"
#pragma hdrstop
#include "I2tMonitor.h"

I2tMonitor::I2tMonitor() {
	reset();
}

double I2tMonitor::updateAndOutput(SHORT current) {
	const LONG current2 = ((LONG)current) * ((LONG)current);

	//Update the integral and the ring buffer
	buffer_integral = buffer_integral + current2 - buffer_current2[buffer_index];
	buffer_current2[buffer_index] = current2;
	buffer_index++;
	if (buffer_index >= I2TMONITOR_BUFFER_SIZE) {
		buffer_index = 0;
	}

	//Compute the I2t monitoring output
	return sqrt_(1.0 / I2TMONITOR_T * I2TMONITOR_T_a * (double)buffer_integral);
}

void I2tMonitor::reset() {
	memset((void*)buffer_current2, 0, I2TMONITOR_BUFFER_SIZE * sizeof(LONG));
	buffer_index = 0;
	buffer_integral = 0;
}
