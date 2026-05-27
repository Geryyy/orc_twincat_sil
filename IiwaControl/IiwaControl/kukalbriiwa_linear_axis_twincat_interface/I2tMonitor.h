#pragma once

class I2tMonitor;

#include"KsoeSlave.h"

#define I2TMONITOR_T_a KSOESLAVE_FILTER_T_a
#define I2TMONITOR_T 0.5 //s
#define I2TMONITOR_BUFFER_SIZE (int)(I2TMONITOR_T/I2TMONITOR_T_a)

class I2tMonitor {
  private:
	LONG buffer_current2[I2TMONITOR_BUFFER_SIZE];
	int buffer_index;
	LONGLONG buffer_integral;
	
  public:
    I2tMonitor();

    double updateAndOutput(SHORT current);
    void reset();
};
