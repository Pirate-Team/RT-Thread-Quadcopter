#include "Sensor.h"
#include "rtthread.h"
Sensor::~Sensor(void)
{
	if(buffer != null)
	{
		rt_free(buffer);
		buffer = null;
	}
}


void Sensor::setDevAddr(uint8_t addr)
{
	devAddr = addr;
}
