#ifndef SENSOR_H
#define SENSOR_H
#include "stm32f4xx.h"
class Sensor
{
public:
	bool initialize(void);
	bool testConnection(void);
	void setDevAddr(uint8_t addr);
protected:
	uint8_t devAddr;
	uint8_t *buffer;
};
#endif
