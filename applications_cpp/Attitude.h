#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "stm32f4xx.h"

class Attitude
{
public:
	Attitude();
	float& operator[](int8_t index);
	float pitch,roll,yaw,altitude;
	int32_t longitude,latitude;//实际读度数*(10^7)
};

extern Attitude att;
#endif
