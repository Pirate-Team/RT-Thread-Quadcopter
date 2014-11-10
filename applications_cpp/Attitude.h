#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "stm32f4xx.h"

class Attitude
{
public:
	Attitude();
	float& operator[](int8_t index);
	float pitch,roll,yaw,altitude;
	double longitude,latitude;
};

extern Attitude att;
#endif
