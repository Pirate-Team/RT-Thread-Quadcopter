#include "Attitude.h"

Attitude att;

Attitude::Attitude()
{
	pitch = roll = yaw = altitude = 0;
}

float& Attitude::operator[](int8_t index)
{
	switch(index)
	{
		case 0:
			return pitch;
		case 1:
			return roll;
		case 2:
			return yaw;
		case 3:
			return altitude;
		default:
			return pitch;
	}
}
