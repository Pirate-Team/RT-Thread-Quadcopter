#ifndef RECEIVER_H
#define RECEIVER_H
#include "stm32f4xx.h"

enum RC_TYPE
{
	PITCH = 0,
	ROLL,
	YAW,
	THROTTLE,
	HOLD,
	AUX,
	RC_NUM
};

class Receiver
{
	Receiver(void);
	~Receiver(void);
	bool initialize(void);
	void getRCValue(int16_t* value);
};

#endif
