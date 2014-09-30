#ifndef RECEIVER_H
#define RECEIVER_H
#include "stm32f4xx.h"

enum RC_TYPE
{
	PITCH = 0,
	ROLL,
	YAW,
	THROTTLE,
//	HOLD,
//	AUX,
	RC_NUM
};

extern uint16_t RCValue[RC_NUM];

class Receiver
{
public:
	static bool initialize(void);
	static void getRCValue(int16_t* value);
};

void rt_thread_entry_receiver_test(void* parameter);
	
#endif
