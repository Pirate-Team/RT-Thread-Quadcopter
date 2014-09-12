#ifndef MOTOR_H
#define MOTOR_H
#include "rtthread.h"
#include "stm32f4xx.h"
enum MOTOR_ENUM
{
	MOTOR1 = 1,
	MOTOR2,
	MOTOR3,
	MOTOR4,
	MOTORALL
};

class Motor
{
public:
	Motor();
	~Motor();
	void init();
	void start();
	void stop();
	void setThrottle(MOTOR_ENUM motor,uint8_t throttle);
	void setThrottle(uint8_t throttle);
	void setThrottle(uint8_t throttle1,uint8_t throttle2,uint8_t throttle3,uint8_t throttle4);
	void getThrottle(MOTOR_ENUM motor,uint8_t& throttle);

	static bool state;
};


#endif
