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
	static void initialize();
	static void setValue(MOTOR_ENUM motor,uint16_t Value);
	static void setValue(uint16_t Value);
	static void setValue(uint16_t Value1,uint16_t Value2,uint16_t Value3,uint16_t Value4);
	static void getValue(MOTOR_ENUM motor,uint16_t& Value);
};

extern uint16_t motorValue[MOTORALL];

#endif
