#ifndef PARAM_H
#define PARAM_H

#include "stm32f4xx.h"
#include "rtthread.h"
#define FLASH_ADDRESS_BASE (0x08020000)

#define PITCH 0
#define ROLL 1
#define YAW 2
#define ALT 3
#define LNG 4
#define LAT 5


struct pid_t
{
	float P,I,D,result;
};

class Parameter
{
public:
	Parameter();
	bool flashRead(void);
	bool flashWrite(void);

	struct pid_t PID[6];
	int16_t accXOffset,accYOffset,accZOffset;
	int16_t gyroXOffset,gyroYOffset,gyroZOffset;
	int16_t magXOffset,magYOffset,magZOffset;
	int16_t magYGain,magZGain;
	uint16_t checkSum;
};//120kb

extern Parameter param;

#endif
