#ifndef SENSOR_H
#define SENSOR_H
#include "stm32f4xx.h"
#define null (0)

//using namespace std;
class Sensor
{
public:
	virtual ~Sensor(void);
	virtual bool initialize(void) = 0;
	virtual bool testConnection(void) = 0;
	virtual uint8_t getData(void* data1,void* data2 = null,void* data3 = null,void* data4 = null,void* data5 = null,void* data6 = null) = 0;
	void setDevAddr(uint8_t addr);
	
	uint8_t devAddr;
	char name[10];
protected:
	uint8_t *buffer;
};
#endif
