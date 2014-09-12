#ifndef MS5611_H
#define MS5611_h
#include "stm32f4xx.h"
#include "Sensor.h"

#define  MS561101BA_SlaveAddress 0x77  //定义器件在IIC总线中的从地址

#define  MS561101BA_D1 0x40 
#define  MS561101BA_D2 0x50 
#define  MS561101BA_RST 0x1E 

//#define  MS561101BA_D1_OSR_256 0x40 
//#define  MS561101BA_D1_OSR_512 0x42 
//#define  MS561101BA_D1_OSR_1024 0x44 
//#define  MS561101BA_D1_OSR_2048 0x46 
#define  MS561101BA_D1_OSR_4096 0x48 

//#define  MS561101BA_D2_OSR_256 0x50 
//#define  MS561101BA_D2_OSR_512 0x52 
//#define  MS561101BA_D2_OSR_1024 0x54 
//#define  MS561101BA_D2_OSR_2048 0x56 
#define  MS561101BA_D2_OSR_4096 0x58 

#define  MS561101BA_ADC_RD 0x00 
#define  MS561101BA_PROM_RD 0xA0 
#define  MS561101BA_PROM_CRC 0xAE 

class MS5611:public Sensor
{
public:
	MS5611();
	~MS5611();
	void initialize(void);
	bool testConnection(void);
	float getPressure(void);
	float getTemperature(void);
	void readPROM(void);
	void reset();
	float getAttitude(void);
private:
	uint16_t C[6];
	int32_t dT,temperature,pressure;
};


#endif
