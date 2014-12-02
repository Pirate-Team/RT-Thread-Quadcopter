#include "MS5611.h"
#include "head.h"
#include "rtthread.h"
#include "I2Cdev.h"
#include "arm_math.h"
#include "string.h"

//#define DEBUG

#define C1 51270
#define C2 51755
#define C3 31295
#define C4 28176
#define C5 32431
#define C6 27811

MS5611 baro;

MS5611::MS5611(void)
{
	devAddr = MS561101BA_SlaveAddress;
	buffer = new uint8_t[3];
	strcpy(name,MS5611_NAME);
	groundPress = SEA_PRESS * 100;
}

MS5611::~MS5611(void)
{
	if(buffer != null)
	{
		delete(buffer);
		buffer = null;
	}
}

bool MS5611::initialize(void)
{
	if(!reset()) return false;
	DELAY_MS(400);
	if(!readPROM()) return false;

	ConvertTemperature();
	DELAY_MS(20);
	getTemperature();
	ConvertPressure();
	DELAY_MS(20);
	getPressure();
	ConvertTemperature();
	DELAY_MS(20);
	getTemperature();
	ConvertPressure();
	DELAY_MS(20);
	getPressure();
	
	return true;
}

bool MS5611::testConnection(void)
{
	return true;
}

uint8_t MS5611::getData(void* data1,void* data2,void* data3,void* data4,void* data5,void* data6)
{
	if(!getAltitude(*(float*)data1)) return false;
	return true;
}

bool MS5611::reset(void)
{
	return I2Cdev::writeBytes(devAddr,MS561101BA_RST,0,0);
}

bool MS5611::readPROM(void)
{
	uint8_t i;
	for(i=0;i<6;i++)
	{
		if(!I2Cdev::readBytes(devAddr,MS561101BA_PROM_RD + i * 2,2,buffer)) return false;
		C[i] = (buffer[0] << 8) | buffer[1];
#ifdef DEBUG
		rt_kprintf("C%d = %d\r\n",i+1,C[i]);
#endif
	}
	return true;
}

bool MS5611::getTemperature()
{
	uint32_t D2Temp;
	static uint32_t D2 = 0;
	
	if(!I2Cdev::readBytes(devAddr,0,3,buffer)) return false;
	
	D2Temp = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
	D2 = (D2 + D2Temp) >>1;
	
	dT =D2 - ((C[4]) << 8);
	temperature = (2000 + (((int64_t)dT * (int64_t)C[5]) >> 23));	
#ifdef DEBUG
	rt_kprintf("D2 = %d\ttemperature = %d\r\n",D2,temperature);
#endif
	return true;
}

bool MS5611::getPressure()
{
	uint32_t D1Temp;
	static uint32_t D1 = 0;
	
	if(!I2Cdev::readBytes(devAddr,0,3,buffer)) return false;
	
	D1Temp = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
	D1 = (D1 + D1Temp) >> 1;
	
	int32_t off2,sens2,delt;
	int64_t off=((int64_t)C[1]<<16)+(((int64_t)C[3]*dT)>>7);
	int64_t sens=((int64_t)C[0]<<15)+(((int64_t)C[2]*dT)>>8);

	if (temperature < 2000) { // temperature lower than 20st.C 
		delt = temperature-2000;
		delt  = 5*delt*delt;
		off2  = delt>>1;
		sens2 = delt>>2;
		if (temperature < -1500) { // temperature lower than -15st.C
			delt  = temperature+1500;
			delt  = delt*delt;
			off2  += 7 * delt;
			sens2 += (11 * delt)>>1;
		}
		off  -= off2; 
		sens -= sens2;
	}
	
	#define BARO_TAB_SIZE 21
    static int32_t baroHistTab[BARO_TAB_SIZE] = {0};
    static uint8_t baroHistIdx = 0;
	static uint32_t pressureSum = 0;
  
    uint8_t indexplus1 = (baroHistIdx + 1);
    if (indexplus1 == BARO_TAB_SIZE) indexplus1 = 0;
    baroHistTab[baroHistIdx] = ((((int64_t)D1 * sens ) >> 21) - off) >> 15;
    pressureSum += baroHistTab[baroHistIdx];
    pressureSum -= baroHistTab[indexplus1];
    baroHistIdx = indexplus1;

	pressure = pressureSum / 20;
	
#ifdef DEBUG
	rt_kprintf("D1 = %d\tpressure = %d\r\n",D1,pressure);
#endif

	return true;
}

bool MS5611::getPressure(float &press)
{
	if(getPressure())
	{
		press = pressure / 100.0f;
		return true;
	}
	return false;
}

bool MS5611::getTemperature(float &temp)
{
	if(getTemperature())
	{
		temp = temperature / 100.0f;
		return true;
	}
	return false;
}

bool MS5611::getAltitude(float &altitude)
{
	if(pressure<50000) return false;
	altitude = ((pow((float)groundPress / (float)pressure, 1/5.257f) - 1.0f) * (temperature/100.0f + 273.15f)) / 0.0065f;
	return true;
}
	
void MS5611::setGround(void)
{
	if(pressure < 50000) return;
	groundPress = (groundPress*3 + pressure) >> 2;
}

void MS5611::ConvertPressure()
{
	I2Cdev::writeBytes(devAddr,MS561101BA_D1_OSR_4096,0,0);
}

void MS5611::ConvertTemperature()
{
	I2Cdev::writeBytes(devAddr,MS561101BA_D2_OSR_4096,0,0);
}
