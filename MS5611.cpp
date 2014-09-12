#include "MS5611.h"
#include "rtthread.h"
#include "I2Cdev.h"
#include "arm_math.h"

#define DEBUG

//在这个程序里读取校正数据有问题，用arduino读取的数据代替了
#define C1 51270
#define C2 51755
#define C3 31295
#define C4 28176
#define C5 32431
#define C6 27811

#define SEA_PRESS 1013.25

MS5611::MS5611(void)
{
	devAddr = MS561101BA_SlaveAddress;
	buffer = (uint8_t*)rt_malloc(3);
}

MS5611::~MS5611(void)
{
	rt_free(buffer);
}

void MS5611::initialize(void)
{
	reset();
	rt_thread_delay(RT_TICK_PER_SECOND/50);
	readPROM();
	rt_thread_delay(RT_TICK_PER_SECOND/50);
	C[0] = C1;
	C[1] = C2;
	C[2] = C3;
	C[3] = C4;
	C[4] = C5;
	C[5] = C6;
	getTemperature();
}

bool MS5611::testConnection(void)
{
	return true;
}

void MS5611::reset(void)
{
	I2Cdev::writeByte(devAddr,MS561101BA_RST,0);
}

//从PROM读取出厂校准数据
void MS5611::readPROM(void)
{
	uint8_t i;
	for(i=0;i<6;i++)
	{
		I2Cdev::readBytes(devAddr,MS561101BA_PROM_RD + i * 2,2,buffer);
		C[i] = (buffer[0] << 8) | buffer[1];
#ifdef DEBUG
		rt_kprintf("C%d = %d\r\n",i+1,C[i]);
#endif
	}
}

//读取数字温度
float MS5611::getTemperature(void)
{
	uint32_t D2;
	I2Cdev::writeByte(MS561101BA_SlaveAddress,MS561101BA_D2_OSR_4096,0);
	rt_thread_delay(RT_TICK_PER_SECOND/50);
	I2Cdev::readBytes(MS561101BA_SlaveAddress,0,3,buffer);
	D2 = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
	
	dT =D2 - ((C[4]) << 8);
	temperature = 2000 + (((int64_t)dT * (int64_t)C[5]) >> 23);	
#ifdef DEBUG
	rt_kprintf("D2 = %d\ttemperature = %d\r\n",D2,temperature);
#endif	
	return temperature / 100.0f;
}

//读取数字气压
float MS5611::getPressure(void)
{
	I2Cdev::writeByte(MS561101BA_SlaveAddress,MS561101BA_D1_OSR_4096,0);
	rt_thread_delay(RT_TICK_PER_SECOND/50);	
	I2Cdev::readBytes(MS561101BA_SlaveAddress,0,3,buffer);
	uint32_t D1 = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
	
	int32_t off2,sens2,delt;
	int64_t off=((int64_t)C[1]<<16)+(((int64_t)C[3]*dT)>>7);
	int64_t sens=((int64_t)C[0]<<15)+(((int64_t)C[2]*dT)>>8);
	
	//温度补偿
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
	
	pressure = ((((int64_t)D1 * sens ) >> 21) - off) >> 15;
#ifdef DEBUG
	rt_kprintf("D1 = %d\tpressure = %d\r\n",D1,pressure);
#endif	
	return pressure / 100.0f;
}

float MS5611::getAttitude(void)
{
	getPressure(); getTemperature();
	//((pow((sea_press / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065  pressure/100   temperature/100
	return ((pow((float)SEA_PRESS / (float)pressure * 100.0f, (float)1/5.257f) - 1.0f) * (temperature/100.0f + 273.15f)) / 0.0065f;
}
