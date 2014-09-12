#include "Led.h"
#include "rtthread.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include "MPU6050.h"
#include "I2Cdev.h"
#include "HMC5883L.h"
#include "MS5611.h"

void rt_thread_entry_mpu6050_test(void* parameter)
{
	I2Cdev::init();
	MPU6050 accelgyro;
	accelgyro.initialize();
	rt_kprintf("\r\nMPU6050 connection test: ");
	rt_kprintf(accelgyro.testConnection()?"success\r\n":"failure\r\n");
	HMC5883L mag;
	mag.initialize();
	rt_kprintf("\r\nHMC5883 connection test: ");
	rt_kprintf(mag.testConnection()?"success\r\n":"failure\r\n");
	accelgyro.setOffSet();
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	float heading;
	char str[100];	
	while(1)
	{	
//		accelgyro.getMotion6Cal(&ax, &ay, &az, &gx, &gy, &gz);	
//		sprintf(str,"accel: %+2.6f, %+2.6f, %+2.6f; gyro: %+2.6f, %+2.6f, %+2.6f\r\n",(float)ax/2048,(float)ay/2048,(float)az/2048,(float)gx/16.4f,(float)gy/16.4f,(float)gz/16.4f);
		mag.getHeadingCal(&heading);
		sprintf(str,"heading: %+f\r\n",heading);
		rt_kprintf("%s",str);
		
		rt_thread_delay(RT_TICK_PER_SECOND/20);
	}
}

void rt_thread_entry_ms5611_test(void* parameter)
{
	I2Cdev::init();
	MS5611 baro;
	baro.initialize();
	rt_thread_delay(RT_TICK_PER_SECOND);
	float a = 2.3333;
	char str[100];
	while(1)
	{
		sprintf(str,"Pressure=%f\tTemperature=%f\r\nAttitude=%f\r\n",baro.getPressure(),baro.getTemperature(),baro.getAttitude());
		rt_kprintf("%s",str);
		rt_thread_delay(RT_TICK_PER_SECOND/10);
	}
}

int  rt_application_init(void)
{
	rt_thread_t led_thread;
	led_thread = rt_thread_create("led",
									rt_thread_entry_led_test,
									RT_NULL,
									2048,
									8,
									20);
	if(led_thread != RT_NULL)
		rt_thread_startup(led_thread);
	
//	rt_thread_t mpu6050_thread;
//	mpu6050_thread = rt_thread_create("mpu6050",
//									rt_thread_entry_mpu6050_test,
//									RT_NULL,
//									2048,
//									8,
//									20);
//	if(mpu6050_thread != RT_NULL)
//		rt_thread_startup(mpu6050_thread);
	
	rt_thread_t ms5611_thread;
	ms5611_thread = rt_thread_create("ms5611",
									rt_thread_entry_ms5611_test,
									RT_NULL,
									2048,
									8,
									20);
	if(ms5611_thread != RT_NULL)
		rt_thread_startup(ms5611_thread);
	
	return 0;
}
