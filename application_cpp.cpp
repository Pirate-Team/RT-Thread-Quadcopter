#include "Led.h"
#include "rtthread.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include "MPU6050.h"
#include "I2Cdev.h"

void rt_thread_entry_mpu6050_test(void* parameter)
{
	I2Cdev::init();
	MPU6050 accelgyro;
	accelgyro.initialize();
	accelgyro.calibrate();
	int16_t ax, ay, az, axa = 0, aya = 0, aza = 0;
	int16_t gx, gy, gz, gxa = 0, gya = 0, gza = 0;
	char str[100];	
	while(1)
	{	
		accelgyro.getMotion6Cal(&ax, &ay, &az, &gx, &gy, &gz);
		axa = (axa + ax) / 2; aya = (aya + ay) / 2; aza = (aza + az) / 2;
		gxa = (gxa + gx) / 2; gya = (gya + gy) / 2; gza = (gza + gz) / 2;
		
		sprintf(str,"accel: %f, %f, %f; gyro: %f, %f, %f\r\n",(float)axa/2048,(float)aya/2048,(float)aza/2048,(float)gxa/16.4f,(float)gya/16.4f,(float)gza/16.4f);
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
	
	rt_thread_t mpu6050_thread;
	mpu6050_thread = rt_thread_create("mpu6050",
									rt_thread_entry_mpu6050_test,
									RT_NULL,
									2048,
									7,
									20);
	if(mpu6050_thread != RT_NULL)
		rt_thread_startup(mpu6050_thread);
	return 0;
}
