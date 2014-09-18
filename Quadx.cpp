#include "quadx.h"
#include "rtthread.h"
#include "stdio.h"
#include "arm_math.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "MS5611.h"
#include "Motor.h"
#include "MadgwickAHRS.h"
#include "Receiver.h"
/*-----------------------------------
	define
-----------------------------------*/
#define M_57_3 57.295779f
#define GAIN 939.6507756f
/*-----------------------------------
	global
-----------------------------------*/
struct pid_t
{
	float P,I,D;
};

int16_t ax,ay,az,gx,gy,gz;
float att[4] = {0};
float PIDResult[4] = {0};
struct pid_t PID[4];
int16_t RCValue[RC_NUM] = {0};
rt_sem_t sensorDataReady = RT_NULL, sensorDataNeed = RT_NULL;
rt_mutex_t sensorDataMutex = RT_NULL;
/*-----------------------------------
	thread
-----------------------------------*/
void rt_thread_quadx_get_sensor_data(void* parameter)
{	
	float heading,preHeading;
	I2Cdev::init();
	MPU6050 accelgyro;
	accelgyro.initialize();
	accelgyro.setOffSet();
	HMC5883L mag;
	mag.initialize();
	mag.getHeadingCal(&heading);
	MS5611 baro;
	baro.initialize();
	
	uint8_t state = 0;
	while(1)
	{
		rt_sem_take(sensorDataNeed,RT_WAITING_FOREVER);
		switch(state)
		{
			case 0:
				accelgyro.getData(&ax, &ay, &az, &gx, &gy, &gz);
				if(gz<5&&gz>-5)
					if(((360-abs(heading - preHeading))<0.01f)||(abs(heading - preHeading)<0.01f)) gz = 0;
				break;
			case 1:
				preHeading = heading;
				mag.getHeadingCal(&heading);
				break;
			case 2:
				baro.getData(&att[4]);
				break;
		}
		rt_sem_release(sensorDataReady);
		state = (state + 1) % 2;
	}
}

void rt_thread_quadx_send_data(void* parameter)
{
	char str[100];
	while(1)
	{
		att[PITCH] = (float)asin(2 * q0*q2 - 2 * q3*q1)*M_57_3;
		att[ROLL] = (float)atan2(2 * q0*q1 + 2 * q2*q3, 1 - 2 * q1*q1 - 2 * q2*q2)*M_57_3;
		att[YAW] = (float)atan2(2 * q0*q3 + 2 * q1*q2, 1 - 2 * q2*q2 - 2 * q3*q3)*M_57_3;
		sprintf(str,"%+f\t%+f\t%+f\t%+d\r\n",att[PITCH],att[ROLL],att[YAW],gz);
		rt_kprintf("%s",str);
		rt_thread_delay(100);
	}
}

void rt_thread_quadx_IMU(void* parameter)
{	
	uint32_t preTick = 0,curTick = 0;
	while(1)
	{
		rt_sem_take(sensorDataReady,RT_WAITING_FOREVER);
		curTick = rt_tick_get();
		sampleInterval = curTick - preTick;
		preTick = curTick;
		MadgwickAHRSupdateIMU((float)gx/GAIN,(float)gy/GAIN,(float)gz/GAIN,(float)ax,(float)ay,(float)az);
		quat.toEuler(att[PITCH],att[ROLL],att[YAW]);
		rt_sem_release(sensorDataNeed);
		rt_thread_delay(5);
	}
}

void rt_thread_quadx_PID(void* parameter)
{
	static float preErr[4] = {0};
	static float sumErr[4] = {0};
	float err[4];
	/*pitch&roll*/
	//×î¶à0.5rad == 28.647¶È
	for(uint8_t i=0;i<2;i++)
	{
		err[i] = (RCValue[i] - 1500) / 1000 - att[i];
		
		PIDResult[i] = PID[i].P * err[i];
		
		if(abs(err[i]) < 0.1f) sumErr[i] += err[i]; else sumErr[i] = 0;
		PIDResult[i] += PID[i].I * sumErr[i];
		
		PIDResult[i] += PID[i].D * (err[i] -preErr[i]);
		
		preErr[i] = err[i];
	}
	
	/*yaw*/
	err[YAW] = ((RCValue[YAW] - 1500) / 500) - (gz / M_57_3);
	
	PIDResult[YAW] = PID[YAW].P * err[YAW];
	
	PIDResult[YAW] += PID[YAW].I * sumErr[YAW];

	PIDResult[YAW] += PID[YAW].D * (err[YAW] -preErr[YAW]);
	
	preErr[YAW] = err[YAW];
}

/*-----------------------------------
	main thread
-----------------------------------*/
void rt_thread_entry_quadx(void* parameter)
{
	sensorDataMutex = rt_mutex_create("sensorDataMutex",RT_IPC_FLAG_PRIO);
	sensorDataReady = rt_sem_create("ready",0,RT_IPC_FLAG_PRIO);
	sensorDataNeed  = rt_sem_create("need",1,RT_IPC_FLAG_PRIO);

	rt_thread_t sensor_thread;
	sensor_thread = rt_thread_create("sensor",
									rt_thread_quadx_get_sensor_data,
									RT_NULL,
									4096,
									7,
									10);
	
	rt_thread_t IMU_thread;
	IMU_thread = rt_thread_create("IMU",
									rt_thread_quadx_IMU,
									RT_NULL,
									4096,
									7,
									20);
	
	rt_thread_t send_thread;
	send_thread = rt_thread_create("send",
									rt_thread_quadx_send_data,
									RT_NULL,
									2048,
									7,
									20);
	rt_thread_delay(1000);
	if(sensor_thread != RT_NULL) rt_thread_startup(sensor_thread);
	if(IMU_thread != RT_NULL) rt_thread_startup(IMU_thread);
	if(send_thread != RT_NULL) rt_thread_startup(send_thread);
	while(1) rt_thread_yield();
}
