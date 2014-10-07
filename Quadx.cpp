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
#define M_57_3 57.29577f
#define GYRO_SCALE 32.8f
#define BETWEEN(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
/*-----------------------------------
	global
-----------------------------------*/
int16_t ax,ay,az,gx,gy,gz;
float heading,preHeading;
float att[4] = {0,0,0,0};

struct pid_t PID[4];

float PIDResult[4] = {0};
uint16_t motorThro[4] = {0};

bool holdAlt = false;
/*-----------------------------------
	thread
-----------------------------------*/
void rt_thread_entry_quadx_get_attitude(void* parameter)
{
	I2Cdev::initialize();
	MPU6050 accelgyro;
	accelgyro.initialize();
	accelgyro.setOffSet();
	HMC5883L mag;
	mag.initialize();
	mag.getHeadingCal(&heading);
	preHeading = heading;
	MS5611 baro;
	baro.initialize();
	
//	quatMutex = rt_mutex_create("quad",RT_IPC_FLAG_PRIO);
	
	uint8_t state = 30;
	uint32_t preTick = 0,curTick = 0;
	while(1)
	{
		/*getSensorData*/
		accelgyro.getMotion6Cal(&ax, &ay, &az, &gx, &gy, &gz);
		if(((2*PI-abs(heading - preHeading))<0.01f)||(abs(heading - preHeading)<0.01f))
			if(gz<20&&gz>-20) gz = 0;
		if(gx<10&&gx>-10) gx /= 2.0f;
		if(gy<10&&gy>-10) gy /= 2.0f;
		if(state == 0 || state == 20)
		{
			preHeading = heading;
			mag.getHeadingCal(&heading);
			if(state == 0) state = 40;
		}
		else if(state == 30)
		{
			baro.getTemperature();
		}
		else if(state == 10)
		{
			baro.getPressure();
			//简单增强动态性吧。。可能是鸡肋
			att[THROTTLE] += ((float)(az -2048) / 2048.0f)*0.2f;
			baro.getAltitude(&att[THROTTLE]);	
		}
		state--;
		/*calculate attitude*/
		preTick = curTick;
		curTick = rt_tick_get();
		sampleInterval = (curTick - preTick) / 1000.0f + 0.0001f;
		//姿态数据丢失
		if(ax||ay||az)
		{
			MadgwickAHRSupdateIMU((float)gx/GYRO_SCALE,(float)gy/GYRO_SCALE,(float)gz/GYRO_SCALE,(float)ax,(float)ay,(float)az);
			quat.toEuler(att[PITCH],att[ROLL],att[YAW]);
		}
		else
			att[PITCH]=att[ROLL]=att[YAW]=0;
		
		rt_thread_delay(4);
	}
}


void rt_thread_entry_quadx_control_attitude(void* parameter)
{
	Receiver::initialize();
	Motor mo;
	mo.init();
	mo.start();
	
	int16_t err[4],preErr[4] = {0},sumErr[4] = {0};
	float alt = 0;
	rt_thread_delay(100);
	while(1)
	{
		/*calculate PID*/
		{
			/*pitch&roll*/
			//最多50度
			for(uint8_t i=0;i<2;i++)
			{
				err[i] = (RCValue[i] - 1500)/10 - att[i];
				PIDResult[i] = PID[i].P * err[i];
				
				//偏差小于5度积分
				if(err[i]<5 && err[i]>-5 && (int32_t)err[i]*(int32_t)preErr[i]>=0)
					sumErr[i] = BETWEEN(sumErr[i]+err[i],-1000,1000);
				else 
					sumErr[i] = 0;
				PIDResult[i] += PID[i].I * sumErr[i];
				
				PIDResult[i] -= PID[i].D * (i==0?gx:gy) / GYRO_SCALE;
				
				preErr[i] = err[i];
			}
			
			/*yaw*/
			err[YAW] = (RCValue[YAW] - 1500)/10  - (gz / GYRO_SCALE);
			PIDResult[YAW] = PID[YAW].P * err[YAW];
			
			if(abs(gz / GYRO_SCALE)<2 && (int32_t)err[YAW]*(int32_t)preErr[YAW]>=0)
				sumErr[YAW] = BETWEEN(sumErr[YAW] + err[YAW],-1000,1000); 
			else 
				sumErr[YAW] = 0;
			PIDResult[YAW] += PID[YAW].I * sumErr[YAW];
			
			PIDResult[YAW] += PID[YAW].D * (err[YAW] -preErr[YAW]);
			
			preErr[YAW] = err[YAW];
			
			/*altitude*/
			if(holdAlt)
			{
				if(alt == 0) alt = att[THROTTLE];
				if(RCValue[THROTTLE]<1350) alt -= 0.005f;
				else if(RCValue[THROTTLE]>1650) alt += 0.005f;
				err[THROTTLE] = alt - att[THROTTLE];
				PIDResult[THROTTLE] = PID[THROTTLE].P * err[THROTTLE];
			}
			else alt = 0;
		}
		/*control motor*/
		{
			//停机条件
			if(abs(att[PITCH])>70||abs(att[ROLL])>70||RCValue[THROTTLE]<1050)
			{
				motorThro[0] = 1000;
				motorThro[1] = 1000;
				motorThro[2] = 1000;
				motorThro[3] = 1000;
			}
			else
			{
				motorThro[0] = RCValue[THROTTLE] + PIDResult[PITCH] + PIDResult[ROLL] + PIDResult[YAW] + PIDResult[THROTTLE];
				motorThro[1] = RCValue[THROTTLE] + PIDResult[PITCH] - PIDResult[ROLL] - PIDResult[YAW] + PIDResult[THROTTLE];
				motorThro[2] = RCValue[THROTTLE] - PIDResult[PITCH] - PIDResult[ROLL] + PIDResult[YAW] + PIDResult[THROTTLE];
				motorThro[3] = RCValue[THROTTLE] - PIDResult[PITCH] + PIDResult[ROLL] - PIDResult[YAW] + PIDResult[THROTTLE];
			}
			mo.setThrottle(motorThro[0],motorThro[1],motorThro[2],motorThro[3]);
		}
		rt_thread_delay(10);
	}
}

