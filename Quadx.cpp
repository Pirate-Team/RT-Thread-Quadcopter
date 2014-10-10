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
struct sensor_data_t sensorData = {0};

float att[4] = {0,0,0,0};

struct pid_t PID[4] = {0};

struct ctrl_t
{
	bool att,thro,coor,alt,track,quadx;
};
extern struct ctrl_t ctrl;
/*-----------------------------------
	thread
-----------------------------------*/
void rt_thread_entry_quadx_get_attitude(void* parameter)
{
	MPU6050 accelgyro;
	HMC5883L mag;
	MS5611 baro;
	mag.getHeadingCal(&sensorData.heading);
	float preHeading = sensorData.heading;
	
	uint8_t state = 30;
	uint32_t preTick = 0,curTick = 0;
	while(1)
	{
		/*getSensorData*/
		accelgyro.getMotion6Cal(&sensorData.ax, &sensorData.ay, &sensorData.az, &sensorData.gx, &sensorData.gy, &sensorData.gz);
		//gz粗暴高通滤波
#define HEADING_DELTA (0.5f)
#define GZ_THRE (60)
		if(((360-abs(sensorData.heading - preHeading))<HEADING_DELTA)||(abs(sensorData.heading - preHeading)<HEADING_DELTA))
			if(sensorData.gz<GZ_THRE&&sensorData.gz>-GZ_THRE) sensorData.gz = 0;
//		if(sensorData.gx<10&&sensorData.gx>-10) sensorData.gx /= 2.0f;
//		if(sensorData.gy<10&&sensorData.gy>-10) sensorData.gy /= 2.0f;
		if(state == 0 || state == 20)
		{
			preHeading = sensorData.heading;
			mag.getHeadingCal(&sensorData.heading);
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
//			att[THROTTLE] += ((float)(sensorData.az -2048) / 2048.0f)*0.5f;
			baro.getAltitude(&att[THROTTLE]);	
		}
		state--;
		/*calculate attitude*/
		preTick = curTick;
		curTick = rt_tick_get();
		sampleInterval = (curTick - preTick) / 1000.0f + 0.0001f;
		//姿态数据
		MadgwickAHRSupdateIMU((float)sensorData.gx/GYRO_SCALE,(float)sensorData.gy/GYRO_SCALE,(float)sensorData.gz/GYRO_SCALE,(float)sensorData.ax,(float)sensorData.ay,(float)sensorData.az);
		quat.toEuler(att[PITCH],att[ROLL],att[YAW]);
		
		if(ctrl.quadx == false) rt_thread_delay(100); 
		rt_thread_delay(4);
	}
}


void rt_thread_entry_quadx_control_attitude(void* parameter)
{
	struct err_t
	{
		float cur,pre,sum;
	}err[4];
	float alt = 0;
	
	rt_thread_delay(100);
	while(1)
	{
		/*calculate PID*/
		{
//			RCValue[THROTTLE] = 1500;
			/*pitch&roll*/
			//最多30度
			for(uint8_t i=0;i<2;i++)
			{
				err[i].cur = BETWEEN(RCValue[i] - 1500,-500,500)/16.0f - att[i];
				PID[i].result = PID[i].P * err[i].cur;
				
				//遥控器小于5度积分
				if(RCValue[i]<1580&&RCValue[i]>1420)
					err[i].sum = BETWEEN(err[i].sum+err[i].cur,-1000,1000);
				else
					err[i].sum = 0;
				PID[i].result += PID[i].I * err[i].sum;
				
				PID[i].result -= PID[i].D * ((i==0?sensorData.gx:sensorData.gy) / GYRO_SCALE);
				PID[i].result = BETWEEN(PID[i].result,-150,150);
				
				err[i].pre = err[i].cur;
			}
			
			/*yaw*/
			err[YAW].cur = (RCValue[YAW] - 1500)/16.0f  - (sensorData.gz / GYRO_SCALE);
			PID[YAW].result = PID[YAW].P * err[YAW].cur;
			
			//遥控器小于5度积分
			if(RCValue[YAW]<1580&&RCValue[YAW]>1420)
				err[YAW].sum = BETWEEN(err[YAW].sum + err[YAW].cur,-1000,1000); 
			else 
				err[YAW].sum = 0;
			PID[YAW].result += PID[YAW].I * err[YAW].sum;
			
			PID[YAW].result += PID[YAW].D * (err[YAW].cur -err[YAW].pre);
			
			err[YAW].pre = err[YAW].cur;
			
			/*altitude*/
			if(ctrl.alt)
			{
				if(alt == 0) alt = att[THROTTLE];
				if(RCValue[THROTTLE]<1350) alt -= 0.005f;
				else if(RCValue[THROTTLE]>1650) alt += 0.005f;
				err[THROTTLE].cur = alt - att[THROTTLE];
				PID[THROTTLE].result = PID[THROTTLE].P * err[THROTTLE].cur;
			}
			else alt = 0;
		}
		/*control motor*/
		{
			//停机条件
			if(abs(att[PITCH])>70||abs(att[ROLL])>70||RCValue[THROTTLE]<1060||ctrl.quadx == false)
			{
				motorValue[0] = 1000;
				motorValue[1] = 1000;
				motorValue[2] = 1000;
				motorValue[3] = 1000;
			}
			else
			{
				motorValue[0] = RCValue[THROTTLE] + PID[PITCH].result + PID[ROLL].result + PID[YAW].result + PID[THROTTLE].result;
				motorValue[1] = RCValue[THROTTLE] + PID[PITCH].result - PID[ROLL].result - PID[YAW].result + PID[THROTTLE].result;
				motorValue[2] = RCValue[THROTTLE] - PID[PITCH].result - PID[ROLL].result + PID[YAW].result + PID[THROTTLE].result;
				motorValue[3] = RCValue[THROTTLE] - PID[PITCH].result + PID[ROLL].result - PID[YAW].result + PID[THROTTLE].result;
			}
			Motor::setValue(motorValue[0],motorValue[1],motorValue[2],motorValue[3]);
		}
		if(ctrl.quadx == false) rt_thread_delay(100);
		rt_thread_delay(10);
	}
}

