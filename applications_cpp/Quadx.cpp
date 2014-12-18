#include "quadx.h"
#include "head.h"
#include "rtthread.h"
#include "stdio.h"
#include "arm_math.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "MS5611.h"
#include "Motor.h"
#include "Receiver.h"
#include "Parameter.h"
#include "Quaternion.h"
#include "Attitude.h"
/*-----------------------------------
	define
-----------------------------------*/
#define M_57_3 57.295779f
#define GYRO_SCALE 32.8f
#define ACCEL_SCALE 2048.0f
#define POS 4
//#define DYNAMIC_PID
/*-----------------------------------
	global
-----------------------------------*/

struct sensor_data_t sensorData = {0};

struct ctrl_t
{
	bool send,quadx,trace;
};
extern struct ctrl_t ctrl;
extern int32_t lng,lat;
extern "C" 
{
	extern int16_t targetX,targetY;
}

/*-----------------------------------
	thread
-----------------------------------*/
void rt_thread_entry_quadx_get_attitude(void* parameter)
{
	uint8_t state = 0;
	quat.sampleInterval = 0.0041f;
	while(1)
	{
		/*getSensorData*/
		accelgyro.getMotion6Cal(sensorData.ax, sensorData.ay, sensorData.az, sensorData.gx, sensorData.gy, sensorData.gz);
		if(state == 0)
		{
			mag.getHeadingCal(sensorData.mx,sensorData.my,sensorData.mz);
			state = 10;
		}
		else if(state == 5)
		{
			baro.getTemperature();
			baro.ConvertPressure();
			mag.getHeadingCal(sensorData.mx,sensorData.my,sensorData.mz);
		}
		else if(state == 2)
		{
			baro.getPressure();	
			baro.ConvertTemperature();
			baro.getAltitude(att[THROTTLE]);
		}
		state--;
		/*calculate attitude*/
		//姿态数据
		quat.MadgwickAHRSupdate((float)sensorData.gx/GYRO_SCALE,(float)sensorData.gy/GYRO_SCALE,(float)sensorData.gz/GYRO_SCALE,(float)sensorData.ax,(float)sensorData.ay,(float)sensorData.az,(float)sensorData.mx,(float)sensorData.my,(float)sensorData.mz);
		//MadgwickAHRSupdateIMU((float)sensorData.gx/GYRO_SCALE,(float)sensorData.gy/GYRO_SCALE,(float)sensorData.gz/GYRO_SCALE,(float)sensorData.ax,(float)sensorData.ay,(float)sensorData.az);
		
		DELAY_MS(4); 
	}
}

int32_t lng = 0,lat = 0;
void rt_thread_entry_quadx_control_attitude(void* parameter)
{
	struct err_t
	{
		float cur,pre,sum;
	}err[6]={0};
	
	float heading = 0;
	float alt = 0;
	float vel = 0;
	uint16_t throttle = 0;
	int16_t accZ = 2048;
	
	DELAY_MS(20);	
	while(1)
	{
/*--------------------------------------------------------*/		
		/*calculate PID*/
		quat.toEuler(att[PITCH],att[ROLL],att[YAW]);
		/*pitch&roll*/
		//trace
		if(ctrl.trace == true)
		{			
			err[PITCH+POS].cur = DEAD_BAND(targetY,0,1);
			err[PITCH+POS].cur = BETWEEN(err[PITCH+POS].cur,-16,16);

			param.PID[PITCH+POS].result =  param.PID[PITCH+POS].P * err[PITCH+POS].cur;
			param.PID[PITCH+POS].result += param.PID[PITCH+POS].D * (err[PITCH+POS].cur-err[PITCH+POS].pre);
			err[PITCH+POS].pre = err[PITCH+POS].cur;
			
			err[ROLL +POS].cur = DEAD_BAND(targetX,0,1);
			err[ROLL +POS].cur = BETWEEN(err[ROLL+POS].cur,-16,16);
			
			param.PID[ROLL +POS].result =  param.PID[ROLL +POS].P * err[ROLL+POS].cur;
			param.PID[ROLL +POS].result += param.PID[ROLL +POS].D * (err[ROLL+POS].cur-err[ROLL+POS].pre);
			err[ROLL +POS].pre = err[ROLL+POS].cur; 
		}
		else
		{
			if(RCValue[HOLD] > 1500 && RCValue[PITCH] == 1500 && RCValue[ROLL] == 1500)
			{
				float _cos = arm_cos_f32(att.yaw / M_57_3);
				float _sin = arm_sin_f32(att.yaw / M_57_3);
				int32_t _lng = lng - att.longitude;
				int32_t _lat = lat - att.latitude;
				
				err[PITCH+POS].cur = DEAD_BAND(-(_lat * _cos - _lng * _sin) / 100,0,2);
				err[PITCH+POS].cur = BETWEEN(err[PITCH+POS].cur,-16,16);

				param.PID[PITCH+POS].result =  param.PID[PITCH+POS].P * err[PITCH+POS].cur;
				param.PID[PITCH+POS].result += param.PID[PITCH+POS].D * (err[PITCH+POS].cur-err[PITCH+POS].pre);
				err[PITCH+POS].pre = err[PITCH+POS].cur;
				
				err[ROLL +POS].cur = DEAD_BAND((_lng * _cos + _lat * _sin) / 100,0,2);
				err[ROLL +POS].cur = BETWEEN(err[ROLL+POS].cur,-16,16);
				
				param.PID[ROLL +POS].result =  param.PID[ROLL +POS].P * err[ROLL+POS].cur;
				param.PID[ROLL +POS].result += param.PID[ROLL +POS].D * (err[ROLL+POS].cur-err[ROLL+POS].pre);
				err[ROLL +POS].pre = err[ROLL+POS].cur; 
			}
			else
			{
				lng = att.longitude; lat = att.latitude;
				param.PID[PITCH+POS].result = param.PID[ROLL+POS].result = 0;
			}
		}
		for(uint8_t i=0;i<2;i++)
		{
			//遥控最多30度
			err[i].cur = BETWEEN(RCValue[i] - 1500,-500,500)/16.0f + param.PID[i+POS].result- att[i];
			param.PID[i].result = param.PID[i].P * err[i].cur;
			//遥控器小于5度积分
			if(RCValue[i]<1580&&RCValue[i]>1420)
				err[i].sum = BETWEEN(err[i].sum+err[i].cur,-500,500);
			else
				err[i].sum = 0;
			param.PID[i].result += param.PID[i].I * err[i].sum;
			param.PID[i].result -= param.PID[i].D * ((i==0?sensorData.gx:sensorData.gy) / GYRO_SCALE);
			param.PID[i].result = BETWEEN(param.PID[i].result,-100,100);
			err[i].pre = err[i].cur;
		}
		
		/*yaw*/
		if(RCValue[YAW] == 1500 && (RCValue[THROTTLE]>1400 || RCValue[HOLD]>1500))
		{
			err[YAW].cur= att[YAW] - heading;
			if(err[YAW].cur>180) err[YAW].cur -= 360;
			else if(err[YAW].cur<-180) err[YAW].cur += 360;
			err[YAW].cur = DEAD_BAND(err[YAW].cur,0,1.2f);
			param.PID[YAW].result = param.PID[YAW].P * err[YAW].cur;
			if(err[YAW].cur == 0) sensorData.gz = 0;
		}
		else
		{
			heading = att[YAW];
			param.PID[YAW].result = 0;
		}
		err[YAW].cur = BETWEEN(RCValue[YAW] - 1500,-500,+500)/8.0f - param.PID[YAW].result  - (sensorData.gz / GYRO_SCALE);
		param.PID[YAW].result = param.PID[YAW].P * err[YAW].cur;
		//遥控器小于5度积分
		if(RCValue[YAW]<1540&&RCValue[YAW]>1460)
			err[YAW].sum = BETWEEN(err[YAW].sum + err[YAW].cur,-500,500); 
		else
			err[YAW].sum = 0;
		param.PID[YAW].result += param.PID[YAW].I * err[YAW].sum;
		param.PID[YAW].result += param.PID[YAW].D * (err[YAW].cur -err[YAW].pre);
		param.PID[YAW].result = BETWEEN(param.PID[YAW].result,-100,+100);
		err[YAW].pre = err[YAW].cur;
		
		/*altitude*/
		if(RCValue[HOLD]>1500)
		{
			static uint8_t state = 0;
			if(state == 0)
			{
				if(alt == 0) 
				{
					alt = att[THROTTLE];
					vel = 0;
					err[THROTTLE].sum = 0;
					err[THROTTLE].pre = att[THROTTLE];
					throttle = RCValue[THROTTLE];
				}
				if((RCValue[THROTTLE]<throttle-100)||(RCValue[THROTTLE]>throttle+100)) 
					alt += (RCValue[THROTTLE]-throttle)/30000.0f;
				err[THROTTLE].cur = (alt - att[THROTTLE]) * 100;//单位从米到厘米
				//死区
				err[THROTTLE].cur = DEAD_BAND(err[THROTTLE].cur,0,20);
				param.PID[THROTTLE].result = param.PID[THROTTLE].P * err[THROTTLE].cur;
				err[THROTTLE].sum = BETWEEN(err[THROTTLE].sum + err[THROTTLE].cur,-500,500);
				param.PID[THROTTLE].result += param.PID[THROTTLE].I * err[THROTTLE].sum;
				vel += (sensorData.az - accZ) / ACCEL_SCALE * 5;//0.05*100
				float baroVel = (att[THROTTLE] - err[THROTTLE].pre) * 2000;//20*100
				//死区
				vel = DEAD_BAND(vel,0,10);
				baroVel = DEAD_BAND(baroVel,0,10);
				err[THROTTLE].pre = att[THROTTLE];
				vel = vel * 0.9f + baroVel * 0.1f;
				param.PID[THROTTLE].result -= param.PID[THROTTLE].D * vel;
				param.PID[THROTTLE].result = BETWEEN(param.PID[THROTTLE].result,-100,+100);
				state = 5;
			}
			state--;
		}
		else 
		{
			alt = 0;
			param.PID[THROTTLE].result = 0;
			throttle = RCValue[THROTTLE];
		}
/*--------------------------------------------------------*/
#ifdef DYNAMIC_PID
		/*动态PID*/
		if(RCValue[THROTTLE]>1400 || RCValue[HOLD] > 1500)
		{
			#define K (4000.0f)
			float t = BETWEEN((RCValue[THROTTLE] - 1000) / K + 1 - 700 / K,0,1.1111f);//油门为1700时t=1
			param.PID[PITCH].result *= t;
			param.PID[ROLL].result *=  t;
			param.PID[THROTTLE].result += BETWEEN((abs(param.PID[PITCH].result) + abs(param.PID[ROLL].result)) / 20.0f,0,5);
		}
#endif
/*--------------------------------------------------------*/		
		/*落地任务*/
		if(ctrl.quadx != true || RCValue[THROTTLE]<1050)
		{
			baro.setGround();
			if(abs(att[PITCH]) < 1 && abs(att[ROLL]) < 1)
				accZ = (accZ + sensorData.az) >> 1;
		}
/*--------------------------------------------------------*/
		/*control motor*/
		//停机条件
		if(abs(att[PITCH])>80||abs(att[ROLL])>80||RCValue[THROTTLE]<1050||ctrl.quadx != true)
		{
			motorValue[0] = 1000;
			motorValue[1] = 1000;
			motorValue[2] = 1000;
			motorValue[3] = 1000;
		}
		else
		{
			motorValue[0] = throttle + param.PID[PITCH].result + param.PID[ROLL].result + param.PID[YAW].result + param.PID[THROTTLE].result;
			motorValue[1] = throttle + param.PID[PITCH].result - param.PID[ROLL].result - param.PID[YAW].result + param.PID[THROTTLE].result;
			motorValue[2] = throttle - param.PID[PITCH].result - param.PID[ROLL].result + param.PID[YAW].result + param.PID[THROTTLE].result;
			motorValue[3] = throttle - param.PID[PITCH].result + param.PID[ROLL].result - param.PID[YAW].result + param.PID[THROTTLE].result;
		}
		Motor::setValue(motorValue[0],motorValue[1],motorValue[2],motorValue[3]);
/*--------------------------------------------------------*/			
		if(ctrl.quadx == true) 
			DELAY_MS(10);//间隔10ms
		else 
			DELAY_MS(20);
	}
}

