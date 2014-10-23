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
#include "Led.h"
/*-----------------------------------
	define
-----------------------------------*/
#define M_57_3 57.29577f
#define GYRO_SCALE 32.8f
#define ACCEL_SCALE 2048.0f
#define BETWEEN(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
/*-----------------------------------
	global
-----------------------------------*/
float att[4] = {0};

struct sensor_data_t sensorData = {0};

struct pid_t PID[4] = {0};

struct ctrl_t
{
	bool att,thro,coor,alt,trace,quadx;
};
extern struct ctrl_t ctrl;
/*-----------------------------------
	thread
-----------------------------------*/
void rt_thread_entry_quadx_get_attitude(void* parameter)
{
	MPU6050 accelgyro;
	accelgyro.getMotion6Cal(&sensorData.ax, &sensorData.ay, &sensorData.az, &sensorData.gx, &sensorData.gy, &sensorData.gz);
	HMC5883L mag;
	MS5611 baro;
	rt_thread_delay(100);
	baro.getTemperature();
	rt_thread_delay(100);
	baro.getPressure();
	
	uint8_t state = 0;
	uint32_t preTick = 0,curTick = 0;
	while(1)
	{
		/*getSensorData*/
		accelgyro.getMotion6Cal(&sensorData.ax, &sensorData.ay, &sensorData.az, &sensorData.gx, &sensorData.gy, &sensorData.gz);
		if(state == 0 || state == 20)
		{
			mag.getHeadingCal(&sensorData.mx,&sensorData.my,&sensorData.mz);
			if(state == 0) state = 40;
//			char str[50];
//			sprintf(str,"heading=%+f\r\n",sensorData.heading);
//			rt_kprintf(str);
		}
		else if(state == 30)
		{
			baro.getTemperature(&sensorData.temperature);
		}
		else if(state == 10)
		{
			baro.getPressure(&sensorData.pressure);
			att[THROTTLE] += ((pow((float)SEA_PRESS / sensorData.pressure, 1/5.257f) - 1.0f) * (sensorData.temperature + 273.15f)) / 0.0065f;
			att[THROTTLE] /= 2;
		}
		state--;
		/*calculate attitude*/
		preTick = curTick;
		curTick = rt_tick_get();
		sampleInterval = (curTick - preTick) / 1000.0f + 0.0001f;
		//姿态数据
		MadgwickAHRSupdate((float)sensorData.gx/GYRO_SCALE,(float)sensorData.gy/GYRO_SCALE,(float)sensorData.gz/GYRO_SCALE,(float)sensorData.ax,(float)sensorData.ay,(float)sensorData.az,(float)sensorData.mx,(float)sensorData.my,(float)sensorData.mz);
		//MadgwickAHRSupdateIMU((float)sensorData.gx/GYRO_SCALE,(float)sensorData.gy/GYRO_SCALE,(float)sensorData.gz/GYRO_SCALE,(float)sensorData.ax,(float)sensorData.ay,(float)sensorData.az);
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
	}err[4]={0};
	
	float heading = 0;
	float alt = 0;
	float vel = 0;
	uint16_t throttle = 0;
	int16_t RC[3] = {0};
	
	rt_thread_delay(100);
	while(1)
	{
//		RCValue[THROTTLE] = 1100;
		/*calculate PID*/
		{
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
				PID[i].result = BETWEEN(PID[i].result,-100,100);
				
				err[i].pre = err[i].cur;
			}
			
			/*yaw*/
			if(RCValue[YAW] == 1500 && RCValue[THROTTLE]>1400)
			{
				float err = att[YAW] - heading;
				if(err>180) err -= 360;
				else if(err<-180) err += 360;
				RC[YAW] = PID[YAW].P * err;
				if(fabs(err) < 1.2f) sensorData.gz = 0;
			}
			else heading = att[YAW];
			
			err[YAW].cur = (RCValue[YAW] - 1500)/8.0f - RC[YAW]  - (sensorData.gz / GYRO_SCALE);
			PID[YAW].result = PID[YAW].P * err[YAW].cur;
			
			//遥控器小于5度积分
			if(RCValue[YAW]<1540&&RCValue[YAW]>1460)
				err[YAW].sum = BETWEEN(err[YAW].sum + err[YAW].cur,-1000,1000); 
			else
				err[YAW].sum = 0;
			PID[YAW].result += PID[YAW].I * err[YAW].sum;
			
			PID[YAW].result += PID[YAW].D * (err[YAW].cur -err[YAW].pre);
			
			PID[YAW].result = BETWEEN(PID[YAW].result,-50,+50);
			
			err[YAW].pre = err[YAW].cur;
			
			/*altitude*/
			if(ctrl.alt)
			{
				static uint8_t state = 0;
				if(state == 0)
				{
					if(alt == 0) 
					{
						alt = att[THROTTLE];
						vel = 0;
						PID[THROTTLE].result = 0;
						err[THROTTLE].sum = 0;
						err[THROTTLE].pre = att[THROTTLE];
						throttle = RCValue[THROTTLE];
					}
					
					if((RCValue[THROTTLE]<throttle-100)||(RCValue[THROTTLE]>throttle+100)) 
						alt -= (RCValue[THROTTLE]-throttle)/50000.0f;
					
					err[THROTTLE].cur = (alt - att[THROTTLE]) * 100;
					PID[THROTTLE].result = PID[THROTTLE].P * err[THROTTLE].cur;
					
					err[THROTTLE].sum = BETWEEN(err[THROTTLE].sum + err[THROTTLE].cur,-5000,5000);
					PID[THROTTLE].result += PID[THROTTLE].I * err[THROTTLE].sum;
					
					vel += (sensorData.az - 2048) / ACCEL_SCALE * 0.05f;
					float baroVel = (att[THROTTLE] - err[THROTTLE].pre) * 20;
					err[THROTTLE].pre = att[THROTTLE];
					vel = vel * 0.9f + baroVel * 0.1f;
					
//					char str[100];
//					sprintf(str,"vel = %+f\tbaro = %+f\r\n",vel,baroVel);
//					//sprintf(str,"gx=%+d\tgy=%+d\tgz=%+d\tax=%+d\tay=%+d\taz=%+d\r\n",sensorData.gx,sensorData.gy,sensorData.gz,sensorData.ax,sensorData.ay,sensorData.az);
//					rt_kprintf(str);
					
					PID[THROTTLE].result -= PID[THROTTLE].D * vel * 100;
					
					PID[THROTTLE].result = BETWEEN(PID[THROTTLE].result,-100,+100);
					
					state = 5;
				}
				state--;
			}
			else 
			{
				alt = 0;
				throttle = RCValue[THROTTLE];
			}
		}
		/*control motor*/
		{
			//停机条件
			if(abs(att[PITCH])>80||abs(att[ROLL])>80||RCValue[THROTTLE]<1060||ctrl.quadx == false)
			{
				motorValue[0] = 1000;
				motorValue[1] = 1000;
				motorValue[2] = 1000;
				motorValue[3] = 1000;
			}
			else
			{
				motorValue[0] = throttle + PID[PITCH].result + PID[ROLL].result + PID[YAW].result + PID[THROTTLE].result;
				motorValue[1] = throttle + PID[PITCH].result - PID[ROLL].result - PID[YAW].result + PID[THROTTLE].result;
				motorValue[2] = throttle - PID[PITCH].result - PID[ROLL].result + PID[YAW].result + PID[THROTTLE].result;
				motorValue[3] = throttle - PID[PITCH].result + PID[ROLL].result - PID[YAW].result + PID[THROTTLE].result;
			}
			Motor::setValue(motorValue[0],motorValue[1],motorValue[2],motorValue[3]);
		}
		if(ctrl.quadx == false) rt_thread_delay(100);
		rt_thread_delay(10);
	}
}

