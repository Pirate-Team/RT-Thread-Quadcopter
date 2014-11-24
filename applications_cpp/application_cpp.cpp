#include "Led.h"
#include "stdio.h"
#include "rtthread.h"
#include "stm32f4xx.h"
#include "Quadx.h"
#include "Communication.h"
#include "cpu_usage.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "MS5611.h"
#include "Receiver.h"
#include "Motor.h"
#include "I2Cdev.h"
#include "Parameter.h"
#include "Attitude.h"

//#define TRACE_TEST

struct ctrl_t
{
	bool send,quadx,trace;
};
struct ctrl_t ctrl = {0};

extern "C" 
{
	extern int16_t targetX,targetY,targetH,targetW;
	void rt_thread_entry_trace(void* parameter);
	uint8_t ov_7725_init(void);
}

void hardware_init(void);
void param_init(void);
void param_save(void);

void rt_thread_entry_main(void* parameter)
{
/*************************************
	declare variables
*************************************/	
	ctrl.quadx = ctrl.send = ctrl.trace = true;
	rx_data_t rxData;
	tx_data_t txData;
	rt_memset(&txData,0,sizeof(tx_data_t));
	
/*************************************
	hardware init
*************************************/
	hardware_init();

/*************************************
	param init
*************************************/
	param_init();
	
/*************************************
	create thread
*************************************/
	/*led_thread*/
	rt_thread_t led_thread = rt_thread_create("led",
												rt_thread_entry_led,
												RT_NULL,
												192,//max used = 140 
												1,
												10);
	/*communication_thread*/
	rt_thread_t communication_thread = rt_thread_create("communication",
												rt_thread_entry_communication,
												RT_NULL,
												512,
												9,
												10);
	/*quadx_get_thread*/
	rt_thread_t quadx_get_thread = rt_thread_create("quadx_get_attitude",
												rt_thread_entry_quadx_get_attitude,
												RT_NULL,
												1024,
												7,
												10);
	/*quadx_control_thread*/
	rt_thread_t quadx_control_thread = rt_thread_create("quadx_control_attitude",
												rt_thread_entry_quadx_control_attitude,
												RT_NULL,
												1024,
												7,
												10);
												
	/*trace_thread*/
	rt_thread_t trace_thread = rt_thread_create("trace",
												rt_thread_entry_trace,
												RT_NULL,
												1024,
												10,
												500);

/*************************************
	start thread
*************************************/			
	if(led_thread != RT_NULL) rt_thread_startup(led_thread);
	if(communication_thread != RT_NULL) rt_thread_startup(communication_thread);
	if(quadx_get_thread != RT_NULL) rt_thread_startup(quadx_get_thread);
	if(quadx_control_thread != RT_NULL) rt_thread_startup(quadx_control_thread);
//	if(trace_thread != RT_NULL) rt_thread_startup(trace_thread); 
	
	
	//�ó�cpu����β�ȴ�����
	rt_thread_delay(10);
/*************************************
	main loop
*************************************/
	while(1)
	{
/***************led control****************/		
		if(ctrl.quadx == false)
			led3.interval = 1000;
		else
			led3.interval = 500;
		if(ctrl.trace == false)
			led1.interval = 1000;
		else
			led1.interval = 500;
/***************recv begin****************/
		if(rt_mq_recv(rxQ,&rxData,RX_DATA_SIZE,0) == RT_EOK)
		{
			if(rxData.type == 'P')
			{
				param.PID[PITCH].P = param.PID[ROLL].P = rxData.pid.level[0] / 10.0f;//P[0,20],����0.1
				param.PID[PITCH].I = param.PID[ROLL].I = rxData.pid.level[1] / 1000.0f;//I[0,0.250],����0.001
				param.PID[PITCH].D = param.PID[ROLL].D = rxData.pid.level[2] / 10.0f;//D[0,20],����0.1

				param.PID[YAW].P = rxData.pid.heading[0] / 10.0f;//P[0,20],����0.1
				param.PID[YAW].I = rxData.pid.heading[1] / 1000.0f;//I[0,0.250],����0.001
				param.PID[YAW].D = rxData.pid.heading[2] / 10.0f;//D[0,20],����0.1
				
				param.PID[ALT].P = rxData.pid.altitude[0] / 10.0f;//P[0,20],����0.1
				param.PID[ALT].I = rxData.pid.altitude[1] / 1000.0f;//I[0,0.250],����0.001
				param.PID[ALT].D = rxData.pid.altitude[2] / 10.0f;//P[0,20],����0.1
				
				param.PID[LNG].P = param.PID[LAT].P = rxData.pid.position[0] / 10.0f;//P[0,20],����0.1
				param.PID[LNG].I = param.PID[LAT].I = rxData.pid.position[1] / 1000.0f;//I[0,0.250],����0.001
				param.PID[LNG].D = param.PID[LAT].D = rxData.pid.position[2] / 10.0f;//D[0,20],����0.1
			}
			else if(rxData.type == 'C')
			{
				if(rxData.ctrl.restart != 0)
					NVIC_SystemReset();
				
				if(rxData.ctrl.quadx != 0)
					ctrl.quadx = true;
				else 
					ctrl.quadx = false;
				
				if(rxData.ctrl.send != 0)
					ctrl.send = true;
				else
					ctrl.send = false;
				
				if(rxData.ctrl.trace != 0)
					ctrl.trace = true;
				else
					ctrl.trace = false;
				
				if(ctrl.quadx == 0)
				{
					//�������
					if(rxData.ctrl.save != 0)
					{
						led3.interval = 100;
						rt_thread_delay(500);
						param_save();
					}
					if(rxData.ctrl.acc != 0)
					{
						led3.interval = 100;
						rt_thread_delay(500);
						accelgyro.setOffset();
					}
					if(rxData.ctrl.mag != 0)
					{
						led3.interval = 100;
						rt_thread_delay(500);
						mag.setOffset();
					}
				}
			}
			else if(rxData.type == 'G')
			{
				//TODO: GPS
			}
		}
/***************recv end****************/
		
/***************send begin****************/
		if(ctrl.send != 0)
		{
			txData.status.type = 'S';
			
			txData.status.gps[0] ++;
			txData.status.gps[1] = 2;
			
			//�Ƕȳ�10���з���
			txData.status.att[0] = att[0] * 10;
			txData.status.att[1] = att[1] * 10;
			txData.status.att[2] = att[2] * 10;
			//�׳�50���޷���
			txData.status.att[3] = att[3] * 50;
			
			rt_memcpy(txData.status.motor,motorValue,8);//������ˣ��޷���,ֱ�ӿ���
			
			txData.status.target[0] = 1;//targetX;//Ŀ��λ�ã������任
			txData.status.target[1] = 2;//targetY;
			txData.status.target[2] = 3;//targetH;//Ŀ�곤�������任
			txData.status.target[3] = 4;//targetW;
			
			rt_mq_send(txQ,&txData,TX_DATA_SIZE);
		}
/***************send end****************/
		rt_thread_delay(100);
	}
}

void hardware_init(void)
{
	rt_thread_delay(500);
	
	led1.initialize();
	led2.initialize();
	led3.initialize();
	led1.interval = 1000;
	led2.interval = 0xff;
	led3.interval = 1000;
	led1.on();
	led2.on();
	led3.on();
	rt_thread_delay(1000);
	led1.off();
	led2.off();
		
	Receiver::initialize();
	I2Cdev::initialize();
	Motor::initialize();
	
#ifndef TRACE_TEST
	while(!accelgyro.initialize())
	{
		led2.toggle();
		rt_thread_delay(50);
	}
	
	while(!mag.initialize())
	{
		led2.toggle();
		rt_thread_delay(200);
	}
	
	while(!baro.initialize())
	{
		led2.toggle();
		rt_thread_delay(350);
	}
#endif
	
//	if(!ov_7725_init())
//		led1.interval = 0;
	
	Communication::initialize();
}

void param_init(void)
{
	if(!param.flashRead())
		led2.interval = 0;
	else
		led2.interval = 0xff;
}

void param_save(void)
{
	if(!param.flashWrite())
		led2.interval = 0;
	else
		led2.interval = 0xff;
}

int  rt_application_init(void)
{	
	cpu_usage_init();
	
	rt_thread_t main_thread;
	main_thread = rt_thread_create("main",
									rt_thread_entry_main,
									RT_NULL,
									512,
									8,
									10);
	if(main_thread != RT_NULL)
		rt_thread_startup(main_thread);
	return 0;
}
