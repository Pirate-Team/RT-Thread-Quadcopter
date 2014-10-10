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

struct ctrl_t
{
	bool att,thro,coor;
	bool alt,track,quadx;
};
struct ctrl_t ctrl = {0};

void hardware_init(void);

void rt_thread_entry_main(void* parameter)
{
/*************************************
	declare variables
*************************************/	
	ctrl.quadx = ctrl.att = ctrl.thro = true; ctrl.coor = ctrl.alt = ctrl.track = false;
	uint8_t rxData[RX_DATA_SIZE] = {0},txData[TX_DATA_SIZE];
	uint8_t major,minor;

/*************************************
	hardware init
*************************************/
	hardware_init();
	
/*************************************
	create thread
*************************************/
	/*led_thread*/
	rt_thread_t led_thread = rt_thread_create("led",
												rt_thread_entry_led_test,
												RT_NULL,
												1024,//max used = 140 
												1,
												10);
	/*communication_thread*/
	rt_thread_t communication_thread = rt_thread_create("communication",
												rt_thread_entry_communication,
												RT_NULL,
												1024,
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

/*************************************
	start thread
*************************************/			
	if(led_thread != RT_NULL) rt_thread_startup(led_thread);
	if(communication_thread != RT_NULL) rt_thread_startup(communication_thread);
	if(quadx_get_thread != RT_NULL) rt_thread_startup(quadx_get_thread);
	if(quadx_control_thread != RT_NULL) rt_thread_startup(quadx_control_thread);
	
	//让出cpu，队尾等待调度
	rt_thread_delay(100);
	
/*************************************
	main loop
*************************************/
	while(1)
	{
		//recv
		if(rt_mq_recv(rxQ,rxData,RX_DATA_SIZE,0) == RT_EOK)
		{
			if(rxData[0]>=0xda&&rxData[0]<=0xdd)
			{
				PID[rxData[0] - 0xda].P = rxData[1] / 10.0f;//P[0,20],精度0.1
				PID[rxData[0] - 0xda].I = rxData[2] / 1000.0f;//I[0,0.250],精度0.001
				PID[rxData[0] - 0xda].D = rxData[3] / 10.0f;//D[0,20],精度0.1
			}
			else if(rxData[0]==0xca)
			{
				//TODO: restart
				NVIC_SystemReset();
			}
			else if(rxData[0]==0xcb)
			{
				//四轴模式
				if(rxData[1] == 0xf1) ctrl.quadx = true;
				else if(rxData[1] == 0xf0) ctrl.quadx = false;
			}
			else if(rxData[0]==0xcc)
			{
				//图像跟踪模式
				if(rxData[1] == 0xf1) ctrl.track = true;
				else if(rxData[1] == 0xf0) ctrl.track = false;
			}
			else if(rxData[0]==0xcd)
			{
				if(rxData[1] == 0xf1) ctrl.att = true;
				else if(rxData[1] == 0xf0) ctrl.att = false;
				if(rxData[2] == 0xf1) ctrl.thro = true;
				else if(rxData[2] == 0xf0) ctrl.thro = false;
				if(rxData[3] == 0xf1) ctrl.coor = true;
				else if(rxData[3] == 0xf0) ctrl.coor = false;
			}
			else if(rxData[0]==0xce)
			{
				//保持高度模式
				if(rxData[1] == 0xf1) ctrl.alt = true;
				else if(rxData[1] == 0xf0) ctrl.alt = false;
			}
			else if(rxData[0]==0xcf)
			{
				if(!ctrl.quadx)
				{
					Led::interval = 100;
					if(rxData[1] == 0xf1)
					{
						MPU6050 *accelgyro = new MPU6050();
						accelgyro->setOffset();
						delete accelgyro;
					}
					else if(rxData[1] == 0xf0)
					{
						HMC5883L *mag = new HMC5883L();
						mag->setOffset();
						delete mag;
					}
					Led::interval = 500;
				}
			}
			else
			{
				rt_kprintf("Unknown command!\r\n");
			}
		}
		//send
		if(ctrl.att)
		{
			uint8_t i;
			txData[0] = 0xea;
			for(i=0;i<3;i++)
				((int16_t*)(txData+1))[i] = att[i] * 10;//角度乘10，有符号
			((uint16_t*)(txData+1))[i] = att[i] * 50;//米乘50，无符号
			rt_mq_send(txQ,txData,TX_DATA_SIZE);
		}
		if(ctrl.thro)
		{
			uint8_t i;
			txData[0] = 0xeb;
			for(i=0;i<4;i++)
				((uint16_t*)(txData+1))[i] = motorValue[i];//电机不乘，有符号
			rt_mq_send(txQ,txData,TX_DATA_SIZE);
		}
		if(ctrl.coor)
		{
			//TODO: send coordinate
		}
		
//		char str[100];
//		sprintf(str,"%+f\t%+f\t%+f\t%+f\r\n",att[PITCH],att[ROLL],att[YAW],att[THROTTLE]);
//		sprintf(str,"%+d\t%+d\t%+d\t%+d\r\n",motorValue[0],motorValue[1],motorValue[2],motorValue[3]);
//		cpu_usage_get(&major,&minor);
//		sprintf(str,"major: %d\tminor: %d\r\n",major,minor);
//		rt_kprintf("%s",str);
		rt_thread_delay(50);
	}
}

void hardware_init(void)
{	
	Receiver::initialize();
	I2Cdev::initialize();
	Motor::initialize();
	
	Led *led = new Led();
	led->initialize();
	led->off();
	delete led;
	
	rt_thread_delay(500);
	
	MPU6050 *accgyro = new MPU6050();
	accgyro->initialize();
	accgyro->setOffset();
	delete accgyro;
	
	HMC5883L *mag = new HMC5883L();
	mag->initialize();
//	mag->setOffset();
	delete mag;
	
	MS5611 *baro = new MS5611();
	baro->initialize();
	delete baro;
}

int  rt_application_init(void)
{	
	cpu_usage_init();

	rt_thread_t main_thread;
	main_thread = rt_thread_create("main",
									rt_thread_entry_main,
									RT_NULL,
									1024,//max used = 140
									8,
									10);
	if(main_thread != RT_NULL)
		rt_thread_startup(main_thread);
	return 0;
}
