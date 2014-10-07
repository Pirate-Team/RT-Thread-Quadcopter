#include "Led.h"
#include "stdio.h"
#include "rtthread.h"
#include "stm32f4xx.h"
#include "Quadx.h"
#include "Communication.h"
#include "cpu_usage.h"

void rt_thread_entry_main(void* parameter)
{	
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
												8,
												10);

/*************************************
	start thread
*************************************/										
	if(led_thread != RT_NULL) rt_thread_startup(led_thread);
	if(communication_thread != RT_NULL) rt_thread_startup(communication_thread);
	if(quadx_get_thread != RT_NULL) rt_thread_startup(quadx_get_thread);
	if(quadx_control_thread != RT_NULL) rt_thread_startup(quadx_control_thread);

/*************************************
	declare variables
*************************************/	
	bool sendAtt = true,sendThro = true,sendCoor = false;
	char str[100];
	uint8_t rxData[RX_DATA_SIZE] = {0},txData[TX_DATA_SIZE];
	uint8_t major,minor;
	
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
				PID[rxData[0] - 0xda].P = rxData[1] / 2000.0f;//P[0,0.1],精度0.005
				PID[rxData[0] - 0xda].I = rxData[2] / 10000.0f;//I[0,0.02],精度0.0001
				PID[rxData[0] - 0xda].D = rxData[3] / 10000.0f;//D[0,0.02],精度0.0001
			}
			else if(rxData[0]==0xca)
			{
				//TODO: restart
				NVIC_SystemReset();
			}
			else if(rxData[0]==0xcb)
			{
				//四轴模式
				if(rxData[1] == 0xf1)
				{
					rt_thread_resume(quadx_get_thread);
					rt_thread_resume(quadx_control_thread);
				}
				else if(rxData[1] == 0xf0)
				{
					rt_thread_suspend(quadx_get_thread);
					rt_thread_suspend(quadx_control_thread);
				}
			}
			else if(rxData[0]==0xcc)
			{
				//图像跟踪模式
				//TODO: track
			}
			else if(rxData[0]==0xcd)
			{
				if(rxData[1] == 0xf1) sendAtt = true;
				else if(rxData[1] == 0xf0) sendAtt = false;
				if(rxData[2] == 0xf1) sendThro = true;
				else if(rxData[2] == 0xf0) sendThro = false;
				if(rxData[3] == 0xf1) sendCoor = true;
				else if(rxData[3] == 0xf0) sendCoor = false;
			}
			else if(rxData[0]==0xce)
			{
				//保持高度模式
				holdAlt = true;
			}
			else
			{
				rt_kprintf("Unknown command!\r\n");
			}
		}
		//send
		if(sendAtt)
		{
			uint8_t i;
			txData[0] = 0xea;
			for(i=0;i<3;i++)
				((int16_t*)(txData+1))[i] = att[i] * 1000;//角度乘10，有符号
			((uint16_t*)(txData+1))[i] = att[i] * 50;//米乘50，无符号
			rt_mq_send(txQ,txData,TX_DATA_SIZE);
		}
		if(sendThro)
		{
			uint8_t i;
			txData[0] = 0xeb;
			for(i=0;i<4;i++)
				((uint16_t*)(txData+1))[i] = motorThro[i];//电机不乘，有符号
			rt_mq_send(txQ,txData,TX_DATA_SIZE);
		}
		if(sendCoor)
		{
			//TODO: send coordinate
		}
		
//		sprintf(str,"%+f\t%+f\t%+f\t%+f\r\n",att[PITCH],att[ROLL],att[YAW],att[THROTTLE]);
//		sprintf(str,"%+d\t%+d\t%+d\t%+d\r\n",motorThro[0],motorThro[1],motorThro[2],motorThro[3]);
//		cpu_usage_get(&major,&minor);
//		sprintf(str,"major: %d\tminor: %d\r\n",major,minor);
//		rt_kprintf("%s",str);
		rt_thread_delay(50);
	}
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
