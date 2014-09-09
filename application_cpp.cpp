#include "Led.h"
#include "rtthread.h"
#include "stm32f4xx.h"
#include <stdio.h>

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
	return 0;
}
