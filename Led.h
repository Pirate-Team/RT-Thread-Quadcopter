#ifndef LED_H
#define LED_H

#include <stdio.h>

#include "stm32f4xx.h"

#include <rtthread.h>

class Led
{
public:
	Led(void);
	~Led(void);
	void on(void);
	void off(void);
	void toggle(void);
	bool getState(void);
private:
	static bool state;
};

void rt_thread_entry_led_test(void* parameter);

#endif
