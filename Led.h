#ifndef LED_H
#define LED_H
#include "stm32f4xx.h"

class Led
{
public:
	Led(void);
	~Led(void);
	void initialize(void);
	void on(void);
	void off(void);
	void toggle(void);
	static uint16_t interval;
};

void rt_thread_entry_led_test(void* parameter);

#endif
