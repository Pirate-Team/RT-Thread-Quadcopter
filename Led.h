#ifndef LED_H
#define LED_H
#include "stm32f4xx.h"

class Led
{
public:
	Led(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
	~Led(void);
	void initialize(void);
	void on(void);
	void off(void);
	void toggle(void);
	uint16_t interval;
	GPIO_TypeDef* GPIO;
	uint16_t Pin;
};

void rt_thread_entry_led(void* parameter);

#endif
