#include "Led.h"
#include "head.h"
#include "rtthread.h"

Led led1(GPIOA,GPIO_Pin_5);
Led led2(GPIOB,GPIO_Pin_8);
Led led3(GPIOB,GPIO_Pin_9);

Led::Led(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO = GPIOx;
	Pin = GPIO_Pin;
	interval = 500;
}

Led::~Led(void)
{
}

void Led::initialize(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

    /* GPIO Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIO, &GPIO_InitStructure);
	off();
}

void Led::on(void)
{
	GPIO_ResetBits(GPIO,Pin);
}

void Led::off(void)
{
	GPIO_SetBits(GPIO,Pin);
}

void Led::toggle(void)
{
	GPIO_ToggleBits(GPIO,Pin);
}

void rt_thread_entry_led(void* parameter)
{
	uint32_t led1Tick,led2Tick,led3Tick,tick;
	led1Tick = led2Tick = led3Tick = tick = 0;
	while(1)
	{	
		tick = rt_tick_get();
		//led1
		if(led1.interval == 0xff)
			led1.off();
		else if(led1.interval == 0)
			led1.on();
		else if(tick>led1Tick)
		{
			led1.toggle();
			led1Tick = tick + led1.interval / MS_PER_TICK;
		}
		//led2
		if(led2.interval == 0xff)
			led2.off();
		else if(led2.interval == 0)
			led2.on();
		else if(tick>led2Tick)
		{
			led2.toggle();
			led2Tick = tick + led2.interval / MS_PER_TICK;
		}
		//led3
		if(led3.interval == 0xff)
			led3.off();
		else if(led3.interval == 0)
			led3.on();
		else if(tick>led3Tick)
		{
			led3.toggle();
			led3Tick = tick + led3.interval / MS_PER_TICK;
		}
		DELAY_MS(100);
	}
}

