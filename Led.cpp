#include "Led.h"
#include "rtthread.h"

uint16_t Led::interval = 500;

Led::Led(void)
{
}

Led::~Led(void)
{
}

void Led::initialize(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

    /* GPIOA Periph clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	off();
}

void Led::on(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
}

void Led::off(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);
}

void Led::toggle(void)
{
	GPIO_ToggleBits(GPIOA,GPIO_Pin_5);
}

void rt_thread_entry_led_test(void* parameter)
{
	Led led;
	while(1)
	{	
		led.toggle();	
		rt_thread_delay(led.interval);
	}
}
