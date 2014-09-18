#include "Led.h"
#include "rtthread.h"
bool Led::state = false;

Led::Led(void)
{
}

Led::~Led(void)
{
}

void Led::init(void)
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
	state = true;
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);
}

void Led::off(void)
{
	state = false;
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
}

void Led::toggle(void)
{
	if(state)
		off();
	else
		on();
}

bool Led::getState(void)
{
	return state;
}

void rt_thread_entry_led_test(void* parameter)
{
	Led led;
	led.init();
	while(1)
	{
//		uint32_t pretick;
//		volatile uint32_t i = 8400000;
//		rt_enter_critical();
//		pretick = rt_tick_get();
//		while (i)
//			i--;
//		rt_kprintf("ticks = %d\r\n",rt_tick_get() - pretick);
//		rt_exit_critical();
		
		led.toggle();
		
		rt_thread_delay(500);
	}
}
