
/* TIM3 CH1 (PC6), TIM3 CH2 (PC7) , TIM3 CH3 (PC8), TIM3 CH4 (PC9) */

#include "Receiver.h"
#include "rtthread.h"


#define COUNTER_FREQ 1000000	//计数器频率，精确到1us
#define PWM_FREQ 50			//PWM频率，周期20ms
#define PRESCALER        ((SystemCoreClock / COUNTER_FREQ) - 1) 
#define ARR              ((COUNTER_FREQ / PWM_FREQ) - 1)

uint16_t RCValue[RC_NUM] = {1500};
static uint16_t preCCR[RC_NUM] = {0};
static uint16_t CCR[RC_NUM] = {0};
uint8_t RCFlag[RC_NUM] = {0};
uint16_t preThrottle = 0;

bool Receiver::initialize(void)
{
	for(uint8_t i =0;i<RC_NUM;i++)
		RCValue[i] = 1500;
	RCValue[THROTTLE] = 1000;
/*-------------------------------------------------
	GPIO配置，时钟，端口，使能，复用，
--------------------------------------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* TIM3 CH1 (PC6), TIM3 CH2 (PC7) , TIM3 CH3 (PC8), TIM3 CH4 (PC9) */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
	GPIO_Init(GPIOC, &GPIO_InitStructure); 

	/* Connect TIM3 pins to AF */  
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
	
	
/*----------------------------------------------
	NVIC
----------------------------------------------*/
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 

/*----------------------------------------------
	TIM3
----------------------------------------------*/
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef        TIM_ICInitStructure;

	RCC_TIMCLKPresConfig(RCC_TIMPrescActivated);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = (uint16_t)ARR;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)PRESCALER;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	TIM_ITConfig(TIM3,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,DISABLE);
	/* PWM1 Mode configuration: Channel1 */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM3,&TIM_ICInitStructure);

	/* PWM1 Mode configuration: Channel2 */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM3,&TIM_ICInitStructure);

	/* PWM1 Mode configuration: Channel3 */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM3,&TIM_ICInitStructure);

	/* PWM1 Mode configuration: Channel4 */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM3,&TIM_ICInitStructure);

	
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4);
	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
	
	return true;
}

void Receiver::getRCValue(int16_t* value)
{
	for(uint8_t i=0;i<RC_NUM;i++)
		value[i] = RCValue[i];
}

//供C文件使用，不能在CPP里直接写终端服务函数，因为CPP的函数名字不一样
extern "C" void MyTIM3_IRQHandler(void);
void MyTIM3_IRQHandler(void)
{
	//update
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
		for(uint8_t i=0;i<RC_NUM;i++)
		{
			if(RCFlag[i] == 2)
			{
				RCValue[THROTTLE] -= preThrottle / 100;
				if(RCValue[THROTTLE] <1000) RCValue[THROTTLE] = 1000;
				for(uint8_t i=0;i<3;i++) RCValue[i] = 1500;
			}
			else
			{
				RCFlag[i] = RCFlag[i] + 1;
				if(i == THROTTLE) preThrottle = RCValue[THROTTLE] - 1000;
			}
		}
	}
	//channel 1
	if(TIM_GetITStatus(TIM3,TIM_IT_CC1) == SET)
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_CC1);
		if(GPIOC->IDR  & GPIO_Pin_6)
		{
			preCCR[PITCH] = TIM_GetCapture1(TIM3);
		}
		else
		{
			CCR[PITCH] = TIM_GetCapture1(TIM3);
			if(CCR[PITCH] > preCCR[PITCH])
				RCValue[PITCH] = (CCR[PITCH] - preCCR[PITCH]);
			else
				RCValue[PITCH] = (ARR + CCR[PITCH] - preCCR[PITCH]);
			if(RCValue[PITCH]>2000) RCValue[PITCH] = 2000;
			else if(RCValue[PITCH]<1000) RCValue[PITCH] = 1000;
			RCFlag[PITCH] = 0;
		}
	}
	//channel 2
	if(TIM_GetITStatus(TIM3,TIM_IT_CC2) == SET)
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_CC2);
		if(GPIOC->IDR  & GPIO_Pin_7)
		{
			preCCR[ROLL] = TIM_GetCapture2(TIM3);
		}
		else
		{
			CCR[ROLL] = TIM_GetCapture2(TIM3);
			if(CCR[ROLL] > preCCR[ROLL])
				RCValue[ROLL] = (CCR[ROLL] - preCCR[ROLL]);
			else
				RCValue[ROLL] = (ARR + CCR[ROLL] - preCCR[ROLL]);
			if(RCValue[ROLL]>2000) RCValue[ROLL] = 2000;
			else if(RCValue[ROLL]<1000) RCValue[ROLL] = 1000;
			RCFlag[ROLL] = 0;
		}
	}
	//channel 3
	if(TIM_GetITStatus(TIM3,TIM_IT_CC3) == SET)
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_CC3);
		if(GPIOC->IDR  & GPIO_Pin_8)
		{
			preCCR[YAW] = TIM_GetCapture3(TIM3);
		}
		else
		{
			CCR[YAW] = TIM_GetCapture3(TIM3);
			if(CCR[YAW] > preCCR[YAW])
				RCValue[YAW] = (CCR[YAW] - preCCR[YAW]);
			else
				RCValue[YAW] = (ARR + CCR[YAW] - preCCR[YAW]);
			if(RCValue[YAW]>2000) RCValue[YAW] = 2000;
			else if(RCValue[YAW]<1000) RCValue[YAW] = 1000;
			RCFlag[YAW] = 0;
		}
	}
	//channel 4
	if(TIM_GetITStatus(TIM3,TIM_IT_CC4) == SET)
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_CC4);
		if(GPIOC->IDR  & GPIO_Pin_9)
		{
			preCCR[THROTTLE] = TIM_GetCapture4(TIM3);
		}
		else
		{
			CCR[THROTTLE] = TIM_GetCapture4(TIM3);
			if(CCR[THROTTLE] > preCCR[THROTTLE])
				RCValue[THROTTLE] = (CCR[THROTTLE] - preCCR[THROTTLE]);
			else
				RCValue[THROTTLE] = (ARR + CCR[THROTTLE] - preCCR[THROTTLE]);
			if(RCValue[THROTTLE]>2000) RCValue[THROTTLE] = 2000;
			else if(RCValue[THROTTLE]<1000) RCValue[THROTTLE] = 1000;
			RCFlag[THROTTLE] = 0;
		}
	}
}

void rt_thread_entry_receiver_test(void* parameter)
{
	Receiver::initialize();
	
	while(1)
	{
		rt_thread_delay(200);
		rt_kprintf("%d\t%d\t%d\t%d\r\n",RCValue[PITCH],RCValue[ROLL],RCValue[YAW],RCValue[THROTTLE]);
	}
}
