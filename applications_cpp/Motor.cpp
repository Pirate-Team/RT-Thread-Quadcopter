#include "Motor.h"
#include "head.h"
/*-------------------------------------------------
	使用TIM1，通道PA8,9,10,11
--------------------------------------------------*/
#define COUNTER_FREQ 1000000	//计数器频率，精确到1us
#define PWM_FREQ 50				//PWM频率，周期20ms
#define PRESCALER        ((SystemCoreClock / COUNTER_FREQ) - 1) 
#define ARR              ((COUNTER_FREQ / PWM_FREQ) - 1)
#define INIT_DUTYCYCLE	 ((uint16_t)1000)	//初始化脉宽0.9ms

uint16_t motorValue[MOTORALL] = {INIT_DUTYCYCLE};

void Motor::initialize()
{
/*-------------------------------------------------
	GPIO配置，时钟，端口，使能，复用，
--------------------------------------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* TIM1 CH1 (PA8), TIM1 CH2 (PA9), TIM1 CH3 (PA10) and TIM1 CH4 (PA11) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

	/* Connect TIM1 pins to AF */  
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1); 
	
/*----------------------------------------------
	定时器4输出4路PWM配置
----------------------------------------------*/
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef        TIM_OCInitStructure;

	RCC_TIMCLKPresConfig(RCC_TIMPrescActivated);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = (uint16_t)ARR;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)PRESCALER;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse       = INIT_DUTYCYCLE;
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel2 */
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	/* PWM1 Mode configuration: Channel3 */
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	/* PWM1 Mode configuration: Channel4 */
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	TIM_ARRPreloadConfig(TIM1, ENABLE);

	/* TIM1 disable counter */
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
}

/*----------------------------------------------
	1000<=Value<=2000
----------------------------------------------*/
void Motor::setValue(MOTOR_ENUM motor,uint16_t Value)
{
	//限速1800
#define LIMIT 1800
	Value = BETWEEN(Value,1000,LIMIT);
//	Value = (Value>LIMIT)?(LIMIT):((Value<INIT_DUTYCYCLE)?(INIT_DUTYCYCLE):(Value));
	switch(motor)
	{
		case MOTOR1:
			TIM_SetCompare1(TIM1,Value);
			break;
		case MOTOR2:
			TIM_SetCompare2(TIM1,Value);
			break;
		case MOTOR3:
			TIM_SetCompare3(TIM1,Value);
			break;
		case MOTOR4:
			TIM_SetCompare4(TIM1,Value);
			break;
		default:
			break;
	}
}

void Motor::setValue(uint16_t Value)
{
	setValue(MOTOR1,Value);
	setValue(MOTOR2,Value);
	setValue(MOTOR3,Value);
	setValue(MOTOR4,Value);
}

void Motor::setValue(uint16_t Value1,uint16_t Value2,uint16_t Value3,uint16_t Value4)
{
	setValue(MOTOR1,Value1);
	setValue(MOTOR2,Value2);
	setValue(MOTOR3,Value3);
	setValue(MOTOR4,Value4);
}

void Motor::getValue(MOTOR_ENUM motor,uint16_t& Value)
{
	switch(motor)
	{
		case MOTOR1:
			Value = (uint16_t)TIM_GetCapture1(TIM1);
			break;
		case MOTOR2:
			Value = (uint16_t)TIM_GetCapture2(TIM1);
			break;
		case MOTOR3:
			Value = (uint16_t)TIM_GetCapture3(TIM1);
			break;
		case MOTOR4:
			Value = (uint16_t)TIM_GetCapture4(TIM1);
			break;
		default:
			break;
	}
}
