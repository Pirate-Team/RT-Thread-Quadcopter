#include "Motor.h"
/*-------------------------------------------------
	使用TIM4，通道PB6,PB7,PB8,PB9
--------------------------------------------------*/

#define COUNTER_FREQ 100000	//计数器频率，精确到10us
#define PWM_FREQ 50				//PWM频率，周期20ms
#define PRESCALER        ((SystemCoreClock / COUNTER_FREQ) - 1) 
#define ARR              ((COUNTER_FREQ / PWM_FREQ) - 1)
#define INIT_DUTYCYCLE	 ((uint8_t)80)	//初始化脉宽0.8ms

bool Motor::isStart = false;	//静态成员变量初始化

Motor::Motor(){}

Motor::~Motor(){}

void Motor::init()
{
/*-------------------------------------------------
	GPIO配置，时钟，端口，使能，复用，
--------------------------------------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* TIM4 CH1 (PB6), TIM4 CH2 (PB7), TIM4 CH3 (PB8) and TIM4 CH4 (PB9) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 

	/* Connect TIM4 pins to AF2 */  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4); 
	
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
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse       = INIT_DUTYCYCLE;
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel2 */
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel3 */
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel4 */
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);

	/* TIM4 disable counter */
	TIM_Cmd(TIM4, DISABLE);

/*----------------------------------------------
	是否启动标志
----------------------------------------------*/
	isStart = false;
}

void Motor::start()
{
	isStart = true;
	TIM_SetCompare1(TIM4,INIT_DUTYCYCLE);
	TIM_SetCompare2(TIM4,INIT_DUTYCYCLE);
	TIM_SetCompare3(TIM4,INIT_DUTYCYCLE);
	TIM_SetCompare4(TIM4,INIT_DUTYCYCLE);
	TIM_Cmd(TIM4,DISABLE);
}

void Motor::stop()
{
	isStart = false;
	TIM_SetCompare1(TIM4,INIT_DUTYCYCLE);
	TIM_SetCompare2(TIM4,INIT_DUTYCYCLE);
	TIM_SetCompare3(TIM4,INIT_DUTYCYCLE);
	TIM_SetCompare4(TIM4,INIT_DUTYCYCLE);
	TIM_Cmd(TIM4,ENABLE);
}
/*----------------------------------------------
	0<=throttle<=100---->100<=CRR<=200
----------------------------------------------*/
void Motor::setThrottle(MOTOR_ENUM motor,uint8_t throttle)
{
	uint8_t crr = throttle + 100;
	if(crr>200) crr = 200;
	else if(crr<100) crr = 100;
	
	switch(motor)
	{
		case MOTOR1:
			TIM_SetCompare1(TIM4,crr);
			break;
		case MOTOR2:
			TIM_SetCompare2(TIM4,crr);
			break;
		case MOTOR3:
			TIM_SetCompare3(TIM4,crr);
			break;
		case MOTOR4:
			TIM_SetCompare4(TIM4,crr);
			break;
		default:
			break;
	}
}

void Motor::setThrottle(uint8_t throttle)
{
	setThrottle(MOTOR1,throttle);
	setThrottle(MOTOR2,throttle);
	setThrottle(MOTOR3,throttle);
	setThrottle(MOTOR4,throttle);
}

void Motor::setThrottle(uint8_t throttle1,uint8_t throttle2,uint8_t throttle3,uint8_t throttle4)
{
	setThrottle(MOTOR1,throttle1);
	setThrottle(MOTOR2,throttle2);
	setThrottle(MOTOR3,throttle3);
	setThrottle(MOTOR4,throttle4);
}

void Motor::getThrottle(MOTOR_ENUM motor,uint8_t& throttle)
{
	switch(motor)
	{
		case MOTOR1:
			throttle = (uint8_t)TIM_GetCapture1(TIM4);
			break;
		case MOTOR2:
			throttle = (uint8_t)TIM_GetCapture2(TIM4);
			break;
		case MOTOR3:
			throttle = (uint8_t)TIM_GetCapture3(TIM4);
			break;
		case MOTOR4:
			throttle = (uint8_t)TIM_GetCapture4(TIM4);
			break;
		default:
			break;
	}
	throttle -= 100;
}
