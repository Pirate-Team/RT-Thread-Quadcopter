#include "Nema_decode.h"
#include "rtthread.h"
#include "head.h"
#include "nmea/nmea.h"
#include "stdio.h"
#include "Attitude.h"
#include "ctype.h"
#include "stm32f4xx.h"

#define GPS_BUFFER_SIZE (320)
#define GPS_BUFFER_SIZE_HALF (GPS_BUFFER_SIZE/2)

volatile uint8_t GPS_TransferEnd = 0, GPS_HalfTransferEnd = 0;
uint8_t gps_buffer[GPS_BUFFER_SIZE];
bool fixed = false;

int GPS_coord_to_degrees(char* s);
void usart1_init(void);

void rt_thread_entry_getgpsdata(void* parameter)
{
	uint32_t tick = 0;
    nmeaINFO info;          //GPS解码后得到的信息
    nmeaPARSER parser;      //解码时使用的数据结构  
    uint8_t new_parse=0;    //是否有新的解码数据标志

    /* 初始化GPS数据结构 */
    nmea_zero_INFO(&info);
    nmea_parser_init(&parser);
	
	DELAY_MS(1000);
	usart1_init();

    while(1)
    {
		if(GPS_HalfTransferEnd == 1)
		{
			new_parse = nmea_parse(&parser, (const char*)&gps_buffer[0], GPS_BUFFER_SIZE_HALF, &info);
			GPS_HalfTransferEnd = 0;
		}
		else if(GPS_TransferEnd)
		{
			new_parse = nmea_parse(&parser, (const char*)&gps_buffer[GPS_BUFFER_SIZE_HALF], GPS_BUFFER_SIZE_HALF, &info);
			GPS_TransferEnd = 0;	
		}
		
		if(new_parse )                //有新的解码消息   
		{    
			if(info.fix == 2 || info.fix == 3)
			{
				char str[16];
				sprintf(str,"%f",info.lon);
				att.longitude = ((int64_t)att.longitude*3 + (int64_t)GPS_coord_to_degrees(str)) >> 2;
				sprintf(str,"%f",info.lat);
				att.latitude = ((int64_t)att.latitude*3 + (int64_t)GPS_coord_to_degrees(str)) >> 2;
				fixed = true;
			}
			else
				fixed = false;
			
			tick = rt_tick_get() + 1000;//2s
			new_parse = 0;
		}
		else if(tick < rt_tick_get())
		{
			fixed = false;
		}
			
		DELAY_MS(200);
	}
}

int GPS_coord_to_degrees(char* s) {
	char *p, *q;
	short deg = 0, min = 0;
	unsigned int frac_min = 0;
	short i;

	// scan for decimal point or end of field
	for (p = s; isdigit(*p); p++) ;
	q = s;

	// convert degrees
	while ((p - q) > 2) {
		if (deg) deg *= 10;
		deg += ((*q++)-'0');
	}
	// convert minutes
	while (p > q) {
		if (min) min *= 10;
		min += ((*q++)-'0');
	}
	// convert fractional minutes
	// expect up to four digits, result is in
	// ten-thousandths of a minute
	if (*p == '.') {
		q = p + 1;
		for (i = 0; i < 5; i++) {
			frac_min *= 10;
			if (isdigit(*q))
				frac_min += *q++ - '0';
		}
	}
	return deg * 10000000UL + (min * 1000000UL + frac_min*10UL) / 6;
}

void usart1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;	
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);
	
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)gps_buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = (uint32_t)GPS_BUFFER_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_DeInit(DMA2_Stream5);
	DMA_Init(DMA2_Stream5, &DMA_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	DMA_ITConfig(DMA2_Stream5,DMA_IT_HT|DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA2_Stream5,ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	USART_Cmd(USART1,ENABLE);
}

extern "C" void GPS_ProcessDMAIRQ(void);
void GPS_ProcessDMAIRQ(void)
{
	if(DMA_GetITStatus(DMA2_Stream5,DMA_IT_HTIF5) )         /* DMA 半传输完成 */
	{
		GPS_HalfTransferEnd = 1;                //设置半传输完成标志位
		DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_HTIF5);
	}
	else if(DMA_GetITStatus(DMA2_Stream5,DMA_IT_TCIF5))     /* DMA 传输完成 */
	{
		GPS_TransferEnd = 1;                    //设置传输完成标志位
		DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);
	}
}
