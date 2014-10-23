#ifndef __SCCB_H
#define __SCCB_H

#include "stm32f4xx.h"

#define SCL_H         GPIO_SetBits(GPIOA , GPIO_Pin_15)
#define SCL_L         GPIO_ResetBits(GPIOA , GPIO_Pin_15) 
   
#define SDA_H         GPIO_SetBits(GPIOA , GPIO_Pin_14) 
#define SDA_L         GPIO_ResetBits(GPIOA , GPIO_Pin_14)

#define SCL_read      GPIO_ReadInputDataBit(GPIOA , GPIO_Pin_15) 
#define SDA_read      GPIO_ReadInputDataBit(GPIOA , GPIO_Pin_14) 

#define ADDR_OV7725   0x42

void SCCB_GPIO_Config(void);
int SCCB_WriteByte( u16 WriteAddress , u8 SendByte);
int SCCB_ReadByte(u8* pBuffer,   u16 length,   u8 ReadAddress);
void SCCB_delay(void);
#endif 
