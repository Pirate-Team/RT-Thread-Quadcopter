#include "Parameter.h"

Parameter param;

Parameter::Parameter()
{
	rt_memset(this,0,sizeof(Parameter));
	accXOffset = -1;
	accYOffset = -12;
	accZOffset = -131;
	gyroXOffset = -35;
	gyroYOffset = -15;
	gyroZOffset = -39;
	magXOffset = -158;
	magYOffset = 93;
	magZOffset = -28;
}
bool Parameter::flashRead(void)
{
	Parameter *param;
	param = (Parameter *)FLASH_ADDRESS_BASE;
	
	uint16_t size = sizeof(Parameter);
	checkSum = 0;
	for(uint8_t i=0;i<(size-2)/2;i++)
		checkSum += ((uint16_t*)param)[i];
	if(checkSum != (*param).checkSum) 
		return false;

	rt_memcpy(this,param,size);
	return true;
}
bool Parameter::flashWrite(void)
{	
	uint16_t size = sizeof(Parameter);
	checkSum = 0;
	for(uint8_t i=0;i<(size-2)/2;i++)
		checkSum += ((uint16_t*)this)[i];
	
	FLASH_Unlock();
	FLASH_EraseSector(FLASH_Sector_5,VoltageRange_3);
	for(uint8_t i=0;i<size/4;i++)
		FLASH_ProgramWord(FLASH_ADDRESS_BASE+i*4,((uint32_t*)this)[i]);
	FLASH_Lock();
	
	return flashRead();
}
