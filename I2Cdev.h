#ifndef I2CDEV_H
#define I2CDEV_H
#include "stm32f4xx.h"

class I2Cdev
{
public:
	static void init(void);
	
	static bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data);
	static bool readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
	
	static bool readByte(uint8_t devAddr,uint8_t regAddr,uint8_t *data);
	static bool writeByte(uint8_t devAddr,uint8_t regAddr,uint8_t data);
	
	static bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
private:
	static void I2C_delay(void);
	static bool start(void);
	static void stop(void);
	static void ack(void);
	static void noAck(void);
	static bool waitAck(void);
	static void sendByte(uint8_t byte);
	static uint8_t receiveByte(void);
};


#endif
