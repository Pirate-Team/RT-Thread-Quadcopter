#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "stm32f4xx.h"
#include "rtthread.h"

#define TX_DATA_SIZE 9
#define RX_DATA_SIZE 4

class Communication
{
public:
	Communication(const char *name);
	~Communication(void);
	
//	static rt_err_t usartInput(rt_device_t dev, rt_size_t size);
	
	bool getData(void);
	void sendData(void);

//	static uint8_t availableData;
	rt_device_t device;
	uint8_t rxData[RX_DATA_SIZE + 1],txData[TX_DATA_SIZE + 1];
};
extern rt_mq_t rxQ,txQ;
void rt_thread_entry_communication(void* parameter);
#endif
