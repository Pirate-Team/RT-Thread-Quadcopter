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
	
	bool getData(void);
	void sendData(void);

	rt_device_t device;
	uint8_t rxData[RX_DATA_SIZE + 1],txData[TX_DATA_SIZE + 1];
};
extern rt_mq_t rxQ,txQ;
void rt_thread_entry_communication(void* parameter);
#endif
