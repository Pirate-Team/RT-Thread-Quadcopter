#include "Communication.h"
#include "head.h"

#define ADDR "192.168.4.255"//"192.168.31.135"
#define PORT 45454

enum GET_DATA_STATE
{
    NEED_AA = 0,
    NEED_BB,
    NEED_DATA
};

rt_mq_t rxQ,txQ;

Communication::Communication(const char *name)
{
	device = rt_device_find(name);
	if (device != RT_NULL)
		rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
	rt_memset(&txFrame,0,TX_FRAME_SIZE);
	rt_memset(&rxFrame,0,RX_FRAME_SIZE);
	txFrame.AA = 0xaa;
	txFrame.BB = 0xbb;
}

bool Communication::initialize(void)
{
	DELAY_MS(1000);
	rt_kprintf("ATE0\n");
	DELAY_MS(100);
	rt_kprintf("AT+CIPMUX=1\n");
	DELAY_MS(100);
	rt_kprintf("AT+CIPSERVER=1,2333\n");
	DELAY_MS(100);
	rt_kprintf("AT+CIPSTART=0\"UDP\",\"%s\",%d\n",ADDR,PORT);
	DELAY_MS(100);
//	rt_kprintf("AT+CIPSEND=0,2\nOK");
	return true;
}

bool Communication::getData(void)
{
	static GET_DATA_STATE state = NEED_AA;
	static uint8_t length;
	static uint8_t byte;
	while(1)
	{
		if(state == NEED_AA)
		{
			if(rt_device_read(device, 0, &byte, 1) != 1) return false;
			if(byte != 0xaa) continue;
			state = NEED_BB;
		}
		if(state == NEED_BB)
		{
			if(rt_device_read(device, 0, &byte, 1) != 1) return false;
			if(byte != 0xbb) 
			{
				state = NEED_AA;
				continue;
			}
			length = 0;
			state = NEED_DATA;
		}
		if(state == NEED_DATA)
		{
			length += rt_device_read(device, 0, ((uint8_t*)&(rxFrame.data)) + length, RX_FRAME_SIZE - 2 - length);
			if(length != RX_FRAME_SIZE - 2) return false;
			uint8_t checkSum = 0;
			for(uint8_t i=0;i<RX_DATA_SIZE;i++)
				checkSum += ((uint8_t*)&(rxFrame.data))[i];
			if(checkSum != rxFrame.checkSum)
			{
				state = NEED_AA;
				continue;
			}
			state = NEED_AA;
			return true;
		}
		state = NEED_AA;
		return false;
	}
}

void Communication::sendData(void)
{
	rt_kprintf("AT+CIPSEND=0,%d\n",TX_FRAME_SIZE);
	DELAY_MS(30);
	txFrame.checkSum = 0;
	for(uint8_t i=0;i<TX_DATA_SIZE;i++)
		txFrame.checkSum += ((uint8_t*)&(txFrame.data))[i];
	rt_device_write(device,0,&txFrame,TX_FRAME_SIZE);
}

void rt_thread_entry_communication(void* parameter)
{
	rxQ = rt_mq_create("rx",RX_DATA_SIZE,3,RT_IPC_FLAG_PRIO);
	txQ = rt_mq_create("tx",TX_DATA_SIZE,3,RT_IPC_FLAG_PRIO);
	Communication com("uart2");
	com.initialize();
	while(1)
	{
		if(com.getData())
		{
			rt_mq_send(rxQ,&(com.rxFrame.data),RX_DATA_SIZE);
		}
		if(rt_mq_recv(txQ,&(com.txFrame.data),TX_DATA_SIZE,0) == RT_EOK)
		{
			com.sendData();
		}
		DELAY_MS(50);
	}
}
