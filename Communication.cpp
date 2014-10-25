#include "Communication.h"

enum GET_DATA_STATE
{
    NEED_AA = 0,
    NEED_BB,
    NEED_TYPE,
    NEED_DATA
};

rt_mq_t rxQ,txQ;

//uint8_t Communication::availableData;

Communication::Communication(const char *name)
{
//	availableData = 0;
	device = rt_device_find(name);
	if (device != RT_NULL)
	{
//		rt_device_set_rx_indicate(device,usartInput);
		rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
	}
}

Communication::~Communication(void)
{

}
	
//rt_err_t Communication::usartInput(rt_device_t dev, rt_size_t size)
//{
//	availableData = size;
//	return RT_EOK;
//}

bool Communication::getData(void)
{
	static GET_DATA_STATE state = NEED_AA;
	static uint8_t length;
	static uint8_t byte;
	if(state == NEED_AA)
	{
		if(rt_device_read(device, 0, &byte, 1) != 1) return false;
		if(byte != 0xaa) return false;
		state = NEED_BB;
	}
	if(state == NEED_BB)
	{
		if(rt_device_read(device, 0, &byte, 1) != 1) return false;
		if(byte != 0xbb) 
		{
			state = NEED_AA;
			return false;
		}
		length = 0;
		state = NEED_DATA;
	}
//	if(state == NEED_TYPE)
//	{
//		if(rt_device_read(device, 0, &byte, 1) != 1) return false;
//		if(!(byte>=0xca && byte<=0xdf))
//		{
//			state = NEED_AA;
//			return false;
//		}
//		length = 0;
//		rxData[0] = byte;
//		state = NEED_DATA;
//	}
	if(state == NEED_DATA)
	{
		length += rt_device_read(device, 0, rxData + length, RX_DATA_SIZE + 1 -length);
		if(length != RX_DATA_SIZE + 1) return false;
		uint8_t checkSum = 0;
		for(uint8_t i=0;i<RX_DATA_SIZE;i++)
			checkSum += rxData[i];
		if(checkSum != rxData[RX_DATA_SIZE])
		{
			state = NEED_AA;
			return false;
		}
		state = NEED_AA;
		return true;
	}
	state = NEED_AA;
	return false;
}

void Communication::sendData(void)
{
	static uint8_t buf[2];
	buf[0] = 0xaa;
	buf[1] = 0xbb;
	rt_device_write(device,0,buf,2);
	rt_device_write(device,0,txData,TX_DATA_SIZE);
	buf[0] = 0;
//	buf[1] = '\r';
	for(uint8_t i=0;i<TX_DATA_SIZE;i++)
		buf[0] += txData[i];
	rt_device_write(device,0,buf,1);
}

void rt_thread_entry_communication(void* parameter)
{
	rxQ = rt_mq_create("rx",RX_DATA_SIZE,5,RT_IPC_FLAG_PRIO);
	txQ = rt_mq_create("tx",TX_DATA_SIZE,5,RT_IPC_FLAG_PRIO);
	Communication com("uart2");
	while(1)
	{
		if(com.getData())
		{
			rt_mq_send(rxQ,com.rxData,RX_DATA_SIZE);
		}
		if(rt_mq_recv(txQ,com.txData,TX_DATA_SIZE,0) == RT_EOK)
		{
			com.sendData();
		}
		rt_thread_delay(10);
	}
}
