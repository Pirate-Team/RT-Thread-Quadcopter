#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "stm32f4xx.h"
#include "rtthread.h"

#define RX_FRAME_SIZE 16
#define RX_DATA_SIZE 13
#define TX_FRAME_SIZE 32
#define TX_DATA_SIZE 29

#pragma pack(push)
#pragma pack(1)
struct status_data_t
{
	uint8_t type;
	int32_t gps[2];
	int16_t att[4];
	uint16_t motor[4];
	int8_t target[4];
};//

struct gps_data_t
{
	uint8_t type;
	int32_t lng,lat;
};//9byte

struct PID_data_t
{
	uint8_t type;
	uint8_t level[3],heading[3],altitude[3],position[3];
};//13byte

struct ctrl_data_t
{
    uint8_t type;
    uint8_t quadx,trace,send,restart,save,acc,mag;
};//8byte

union rx_data_t
{
	uint8_t type;
	struct gps_data_t gps;
	struct PID_data_t pid;
	struct ctrl_data_t ctrl;
};

struct rx_frame_t
{
	uint8_t AA,BB;
	union rx_data_t data;
	uint8_t checkSum;
};//16byte

union tx_data_t
{
	uint8_t type;
	struct status_data_t status;
};

struct tx_frame_t
{
	uint8_t AA,BB;
	union tx_data_t data;
	uint8_t checkSum;
};//32byte
#pragma pack(pop)

class Communication
{
public:
	Communication(const char *name);
	bool initialize(void);
	
	bool getData(void);
	void sendData(void);

	struct rx_frame_t rxFrame;
	struct tx_frame_t txFrame;
private:
	rt_device_t device;
};
extern rt_mq_t rxQ,txQ;
void rt_thread_entry_communication(void* parameter);
#endif
