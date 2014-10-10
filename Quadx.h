#ifndef QUATX_H
#define QUATX_H
#include "stm32f4xx.h"
struct pid_t
{
	float P,I,D,result;
};

struct sensor_data_t
{
	int16_t ax,ay,az,gx,gy,gz;
	float heading,altitude;
};

void rt_thread_entry_quadx_get_attitude(void* parameter);
void rt_thread_entry_quadx_control_attitude(void* parameter);
extern struct pid_t PID[4];
extern float att[4];
#endif
