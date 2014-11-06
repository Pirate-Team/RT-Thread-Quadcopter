#ifndef QUATX_H
#define QUATX_H
#include "stm32f4xx.h"

struct sensor_data_t
{
	int16_t ax,ay,az,gx,gy,gz,mx,my,mz;
	float temperature,pressure;
};

void rt_thread_entry_quadx_get_attitude(void* parameter);
void rt_thread_entry_quadx_control_attitude(void* parameter);
extern struct pid_t PID[6];
extern float att[4];
#endif
