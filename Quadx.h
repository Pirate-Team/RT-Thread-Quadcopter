#ifndef QUATX_H
#define QUATX_H
#include "stm32f4xx.h"

void rt_thread_quadx_get_sensor_data(void* parameter);
void rt_thread_quadx_send_data(void* parameter);
void rt_thread_quadx_AHRS(void* parameter);
void rt_thread_quadx_PID(void* parameter);
void rt_thread_entry_quadx(void* parameter);

#endif
