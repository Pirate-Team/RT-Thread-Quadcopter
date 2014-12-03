#ifndef HEAD_H

#include "stm32f4xx.h"
#include "rtthread.h"

#define MS_PER_TICK (1000/RT_TICK_PER_SECOND)
#define DELAY_MS(x) rt_thread_delay((x)/MS_PER_TICK)
#define BETWEEN(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define DEAD_BAND(value,mid,ban) (((value)<(mid)-(ban))?((value)+(ban)):(((value)>(mid)+(ban))?((value)-(ban)):(mid)))
#define MIN(x,y) ((x)<(y)?(x):(y))
#define MAX(x,y) ((x)>(y)?(x):(y))

#endif
