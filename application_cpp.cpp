#include "Led.h"
#include "stdio.h"
#include "rtthread.h"
#include "stm32f4xx.h"
#include "MPU6050.h"
#include "I2Cdev.h"
#include "HMC5883L.h"
#include "MS5611.h"
#include "arm_math.h"
#include "quadx.h"
#include "serial.h"
void rt_thread_entry_sensor_test(void* parameter)
{
	I2Cdev::init();
	rt_thread_delay(200);
	
	MPU6050 accelgyro;
	rt_kprintf("\r\nMPU6050 initialize: ");
	rt_kprintf(accelgyro.initialize()?"success\r\n":"failure\r\n");
	HMC5883L mag;
	rt_kprintf("\r\nHMC5883 initialize: ");
	rt_kprintf(mag.initialize()?"success\r\n":"failure\r\n");
	MS5611 baro;
	rt_kprintf("\r\nMS5611 initialize: ");
	rt_kprintf(baro.initialize()?"success\r\n":"failure\r\n");
	
	accelgyro.setOffSet();
	int16_t ax, ay, az;
	int16_t gx, gy, gz, gzCal;
	int16_t mx, my, mz;
	float heading,preHeading,altitude;
	char str[100];
	uint32_t preTick;
	while(1)
	{	
		preTick = rt_tick_get();
		accelgyro.getMotion6Cal(&ax, &ay, &az, &gx, &gy, &gz);
//		sprintf(str,"accel: %+f\t%+f\t%+f\tgyro: %+f\t%+f\t%+f\t%d\r\n",(float)ax/2048,(float)ay/2048,(float)az/2048,(float)gx/16.4f,(float)gy/16.4f,(float)gz/16.4f,rt_tick_get() - preTick);
//		rt_kprintf("%s",str);
		mag.getData(&mx,&my,&mz,&heading);
		gzCal = (gz - (heading - preHeading)*100) / 2;
		preHeading = heading;
		sprintf(str,"%+d\t%+d\t%+f\r\n",gz,gzCal,heading);
		rt_kprintf("%s",str);
//		sprintf(str,"mag: %+d\t%+d\t%+d\theading: %+f\t%d\r\n",mx,my,mz,heading,rt_tick_get() - preTick);
//		rt_kprintf("%s",str);
//		baro.getAltitude(&altitude);
//		sprintf(str,"altitude: %+f\t%d\r\n",altitude,rt_tick_get() - preTick);
//		rt_kprintf("%s",str);
		rt_thread_delay(RT_TICK_PER_SECOND/10);
	}
}

struct rx_msg
{
    rt_device_t dev;
    rt_size_t   size;
};
 
rt_mq_t rx_mq;
static char uart_rx_buffer[64];
 
// ????????
rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    msg.dev = dev;
    msg.size = size;
    rt_mq_send(rx_mq, &msg, sizeof(struct rx_msg));
    return RT_EOK;
}
 
// ??????
void usr_echo_thread_entry(void* parameter)
{
    struct rx_msg msg;
   
    rt_device_t device;
    rt_err_t result = RT_EOK;
   
        // ?RT???????1??
    device = rt_device_find("uart2");
    if (device != RT_NULL)
    {
//		rt_device_init();
                           // ?????????????
        rt_device_set_rx_indicate(device, uart_input);
                           // ?????????
        rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
    }
   
    while(1)
    {
        result = rt_mq_recv(rx_mq, &msg, sizeof(struct rx_msg), 100);
        if (result == -RT_ETIMEOUT)
        {
            // timeout, do nothing
        }
       
        if (result == RT_EOK)
        {
            rt_uint32_t rx_length;
           
            rx_length = (sizeof(uart_rx_buffer) - 1) > msg.size ?
                msg.size : sizeof(uart_rx_buffer) - 1;
           
            rx_length = rt_device_read(msg.dev, 0, &uart_rx_buffer[0], rx_length);
            uart_rx_buffer[rx_length] = '\0';
            rt_device_write(msg.dev, 0, &uart_rx_buffer[0], rx_length);
        }
    }
}

int  rt_application_init(void)
{	
	
	rx_mq = rt_mq_create("mq",sizeof(struct rx_msg),100*sizeof(struct rx_msg),RT_IPC_FLAG_PRIO);
	
	rt_thread_t led_thread;
	led_thread = rt_thread_create("led",
									rt_thread_entry_led_test,
									RT_NULL,
									1024,
									1,
									100);
	if(led_thread != RT_NULL)
		rt_thread_startup(led_thread);
	
	rt_thread_t usart_thread;
	usart_thread = rt_thread_create("usart",
									usr_echo_thread_entry,
									RT_NULL,
									1024,
									1,
									100);
	if(usart_thread != RT_NULL)
		rt_thread_startup(usart_thread);
	
//	rt_thread_t quadx_thread;
//	quadx_thread = rt_thread_create("quadx",
//									rt_thread_entry_quadx,
//									RT_NULL,
//									4096,
//									10,
//									100);
//	if(quadx_thread != RT_NULL)
//		rt_thread_startup(quadx_thread);
	
	
	return 0;
}
