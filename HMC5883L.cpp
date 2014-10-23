#include "HMC5883L.h"
#include "I2Cdev.h"
#include "arm_math.h"
#include "string.h"
#include "rtthread.h"

#define M_57_3 57.29577f
int16_t magXOffset = -162;
int16_t magYOffset = 94;
int16_t magZOffset = -26;
float xGain = 0,yGain = 0,zGain = 0;

HMC5883L::HMC5883L(void)
{
	devAddr = HMC5883L_ADDRESS;
	buffer = (uint8_t*)rt_malloc(6);
	strcpy(name,HMC5883L_NAME);
}

HMC5883L::~HMC5883L(void)
{
	if(buffer != null)
	{
		rt_free(buffer);
		buffer = null;
	}
}

bool HMC5883L::initialize(void)
{
    // write CONFIG_A register
    if(!I2Cdev::writeByte(devAddr, HMC5883L_RA_CONFIG_A,
        (HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
        (HMC5883L_RATE_15     << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
        (HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)))) return false;

    // write CONFIG_B register
	if(!I2Cdev::writeByte(devAddr, HMC5883L_RA_CONFIG_B, HMC5883L_GAIN_440 << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1))) return false;
    
    // write MODE register
    return I2Cdev::writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_CONTINUOUS << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
}

bool HMC5883L::testConnection(void) 
{
	uint8_t buffer[3];
	I2Cdev::readBytes(devAddr, HMC5883L_RA_ID_A, 3, buffer);
	return (buffer[0] == 'H' && buffer[1] == '4' && buffer[2] == '3');
}

uint8_t HMC5883L::getData(void* data1,void* data2,void* data3,void* data4,void* data5,void* data6)
{
	getHeadingCal((int16_t*)data1,(int16_t*)data2,(int16_t*)data3);
	if(data4 != null) getHeadingCal((float*)data4);
	return 4;
}

// DATA* registers

/** Get 3-axis heading measurements.
 * In the event the ADC reading overflows or underflows for the given channel,
 * or if there is a math overflow during the bias measurement, this data
 * register will contain the value -4096. This register value will clear when
 * after the next valid measurement is made. Note that this method automatically
 * clears the appropriate bit in the MODE register if Single mode is active.
 * @param x 16-bit signed integer container for X-axis heading
 * @param y 16-bit signed integer container for Y-axis heading
 * @param z 16-bit signed integer container for Z-axis heading
 * @see HMC5883L_RA_DATAX_H
 */
void HMC5883L::getHeadingRaw(int16_t *x, int16_t *y, int16_t *z) 
{
	I2Cdev::readBytes(devAddr, HMC5883L_RA_DATAX_H, 6, buffer);
//	I2Cdev::writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	*x = (((int16_t)buffer[0]) << 8) | buffer[1];
	*y = (((int16_t)buffer[4]) << 8) | buffer[5];
	*z = (((int16_t)buffer[2]) << 8) | buffer[3];
}

void HMC5883L::getHeadingCal(int16_t *x, int16_t *y, int16_t *z)
{
	static int16_t avgX = 0,avgY = 0,avgZ = 0;
	
	I2Cdev::readBytes(devAddr, HMC5883L_RA_DATAX_H, 6, buffer);
//	I2Cdev::writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	*x = ((((int16_t)buffer[0]) << 8) | buffer[1]) - magXOffset;
	*y = ((((int16_t)buffer[4]) << 8) | buffer[5]) - magYOffset;
	*z = ((((int16_t)buffer[2]) << 8) | buffer[3]) - magZOffset;
	
	if(avgX == 0 && avgY == 0) 
	{
		avgX = *x;
		avgY = *y;
		avgZ = *z;
	}
	avgX = (((int32_t)*x)*3 + (int32_t)avgX*5) >> 3;
	avgY = (((int32_t)*y)*3 + (int32_t)avgY*5) >> 3;
	avgZ = (((int32_t)*z)*3 + (int32_t)avgZ*5) >> 3;
	
	*x = avgX;
	*y = avgY;
	*z = avgZ;
}

void HMC5883L::getHeadingCal(float *heading)
{
	int16_t x,y,z;
	getHeadingCal(&x,&y,&z);
	*heading = atan2((float)y, (float)x);
    if(*heading < 0)
      *heading += 2 * PI;
	*heading = *heading * M_57_3;
}

void HMC5883L::setOffset(void)
{	
	uint32_t tick = rt_tick_get() + 15000; //15s
	int16_t data[3],min[3],max[3];
	for(uint8_t i=0;i<3;i++)
	{
		min[i] = 30000;
		max[i] = -30000;
	}
	while(tick>rt_tick_get())
	{
		getHeadingRaw(&data[0],&data[1],&data[2]);
		for(uint8_t i=0;i<3;i++)
		{
			if(data[i]<min[i]) min[i] = data[i];
			if(data[i]>max[i]) max[i] = data[i];
		}
		rt_thread_delay(10);
	}
	magXOffset = (min[0] + max[0]) / 2;
	magYOffset = (min[1] + max[1]) / 2;
	magZOffset = (min[2] + max[2]) / 2;
}
