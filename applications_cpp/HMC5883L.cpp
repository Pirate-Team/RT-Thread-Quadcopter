#include "HMC5883L.h"
#include "head.h"
#include "I2Cdev.h"
#include "arm_math.h"
#include "string.h"
#include "rtthread.h"
#include "Parameter.h"
#define M_57_3 57.29577f
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16)                       //!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16)   //!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08)                       //!< Y axis level when bias current is applied.

HMC5883L mag;

HMC5883L::HMC5883L(void)
{
	devAddr = HMC5883L_ADDRESS;
	buffer = new uint8_t[6];
	strcpy(name,HMC5883L_NAME);
	magGain[0] = magGain[1] = magGain[2] = 1;
}

HMC5883L::~HMC5883L(void)
{
	if(buffer != null)
	{
		delete(buffer);
		buffer = null;
	}
}

bool HMC5883L::initialize(void)
{
//    // write CONFIG_A register
//    if(!I2Cdev::writeByte(devAddr, HMC5883L_RA_CONFIG_A,
//        (HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
//        (HMC5883L_RATE_30     << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
//        (HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)))) return false;

//    // write CONFIG_B register
//	if(!I2Cdev::writeByte(devAddr, HMC5883L_RA_CONFIG_B, HMC5883L_GAIN_440 << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1))) return false;
//    
//    // write MODE register
//    return I2Cdev::writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_CONTINUOUS << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	
	bool bret=true;                // Error indicator
	int32_t xyz_total[3]={0,0,0};  // 32 bit totals so they won't overflow.
	int16_t xyz[3] = {0,0,0};
/*************calc gain*****************/	
	if(!I2Cdev::writeByte(devAddr, HMC5883L_RA_CONFIG_A,
        (HMC5883L_AVERAGING_1 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
        (HMC5883L_RATE_15     << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
        (HMC5883L_BIAS_POSITIVE << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)))) return false;
	if(!I2Cdev::writeByte(devAddr, HMC5883L_RA_CONFIG_B, HMC5883L_GAIN_660 << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1))) return false;
	if(!I2Cdev::writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1))) return false;
	DELAY_MS(100);
	getHeadingRaw(&xyz[0],&xyz[1],&xyz[2]);
	
	//POS
	for (uint8_t i=0; i<10; i++)  //Collect 10 samples
	{
		I2Cdev::writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
		DELAY_MS(100);
		getHeadingRaw(&xyz[0],&xyz[1],&xyz[2]);   // Get the raw values in case the scales have already been changed.
					
		// Since the measurements are noisy, they should be averaged rather than taking the max.
		xyz_total[0]+=xyz[0];
		xyz_total[1]+=xyz[1];
		xyz_total[2]+=xyz[2];
					
		// Detect saturation.
		if (-(1<<12) >= MIN(xyz[0],MIN(xyz[1],xyz[2])))
		{
			bret=false;
			break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
		}
	}
	//NAG
	if(!I2Cdev::writeByte(devAddr, HMC5883L_RA_CONFIG_A,
        (HMC5883L_AVERAGING_1 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
        (HMC5883L_RATE_15     << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
        (HMC5883L_BIAS_NEGATIVE << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)))) return false;
	DELAY_MS(100);
	getHeadingRaw(&xyz[0],&xyz[1],&xyz[2]);
	
	for (uint8_t i=0; i<10; i++)  //Collect 10 samples
	{
		I2Cdev::writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
		DELAY_MS(100);
		getHeadingRaw(&xyz[0],&xyz[1],&xyz[2]);   // Get the raw values in case the scales have already been changed.
					
		// Since the measurements are noisy, they should be averaged rather than taking the max.
		xyz_total[0]-=xyz[0];
		xyz_total[1]-=xyz[1];
		xyz_total[2]-=xyz[2];
					
		// Detect saturation.
		if (-(1<<12) >= MIN(xyz[0],MIN(xyz[1],xyz[2]))) 
		{
			bret=false;
			break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
		}
	}
	
	magGain[0]=fabs(660.0*HMC58X3_X_SELF_TEST_GAUSS*2.0*10.0/xyz_total[0]);
	magGain[1]=fabs(660.0*HMC58X3_Y_SELF_TEST_GAUSS*2.0*10.0/xyz_total[1]);
	magGain[2]=fabs(660.0*HMC58X3_Z_SELF_TEST_GAUSS*2.0*10.0/xyz_total[2]);
	
	if (!bret)  //Something went wrong so get a best guess
	{
		magGain[0] = 1.0;
		magGain[1] = 1.0;
		magGain[2] = 1.0;
	}
/*************calc gain end*****************/	
	if(!I2Cdev::writeByte(devAddr, HMC5883L_RA_CONFIG_A,
        (HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
        (HMC5883L_RATE_30     << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
        (HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)))) return false;
	if(!I2Cdev::writeByte(devAddr, HMC5883L_RA_CONFIG_B, HMC5883L_GAIN_660 << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1))) return false;
	if(!I2Cdev::writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_CONTINUOUS << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1))) return false;
	return true;
}

bool HMC5883L::testConnection(void) 
{
	uint8_t buffer[3];
	I2Cdev::readBytes(devAddr, HMC5883L_RA_ID_A, 3, buffer);
	return (buffer[0] == 'H' && buffer[1] == '4' && buffer[2] == '3');
}

uint8_t HMC5883L::getData(void* data1,void* data2,void* data3,void* data4,void* data5,void* data6)
{
//	getHeadingCal((int16_t*)data1,(int16_t*)data2,(int16_t*)data3);
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
	*x = ((((int16_t)buffer[0]) << 8) | buffer[1]);
	*y = ((((int16_t)buffer[4]) << 8) | buffer[5]);
	*z = ((((int16_t)buffer[2]) << 8) | buffer[3]);
	
	*x *= magGain[0];
	*y *= magGain[1];
	*z *= magGain[2];
}

//void HMC5883L::getHeadingCal(int16_t *x, int16_t *y, int16_t *z)
//{
//	static int16_t avgX = 0,avgY = 0,avgZ = 0;
//	
//	I2Cdev::readBytes(devAddr, HMC5883L_RA_DATAX_H, 6, buffer);
////	I2Cdev::writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
//	*x = ((((int16_t)buffer[0]) << 8) | buffer[1]) - param.magXOffset;
//	*y = ((((int16_t)buffer[4]) << 8) | buffer[5]) - param.magYOffset;
//	*z = ((((int16_t)buffer[2]) << 8) | buffer[3]) - param.magZOffset;
//	
//	if(avgX == 0 && avgY == 0) 
//	{
//		avgX = *x;
//		avgY = *y;
//		avgZ = *z;
//	}
//	avgX = (((int32_t)*x)*3 + (int32_t)avgX*5) >> 3;
//	avgY = (((int32_t)*y)*3 + (int32_t)avgY*5) >> 3;
//	avgZ = (((int32_t)*z)*3 + (int32_t)avgZ*5) >> 3;
//		
//	*x = avgX;
//	*y = avgY;
//	*z = avgZ;
//	
////	#define MAG_TAB_SIZE 5
////	static int16_t magXHistTab[MAG_TAB_SIZE] = {0},magYHistTab[MAG_TAB_SIZE] = {0},magZHistTab[MAG_TAB_SIZE] = {0};
////	static int32_t magXSum = 0,magYSum = 0,magZSum = 0;
////	static uint8_t magHistIdx = 0;
////	uint8_t indexplus1 = (magHistIdx + 1);
////	if (indexplus1 == MAG_TAB_SIZE) indexplus1 = 0;

////	magXHistTab[magHistIdx] = *x;
////	magXSum += magXHistTab[magHistIdx];
////	magXSum -= magXHistTab[indexplus1];

////	magYHistTab[magHistIdx] = *y;
////	magYSum += magYHistTab[magHistIdx];
////	magYSum -= magYHistTab[indexplus1];

////	magZHistTab[magHistIdx] = *z;
////	magZSum += magZHistTab[magHistIdx];
////	magZSum -= magZHistTab[indexplus1];

////	magHistIdx = indexplus1;

////	*x = magXSum >> 2;
////	*y = magYSum >> 2;
////	*z = magZSum >> 2;
//}

void HMC5883L::getHeadingCal(int16_t &x, int16_t &y, int16_t &z)
{
	if(I2Cdev::readBytes(devAddr, HMC5883L_RA_DATAX_H, 6, buffer))
	{
		int16_t xt = ((((int16_t)buffer[0]) << 8) | buffer[1]);
		int16_t yt = ((((int16_t)buffer[4]) << 8) | buffer[5]);
		int16_t zt = ((((int16_t)buffer[2]) << 8) | buffer[3]);
		
		float yGain = param.magYGain / 10000.0f;
		float zGain = param.magZGain / 10000.0f;

		xt = xt * magGain[0] * 1.0f  - param.magXOffset;
		yt = yt * magGain[1] * yGain - param.magYOffset;
		zt = zt * magGain[2] * zGain - param.magZOffset;
		
		x = (((int32_t)x)*3 + (int32_t)xt*5) >> 3;
		y = (((int32_t)y)*3 + (int32_t)yt*5) >> 3;
		z = (((int32_t)z)*3 + (int32_t)zt*5) >> 3;
	}
	else
	{
		x = y = z = 0;
	}
}

void HMC5883L::getHeadingCal(float *heading)
{
	int16_t x,y,z;
	getHeadingCal(x,y,z);
	*heading = atan2((float)y, (float)x);
    if(*heading < 0)
      *heading += 2 * PI;
	*heading = *heading * M_57_3;
}

void HMC5883L::setOffset(void)
{
#define CALI_TIME_S 30
	uint32_t tick = rt_tick_get() + CALI_TIME_S * RT_TICK_PER_SECOND;
	int16_t data[3],min[3],max[3];
	DELAY_MS(50);
	I2Cdev::writeByte(devAddr, HMC5883L_RA_CONFIG_A,
        (HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
        (HMC5883L_RATE_75     << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
        (HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)));
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
		DELAY_MS(16);
	}
	param.magXOffset = (min[0] + max[0]) >> 1;
	param.magYOffset = (min[1] + max[1]) >> 1;
	param.magZOffset = (min[2] + max[2]) >> 1;
	param.magYGain = ((max[0] - min[0])*10000)/(max[1] - min[1]);
	param.magZGain = ((max[0] - min[0])*10000)/(max[2] - min[2]);
	
	DELAY_MS(50);
	I2Cdev::writeByte(devAddr, HMC5883L_RA_CONFIG_A,
        (HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
        (HMC5883L_RATE_30     << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
        (HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)));
}
