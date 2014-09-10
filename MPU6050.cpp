#include "MPU6050.h"
#include "I2Cdev.h"

MPU6050::MPU6050()
{
	devAddr = (uint8_t)MPU6050_DEFAULT_ADDRESS;
	buffer = (uint8_t*)rt_malloc(15);
	accXZero = accYZero = accZZero = gyroXZero = gyroYZero = gyroZZero = 0;
}

MPU6050::~MPU6050()
{
	if(buffer != RT_NULL)
		rt_free(buffer);
	buffer = RT_NULL;
}

bool MPU6050::initialize(void)
{
	I2Cdev::writeByte(devAddr, MPU6050_RA_PWR_MGMT_1, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
	
	I2Cdev::writeByte(devAddr, MPU6050_RA_PWR_MGMT_1, 0x01);
	I2Cdev::writeByte(devAddr, MPU6050_RA_GYRO_CONFIG, 0x18);
    I2Cdev::writeByte(devAddr, MPU6050_RA_ACCEL_CONFIG, 0x18);
	I2Cdev::writeByte(devAddr, MPU6050_RA_USER_CTRL, 0x00);
	I2Cdev::writeByte(devAddr, MPU6050_RA_INT_PIN_CFG, 0xB2);	
	
/*
	
	I2Cdev::writeByte(devAddr, MPU6050_RA_PWR_MGMT_1, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
	rt_thread_delay(5);
	I2Cdev::writeByte(devAddr, MPU6050_RA_PWR_MGMT_1, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
	I2Cdev::writeByte(devAddr, MPU6050_RA_CONFIG, 0x00); 				//CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
	I2Cdev::writeByte(devAddr, MPU6050_RA_GYRO_CONFIG, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec  0x18
	
	// enable I2C bypass for AUX I2C
	I2Cdev::writeByte(devAddr, MPU6050_RA_INT_PIN_CFG, 0x02);           	//INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
	
	I2Cdev::writeByte(devAddr, MPU6050_RA_ACCEL_CONFIG, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec. 0x10
	//note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
	//confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480

*/
/*-------------------------------------------------
	mag uses bypass
--------------------------------------------------*/
	//at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
	//now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
//	I2Cdev::writeByte(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
//	I2Cdev::writeByte(MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 0x00);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
//	I2Cdev::writeByte(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_CTRL, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
//	I2Cdev::writeByte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_ADDR, 0x80|MAG_ADDRESS);//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
//	I2Cdev::writeByte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_REG, MAG_DATA_REGISTER);//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
//	I2Cdev::writeByte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_CTRL, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)
	return true;
}

bool MPU6050::testConnection(void)
{
	uint8_t who_am_i;
	I2Cdev::readByte(devAddr,MPU6050_RA_WHO_AM_I,&who_am_i);
	return who_am_i == MPU6050_DEFAULT_ADDRESS;
}
/*-------------------------------------------------
	ACCEL_*OUT_* registers
--------------------------------------------------*/
//void MPU6050::getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz)
//{
//    getMotion6(ax, ay, az, gx, gy, gz);
//    // TODO: magnetometer integration
//}

void MPU6050::getMotion6Cal(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
    I2Cdev::readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
    *ax = ((((int16_t)buffer[0]) << 8) | buffer[1]) - accXZero;
    *ay = ((((int16_t)buffer[2]) << 8) | buffer[3]) - accYZero;
    *az = ((((int16_t)buffer[4]) << 8) | buffer[5]) - accZZero;
    *gx = ((((int16_t)buffer[8]) << 8) | buffer[9]) - gyroXZero;
    *gy = ((((int16_t)buffer[10]) << 8) | buffer[11]) - gyroYZero;
    *gz = ((((int16_t)buffer[12]) << 8) | buffer[13]) - gyroZZero;	
}

void MPU6050::getAccelerationRaw(int16_t* x, int16_t* y, int16_t* z)
{
    I2Cdev::readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}

void MPU6050::getAccelerationCal(int16_t* x, int16_t* y, int16_t* z)
{
    I2Cdev::readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}

//int16_t MPU6050::getAccelerationX()
//{

//}
//int16_t MPU6050::getAccelerationY()
//{

//}
//int16_t MPU6050::getAccelerationZ()
//{

//}

/*-------------------------------------------------
	TEMPERATURE_*OUT_* registers
--------------------------------------------------*/
int16_t MPU6050::getTemperature()
{
    I2Cdev::readBytes(devAddr, MPU6050_RA_TEMP_OUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/*-------------------------------------------------
	GYRO_*OUT_* registers
--------------------------------------------------*/
void MPU6050::getRotationRaw(int16_t* x, int16_t* y, int16_t* z)
{
    I2Cdev::readBytes(devAddr, MPU6050_RA_GYRO_XOUT_H, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}

void MPU6050::getRotationCal(int16_t* x, int16_t* y, int16_t* z)
{
    I2Cdev::readBytes(devAddr, MPU6050_RA_GYRO_XOUT_H, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}

//int16_t MPU6050::getRotationX();
//int16_t MPU6050::getRotationY();
//int16_t MPU6050::getRotationZ();


void MPU6050::calibrate(void)
{
	int16_t ax, ay, az, axa = 0, aya = 0, aza = 0;
	int16_t gx, gy, gz, gxa = 0, gya = 0, gza = 0;
	for(uint16_t i=0;i<512;i++)
	{	
		getAccelerationRaw(&ax,&ay,&az);
		getRotationRaw(&gx,&gy,&gz);
		axa = (axa + ax) / 2; aya = (aya + ay) / 2; aza = (aza + az) / 2;
		gxa = (gxa + gx) / 2; gya = (gya + gy) / 2; gza = (gza + gz) / 2;
		
		accXZero = (accXZero + axa) >> 1;
		accYZero = (accYZero + aya) >> 1;
		accZZero = (accZZero + aza) >> 1;
		
		gyroXZero = (gyroXZero + gxa) >> 1;
		gyroYZero = (gyroYZero + gya) >> 1;
		gyroZZero = (gyroZZero + gza) >> 1;
		
		rt_thread_delay(1);
	}
	accZZero -= 2048;
	rt_kprintf("accelZero: %d, %d,%d; gyroZero: %d, %d,%d\r\n",accXZero,accYZero,accZZero,gyroXZero,gyroYZero,gyroZZero);
	rt_kprintf("accelraw: %d, %d,%d; gyroraw: %d, %d,%d\r\n",axa,aya,aza,gxa,gya,gza);
	rt_thread_delay(RT_TICK_PER_SECOND * 2);
}
