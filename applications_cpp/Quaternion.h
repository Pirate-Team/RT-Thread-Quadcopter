#ifndef QUATERNION_H
#define QUATERNION_H
#include "stm32f4xx.h"

class Quaternion
{
public:
	Quaternion(void);
	Quaternion(float _w,float _x,float _y,float _z);
	Quaternion(float pitch,float roll,float yaw);
	~Quaternion(void);
	
	float& operator[](int8_t index);
	void toEuler(float& pitch,float& roll, float& yaw);
	void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);	
	float w,x,y,z;
	float beta;								// 2 * proportional gain (Kp)
	float sampleInterval;
};

extern Quaternion quat;

#endif
