#include "quaternion.h"
#include "math.h"
Quaternion::Quaternion(void)
{
	w=1; x=0; y=0; z=0;
}
Quaternion::Quaternion(float _w,float _x,float _y,float _z)
{
	w=_w; x=_x; y=_y; z=_z;
}
Quaternion::Quaternion(float pitch,float roll,float yaw)
{
	float   sinpitch_2 = (float)sin(pitch / 2),
			cospitch_2 = (float)cos(pitch / 2),
			sinroll_2 = (float)sin(roll / 2),
			cosroll_2 = (float)cos(roll / 2),
			sinyaw_2 = (float)sin(yaw / 2),
			cosyaw_2 = (float)cos(yaw / 2);

    //pitch->y roll->x yaw->z
    w = cosroll_2*cospitch_2*cosyaw_2 + sinroll_2*sinpitch_2*sinyaw_2;
    x = sinroll_2*cospitch_2*cosyaw_2 - cosroll_2*sinpitch_2*sinyaw_2;
    y = cosroll_2*sinpitch_2*cosyaw_2 + sinroll_2*cospitch_2*sinyaw_2;
    z = cosroll_2*cospitch_2*sinyaw_2 - sinroll_2*sinpitch_2*cosyaw_2;
}
Quaternion::~Quaternion(void)
{
}

float& Quaternion::operator[](int8_t index)
{
	switch(index)
	{
		case 0:
			return w;
		case 1:
			return x;
		case 2:
			return y;
		case 3:
			return z;
		default:
			return w;
	}
}
void Quaternion::toEuler(float& pitch,float& roll, float& yaw)
{
	pitch = (float)asin(2 * w*y - 2 * z*x);
    roll = (float)atan2(2 * w*x + 2 * y*z, 1 - 2 * x*x - 2 * y*y);
    yaw = (float)atan2(2 * w*z + 2 * x*y, 1 - 2 * y*y - 2 * z*z);
}
