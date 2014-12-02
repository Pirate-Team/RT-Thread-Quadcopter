#ifndef __NEMA_DECODE_H
#define __NEMA_DECODE_H

#include "stdlib.h"
#include "string.h"
#include "ctype.h"
#include "rtthread.h"
#include "stm32f4xx.h"

enum GPSDATA_TYPE{
 GPGGA=1,   //全球定位数据 
 GPGSA=2,	//卫星PRN数据  
 GPGSV=3,	//卫星状态信息
 GPRMC=4,	//推荐最小数据 
 GPVTG=5,   //地面速度信息
 GPGLL=6,   //大地坐标信息 
 GPZDA=7	//UTC时间和日期
};

typedef struct UTC{
	int year;
	int mon;
	int day;
	int hour;
	int minute;
	double second;
}UTC_TIME;

typedef struct GPS_GPGGA {
		UTC_TIME	Utc_Time;   		//格林威治时间 
		int 	  	Latitude;			// 纬度值
		char 		Latitude_Directon;  // 北纬或南纬 
		int		  	Longitude; 			//  经度值
		char 		Longitude_Direction;//东经或西经 
		int 		Location_Quality;  	//定位质量  0：无效 1：标准 2：差分 6：估算 
		int 		Satelite_Quantity; 	//使用卫星数
		double 		Level_Accuracy;  	//水平精确度 0.5-99.9 
		double 		Sea_Dials;			//天线离海平面高度
		char 		UnitOfHeight1;		// 高度单位
		double 		Relative_Height;	//大地椭球面相对海平面高度
		char 		UnitOfHeight2;		// 高度单位
		int			RTCM;				// 差分GPS数据期限，最后设立RTCM 传送秒数量 
		int 		BaseStation_Number;	//差分参考基站标号 
		int 		Sum;				//  校验和 
}GPS_GPGGA;

typedef struct GPGSA{
	int l;
}GPS_GPGSA;

typedef struct GPGSV{
	int num;
}GPS_GPGSV;

typedef struct GPRMC{
	int num;
}GPS_GPRMC;

typedef struct GPVTG{
	int num;
}GPS_GPVTG;

typedef struct GPGLL{
	int num;
}GPS_GPGLL;

typedef struct GPZDA{
	int num;
}GPS_GPZDA;


class GPS_GetData{
	public:
		GPS_GPGGA Gpgga;
		rt_device_t uart_device;
	public:
		GPS_GetData(){}
	    GPS_GetData(rt_device_t& Uart_device);
		~GPS_GetData();
		void Get_Gps_GPGGA(void);
		void Get_Gps_GPGLL(void);
		void Get_Gps_GPGSV(void);
		void Get_Gps_GPRMC(void);
		void Get_Gps_GPVTG(void);
		void Get_Gps_GPGSA(void);
		void Get_Gps_GPZDA(void);
		int  Get_DataType(char* data); 
		inline	char Get_next(void);
		int GPS_coord_to_degrees(char* s);
		GPS_GPGGA  Nema_decode_gpgga();
		void Get_Coor(int32_t* lng,int32_t* lat);
		void Get_Altitude(int32_t &alt0);
		void Get_Speed(int32_t &speed);
		void Get_GPS_Direction(char &lng_dir,char &lat_dir); //得到N/S 和 E/W
};

void rt_thread_entry_getgpsdata(void* parameter);
#endif
