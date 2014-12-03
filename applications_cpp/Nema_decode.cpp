#include "Nema_decode.h"
#include "rtthread.h"

void rt_thread_entry_getgpsdata(void* parameter){
	    int32_t lat,log;  //经纬度数据
		rt_device_t uart1_device = rt_device_find("uart1");
	
		if(uart1_device!=RT_NULL){
			rt_device_open(uart1_device,RT_DEVICE_OFLAG_RDWR);
			rt_kprintf("usart1 suu\n");
		}
		else
		   rt_kprintf("uart fail\n");
		
		GPS_GetData  gps_data(uart1_device); //初始化时传入设备引用
		while(true){
			//此函数用来解析GPS发过来的数据，参数1是经度，参数2是纬度
			gps_data.Get_Coor(&log,&lat);
			rt_kprintf("the Longitude:%d  the latitude:%d\n",log,lat);
			rt_thread_delay(1000);
		}
}


GPS_GetData::GPS_GetData(rt_device_t& Uart_device){
			memset(&Gpgga,0,sizeof(GPS_GPGGA));
			uart_device=Uart_device;
}

GPS_GetData::~GPS_GetData(){}
	

GPS_GPGGA  GPS_GetData::Nema_decode_gpgga()
{
	
	char str[10];
	char cc=0;
	
	if(uart_device==NULL){
		memset(&Gpgga,0,sizeof(GPS_GPGGA));
		return Gpgga;
	}
	
	while((cc=Get_next())!=0)
	{
		switch(cc)
		{
		case '$':
			if(((cc=Get_next())=='G')&&((cc=Get_next())=='P'))
			{   
				memset(str,0,sizeof(str));
				while((cc=Get_next())!=','){
					strncat(str,&cc,1);
				}
				switch(Get_DataType(str)){
				case GPGGA:
					Get_Gps_GPGGA();
					return Gpgga;
				case GPGSA:
					Get_Gps_GPGSA();
					break;
				case GPGSV:
					Get_Gps_GPGSV();
					break;	  
				case GPRMC:
					Get_Gps_GPRMC();
					break;
				case GPVTG:
					Get_Gps_GPVTG();
					break;
				case GPGLL:
					Get_Gps_GPGLL();
					break;
				case GPZDA:
					Get_Gps_GPZDA();
					break;
				default:
					break;
				}
			}

			else{
				break;	
			}
		default:
			continue;
		}
	}
	return Gpgga;
}

inline char GPS_GetData::Get_next()
{  
	char ch;
	while(0 == rt_device_read(uart_device,0,&ch,1))
		rt_thread_delay(10);
	return ch;
}

void GPS_GetData::Get_Gps_GPGSV(void)
{

}
void GPS_GetData::Get_Gps_GPGLL(void)
{

}
void GPS_GetData::Get_Gps_GPRMC(void)
{

}
void GPS_GetData::Get_Gps_GPVTG(void)
{

}

void GPS_GetData::Get_Gps_GPGSA(void)
{

}
void GPS_GetData::Get_Gps_GPZDA(void){
	

}

void GPS_GetData::Get_Coor(int32_t* lng,int32_t* lat){
	Nema_decode_gpgga();
	*lng=Gpgga.Longitude;
	*lat=Gpgga.Latitude;	
}

void GPS_GetData::Get_Gps_GPGGA(void){
	char temp[15];
	char ch;
	unsigned int i=0;

	memset(&Gpgga,0,sizeof(Gpgga));

	//读时间 hhmmss.sss,
	memset(temp,0,sizeof(temp));
	ch=Get_next();
	if(ch!=','){
		strncat(temp,&ch,1);
		ch=Get_next();
		strncat(temp,&ch,1);
		Gpgga.Utc_Time.hour=atoi(temp);	

		memset(temp,0,sizeof(temp));
		for(i=0;i<2;i++){
			ch=Get_next();
			strncat(temp,&ch,1);
		}
		Gpgga.Utc_Time.minute=atoi(temp);

		memset(temp,0,sizeof(temp));
		for(i=0;i<5;i++){
			ch=Get_next();
			strncat(temp,&ch,1);
		}
		Gpgga.Utc_Time.second=atof(temp);
		ch=Get_next();
	}
	// 读纬度 :ddmm.mmmm,
	memset(temp,0,sizeof(temp));
	while((ch=Get_next())!=','){
		strncat(temp,&ch,1);
	}
	Gpgga.Latitude=GPS_coord_to_degrees(temp);
	
	
	//读纬度半球  N或S
	memset(temp,0,sizeof(temp));
	ch=Get_next();
	if(ch!=','){
		Gpgga.Latitude_Directon=ch;
		ch=Get_next();
	}

	//读经度:dddmm.mmmm,
	memset(temp,0,sizeof(temp));
	while((ch=Get_next())!=','){
		strncat(temp,&ch,1);
	}
	
	Gpgga.Longitude=GPS_coord_to_degrees(temp);
	
	//读经度半球 E?W
	memset(temp,0,sizeof(temp));
	ch=Get_next();
	if(ch!=','){
		Gpgga.Longitude_Direction=ch;
		ch=Get_next();
	}

	//读定位质量
	memset(temp,0,sizeof(temp));
	while((ch=Get_next())!=','){
		strncat(temp,&ch,1);
	 }
	Gpgga.Location_Quality=atoi(temp);
	


	//读指定卫星数
	memset(temp,0,sizeof(temp));
	while((ch=Get_next())!=','){
		strncat(temp,&ch,1);
	}
	Gpgga.Satelite_Quantity=atoi(temp);
		
	//读水平精确度
	memset(temp,0,sizeof(temp));
	while((ch=Get_next())!=','){
		strncat(temp,&ch,1);
	}
	Gpgga.Level_Accuracy=atof(temp);

	// 天线离海平面 高度
	memset(temp,0,sizeof(temp));
	while((ch=Get_next())!=','){
		strncat(temp,&ch,1);
	}
	Gpgga.Sea_Dials=atof(temp);
	//高度单位1
	memset(temp,0,sizeof(temp));
	while((ch=Get_next())!=','){
		strncat(temp,&ch,1);
	}
	Gpgga.UnitOfHeight1='M';      //注意改
	//大地 椭球面离相对海平面高度
	memset(temp,0,sizeof(temp));
	while((ch=Get_next())!=','){
		strncat(temp,&ch,1);
	}
	Gpgga.Relative_Height=atof(temp);

	//高度单位2
	memset(temp,0,sizeof(temp));
	while((ch=Get_next())!=','){
		strncat(temp,&ch,1);
	}
	Gpgga.UnitOfHeight2='M';    //注意改
	// 差分GPS数据期限
	memset(temp,0,sizeof(temp));
	while((ch=Get_next())!=','){
		strncat(temp,&ch,1);
	}
	Gpgga.RTCM=atoi(temp);
	//差分参考基站标号
	memset(temp,0,sizeof(temp));
	while((ch=Get_next())!='*'){
		strncat(temp,&ch,1);
	}
	Gpgga.BaseStation_Number=atoi(temp);
	//校验和
	memset(temp,0,sizeof(temp));
	while((ch=Get_next())!='\n'){
		strncat(temp,&ch,1);
	}
	Gpgga.Sum=atoi(temp);
	//rt_kprintf("c:%d\n",Gpgga.Longitude);
	return;
}

int  GPS_GetData::Get_DataType(char* data){
	char GGA[4]="GGA";
	char GSA[4]="GSA";
	char GSV[4]="GSV";
	char RMC[4]="RMC";
	char VTG[4]="VTG";
	char GLL[4]="GLL";
	char ZDA[4]="ZDA";
	if((strncmp(data,GGA,3))==0){
		return GPGGA;
	}
	else if((strncmp(data,GSA,3))==0){
		return GPGSA;
	}
	else if(strncmp(data,GSV,3)==0){
		return GPGSV;
	}
	else if(strncmp(data,RMC,3)==0){
		return GPRMC;
	}
	else if(strncmp(data,VTG,3)==0){
		return GPVTG;
	}
	else if(strncmp(data,GLL,3)==0){
		return GPGLL;
	}
	else if(strncmp(data,ZDA,3)==0){
		return GPZDA;
	}
	else
		return 0;
}

int GPS_GetData::GPS_coord_to_degrees(char* s) {
	char *p, *q;
	short deg = 0, min = 0;
	unsigned int frac_min = 0;
	short i;

	// scan for decimal point or end of field
	for (p = s; isdigit(*p); p++) ;
	q = s;

	// convert degrees
	while ((p - q) > 2) {
		if (deg) deg *= 10;
		deg += ((*q++)-'0');
	}
	// convert minutes
	while (p > q) {
		if (min) min *= 10;
		min += ((*q++)-'0');
	}
	// convert fractional minutes
	// expect up to four digits, result is in
	// ten-thousandths of a minute
	if (*p == '.') {
		q = p + 1;
		for (i = 0; i < 5; i++) {
			frac_min *= 10;
			if (isdigit(*q))
				frac_min += *q++ - '0';
		}
	}
	return deg * 10000000UL + (min * 1000000UL + frac_min*10UL) / 6;
}

void GPS_GetData::Get_Altitude(int32_t &alt0){
	alt0=Gpgga.Sea_Dials;
	
}

void GPS_GetData::Get_GPS_Direction(char &lng_dir,char &lat_dir){ //得到N/S 和 E/W
	lat_dir=Gpgga.Latitude_Directon;
	lng_dir=Gpgga.Longitude_Direction;
}

