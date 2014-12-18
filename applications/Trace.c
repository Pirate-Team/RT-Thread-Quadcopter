#include <stdio.h>
#include "stm32f4xx.h"
#include "rtthread.h"
#include "ov_7725.h"
#include "Sccb.h"
#include "Meanshift.h"
#include "head.h"


extern uint8_t Ov7725_vsync;
uint8_t Cam_data[240][320]; //70kb

TARGET_CONDI condition_blue  = {141 ,153 ,30  ,240 ,30  ,220 ,  10   , 10   ,   270  ,  180    }; //Blue
TARGET_CONDI condition_green = {124 ,136 ,20  ,240 ,10  ,180 ,  24   , 24   ,   270  ,   180    }; //green
TARGET_CONDI condition_yellow1 = {0x18,0x2c,0x16,0xe0,0x1a,0xef,  20   , 20   , 270  ,   180   }; //yellow
TARGET_CONDI condition_white = {  0 ,   1,   0,  1 ,180 ,240 ,  10   , 10   ,   270  ,   180   }; //white light
TARGET_CONDI condition_red = {   230, 11 ,60 ,250 ,35  ,225 ,  20   , 20   ,     180  ,   100   }; //red
TARGET_CONDI condition_yellow = { 27 , 60 , 20, 250, 40   ,220 ,  24   , 24   , 270 ,   180   }; //yellow
TARGET_CONDI condition_darkBlue = {160 ,170 ,50  ,240 ,0   ,200 ,  60   , 60   ,270  ,   180    }; //blue

RESULT result;//识别结果
int16_t targetX=0,targetY=0,targetH=0,targetW=0;

int  ShowImage(void){
	uint8_t i,j;
	char str[5];
	for(i=0;i<180;i++)
		for(j=0;j<200;j++){
			sprintf(str,"%5d",Cam_data[i][j]);
			rt_kprintf("%s ",str);
	}
	return 0;
}
int showHSl(int x,int y){
	char str[5];
	COLOR_RGB Rgb;
	COLOR_HSL Hsl;
	
	ReadColor(x,y,&Rgb);
	RGBtoHSL(&Rgb, &Hsl);
	
	sprintf(str,"%5d",Hsl.hue);
	rt_kprintf("the H:%s  ",str);
	sprintf(str,"%5d",Hsl.saturation);
	rt_kprintf("the S:%s  ",str);
	sprintf(str,"%5d",Hsl.luminance);
	rt_kprintf("the L:%s  \n",str);
	return 0;
}
int ShowLocation(RESULT result){
				char str[5];    
				sprintf(str,"%3d",(result.x-180));
				rt_kprintf("the X:%s  ",str);
				sprintf(str,"%3d",(result.y-120));
				rt_kprintf("the Y:%s  \n",str);
				return 0;
}

uint8_t ov_7725_init(void)
{
	Ov7725_GPIO_Config();
	if(Ov7725_Init() != SUCCESS) return 0;
	VSYNC_Init();
	return 1;
}

void rt_thread_entry_trace(void* parameter)
{
	Ov7725_vsync = 0;
	ov_7725_init();
	while(1)
	{
		if(Ov7725_vsync == 2)
		{ 
			FIFO_PREPARE();  			/*FIFO准备*/					
			ImagDisp(&Cam_data[0][0],&condition_red);	/*采集并显示*/	
			
			if(Trace(&condition_red,&result))
			{
				targetX=result.x-160;
				targetY=result.y-120;
				targetX/=5;
				targetY/=5;
				targetH = result.h/5;
				targetW = result.w/5;
			}
			else{
				targetX *= 0.7;
				targetY *= 0.7;
				targetH *= 0.7;
				targetW *= 0.7;
//				result.x=0;
//				result.y=0;
			}
			Ov7725_vsync = 0;		//处理结束标志
		}
	}
}

