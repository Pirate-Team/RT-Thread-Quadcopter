#include <stdio.h>
#include "stm32f4xx.h"
#include "rtthread.h"
#include "ov_7725.h"
#include "Sccb.h"
#include "Meanshift.h"


extern uint8_t Ov7725_vsync;
//uint16_t Cam_data[180][200]; //70kb
uint16_t Cam_data[1][1]; //70kb
TARGET_CONDI condition_blue  = {143 ,151 ,30  ,240 ,30  ,180 ,  10   , 10   ,   150  ,  120    }; //Blue
TARGET_CONDI condition_green = {124 ,136 ,20  ,240 ,10  ,180 ,  24   , 24   ,   120  ,   60    }; //green
TARGET_CONDI condition_yellow1 = {0x18,0x2c,0x16,0xe0,0x1a,0xef,  20   , 20   ,   100  ,   80    }; //yellow
TARGET_CONDI condition_white = {  0 ,   1,   0,  1 ,180 ,240 ,  10   , 10   ,   100  ,   80    }; //white light
TARGET_CONDI condition_red = {   0, 11 ,60 ,250 ,35  ,225 ,  20   , 20   ,   180  ,  150   }; //red
TARGET_CONDI condition_yellow = { 27 , 60 , 20, 250, 40   ,220 ,  24   , 24   ,   180  ,   150    }; //yellow
TARGET_CONDI condition_darkBlue = {160 ,170 ,50  ,240 ,0   ,200 ,  60   , 60   ,   100  ,   70    }; //blue

RESULT result,result1;//识别结果
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
int ShowLocation(RESULT result){
				char str[5];    
				sprintf(str,"%3d",(result.x-100));
				rt_kprintf("the X:%s  ",str);
				sprintf(str,"%3d",(result.y-90));
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
//	uint32_t tick=0;
	Ov7725_vsync = 0;
	while(1)
	{
		if(Ov7725_vsync == 2)
		{
			FIFO_PREPARE();  			/*FIFO准备*/					
			ImagDisp(&Cam_data[0][0]);	/*采集并显示*/		
			if(Trace(&condition_yellow,&result))
			{
				targetX=result.x-100;
				targetY=result.y-90;
				targetX/=5;
				targetY/=5;
				targetH = result.h/5;
				targetW = result.w/5;
//				ShowImage();
//				ShowLocation(result);
			}
			else{
				targetX=0;
				targetY=0;
				targetH=0;
				targetW=0;
			}
			Ov7725_vsync = 0;		//处理结束标志			
		}
	}
}

