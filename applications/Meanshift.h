#ifndef EASY_TRACER_H
#define EASY_TRACER_H
#include "head.h"

#define IMG_X 0	  //图片x坐标
#define IMG_Y 0	  //图片y坐标
#define IMG_W 320 //图片宽度
#define IMG_H 240 //图片高度

#define ALLOW_FAIL_PER 2 //容错率，没1<<ALLOW_FAIL_PER个点允许出现一个错误点，容错率越大越容易识别，但错误率越大
#define ITERATE_NUM    10 //迭代次数，迭代次数越多识别越精确，但计算量越大

#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //棕色
#define BRRED 			 0XFC07 //棕红色
#define GRAY  			 0X8430 //灰色

typedef struct{
    unsigned char  H_MIN;//目标最小色调
    unsigned char  H_MAX;//目标最大色调	
    
	unsigned char  S_MIN;//目标最小饱和度  
    unsigned char  S_MAX;//目标最大饱和度
	
	unsigned char  L_MIN;//目标最小亮度  
    unsigned char  L_MAX;//目标最大亮度
	
	unsigned int  WIDTH_MIN;//目标最小宽度
	unsigned int  HIGHT_MIN;//目标最小高度

	unsigned int  WIDTH_MAX;//目标最大宽度
	unsigned int  HIGHT_MAX;//目标最大高度

}TARGET_CONDI;//判定为的目标条件

typedef struct{
	 uint16_t x;//目标的x坐标
	 uint16_t y;//目标的y坐标
	 uint16_t w;//目标的宽度
	 uint16_t h;//目标的高度
}RESULT;//识别结果

typedef struct{
    unsigned char  red;             // [0,255]
    unsigned char  green;           // [0,255]
    unsigned char  blue;            // [0,255]
}COLOR_RGB;//RGB格式颜色

typedef struct{
    unsigned char hue;              // [0,240]
    unsigned char saturation;       // [0,240]
    unsigned char luminance;        // [0,240]
}COLOR_HSL;//HSL格式颜色

typedef struct{
    unsigned int X_Start;              
    unsigned int X_End;
	unsigned int Y_Start;              
    unsigned int Y_End;
}SEARCH_AREA;//区域

typedef enum{
	TS_FIND_IN_FULL_FRAME,//full frame 
	TS_FIND_IN_LAST_FRAME,//last frame
	TS_TRACING,// trace
}TRACE_STATE;

//唯一的API，用户将识别条件写入Condition指向的结构体中，该函数将返回目标的x，y坐标和长宽
//返回1识别成功，返回1识别失败
int Trace(const TARGET_CONDI *Condition,RESULT *Resu);
uint16_t  GUI_ReadBit16Point(unsigned int x,unsigned int y);

void RGBtoHSL(const COLOR_RGB *Rgb, COLOR_HSL *Hsl);
void ReadColor(unsigned int x,unsigned int y,COLOR_RGB *Rgb);
int ColorMatch(const COLOR_HSL *Hsl,const TARGET_CONDI *Condition);
#endif
