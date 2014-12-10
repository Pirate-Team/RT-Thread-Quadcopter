#include "ov_7725.h"
#include "Sccb.h"
#include "Usart.h"
#include "Meanshift.h"


extern TARGET_CONDI condition_blue;
extern TARGET_CONDI condition_green;
extern TARGET_CONDI condition_yellow1;
extern TARGET_CONDI condition_white;
extern TARGET_CONDI condition_red;
extern TARGET_CONDI condition_yellow;
extern TARGET_CONDI condition_darkBlue;
typedef struct Reg
{
	uint8_t Address;			       /*寄存器地址*/
	uint8_t Value;		           /*寄存器值*/
}Reg_Info;


/* 寄存器参数配置 */
Reg_Info Sensor_Config[] =
{
	{CLKRC,     0x00}, /*clock config*/
	{COM7,      0x46}, /*QVGA RGB565 */
	{HSTART,    0x3f},
	{HSIZE,     0x50},
	{VSTRT,     0x03},
	{VSIZE,     0x78},
	{HREF,      0x00},
	{HOutSize,  0x50},
	{VOutSize,  0x78},
	{EXHCH,     0x00},

	/*DSP control*/
	{TGT_B,     0x7f},
	{FixGain,   0x09},
	{AWB_Ctrl0, 0xe0},
	{DSP_Ctrl1, 0xff},
	{DSP_Ctrl2, 0x20},
	{DSP_Ctrl3,	0x00},
	{DSP_Ctrl4, 0x00},

	/*AGC AEC AWB*/
	{COM8,		  0xf0},
	{COM4,		  0x81}, /*Pll AEC CONFIG*/
	{COM6,		  0xc5},
	{COM9,		  0x21},
	{BDBase,	  0xFF},
	{BDMStep,	  0x01},
	{AEW,		    0x34},
	{AEB,		    0x3c},
	{VPT,		    0xa1},
	{EXHCL,		  0x00},
	{AWBCtrl3,  0xaa},
	{COM8,		  0xff},
	{AWBCtrl1,  0x5d},

	{EDGE1,		  0x0a},
	{DNSOff,	  0x01},
	{EDGE2,		  0x01},
	{EDGE3,		  0x01},

	{MTX1,		  0x5f},
	{MTX2,		  0x53},
	{MTX3,		  0x11},
	{MTX4,		  0x1a},
	{MTX5,		  0x3d},
	{MTX6,		  0x5a},
	{MTX_Ctrl,  0x1e},

	{BRIGHT,	  0x00},
	{CNST,		  0x25},  //白平衡
	{USAT,		  0x50},
	{VSAT,		  0x50},
	{UVADJ0,	  0x81},
	{SDE,		    0x06},
	//{UFix,      0xa0},
	//{VFix,      0x40},
    /*GAMMA config*/
	{GAM1,		  0x0c},
	{GAM2,		  0x16},
	{GAM3,		  0x2a},
	{GAM4,		  0x4e},
	{GAM5,		  0x61},
	{GAM6,		  0x6f},
	{GAM7,		  0x7b},
	{GAM8,		  0x86},
	{GAM9,		  0x8e},
	{GAM10,		  0x97},
	{GAM11,		  0xa4},
	{GAM12,		  0xaf},
	{GAM13,		  0xc5},
	{GAM14,		  0xd7},
	{GAM15,		  0xe8},
	{SLOP,		  0x20},

	{HUECOS,	  0x80},
	{HUESIN,	  0x80},
	{DSPAuto,	  0xff},
	{DM_LNL,	  0x00},
	{BDBase,	  0x99},
	{BDMStep,	  0x03},
	{LC_RADI,	  0x00},
	{LC_COEF,	  0x13},
	{LC_XC,		  0x08},
	{LC_COEFB,  0x14},
	{LC_COEFR,  0x17},
	{LC_CTR,	  0x05},
	
	{COM3,		  0xd0},/*Horizontal mirror image*/

	/*night mode auto frame rate control*/
	{COM5,		0xf5},	 /*在夜视环境下，自动降低帧率，保证低照度画面质量*/
	//{COM5,		0x31},	/*夜视环境帧率不变*/
};

u8 OV7725_REG_NUM = sizeof(Sensor_Config)/sizeof(Sensor_Config[0]);	  /*结构体数组成员数目*/

uint8_t Ov7725_vsync = 0;	 /* 帧同步信号标志，在中断函数和main函数里面使用 */


/************************************************
 * 函数名：FIFO_GPIO_Config
 * 描述  ：FIFO GPIO配置
 * 输入  ：无
 * 输出  ：无
 * 注意  ：无
 ************************************************/
static void FIFO_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA | 
		  RCC_AHB1Periph_GPIOB | 
		  RCC_AHB1Periph_GPIOC | 
		  RCC_AHB1Periph_GPIOD , ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//GPIO_PuPd_UP; //上拉输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        //IO速度


	/* 1W LED 灯控制 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//GPIO_ResetBits(GPIOA, GPIO_Pin_8);
	//GPIO_SetBits(GPIOA, GPIO_Pin_10);

	/*PD3(FIFO_WEN--FIFO写使能)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*PB5(FIFO_WRST--FIFO写复位)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*PA2(FIFO_RRST--FIFO读复位)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

	// PA3(FIFO_OE--FIFO输出使能)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 

	/*PC5(FIFO_RCLK-FIFO读时钟)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*PB8-PB15(FIFO_DATA--FIFO输出数据)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;						
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;           
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        //IO速度
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;						
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;           
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        //IO速度
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	FIFO_OE_L();	  					/*拉低使FIFO输出使能*/
	FIFO_WE_H();   						/*拉高使FIFO写允许*/
}

void Ov7725_GPIO_Config(void)
{
	SCCB_GPIO_Config();
	FIFO_GPIO_Config();
}

/************************************************
 * 函数名：VSYNC_GPIO_Configuration
 * 描述  ：OV7725 GPIO配置
 * 输入  ：无
 * 输出  ：无
 * 注意  ：无
 ************************************************/
static void VSYNC_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD , ENABLE);	  /*PA0---VSYNC*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);  //打开systemconfig时钟

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;          
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;        //IO速度

	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/************************************************
 * 函数名：VSYNC_NVIC_Configuration
 * 描述  ：VSYNC中断配置
 * 输入  ：无
 * 输出  ：无
 * 注意  ：无
 ************************************************/
static void VSYNC_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/************************************************
 * 函数名：VSYNC_EXTI_Configuration
 * 描述  ：OV7725 VSYNC中断管脚配置
 * 输入  ：无
 * 输出  ：无
 * 注意  ：无
 ************************************************/
/*               ___                            ___
 * VSYNC:     __|   |__________________________|   |__     
 */
static void VSYNC_EXTI_Configuration(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource2);

	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	// EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising ; /*上升沿触发*/
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling ; 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

}

/************************************************
 * 函数名：VSYNC_Init
 * 描述  ：OV7725 VSYNC中断相关配置
 * 输入  ：无
 * 输出  ：无
 * 注意  ：无
 ************************************************/
void VSYNC_Init(void)
{
    VSYNC_GPIO_Configuration();
    VSYNC_EXTI_Configuration();
    VSYNC_NVIC_Configuration();
}

/************************************************
 * 函数名：Sensor_Init
 * 描述  ：Sensor初始化
 * 输入  ：无
 * 输出  ：返回1成功，返回0失败
 * 注意  ：无
 ************************************************/
ErrorStatus Ov7725_Init(void)
{
	uint16_t i = 0;
	uint8_t Sensor_IDCode = 0;	
	
	//DEBUG("ov7725 Register Config Start......");
	
	if( 0 == SCCB_WriteByte ( 0x12, 0x80 ) ) /*复位sensor */
	{
		//DEBUG("sccb write data error");		
		return ERROR ;
	}	

	if( 0 == SCCB_ReadByte( &Sensor_IDCode, 1, 0x0b ) )	 /* 读取sensor ID号*/
	{
		//DEBUG("read id faild");		
		return ERROR;
	}
	//DEBUG("Sensor ID is 0x%x", Sensor_IDCode);	
	
	if(Sensor_IDCode == OV7725_ID)
	{
		for( i = 0 ; i < OV7725_REG_NUM ; i++ )
		{
			if( 0 == SCCB_WriteByte(Sensor_Config[i].Address, Sensor_Config[i].Value) )
			{                
				//DEBUG("write reg faild", Sensor_Config[i].Address);
				return ERROR;
			}
		}

	}
	else
	{
		return ERROR;
	}
	//DEBUG("ov7725 Register Config Success");
	
	return SUCCESS;
}


/*       320
 * -------------------
 *|                   |
 *|                   |
 *|                   |  240
 *|                   |
 *|                   |
 * -------------------
 */
#define min3v(v1, v2, v3)   ((v1)>(v2)? ((v2)>(v3)?(v3):(v2)):((v1)>(v3)?(v3):(v1)))
#define max3v(v1, v2, v3)   ((v1)<(v2)? ((v2)<(v3)?(v3):(v2)):((v1)<(v3)?(v3):(v1)))

void ImagDisp(uint8_t*  Cam_data,TARGET_CONDI* Condition)
{
	uint32_t i;
	int16_t r;
	int16_t g;
	int16_t b;
	uint16_t Camera_Data;
	COLOR_HSL Hsl;
	int16_t h,s,l,maxVal,minVal,difVal;
	
	i=76800;
	while(i--){
			READ_FIFO_PIXEL(Camera_Data);		/* 从FIFO读出一个rgb565像素到Camera_Data变量 */
			//读取单个像素点数据进行判定，符合判定条件的在相应数组内置1，不符合的置0，并将跳转到下一个像素点处理
			asm(
				"MOV R5,%0\n"
				"AND R5,#0xf800\n"
				"MOV R5,ROR#8\n"
				"MOV %0,R5\n"
				:"=r"(r)
				:"r"(Camera_Data)
				:"r5"
				);
			asm(
				"MOV R5,%0\n"
				"AND R5,#0x07e0\n"
				"MOV R5,ROR#3\n"
				"MOV %0,R5\n"
				:"=r"(g)
				:"r"(Camera_Data)
				:"r5"
				);
			asm(
				"MOV R5,%0\n"
				"AND R5,#0x00lf\n"
				"MOV R5,ROL#3\n"
				"MOV %0,R5\n"
				:"=r"(g)
				:"r"(Camera_Data)
				:"r5"
				);
//			r  = (unsigned char)((Camera_Data&0xf800)>>8);
//			g  = (unsigned char)((Camera_Data&0x07e0)>>3);
//			b  = (unsigned char)((Camera_Data&0x001f)<<3);
//		
			maxVal = max3v(r, g, b);
			minVal = min3v(r, g, b);
			if(maxVal == minVal)//若r=g=b
			{
				*(Cam_data++)=0;
				continue;
			}
			else
			{
				difVal = maxVal-minVal;
				//计算色调
				if(maxVal==r)
				{
					if(g>=b)
						h = 40*(g-b)/(difVal);
					else
						h = 40*(g-b)/(difVal) + 240;
				}
				else if(maxVal==g)
					h = 40*(b-r)/(difVal) + 80;
				else if(maxVal==b)
					h = 40*(r-g)/(difVal) + 160;
				
				Hsl.hue = (unsigned char)(((h>240)? 240 : ((h<0)?0:h)));
				if(Hsl.hue > Condition->H_MAX && Hsl.hue < Condition->H_MIN)
				{
					*(Cam_data++)=0;
					continue;
				}
				//计算亮度
				l = (maxVal+minVal)*240/255/2;
				Hsl.luminance  = (unsigned char)(((l>240)? 240 : ((l<0)?0:l)));
			
				if(Hsl.luminance > Condition->L_MAX||Hsl.luminance <	Condition->L_MIN){
					*(Cam_data++)=0;
					continue;
				}
				//计算饱和度
				if(l == 0)
					s = 0;
				else if(l<=120)
					s = (difVal)*240/(maxVal+minVal);
				else
					s = (difVal)*240/(511 - (maxVal+minVal));//为什么不是480二十511
				Hsl.saturation = (unsigned char)(((s>240)? 240 : ((s<0)?0:s)));
				if(Hsl.saturation < Condition->S_MIN || Hsl.saturation > Condition->S_MAX){
				    *(Cam_data++)=0;
					continue;
				}
			  *(Cam_data++)=1;
			}	
	}
}

void Set_15fps()  //pclk=12M
{
  SCCB_WriteByte(0x11, 0x03);
  SCCB_WriteByte(0x0d, 0x41);
  SCCB_WriteByte(0x2a, 0x00);
  SCCB_WriteByte(0x2b, 0x00);
  SCCB_WriteByte(0x33, 0x00);
  SCCB_WriteByte(0x34, 0x00);
  SCCB_WriteByte(0x2d, 0x00);
  SCCB_WriteByte(0x2e, 0x00);
  SCCB_WriteByte(0x0e, 0x65);
}

void set_25fps()
{
	SCCB_WriteByte(0x11, 0x01);//
  SCCB_WriteByte(0x0d, 0x41);
  SCCB_WriteByte(0x2a, 0x00);
  SCCB_WriteByte(0x2b, 0x00);
  SCCB_WriteByte(0x33, 0x66);
  SCCB_WriteByte(0x34, 0x00);
  SCCB_WriteByte(0x2d, 0x00);
  SCCB_WriteByte(0x2e, 0x00);
  SCCB_WriteByte(0x0e, 0x65);
}

void set_30fps()
{
	SCCB_WriteByte(0x11, 0x01);//
	SCCB_WriteByte(0x0d, 0x41);//
	SCCB_WriteByte(0x2a, 0x00);//
	SCCB_WriteByte(0x2b, 0x00);//
	SCCB_WriteByte(0x33, 0x00);//66
	SCCB_WriteByte(0x34, 0x00);//
	SCCB_WriteByte(0x2d, 0x00);//
	SCCB_WriteByte(0x2e, 0x00);//
    SCCB_WriteByte(0x0e, 0x65);//
}

void rmov_banding_50hz()  //for 25fps
{
	SCCB_WriteByte(0x13, 0xff); //banding filter enable
	SCCB_WriteByte(0x22, 0x98); //50Hz banding filter
	SCCB_WriteByte(0x23, 0x03); //4 step for 50hz
}

void rmov_banding_60hz()  //for 30fps
{
  SCCB_WriteByte(0x13, 0xff); //banding filter enable
  SCCB_WriteByte(0x22, 0x7f); //60Hz banding filter
  SCCB_WriteByte(0x23, 0x03); //4 step for 60hz
}
/*--------------------------White Balance----------------------*/
void set_wb_sunny()
{
  SCCB_WriteByte(0x13, 0xfd); //AWB off
  SCCB_WriteByte(0x01, 0x5a);
  SCCB_WriteByte(0x02, 0x5c);
  SCCB_WriteByte(0x0e, 0x65); 
  SCCB_WriteByte(0x2d, 0x00);
  SCCB_WriteByte(0x2e, 0x00);
}

void set_wb_cloudy()
{
  SCCB_WriteByte(0x13, 0xfd); //AWB off
  SCCB_WriteByte(0x01, 0x58);
  SCCB_WriteByte(0x02, 0x60);
  SCCB_WriteByte(0x0e, 0x65); 
  SCCB_WriteByte(0x2d, 0x00);
  SCCB_WriteByte(0x2e, 0x00);
}

void set_wb_office()
{
  SCCB_WriteByte(0x13, 0xfd); //AWB off
  SCCB_WriteByte(0x01, 0x84);
  SCCB_WriteByte(0x02, 0x4c);
  SCCB_WriteByte(0x0e, 0x65); 
  SCCB_WriteByte(0x2d, 0x00);
}

void set_wb_home()
{
  SCCB_WriteByte(0x13, 0xfd); //AWB off
  SCCB_WriteByte(0x01, 0x96);
  SCCB_WriteByte(0x02, 0x40);
  SCCB_WriteByte(0x0e, 0x65); 
  SCCB_WriteByte(0x2d, 0x00);
  SCCB_WriteByte(0x2e, 0x00);
}

void set_wb_night()
{
  SCCB_WriteByte(0x13, 0xff); //AWB on
  SCCB_WriteByte(0x0e, 0xe5); 
}
/*-----------------------Saturation-----------------------*/
void set_saturation(int n)
{
  SCCB_WriteByte(0xa7, 0x40+n*0x10);
  SCCB_WriteByte(0xa8, 0x40+n*0x10);
}
/*-----------------------Brightness-----------------------*/
void set_brightness_plus(int n)
{
  SCCB_WriteByte(0x9b, 0x08+n*0x10);
  SCCB_WriteByte(0xab, 0x06);
}

void set_brightness_minus(int n)
{
  SCCB_WriteByte(0x9b, 0x08+(-1*n)*0x10);
  SCCB_WriteByte(0xab, 0x0e);
}
/*---------------------Special Effects--------------------*/
void set_eff_normal()
{
  SCCB_WriteByte(0xa6, 0x06);
  SCCB_WriteByte(0x60, 0x80);
  SCCB_WriteByte(0x61, 0x80);
}

void set_eff_bw()
{
  SCCB_WriteByte(0xa6, 0x26);
  SCCB_WriteByte(0x60, 0x80);
  SCCB_WriteByte(0x61, 0x80);
}

void set_eff_bluish()
{
  SCCB_WriteByte(0xa6, 0x1e);
  SCCB_WriteByte(0x60, 0xa0);
  SCCB_WriteByte(0x61, 0x40);
}

void set_eff_sepia()
{
  SCCB_WriteByte(0xa6, 0x1e);
  SCCB_WriteByte(0x60, 0x40);
  SCCB_WriteByte(0x61, 0xa0);
}

void set_eff_redish()
{
  SCCB_WriteByte(0xa6, 0x1e);
  SCCB_WriteByte(0x60, 0x80);
  SCCB_WriteByte(0x61, 0xc0);
}

void set_eff_greenish()
{
  SCCB_WriteByte(0xa6, 0x1e);
  SCCB_WriteByte(0x60, 0x60);
  SCCB_WriteByte(0x61, 0x60);
}

void set_eff_negative()
{
	SCCB_WriteByte(0xa6, 0x46);
}
