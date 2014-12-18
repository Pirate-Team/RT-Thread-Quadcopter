#include "Meanshift.h"
#include "ov_7725.h"

#define min3v(v1, v2, v3)   ((v1)>(v2)? ((v2)>(v3)?(v3):(v2)):((v1)>(v3)?(v3):(v1)))
#define max3v(v1, v2, v3)   ((v1)<(v2)? ((v2)<(v3)?(v3):(v2)):((v1)<(v3)?(v3):(v1)))

extern uint8_t Cam_data[240][320];

uint16_t  GUI_ReadBit16Point(unsigned int x,unsigned int y){
	     if((x<200)&&(y<180))
         return Cam_data[y][x];
			 else 
				 return 0;
}


//读取RBG格式颜色，唯一需要移植的函数
//extern unsigned short GUI_ReadBit16Point(unsigned short x,unsigned short y);
void ReadColor(unsigned int x,unsigned int y,COLOR_RGB *Rgb)
{
	unsigned short C16;

	C16 = Cam_data[y][x];
	
//	Rgb->red   =	 (unsigned char)(((C16&0xf800)>>11) * 255) / 31;
//	Rgb->green =	 (unsigned char)(((C16&0x07e0)>>5)  * 255) / 63;
//	Rgb->blue  =     (unsigned char)(((C16&0x001f)>>0)  * 255) / 31;
	Rgb->red   =	 (unsigned char)((C16&0xf800)>>8);
	Rgb->green =	 (unsigned char)((C16&0x07e0)>>3);
	Rgb->blue  =     (unsigned char)((C16&0x001f)<<3);
}



//RGB转HSL
void RGBtoHSL(const COLOR_RGB *Rgb, COLOR_HSL *Hsl)
{
    int h,s,l,maxVal,minVal,difVal;
	int r  = Rgb->red;
	int g  = Rgb->green;
	int b  = Rgb->blue;
	
	maxVal = max3v(r, g, b);
	minVal = min3v(r, g, b);
	
	difVal = maxVal-minVal;
	
	//计算亮度
    l = (maxVal+minVal)*240/255/2;
	
	if(maxVal == minVal)//若r=g=b
	{
		h = 0; 
		s = 0;
	}
	else
	{
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
		//计算饱和度
		if(l == 0)
			s = 0;
		else if(l<=120)
			s = (difVal)*240/(maxVal+minVal);
		else
			s = (difVal)*240/(511 - (maxVal+minVal));//为什么不是480二十511
	}
    Hsl->hue =        (unsigned char)(((h>240)? 240 : ((h<0)?0:h)));
    Hsl->saturation = (unsigned char)(((s>240)? 240 : ((s<0)?0:s)));
    Hsl->luminance =  (unsigned char)(((l>240)? 240 : ((l<0)?0:l)));
}

//匹配颜色
int ColorMatch(const COLOR_HSL *Hsl,const TARGET_CONDI *Condition)
{
	if( 
		Hsl->hue		>=	Condition->H_MIN &&
		Hsl->hue		<=	Condition->H_MAX &&
		Hsl->saturation	>=	Condition->S_MIN &&
		Hsl->saturation	<=   Condition->S_MAX &&
		Hsl->luminance	>=	Condition->L_MIN &&
		Hsl->luminance	<=   Condition->L_MAX 
    )
		return 1;
	else
		return 0;
}

//搜索腐蚀中心
static int SearchCentre(unsigned int *x,unsigned int *y,const TARGET_CONDI *Condition,const SEARCH_AREA *Area)
{
	unsigned int SpaceX,SpaceY,i,j,k,FailCount=0;
	//COLOR_RGB Rgb;
//	COLOR_HSL Hsl;
	uint8_t temp;
	SpaceX = Condition->WIDTH_MIN/3;
	SpaceY = Condition->HIGHT_MIN/3;

	for(i=Area->Y_Start;i<Area->Y_End;i+=SpaceY)
	{
		for(j=Area->X_Start;j<Area->X_End;j+=SpaceX)
		{
			FailCount=0;
			for(k=0;k<SpaceX+SpaceY;k++)
			{
				if(k<SpaceX)
					temp = Cam_data[i+SpaceY/2][j+k];//ReadColor(j+k,i+SpaceY/2,&Rgb);
				else
					temp = Cam_data[i+(k-SpaceX)][j+SpaceX/2];//ReadColor(j+SpaceX/2,i+(k-SpaceX),&Rgb);
				//RGBtoHSL(&Rgb,&Hsl);
				
				//if(!ColorMatch(&Hsl,Condition))
				if(temp == 0)
					FailCount++;
				if(FailCount>((SpaceX+SpaceY)>>ALLOW_FAIL_PER))
					break;
			}
			if(k==SpaceX+SpaceY)
			{
				*x = j+SpaceX/2;
				*y = i+SpaceY/2;
				return 1;
			}
		}
	}
	return 0;
}

//从腐蚀中心向外腐蚀，得到新的腐蚀中心
static int Corrode(unsigned int oldx,unsigned int oldy,const TARGET_CONDI *Condition,RESULT *Resu)
{
	unsigned int Xmin,Xmax,Ymin,Ymax,i,FailCount=0;
	//COLOR_RGB Rgb;
//	COLOR_HSL Hsl;
	uint8_t  temp;
	for(i=oldx;i>IMG_X;i--)
	{
		temp = Cam_data[oldy][i];//ReadColor(i,oldy,&Rgb);
		//RGBtoHSL(&Rgb,&Hsl);
		//if(!ColorMatch(&Hsl,Condition))
		if(temp==0)
			FailCount++;
		if(FailCount>(((Condition->WIDTH_MIN+Condition->WIDTH_MAX)>>2)>>ALLOW_FAIL_PER))
			break;	
	}
	Xmin=i;
	FailCount=0;
	
	for(i=oldx;i<IMG_X+IMG_W;i++)
	{
		temp = Cam_data[oldy][i];//ReadColor(i,oldy,&Rgb);
		//RGBtoHSL(&Rgb,&Hsl);
		//if(!ColorMatch(&Hsl,Condition))
		if(temp==0)
			FailCount++;
		if(FailCount>(((Condition->WIDTH_MIN+Condition->WIDTH_MAX)>>2)>>ALLOW_FAIL_PER))
			break;	
	}
	Xmax=i;
	FailCount=0;
	
	for(i=oldy;i>IMG_Y;i--)
	{
		temp = Cam_data[i][oldx];//ReadColor(oldx,i,&Rgb);
		//RGBtoHSL(&Rgb,&Hsl);
		//if(!ColorMatch(&Hsl,Condition))
		if(temp==0)
			FailCount++;
		if(FailCount>(((Condition->HIGHT_MIN+Condition->HIGHT_MAX)>>2)>>ALLOW_FAIL_PER))
			break;	
	}
	Ymin=i;
	FailCount=0;
	
	for(i=oldy;i<IMG_Y+IMG_H;i++)
	{
		temp = Cam_data[i][oldx];//ReadColor(oldx,i,&Rgb);
		//RGBtoHSL(&Rgb,&Hsl);
		//if(!ColorMatch(&Hsl,Condition))
		if(temp==0)
			FailCount++;
		if(FailCount>(((Condition->HIGHT_MIN+Condition->HIGHT_MAX)>>2)>>ALLOW_FAIL_PER))
			break;	
	}
	Ymax=i;
	FailCount=0;
	
	Resu->x	= (Xmin+Xmax)/2;
	Resu->y	= (Ymin+Ymax)/2;
	Resu->w	= Xmax-Xmin;
	Resu->h	= Ymax-Ymin;

	if(((Xmax-Xmin)>(Condition->WIDTH_MIN)) && ((Ymax-Ymin)>(Condition->HIGHT_MIN)) &&\
	   ((Xmax-Xmin)<(Condition->WIDTH_MAX)) && ((Ymax-Ymin)<(Condition->HIGHT_MAX)) )
		return 1;	
	else
		return 0;	
}

//single API, caculates the target x,y width and height
//return 1 for success,0 for failure
int Trace(const TARGET_CONDI *Condition,RESULT *Resu)
{
	unsigned int i;
	static unsigned int x0,y0,flag=0;
	static SEARCH_AREA Area={IMG_X,IMG_X+IMG_W,IMG_Y,IMG_Y+IMG_H};
	RESULT Result;	
	

	if(flag==0)
	{
		if(SearchCentre(&x0,&y0,Condition,&Area))
			flag=1;
		else
		{
			Area.X_Start= IMG_X	;
			Area.X_End  = IMG_X+IMG_W  ;
			Area.Y_Start= IMG_Y		;
			Area.Y_End  = IMG_Y+IMG_H;

			if(!SearchCentre(&x0,&y0,Condition,&Area))	
			{
				flag=0;
				return 0;
			}	
		}
	}
	Result.x = x0;
	Result.y = y0;
	
	for(i=0;i<ITERATE_NUM;i++)
		Corrode(Result.x,Result.y,Condition,&Result);
		
	if(Corrode(Result.x,Result.y,Condition,&Result))
	{
		x0=Result.x;
		y0=Result.y;
		Resu->x=Result.x;
		Resu->y=Result.y;
		Resu->w=Result.w;
		Resu->h=Result.h;
		flag=1;

		Area.X_Start= Result.x - ((Result.w)>>1);
		Area.X_End  = Result.x + ((Result.w)>>1);
		Area.Y_Start= Result.y - ((Result.h)>>1);
		Area.Y_End  = Result.y + ((Result.h)>>1);


		return 1;
	}
	else
	{
		flag=0;
		return 0;
	}

}

int TraceinFrame(const TARGET_CONDI *Condition,RESULT *Resu)
{
	static unsigned int x0,y0;
	static SEARCH_AREA FullFrameArea1={IMG_X,IMG_X+IMG_W-1,IMG_Y,IMG_Y+IMG_H-1};
	static SEARCH_AREA LastFrameArea1;
	static TRACE_STATE TraceState1 = TS_FIND_IN_FULL_FRAME;
	
	RESULT TempResult;	
	
	switch(TraceState1)
	{
		case TS_FIND_IN_FULL_FRAME:
			if(SearchCentre(&x0,&y0,Condition,&FullFrameArea1))
			{
				TraceState1 = TS_TRACING;
			}
			else
			{
				return 0;
			}
		break;
		
		case TS_FIND_IN_LAST_FRAME:
			if(SearchCentre(&x0,&y0,Condition,&LastFrameArea1))
			{
				TraceState1 = TS_TRACING;
			}
			else
			{
				TraceState1 = TS_FIND_IN_FULL_FRAME;
				return 0;
			}
		break;
		
		case TS_TRACING:
			break;
	}
	
	TempResult.x = x0;
	TempResult.y = y0;

	
	
	if(Corrode(TempResult.x,TempResult.y,Condition,&TempResult)==0)
	{
			TraceState1 = TS_FIND_IN_LAST_FRAME;
			return 0;
	}
			
	Resu->x=TempResult.x;
	Resu->y=TempResult.y;
	Resu->w=TempResult.w;
	Resu->h=TempResult.h;
	
	LastFrameArea1.X_Start= TempResult.x - ((TempResult.w)>>1);
	LastFrameArea1.X_End  = TempResult.x + ((TempResult.w)>>1);
	LastFrameArea1.Y_Start= TempResult.y - ((TempResult.h)>>1);
	LastFrameArea1.Y_End  = TempResult.y + ((TempResult.h)>>1);
	
	x0=TempResult.x;
	y0=TempResult.y;
	
	
	return 1;
}




