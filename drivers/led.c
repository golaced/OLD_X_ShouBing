/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：led.c
 * 描述    ：LED驱动
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "led.h"
#include "include.h"
#include "mpu6050.h"
#include "ak8975.h"
#include "rc_mine.h"
void LEDRGB_COLOR(u8 color);
void LED_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(ANO_RCC_LED,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = ANO_Pin_LED1| ANO_Pin_LED2| ANO_Pin_LED3| ANO_Pin_LED4;
	GPIO_Init(ANO_GPIO_LED, &GPIO_InitStructure);
	
	LEDRGB_COLOR(BLUE);
	Delay_ms(500);
	LEDRGB_COLOR(RED);
	Delay_ms(500);
	LEDRGB_COLOR(GREEN);
  Delay_ms(500);
}
//中值滤波
float GetMedianNum(float * bArray, u16 iFilterLen)
{  
    int i,j;// 循环变量  
    float bTemp;  
      
    // 用冒泡法对数组进行排序  
    for (j = 0; j < iFilterLen - 1; j ++)  
    {  
        for (i = 0; i < iFilterLen - j - 1; i ++)  
        {  
            if (bArray[i] > bArray[i + 1])  
            {  
                // 互换  
                bTemp = bArray[i];  
                bArray[i] = bArray[i + 1];  
                bArray[i + 1] = bTemp;  
            }  
        }  
    }  
      
    // 计算中值  
    if ((iFilterLen & 1) > 0)  
    {  
        // 数组有奇数个元素，返回中间一个元素  
        bTemp = bArray[(iFilterLen + 1) / 2];  
    }  
    else  
    {  
        // 数组有偶数个元素，返回中间两个元素平均值  
        bTemp = (bArray[iFilterLen / 2] + bArray[iFilterLen / 2 + 1]) / 2;  
    }  
  
    return bTemp;  
}  

#define MID_RC_GET 5
u8 MODE_KEY[8];
u8 RC_GET[8][MID_RC_GET];
void READ_MODE(void)
{u8 i,j;
u8 RC_GET_TEMP[8];
float RC_GETR[8][MID_RC_GET];	
RC_GET_TEMP[0]=!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6);
RC_GET_TEMP[1]=!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_7);
RC_GET_TEMP[2]=!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_8);
RC_GET_TEMP[3]=!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_9);
RC_GET_TEMP[4]=!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_10);
RC_GET_TEMP[5]=!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11);
RC_GET_TEMP[6]=!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_12);
RC_GET_TEMP[7]=!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_13);

for(j=0;j<8;j++)
for(i=0;i<=MID_RC_GET-2;i++)
RC_GET[j][i]=RC_GET[j][i+1];
for(j=0;j<8;j++)
RC_GET[j][MID_RC_GET-2]=RC_GET_TEMP[j];

for(j=0;j<8;j++)
for(i=0;i<=MID_RC_GET-2;i++)
RC_GETR[j][i]=RC_GET[j][i];	
for(j=0;j<8;j++)
MODE_KEY[j]=RC_GET[j][MID_RC_GET-1]=GetMedianNum(RC_GETR[j], MID_RC_GET-2);//*0.7+0.3*reg1;	

}

u8 SEL_KEY[4];
u8 RC_GET2[4][MID_RC_GET];
void READ_SEL(void)
{
u8 i,j;
u8 RC_GET_TEMP[4];
float RC_GETR[4][MID_RC_GET];		
RC_GET_TEMP[0]=GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0);	
RC_GET_TEMP[1]=GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_1);
RC_GET_TEMP[2]=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13);
RC_GET_TEMP[3]=GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3);
for(j=0;j<4;j++)
for(i=0;i<=MID_RC_GET-2;i++)
RC_GET2[j][i]=RC_GET2[j][i+1];
for(j=0;j<4;j++)
RC_GET2[j][MID_RC_GET-2]=RC_GET_TEMP[j];

for(j=0;j<4;j++)
for(i=0;i<=MID_RC_GET-2;i++)
RC_GETR[j][i]=RC_GET2[j][i];	
for(j=0;j<4;j++)
SEL_KEY[j]=RC_GET2[j][MID_RC_GET-1]=GetMedianNum(RC_GETR[j], MID_RC_GET-2);//*0.7+0.3*reg1;	
}


void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6| GPIO_Pin_7| GPIO_Pin_8| GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	READ_MODE();
	READ_SEL();
}


void BEEP_TASK(u8 sel)
{
	switch(sel)
		{case 1:
			GPIO_SetBits(GPIOA, GPIO_Pin_3);	break;
			
		// case 2:	GPIO_SetBits(GPIOA, GPIO_Pin_3);Delay_ms(500);GPIO_ResetBits(GPIOA, GPIO_Pin_3);Delay_ms(500);break;	
			
		default:GPIO_ResetBits(GPIOA, GPIO_Pin_3);	break;	
	}
 	
}
void BEEP_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_3);
	BEEP_TASK(0);

}
void LED_Display( u8 duty[4] ) //0~20
{
	static u8 LED_cnt[4];
	u8 i;
	
	for(i=0;i<4;i++)
	{
		if( LED_cnt[i] < 19 )
		{
			LED_cnt[i]++;
		}
		else
		{
			LED_cnt[i] = 0;
		}
			
		if( LED_cnt[i] < duty[i] )
		{
			switch(i)
			{
				case 0:	
					LED1_ON;
				break;
				case 1:	
					LED2_ON;
				break;
				case 2:	
					LED3_ON;
				break;
				case 3:	
					LED4_ON;
				break;
			}
		}
		else
		{
						switch(i)
			{
				case 0:	
					LED1_OFF;
				break;
				case 1:	
					LED2_OFF;
				break;
				case 2:	
					LED3_OFF;
				break;
				case 3:	
					LED4_OFF;
				break;
			}
		}
	}
}

#include "rc.h"
#include "ak8975.h"

extern u8 height_ctrl_mode;

u8 LED_Brightness[4] = {0,20,0,0}; //TO 20
u8 LED_status[2];  //  0:old;  1:now
void LED_Duty()
{
	
 if(Rc_Get.THROTTLE<1500+THR_DEAD && Rc_Get.THROTTLE>1500-THR_DEAD)
	{
		LEDRGB_COLOR(WHITE);
	}
	
}

void LED_MPU_Err(void)
{

}

void LED_Mag_Err(void)
{
}

void LED_MS5611_Err(void)
{

}



void LEDS_SHINE(u8 sel,u16 delay)
{
}

/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/


void LEDRGB(u8 sel,u8 on)
{
switch(sel)
{
case RED:
if(on)
GPIO_ResetBits(GPIOE,GPIO_Pin_0);
else
GPIO_SetBits(GPIOE,GPIO_Pin_0);
break;
case GREEN:
if(on)
GPIO_ResetBits(GPIOE,GPIO_Pin_1);
else
GPIO_SetBits(GPIOE,GPIO_Pin_1);
break;
case BLUE:
if(on)
GPIO_ResetBits(GPIOE,GPIO_Pin_2);
else
GPIO_SetBits(GPIOE,GPIO_Pin_2);
break;
}
}

void LEDRGB_COLOR(u8 color)
{
switch (color)
{
case RED:
LEDRGB(RED,1);
LEDRGB(BLUE,0);
LEDRGB(GREEN,0);
break;
case BLUE:
LEDRGB(RED,0);
LEDRGB(BLUE,1);
LEDRGB(GREEN,0);
break;
case GREEN:
LEDRGB(RED,0);
LEDRGB(BLUE,0);
LEDRGB(GREEN,1);
break;
case WHITE:
LEDRGB(RED,1);
LEDRGB(BLUE,1);
LEDRGB(GREEN,1);
break;
case BLACK:
LEDRGB(RED,0);
LEDRGB(BLUE,0);
LEDRGB(GREEN,0);
break;
case YELLOW:
LEDRGB(RED,1);
LEDRGB(BLUE,0);
LEDRGB(GREEN,1);
break;
}
}
#define IDLE 0
#define CAL_MPU 1
#define CAL_M  2
u8 mode_control;
void LEDRGB_STATE(void)
{
static u8 main_state;
static u8 mpu_state,m_state,idle_state;
static u16 cnt,cnt_idle;
u8 mode_control;
	
	
mode_control=mode.mode_fly;
switch(main_state)
{ 
	
	case IDLE:
	if(mpu6050.Gyro_CALIBRATE)
	{idle_state=0;main_state=CAL_MPU;}
	else if(Mag_CALIBRATED)
	{idle_state=0;main_state=CAL_M;}
	break;
	case CAL_MPU:
  break;
	case CAL_M:
  break;
}
//   | | | |    | | | |   | | | |   | | | |   | | | |
//    ARM          GPS1     GPS2      GPS3      MODE  
u8 RGB_DELAY=2;
switch(idle_state)
{//ARM
	case 0:
		if(main_state==IDLE)
			{idle_state=1;cnt_idle=0;}
	break;
	case 1:
	 if(fly_ready==0)
				LEDRGB_COLOR(WHITE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY*2)
	{idle_state=2;cnt_idle=0;}
	break;
	case 2:
			LEDRGB_COLOR(BLACK); 
	if(cnt_idle++>RGB_DELAY)
	{idle_state=3;cnt_idle=0;}
	break;
	case 3:
			
	   if(fly_ready==0)
				LEDRGB_COLOR(WHITE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=4;cnt_idle=0;}
	break;
//GPS1	
	case 4:
		if(data_rate>5)
			LEDRGB_COLOR(WHITE);
		else
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=5;cnt_idle=0;}
	break;
	case 5:
			if(data_rate>5)
			LEDRGB_COLOR(WHITE);
		else
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=6;cnt_idle=0;}
	break;
	case 6:
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=7;cnt_idle=0;}
	break;
	case 7:
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=8;cnt_idle=0;}
	break;
		
//GPS2
	case 8:
			if(data_rate>5)
			LEDRGB_COLOR(WHITE);
		else
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=9;cnt_idle=0;}
	break;
	case 9:
				if(data_rate>5)
			LEDRGB_COLOR(WHITE);
		else
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=10;cnt_idle=0;}
	break;
	case 10:
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=11;cnt_idle=0;}
	break;
	case 11:
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=12;cnt_idle=0;}
	break;
//GPS3	
	case 12:
				if(data_rate>5)
			LEDRGB_COLOR(WHITE);
		else
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=13;cnt_idle=0;}
	break;
	case 13:
				if(data_rate>5)
			LEDRGB_COLOR(WHITE);
		else
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=14;cnt_idle=0;}
	break;
	case 14:
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=15;cnt_idle=0;}
	break;
	case 15:
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=16;cnt_idle=0;}
	break;
//MODE
	case 16:
		switch(fly_mode)
	   {
			 case 0:	LEDRGB_COLOR(BLUE);break;//zit
			 case 1:  LEDRGB_COLOR(WHITE);break;//gps
			 case 2:  LEDRGB_COLOR(GREEN);break;//gps
		 }
		
	if(cnt_idle++>RGB_DELAY)
	{idle_state=17;cnt_idle=0;}
	break;
	case 17:
			switch(fly_mode)
	   {
			 case 0:	LEDRGB_COLOR(BLUE);break;//zit
			 case 1:  LEDRGB_COLOR(WHITE);break;//gps
			 case 2:  LEDRGB_COLOR(GREEN);break;//gps
		 }
	if(cnt_idle++>RGB_DELAY)
	{idle_state=18;cnt_idle=0;}
	break;
	case 18:
			switch(mode_control)
	   {
			 case 0:	LEDRGB_COLOR(BLACK);break;//zit
			 case 1:  LEDRGB_COLOR(BLACK);break;//gps
		 }
	if(cnt_idle++>RGB_DELAY)
	{idle_state=19;cnt_idle=0;}
	break;
	case 19:
				switch(mode_control)
	   {
			 case 0:	LEDRGB_COLOR(BLACK);break;//zit
			 case 1:  LEDRGB_COLOR(BLACK);break;//gps
		 }
	if(cnt_idle++>RGB_DELAY)
	{idle_state=20;cnt_idle=0;}
	break;
//-END
	case 20:
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>20)
	{idle_state=0;cnt_idle=0;}
	break;
}

switch(mpu_state){
	case 0:if(main_state==CAL_MPU)
	  {mpu_state=1;LEDRGB_COLOR(YELLOW);}
	       break;
	case 1:
		if(!mpu6050.Gyro_CALIBRATE)
		{	mpu_state=2;LEDRGB_COLOR(BLACK);}
		    break;
	case 2:
		 if(cnt++>200)
		 {mpu_state=0;cnt=0;main_state=IDLE;}
		 break;
	default:mpu_state=0;break;
	 }	 

switch(m_state){
	case 0:if(main_state==CAL_M)
	  {m_state=1;LEDRGB_COLOR(BLUE);}
	       break;
	case 1:
		if(HMC5883_calib==2)
		{	m_state=2;LEDRGB_COLOR(WHITE);}
		    break;
	case 2:
			if(!Mag_CALIBRATED)
		{	m_state=3;LEDRGB_COLOR(GREEN);}
		  else if(HMC5883_calib==1)
	 {	m_state=1;LEDRGB_COLOR(YELLOW);}
		  break;
	case 3:
		 if(cnt++>200)
		 {m_state=0;cnt=0;HMC5883_calib=0;main_state=IDLE;}
		 break;
	default:m_state=0;break;
	 }	 	 
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

