#include "include.h"
#include "ultrasonic.h"
#include "usart.h"
#include "filter.h"
#include "IMU.h"
void Ultrasonic_Init()
{
  Uart5_Init(9600);			//串口5初始化，函数参数为波特率
	
}

s8 ultra_start_f;
u8 ultra_time;
u8 ultra_ok = 0;
void Ultra_Duty()
{
	u8 temp[3];

	ultra_time++;
	ultra_time = ultra_time%2;
	
	
	if( ultra_time == 0 )  //100ms//改用发送中断，节省时间。
	{
//////////////////////////////////////////////
/*		UART5->DR = 0xe8;   //ks103地址（可设置）
		while( (UART5->SR & USART_FLAG_TXE) == 0 );
		
		UART5->DR = 0x02;   //++++
		while( (UART5->SR & USART_FLAG_TXE) == 0 );

		UART5->DR = 0xbc;  //70ms,带温度补偿
		while( (UART5->SR & USART_FLAG_TXE) == 0 );*/
//////////////////////////////////////////////	
	#if defined(USE_KS103)
		temp[0] = 0xe8;
		temp[1] = 0x02;
		temp[2] = 0xbc;
		Uart5_Send(temp ,3);
	#elif defined(USE_US100)
		temp[0] = 0x55;
		Uart5_Send(temp ,1);
	#endif
///////////////////////////////////////////////
		ultra_start_f = 1;
	}
}


// float t1r=1;
/* kalman filter states */
double x_pred = 0.0f; // m   0
double v_pred = 0.0f; //       1
double x_post = 0.0f; // m    2
double v_post = 0.0f; // m/s  3
float sonar_raw = 0.0f;  // m
float scale_kal_sonar_v=1;
float sonar_filter(float hight,float dt_sonar)
{float x_new;
 static float reg;
	float LPF_1=0.75; 
	float MAX_SPEED=0.5;
	/* no data for long time */
	if (dt_sonar > 0.25f) // more than 2 values lost
	{
		v_pred = 0;
	}

	x_pred = x_post + dt_sonar * v_pred;
	v_pred = v_post;
   v_pred=limit_mine	(v_pred,MAX_SPEED);
	 if(fabs(v_pred) < 0.01)           \
    v_pred = 0;   
	 
	 v_pred=reg*(1-LPF_1)+v_pred*(LPF_1);
	 reg=v_pred;
	 x_new = hight;
	sonar_raw = x_new;
	x_post = x_pred +  0.91* (x_new - x_pred);//0.8461f
	v_post = v_pred +  6.2034f* (x_new - x_pred)*scale_kal_sonar_v;
  v_post=limit_mine(v_post,MAX_SPEED);
	  if(fabs(v_post) < 0.01)           \
    v_post = 0;   
	return x_pred;//m/s
}


int ultra_distance;
int ultra_delta;
void Ultra_Get(u8 com_data)
{
	static u8 ultra_tmp;
	int temp1,temp2,temp;
	float dt;
	static int ultra_distance_old;
	if( ultra_start_f == 1 )
	{
		ultra_tmp = com_data;
		ultra_start_f = 2;
	}
	else if( ultra_start_f == 2 )
	{
		temp = (ultra_tmp<<8) + com_data;
		ultra_start_f = 0; 
		dt=Get_Cycle_T(3);
		temp1=limit_mine(Roll,45);
		temp2=limit_mine(Pitch,45);
		temp=temp*cos(temp1*0.017)*cos(temp2*0.017);
	  temp=((temp)<(0)?(0):((temp)>(3500)?(3500):(temp)));
   	sonar_filter((float)temp/1000.,dt);
	  ultra_distance=((x_pred*1000)<(0)?(0):((x_pred*1000)>(3500)?(3500):(x_pred*1000)));
		ultra_ok = 1;
	}
	 
	
	 ultra_delta = ultra_distance - ultra_distance_old;
	
	 ultra_distance_old = ultra_distance;
	
}


