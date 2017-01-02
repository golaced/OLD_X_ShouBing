/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：scheduler.c
 * 描述    ：任务调度
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "scheduler.h"
#include "include.h"
#include "time.h"
#include "mpu6050.h"
#include "ak8975.h"
#include "led.h"
#include "rc.h"
#include "imu.h"
#include "pwm_in.h"
#include "ctrl.h"
#include "ms5611.h"
#include "parameter.h"
#include "ultrasonic.h"
#include "height_ctrl.h"
#include "head.h"
s16 loop_cnt;


loop_t loop;
u8 fly_ready;
void Loop_check()  //TIME INTTERRUPT
{
	loop.time++; //u16
	loop.cnt_2ms++;
	loop.cnt_5ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;

	if( loop.check_flag == 1)
	{
		loop.err_flag ++;     //每累加一次，证明代码在预定周期内没有跑完。
	}
	else
	{	
		loop.check_flag = 1;	//该标志位在循环的最后被清零
	}
}

void Duty_1ms()
{
	Get_Cycle_T(1);
//	LED_Display( LED_Brightness );								//20级led渐变显示
//	ANO_DT_Data_Exchange();												//数传通信定时调用
}

float test[5];
void Duty_2ms()
{
	float inner_loop_time;
	
	inner_loop_time = Get_Cycle_T(0); 						//获取内环准确的执行周期
	
	test[0] = GetSysTime_us()/1000000.0f;
	
	MPU6050_Read(); 															//读取mpu6轴传感器

	MPU6050_Data_Prepare( inner_loop_time );			//mpu6轴传感器数据处理

	//CTRL_1( inner_loop_time ); 										//内环角速度控制。输入：执行周期，期望角速度，测量角速度，角度前馈；输出：电机PWM占空比。<函数未封装>
	
	//RC_Duty( inner_loop_time , Rc_Pwm_In );				// 遥控器通道数据处理 ，输入：执行周期，接收机pwm捕获的数据。
	
	
	
	test[1] = GetSysTime_us()/1000000.0f;
}

void Duty_5ms()
{ static u8 nrf_cnt;
	float outer_loop_time;
	
	outer_loop_time = Get_Cycle_T(2);								//获取外环准确的执行周期
	AD_TASK(6);
	test[2] = GetSysTime_us()/1000000.0f;
	
	/*IMU更新姿态。输入：半个执行周期，三轴陀螺仪数据（转换到度每秒），三轴加速度计数据（4096--1G）；输出：ROLPITYAW姿态角*/
 	IMUupdate(0.5f *outer_loop_time,mpu6050.Gyro_deg.x, mpu6050.Gyro_deg.y, mpu6050.Gyro_deg.z, mpu6050.Acc.x, mpu6050.Acc.y, mpu6050.Acc.z,&Roll,&Pitch,&Yaw);

 	//CTRL_2( outer_loop_time ); 														// 外环角度控制。输入：执行周期，期望角度（摇杆量），姿态角度；输出：期望角速度。<函数未封装>
	if(nrf_cnt++>2)
	{nrf_cnt=0;
	
			RC_Send_Task();	
	}
	//------------
	fly_ready=SEL_KEY[3];
	mode.test1=KEY[2];//6  
	mode.no_head= KEY[3];//1
	mode.mpu6050_bw_42=KEY[4];//5
	mode.en_moto_smooth=KEY[5];//
		//------------
	test[3] = GetSysTime_us()/1000000.0f;
	 GOL_LINK_TASK();
}

void Duty_10ms()
{
 	//	MS5611_Update(); 				//更新ms5611气压计数据
			READ_MODE();
	     READ_SEL();
		//baro_ctrl_start = 1;
	 Nrf_Check_Event();
	 CAL_CHECK();
	  ANO_AK8975_Read();			//获取电子罗盘数据	
	//IMU_AHRSupdate(0,0,0,//mpu6050.Gyro_deg.x* 3.14/180, mpu6050.Gyro_deg.y* 3.14/180, mpu6050.Gyro_deg.z* 3.14/180, 
	//mpu6050.Acc.x, mpu6050.Acc.y, mpu6050.Acc.z,ak8975.Mag_Val.x ,ak8975.Mag_Val.y,-ak8975.Mag_Val.z);
	//GPS_calc_poshold();
}

void Duty_20ms()
{ 
	//Parameter_Save();
}

void Duty_50ms()
{
static u8 flag=0,cnt=0,cnt_1s,state;
u8 i;	
	//Mode();
						//LED任务
	LEDRGB_STATE();	
LED_Duty();		
	//Ultra_Duty();
//	if(bat_fly<740||Rc_Get.AUX5<3555)
//		;///BEEP_TASK(2);
//	//else if(Rc_Get.AUX5<3555)
//		//;//BEEP_TASK(1);
	switch(state){
		case 0:
			if(Rc_Get.THROTTLE<1500+THR_DEAD && Rc_Get.THROTTLE>1500-THR_DEAD)
			{	BEEP_TASK(1);state=1;}
		
		break;
		case 1:
			if(cnt++>2){cnt=0;state=2;BEEP_TASK(0);}
			break;
		case 2:
			if(Rc_Get.THROTTLE>1500+THR_DEAD ||Rc_Get.THROTTLE<1500-THR_DEAD)
			{	state=0;}
			break;
	}
		
		
//	if(Rc_Get.THROTTLE<1500+THR_DEAD && Rc_Get.THROTTLE>1500-THR_DEAD)
//	{
//	if(cnt++>1){cnt=0;
//	flag=!flag;
//	BEEP_TASK(flag);
//	}
//	}
//	else
//	BEEP_TASK(0);	
			if(Rc_Get.AUX5<3555)
		BEEP_TASK(1);
	if(cnt_1s++>20)
	{
	cnt_1s=0;
		for(i=0;i<10;i++)
		 rc_rate_cnt[i]=0;
	
	}
}


void Duty_Loop()   					//最短任务周期为1ms，总的代码执行时间需要小于1ms。
{

	if( loop.check_flag == 1 )
	{
		loop_cnt = time_1ms;
		
		Duty_1ms();							//周期1ms的任务
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//周期2ms的任务
		}
		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;
			Duty_5ms();						//周期5ms的任务
		}
		if( loop.cnt_10ms >= 10 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//周期10ms的任务
		}
		if( loop.cnt_20ms >= 20 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//周期20ms的任务
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//周期50ms的任务
		}
		
		loop.check_flag = 0;		//循环运行完毕标志
	}
}




	/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
	

