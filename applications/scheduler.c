/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * ����   �������ƴ�
 * �ļ���  ��scheduler.c
 * ����    ���������
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
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
		loop.err_flag ++;     //ÿ�ۼ�һ�Σ�֤��������Ԥ��������û�����ꡣ
	}
	else
	{	
		loop.check_flag = 1;	//�ñ�־λ��ѭ�����������
	}
}

void Duty_1ms()
{
	Get_Cycle_T(1);
//	LED_Display( LED_Brightness );								//20��led������ʾ
//	ANO_DT_Data_Exchange();												//����ͨ�Ŷ�ʱ����
}

float test[5];
void Duty_2ms()
{
	float inner_loop_time;
	
	inner_loop_time = Get_Cycle_T(0); 						//��ȡ�ڻ�׼ȷ��ִ������
	
	test[0] = GetSysTime_us()/1000000.0f;
	
	MPU6050_Read(); 															//��ȡmpu6�ᴫ����

	MPU6050_Data_Prepare( inner_loop_time );			//mpu6�ᴫ�������ݴ���

	//CTRL_1( inner_loop_time ); 										//�ڻ����ٶȿ��ơ����룺ִ�����ڣ��������ٶȣ��������ٶȣ��Ƕ�ǰ������������PWMռ�ձȡ�<����δ��װ>
	
	//RC_Duty( inner_loop_time , Rc_Pwm_In );				// ң����ͨ�����ݴ��� �����룺ִ�����ڣ����ջ�pwm��������ݡ�
	
	
	
	test[1] = GetSysTime_us()/1000000.0f;
}

void Duty_5ms()
{ static u8 nrf_cnt;
	float outer_loop_time;
	
	outer_loop_time = Get_Cycle_T(2);								//��ȡ�⻷׼ȷ��ִ������
	AD_TASK(6);
	test[2] = GetSysTime_us()/1000000.0f;
	
	/*IMU������̬�����룺���ִ�����ڣ��������������ݣ�ת������ÿ�룩��������ٶȼ����ݣ�4096--1G���������ROLPITYAW��̬��*/
 	IMUupdate(0.5f *outer_loop_time,mpu6050.Gyro_deg.x, mpu6050.Gyro_deg.y, mpu6050.Gyro_deg.z, mpu6050.Acc.x, mpu6050.Acc.y, mpu6050.Acc.z,&Roll,&Pitch,&Yaw);

 	//CTRL_2( outer_loop_time ); 														// �⻷�Ƕȿ��ơ����룺ִ�����ڣ������Ƕȣ�ҡ����������̬�Ƕȣ�������������ٶȡ�<����δ��װ>
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
 	//	MS5611_Update(); 				//����ms5611��ѹ������
			READ_MODE();
	     READ_SEL();
		//baro_ctrl_start = 1;
	 Nrf_Check_Event();
	 CAL_CHECK();
	  ANO_AK8975_Read();			//��ȡ������������	
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
						//LED����
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


void Duty_Loop()   					//�����������Ϊ1ms���ܵĴ���ִ��ʱ����ҪС��1ms��
{

	if( loop.check_flag == 1 )
	{
		loop_cnt = time_1ms;
		
		Duty_1ms();							//����1ms������
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//����2ms������
		}
		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;
			Duty_5ms();						//����5ms������
		}
		if( loop.cnt_10ms >= 10 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//����10ms������
		}
		if( loop.cnt_20ms >= 20 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//����20ms������
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//����50ms������
		}
		
		loop.check_flag = 0;		//ѭ��������ϱ�־
	}
}




	/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
	

