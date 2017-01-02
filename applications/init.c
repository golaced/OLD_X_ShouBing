/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * ����   �������ƴ�
 * �ļ���  ��init.c
 * ����    ���ɿس�ʼ��
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/

#include "include.h"
#include "pwm_out.h"
#include "mpu6050.h"
#include "i2c_soft.h"
#include "led.h"
#include "ctrl.h"
#include "ms5611.h"
#include "ak8975.h"
#include "ultrasonic.h"
#include "head.h"
u8 All_Init()
{
	NVIC_PriorityGroupConfig(NVIC_GROUP);		//�ж����ȼ��������
	
	SysTick_Configuration(); 	//�δ�ʱ��
	
	I2c_Soft_Init();					//��ʼ��ģ��I2C
	KEY_Init();
	BEEP_Init();
  Spi1_Init();	
		Adc_Init();

//---------------------------------------	
//	MS5611_Init();						//��ѹ�Ƴ�ʼ��
	
	Delay_ms(200);						//��ʱ
	
	MPU6050_Init(5);   			//���ٶȼơ������ǳ�ʼ��������20hz��ͨ
			Delay_ms(100);	
		HMC5883L_SetUp();
	LED_Init();								//LED���ܳ�ʼ��
	
	Usart2_Init(115200);			//����2��ʼ������������Ϊ������
	//Usart2_Init(256000);

	
	
	//Delay_ms(100);						//��ʱ
	
	//Ultrasonic_Init();   			//��������ʼ��
	
	

	//ak8975_ok =1;// !(ANO_AK8975_Run());
	

		
	Nrf24l01_Init(MODEL_TX2,40);// ???  ???
	if(!Nrf24l01_Check())
	{
	while(1){	
	BEEP_TASK(1);
  Delay_ms(500);
	BEEP_TASK(0);
	}
	}
//	BEEP_TASK(1);
//  Delay_ms(150);
//	BEEP_TASK(0);
//	Delay_ms(150);
//	BEEP_TASK(1);
//  Delay_ms(150);
//	BEEP_TASK(0);
	Cycle_Time_Init();
	//	Delay_ms(100);	
	BEEP_TASK(0);
	return (1);
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
