/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：init.c
 * 描述    ：飞控初始化
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
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
	NVIC_PriorityGroupConfig(NVIC_GROUP);		//中断优先级组别设置
	
	SysTick_Configuration(); 	//滴答时钟
	
	I2c_Soft_Init();					//初始化模拟I2C
	KEY_Init();
	BEEP_Init();
  Spi1_Init();	
		Adc_Init();

//---------------------------------------	
//	MS5611_Init();						//气压计初始化
	
	Delay_ms(200);						//延时
	
	MPU6050_Init(5);   			//加速度计、陀螺仪初始化，配置20hz低通
			Delay_ms(100);	
		HMC5883L_SetUp();
	LED_Init();								//LED功能初始化
	
	Usart2_Init(115200);			//串口2初始化，函数参数为波特率
	//Usart2_Init(256000);

	
	
	//Delay_ms(100);						//延时
	
	//Ultrasonic_Init();   			//超声波初始化
	
	

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
