#include "adc.h"
#include "head.h"		 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//ADC 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


//初始化ADC															   
void  Adc_Init(void)
{    
  GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOA时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟

  //先初始化ADC1通道5 IO口
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;//PA5 通道5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化  
  
	 //先初始化ADC1通道5 IO口
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;//PA5 通道5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化  
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 
 
	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//初始化
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
	
 
	ADC_Cmd(ADC1, ENABLE);//开启AD转换器	

}				  
//获得ADC值
//ch: @ref ADC_channels 
//通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
//返回值:转换结果
u16 Get_Adc(u8 ch)   
{
	  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}
//获取通道ch的转换值，取times次,然后平均 
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	//for(t=0;t<times;t++)
	//{
		temp_val=Get_Adc(ch);
	//	delay_ms(5);
	//}
	return temp_val;///times;
} 
	 
u16  ADC_FIFO[9][11];
void  ADC_NEW_DATA(u16 adc0,u16 adc1,u16 adc2,u16 adc3,u16 adc4,u16 adc5,u16 adc6,u16 adc7,u16 adc8)
{
unsigned char i ;
int32_t sum=0;
for(i=1;i<10;i++){	//FIFO 操作
ADC_FIFO[0][i-1]=ADC_FIFO[0][i];
ADC_FIFO[1][i-1]=ADC_FIFO[1][i];
ADC_FIFO[2][i-1]=ADC_FIFO[2][i];
ADC_FIFO[3][i-1]=ADC_FIFO[3][i];
ADC_FIFO[4][i-1]=ADC_FIFO[4][i];
ADC_FIFO[5][i-1]=ADC_FIFO[5][i];
ADC_FIFO[6][i-1]=ADC_FIFO[6][i];
ADC_FIFO[7][i-1]=ADC_FIFO[7][i];
ADC_FIFO[8][i-1]=ADC_FIFO[8][i];

}
ADC_FIFO[0][9]=adc0;//将新的数据放置到 数据的最后面
ADC_FIFO[1][9]=adc1;
ADC_FIFO[2][9]=adc2;
ADC_FIFO[3][9]=adc3;
ADC_FIFO[4][9]=adc4;
ADC_FIFO[5][9]=adc5;
ADC_FIFO[6][9]=adc6;
ADC_FIFO[7][9]=adc7;
ADC_FIFO[8][9]=adc8;
sum=0;
for(i=0;i<10;i++){	//求当前数组的合，再取平均值
   sum+=ADC_FIFO[0][i];
}
ADC_FIFO[0][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=ADC_FIFO[1][i];
}
ADC_FIFO[1][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=ADC_FIFO[2][i];
}
ADC_FIFO[2][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=ADC_FIFO[3][i];
}
ADC_FIFO[3][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=ADC_FIFO[4][i];
}
ADC_FIFO[4][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=ADC_FIFO[5][i];
}
ADC_FIFO[5][10]=sum/10;
sum=0;
for(i=0;i<10;i++){
   sum+=ADC_FIFO[6][i];
}
ADC_FIFO[6][10]=sum/10;
sum=0;
for(i=0;i<10;i++){
   sum+=ADC_FIFO[7][i];
}
ADC_FIFO[7][10]=sum/10;
sum=0;
for(i=0;i<10;i++){
   sum+=ADC_FIFO[8][i];
}
ADC_FIFO[8][10]=sum/10;


}


#define ADC_MAX 4090
#define ADC_AUX_MAX 1000
u16 adcx_out[9];
void AD_TASK(u8 times){
u16 adcx[9];
adcx[0]=Get_Adc_Average(ADC_Channel_0,5);//获取通道5的转换值，20次取平均
adcx[1]=Get_Adc_Average(ADC_Channel_1,5);//获取通道5的转换值，20次取平均
adcx[2]=Get_Adc_Average(ADC_Channel_2,5);//获取通道5的转换值，20次取平均

adcx[3]=Get_Adc_Average(ADC_Channel_11,5);//获取通道5的转换值，20次取平均
adcx[4]=Get_Adc_Average(ADC_Channel_12,5);//获取通道5的转换值，20次取平均
adcx[5]=Get_Adc_Average(ADC_Channel_13,5);//获取通道5的转换值，20次取平均
adcx[6]=Get_Adc_Average(ADC_Channel_14,5);//获取通道5的转换值，20次取平均
adcx[7]=Get_Adc_Average(ADC_Channel_15,5);//获取通道5的转换值，20次取平均
adcx[8]=Get_Adc_Average(ADC_Channel_10,5);//获取通道5的转换值，20次取平均
ADC_NEW_DATA(adcx[0],adcx[1],adcx[2],adcx[3],adcx[4],adcx[5],adcx[6],adcx[7],adcx[8]);
Rc_Get.AUX5=adcx_out[0]=(float)ADC_FIFO[0][10]/1405*7070/2;//POWER
Rc_Get.AUX1=adcx_out[1]=(float)ADC_FIFO[1][10]/ADC_MAX*ADC_AUX_MAX;
Rc_Get.AUX2=adcx_out[2]=(float)ADC_FIFO[2][10]/ADC_MAX*ADC_AUX_MAX;
Rc_Get.AUX3=adcx_out[6]=(float)ADC_FIFO[6][10]/ADC_MAX*ADC_AUX_MAX;
Rc_Get.AUX4=adcx_out[7]=(float)ADC_FIFO[7][10]/ADC_MAX*ADC_AUX_MAX;
	
Rc_Get.PITCH=adcx_out[3]=(float)ADC_FIFO[3][10]/ADC_MAX*1000+1000;
Rc_Get.YAW=adcx_out[4]=(float)ADC_FIFO[4][10]/ADC_MAX*1000+1000;
Rc_Get.THROTTLE=adcx_out[5]=(float)ADC_FIFO[5][10]/ADC_MAX*1000+1000;
//if(Rc_Get.THROTTLE<1500+THR_DEAD&&Rc_Get.THROTTLE>1500-THR_DEAD)
//Rc_Get.THROTTLE=1500;
Rc_Get.ROLL=adcx_out[8]=(float)ADC_FIFO[8][10]/ADC_MAX*1000+1000;

}





