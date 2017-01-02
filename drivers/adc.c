#include "adc.h"
#include "head.h"		 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//ADC ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


//��ʼ��ADC															   
void  Adc_Init(void)
{    
  GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOAʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADC1ʱ��

  //�ȳ�ʼ��ADC1ͨ��5 IO��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;//PA5 ͨ��5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��  
  
	 //�ȳ�ʼ��ADC1ͨ��5 IO��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;//PA5 ͨ��5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��  
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1��λ
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//��λ����	 
 
	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//���������׶�֮����ӳ�5��ʱ��
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMAʧ��
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//��ɨ��ģʽ	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//�ر�����ת��
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1��ת���ڹ��������� Ҳ����ֻת����������1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��
	
 
	ADC_Cmd(ADC1, ENABLE);//����ADת����	

}				  
//���ADCֵ
//ch: @ref ADC_channels 
//ͨ��ֵ 0~16ȡֵ��ΧΪ��ADC_Channel_0~ADC_Channel_16
//����ֵ:ת�����
u16 Get_Adc(u8 ch)   
{
	  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADCͨ��,480������,��߲���ʱ�������߾�ȷ��			    
  
	ADC_SoftwareStartConv(ADC1);		//ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}
//��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ�� 
//ch:ͨ�����
//times:��ȡ����
//����ֵ:ͨ��ch��times��ת�����ƽ��ֵ
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
for(i=1;i<10;i++){	//FIFO ����
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
ADC_FIFO[0][9]=adc0;//���µ����ݷ��õ� ���ݵ������
ADC_FIFO[1][9]=adc1;
ADC_FIFO[2][9]=adc2;
ADC_FIFO[3][9]=adc3;
ADC_FIFO[4][9]=adc4;
ADC_FIFO[5][9]=adc5;
ADC_FIFO[6][9]=adc6;
ADC_FIFO[7][9]=adc7;
ADC_FIFO[8][9]=adc8;
sum=0;
for(i=0;i<10;i++){	//��ǰ����ĺϣ���ȡƽ��ֵ
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
adcx[0]=Get_Adc_Average(ADC_Channel_0,5);//��ȡͨ��5��ת��ֵ��20��ȡƽ��
adcx[1]=Get_Adc_Average(ADC_Channel_1,5);//��ȡͨ��5��ת��ֵ��20��ȡƽ��
adcx[2]=Get_Adc_Average(ADC_Channel_2,5);//��ȡͨ��5��ת��ֵ��20��ȡƽ��

adcx[3]=Get_Adc_Average(ADC_Channel_11,5);//��ȡͨ��5��ת��ֵ��20��ȡƽ��
adcx[4]=Get_Adc_Average(ADC_Channel_12,5);//��ȡͨ��5��ת��ֵ��20��ȡƽ��
adcx[5]=Get_Adc_Average(ADC_Channel_13,5);//��ȡͨ��5��ת��ֵ��20��ȡƽ��
adcx[6]=Get_Adc_Average(ADC_Channel_14,5);//��ȡͨ��5��ת��ֵ��20��ȡƽ��
adcx[7]=Get_Adc_Average(ADC_Channel_15,5);//��ȡͨ��5��ת��ֵ��20��ȡƽ��
adcx[8]=Get_Adc_Average(ADC_Channel_10,5);//��ȡͨ��5��ת��ֵ��20��ȡƽ��
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





