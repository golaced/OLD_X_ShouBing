/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * ����   �������ƴ�
 * �ļ���  ��main.c
 * ����    ����ѭ��
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/
#include "include.h"
#include "head.h"
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
  while (1)
  {
		
  }
}
#endif
//=======================================================================================


//=======================================================================================
u8 Init_Finish = 0;
int main(void)
{
                    	Init_Finish = All_Init();		
	while(1)
	{
		Duty_Loop(); 

//		if(bat_fly*10<7400)
//		;//BEEP_TASK(2);
//  	else
	
	}
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

