#ifndef _LED_H_
#define	_LED_H_

#include "stm32f4xx.h"

#define LED1_OFF         ANO_GPIO_LED->BSRRL = ANO_Pin_LED1   //H
#define LED1_ON          ANO_GPIO_LED->BSRRH = ANO_Pin_LED1		//L
#define LED2_OFF         ANO_GPIO_LED->BSRRL = ANO_Pin_LED2
#define LED2_ON          ANO_GPIO_LED->BSRRH = ANO_Pin_LED2
#define LED3_OFF         ANO_GPIO_LED->BSRRL = ANO_Pin_LED3
#define LED3_ON          ANO_GPIO_LED->BSRRH = ANO_Pin_LED3
#define LED4_OFF         ANO_GPIO_LED->BSRRL = ANO_Pin_LED4
#define LED4_ON          ANO_GPIO_LED->BSRRH = ANO_Pin_LED4

/***************LED GPIO����******************/
#define ANO_RCC_LED			RCC_AHB1Periph_GPIOE
#define ANO_GPIO_LED		GPIOE
#define ANO_Pin_LED1		GPIO_Pin_3
#define ANO_Pin_LED2		GPIO_Pin_2
#define ANO_Pin_LED3		GPIO_Pin_1
#define ANO_Pin_LED4		GPIO_Pin_0
/*********************************************/

void LED_Init(void);
void LED_Display( u8 []);
void LED_Duty(void);

void LED_MPU_Err(void);
void LED_Mag_Err(void);
void LED_MS5611_Err(void);

extern u8 LED_Brightness[4];


		//���κ꣬��������������һ��ʹ��
#define LED1S(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_0);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_0)
#define LED2S(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_1);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_1)
#define LED3S(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_2);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_2)
#define LED4S(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_3);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_3)	 

#define RED 0
#define BLUE 1
#define GREEN 2
#define YELLOW 3
#define BLACK 4
#define WHITE 5
extern void LEDRGB_STATE(void);
					
					
extern u8 MODE_KEY[8];
extern void READ_MODE(void);
extern void KEY_Init(void);
extern u8 SEL_KEY[4];
extern void READ_SEL(void);
extern void BEEP_Init(void);
extern void BEEP_TASK(u8 sel);					
#endif
