#ifndef _ULTRASONIC_H
#define _ULTRASONIC_H

#include "stm32f4xx.h"

void Ultrasonic_Init(void);
void Ultra_Duty(void);
void Ultra_Get(u8);

extern s8 ultra_start_f;
extern int ultra_distance;
extern int ultra_delta;
extern double x_pred ; // m   0
extern double v_pred ; //       1
#endif


