#ifndef __SPI_H
#define __SPI_H
#include "head.h"

#define SPI_CE_H()   GPIO_SetBits(GPIOB, GPIO_Pin_5) 
#define SPI_CE_L()   GPIO_ResetBits(GPIOB, GPIO_Pin_5)

#define SPI_CSN_H()  GPIO_SetBits(GPIOB, GPIO_Pin_4)
#define SPI_CSN_L()  GPIO_ResetBits(GPIOB, GPIO_Pin_4)

void Spi1_Init(void);
u8 Spi_RW(u8 dat);
		 
#endif


