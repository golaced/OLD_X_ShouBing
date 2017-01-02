/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * ����   �������ƴ�
 * �ļ���  ��ms5611.c
 * ����    ����ѹ������
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/

#include "ms5611.h"
#include "math.h"
#include "filter.h"

#define BARO_CAL_CNT 200

	int32_t baroAlt,baroAltOld;
	float baro_alt_speed;
	int32_t baro_Offset;
	uint32_t ms5611_ut;  // static result of temperature measurement
	uint32_t ms5611_up;  // static result of pressure measurement
	uint16_t ms5611_prom[PROM_NB];  // on-chip ROM
	uint8_t t_rxbuf[3],p_rxbuf[3];

void MS5611_Reset(void)
{
    IIC_Write_1Byte(MS5611_ADDR, CMD_RESET, 1);
}

u8 MS5611_Read_Prom(void)
{
	uint8_t rxbuf[2] = { 0, 0 };
	u8 check = 0;
	u8 i;

	for (i = 0; i < PROM_NB; i++)
	{
		check += IIC_Read_nByte(MS5611_ADDR, CMD_PROM_RD + i * 2, 2, rxbuf); // send PROM READ command
		ms5611_prom[i] = rxbuf[0] << 8 | rxbuf[1];
	}

	if(check==PROM_NB)
		return 1;
	else
		return 0;
}


void MS5611_Read_Adc_T(void)
{
	IIC_Read_nByte( MS5611_ADDR, CMD_ADC_READ, 3, t_rxbuf ); // read ADC
}

void MS5611_Read_Adc_P(void)
{
	IIC_Read_nByte(MS5611_ADDR, CMD_ADC_READ, 3, p_rxbuf); // read ADC
}

void MS5611_Start_T(void)
{
	IIC_Write_1Byte(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + MS5611_OSR, 1); // D2 (temperature) conversion start!
}

void MS5611_Start_P(void)
{
  IIC_Write_1Byte(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + MS5611_OSR, 1); // D1 (pressure) conversion start!
}

u8 ms5611_ok;
void MS5611_Init(void)
{
	
	Delay_ms(10);
	//��������λ
	MS5611_Reset();
	Delay_ms(3);
	ms5611_ok = !( MS5611_Read_Prom() );
	//��ʼ��ȡ�¶�
	MS5611_Start_T();
}

void MS5611_Update(void)
{
	static int state = 0;
	
//	I2C_FastMode = 0;
	
	if (state) 
	{
			MS5611_Read_Adc_P();
			MS5611_Start_T();
			MS5611_BaroAltCalculate();
			state = 0;
	} 
	else 
	{
			MS5611_Read_Adc_T();
			MS5611_Start_P();
			state = 1;
	}
}
#define MOVAVG_SIZE   32  //��������� MOVAVG_SIZE ��ֵ 
//�Ƚ��ȳ�����������
int32_t  Temp_buffer[MOVAVG_SIZE],Press_buffer[MOVAVG_SIZE],Alt_buffer[MOVAVG_SIZE];
uint8_t temp_index=0,press_index=0,alt_index=0; //����ָ��
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void BMP180_newALT(int32_t A)
*��������:		����һ���µ�ֵ���߶ȹ�����
*******************************************************************************/
int32_t last_Temperature,last_Pressure,last_Alt;

int32_t MS561101BA_getAvg(int32_t * buff, int size) {
  float sum = 0.0;
  int i;
  for(i=0; i<size; i++) {
    sum += buff[i];
  }
  return sum / size;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void BMP180_newALT(int32_t A)
*��������:		����һ���µ�ֵ���߶ȹ�����
*******************************************************************************/
void BMP180_newALT(int32_t val)
{	
  Alt_buffer[alt_index] = val;
  alt_index = (alt_index + 1) % MOVAVG_SIZE;

  last_Alt = MS561101BA_getAvg(Alt_buffer,MOVAVG_SIZE);	//ȡƽ��ֵ
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void BMP180_newTemperature(int32_t T)
*��������:		����һ���µ�ֵ���¶ȹ�����
*******************************************************************************/
void BMP180_newTemperature(int32_t val)
{	
  Temp_buffer[temp_index] = val;
  temp_index = (temp_index + 1) % MOVAVG_SIZE;

  last_Temperature=MS561101BA_getAvg(Temp_buffer,MOVAVG_SIZE);	//ȡƽ��ֵ
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void BMP180_newPressure(int32_t P)
*��������:		����һ���µ�ֵ����ѹ������
*******************************************************************************/
void BMP180_newPressure(int32_t val)
{	
  Press_buffer[press_index] = val;
  press_index = (press_index + 1) % MOVAVG_SIZE;

  last_Pressure = MS561101BA_getAvg(Press_buffer,MOVAVG_SIZE);	//ȡƽ��ֵ
}

float temperature_5611;
void MS5611_BaroAltCalculate(void)
{
	static u8 baro_start;
	
  int32_t temperature, off2 = 0, sens2 = 0, delt;
  int32_t pressure;
	float alt_3;
	
	int32_t dT;
	int64_t off;
	int64_t sens;
	
		static vs32 sum_tmp_5611 = 0;
		static u8 sum_cnt = BARO_CAL_CNT + 10;
	
		ms5611_ut = (t_rxbuf[0] << 16) | (t_rxbuf[1] << 8) | t_rxbuf[2];
		ms5611_up = (p_rxbuf[0] << 16) | (p_rxbuf[1] << 8) | p_rxbuf[2];
		
    dT = ms5611_ut - ((uint32_t)ms5611_prom[5] << 8);
    off = ((uint32_t)ms5611_prom[2] << 16) + (((int64_t)dT * ms5611_prom[4]) >> 7);
    sens = ((uint32_t)ms5611_prom[1] << 15) + (((int64_t)dT * ms5611_prom[3]) >> 8);
    temperature = 2000 + (((int64_t)dT * ms5611_prom[6]) >> 23);
    BMP180_newTemperature(temperature);
    if (temperature < 2000) { // temperature lower than 20degC 
        delt = last_Temperature - 2000;
        delt = delt * delt;
        off2 = (5 * delt) >> 1;
        sens2 = (5 * delt) >> 2;
        if (last_Temperature < -1500) { // temperature lower than -15degC
            delt = last_Temperature + 1500;
            delt = delt * delt;
            off2  += 7 * delt;
            sens2 += (11 * delt) >> 1;
        }
    }
    off  -= off2; 
    sens -= sens2;
    pressure = (((ms5611_up * sens ) >> 21) - off) >> 15;
		//pressure = (int)((1.0f - pow(pressure / 101325.0f, 0.190295f)) * 4433000.0f); // centimeter
		
		alt_3 = (101000 - pressure)/1000.0f;
		pressure = 0.0082f *alt_3 * alt_3 *alt_3 + 0.09f *(101000 - pressure)*100.0f ;
    BMP180_newPressure(pressure);
							// - ( temperature_5611  ) *650 *(pressure/101000.0);
		
// 		if( pressure < 101000 )
// 		{
// 			pressure = 80 *my_pow( (101000 - pressure)/1000.0 ) + 0.08 *(101000 - pressure)*100.0;
// 			
// 		}
// 		else
// 		{
// 			pressure = baro_Offset;
// 		}
		
		//if(!(baro_Offset == 0))

				
			
			BMP180_newALT(10 *(s32)( 0.1f *( last_Pressure - baro_Offset) ) ); 
			//baroAlt = 10 *(s32)( 0.1f *( pressure - baro_Offset) ) ; //cm
			baroAlt=last_Alt;
			baro_alt_speed += 5 *0.02 *3.14 *( 50 *( baroAlt - baroAltOld ) - baro_alt_speed ); // 20ms һ�� /0.02 = *50 ��λcm/s
			
		  baroAltOld = baroAlt;
			
			if( baro_start < 100 )
			{
				baro_start++;
				baro_alt_speed = 0;
				baroAlt = 0;
			}	
			
		if(sum_cnt)
		{
			sum_cnt--;
// 			if(sum_cnt < BARO_CAL_CNT)
// 				sum_tmp_5611 += pressure;
// 			if(sum_cnt==0)
// 				baro_Offset = sum_tmp_5611 / (BARO_CAL_CNT - 1);
		}
			
		temperature_5611 += 0.01f *( ( 0.01f *last_Temperature ) - temperature_5611 );
}



int32_t MS5611_Get_BaroAlt(void)
{
	return baroAlt;
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
