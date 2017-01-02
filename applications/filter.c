#include "include.h"
#include "filter.h"

// #define WIDTH_NUM 101
// #define FIL_ITEM  10

 void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out)
{
	u16 width_num;
	
	width_num = len ;
	
	if( ++fil_cnt[0] > width_num )	
	{
		fil_cnt[0] = 0; //now
		fil_cnt[1] = 1; //old
	}
	else
	{
		fil_cnt[1] = (fil_cnt[0] == width_num)? 0 : (fil_cnt[0] + 1);
	}
	
	moavarray[ fil_cnt[0] ] = in;
	*out += ( in - ( moavarray[ fil_cnt[1] ]  ) )/(float)( width_num ) ;
	
}


// float filter_tmp[FIL_ITEM][WIDTH_NUM ];
// float filter_out[FIL_ITEM];

// u8 fil_cnt[FIL_ITEM],fil_cnt_old[FIL_ITEM];

// float Moving_Average(u8 item,u8 width_num,float in)
// {
// 	if(item >= FIL_ITEM || width_num >= WIDTH_NUM )
// 	{
// 		return 0;
// 	}
// 	else
// 	{
// 		if( ++fil_cnt[item] > width_num )	
// 		{
// 			fil_cnt[item] = 0;
// 			fil_cnt_old[item] = 1;
// 		}
// 		else
// 		{
// 			fil_cnt_old[item] = (fil_cnt[item] == width_num)? 0 : (fil_cnt[item] + 1);
// 		}
// 		
// 		filter_tmp[item][ fil_cnt[item] ] = in;
// 		filter_out[item] += ( in - ( filter_tmp[item][ fil_cnt_old[item] ]  ) )/(float)( width_num ) ;//+ 0.01 *filter_out[item]
// 								//test_array( array ); /////////////////
// 		return ( filter_out[item] );
// 	}

// 	
// }



#define MED_WIDTH_NUM 11
#define MED_FIL_ITEM  2

float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM ];
float med_filter_out[MED_FIL_ITEM];

u8 med_fil_cnt[MED_FIL_ITEM];

float Moving_Median(u8 item,u8 width_num,float in)
{
	u8 i,j;
	float t;
	float tmp[MED_WIDTH_NUM];
	
	if(item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM )
	{
		return 0;
	}
	else
	{
		if( ++med_fil_cnt[item] >= width_num )	
		{
			med_fil_cnt[item] = 0;
		}
		
		med_filter_tmp[item][ med_fil_cnt[item] ] = in;
		
		for(i=0;i<width_num;i++)
		{
			tmp[i] = med_filter_tmp[item][i];
		}
		
		for(i=0;i<width_num-1;i++)
		{
			for(j=0;j<(width_num-1-i);j++)
			{
				if(tmp[j] > tmp[j+1])
				{
					t = tmp[j];
					tmp[j] = tmp[j+1];
					tmp[j+1] = t;
				}
			}
		}

		
		return ( tmp[(u16)width_num/2] );
	}
}


/*====================================================================================================*/
/*====================================================================================================*
** ��������: IIR_I_Filter
** ��������: IIRֱ��I���˲���
** ��    ��: InData Ϊ��ǰ����
**           *x     ����δ�˲�������
**           *y     �����˲��������
**           *b     ����ϵ��b
**           *a     ����ϵ��a
**           nb     ����*b�ĳ���
**           na     ����*a�ĳ���
**           LpfFactor
** ��    ��: OutData         
** ˵    ��: ��
** ����ԭ��: y(n) = b0*x(n) + b1*x(n-1) + b2*x
n-2) -
                    a1*y(n-1) - a2*y(n-2)
**====================================================================================================*/
/*====================================================================================================*/
double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na)
{
  double z1,z2;
  short i;
  double OutData;
  
  for(i=nb-1; i>0; i--)
  {
    x[i]=x[i-1];
  }
  
  x[0] = InData;
  
  for(z1=0,i=0; i<nb; i++)
  {
    z1 += x[i]*b[i];
  }
  
  for(i=na-1; i>0; i--)
  {
    y[i]=y[i-1];
  }
  
  for(z2=0,i=1; i<na; i++)
  {
    z2 += y[i]*a[i];
  }
  
  y[0] = z1 - z2; 
  OutData = y[0];
    
  return OutData;
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : KalmanFilter
**���� : �������˲�
**���� :  
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R,double x_last,double p_last)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   double x_mid = x_last;
   double x_now;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last;          //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q;        //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R);    //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;   //����ֵ��Ӧ��covariance       
   p_last = p_now;       //����covarianceֵ
   x_last = x_now;       //����ϵͳ״ֵ̬
   return x_now;                
}

/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
fp32 LPF_1st(fp32 oldData, fp32 newData, fp32 lpf_factor)
{
	return oldData * (1 - lpf_factor) + newData * lpf_factor;
}