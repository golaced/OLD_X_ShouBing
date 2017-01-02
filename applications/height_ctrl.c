#include "height_ctrl.h"
#include "ultrasonic.h"
#include "mymath.h"
#include "filter.h"
#include "ctrl.h"
#include "ms5611.h"
#include "rc.h"

_st_height_pid_v wz_speed_pid_v;
_st_height_pid wz_speed_pid;

float baro_speed;

float height_ctrl_out;
float wz_acc;

#define BARO_SPEED_NUM 100
float baro_speed_arr[BARO_SPEED_NUM + 1];
u16 baro_cnt;


void Height_Ctrl(float T,float thr)
{	
	static float wz_speed_t;
	static u8 height_ctrl_start_f;
	static u16 hc_start_delay;
	
	switch( height_ctrl_start_f )
	{
		case 0:
	
		if( mpu6050.Acc.z > 4000 )
		{
			height_ctrl_start_f = 1;
			
			if( ++hc_start_delay > 500 )
			{
				height_ctrl_start_f = 1;
			}
		}
		break;
		
		case 1:
		
		wz_acc += ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) ) *my_deathzoom( (reference_v.z *mpu6050.Acc.z + reference_v.x *mpu6050.Acc.x + reference_v.y *mpu6050.Acc.y - 4096 - wz_acc),100 );
		
		wz_speed_t += ( 1 / ( 1 + 1 / ( 0.5f *3.14f *T ) ) ) *(0.4f*(thr-500) - wz_speed_t);
		
		Moving_Average( (float)( baro_alt_speed *10),baro_speed_arr,BARO_SPEED_NUM,&baro_cnt ,&baro_speed ); //单位mm/s
// 		if( baro_alt_speed > 2000 )
// 		{
// 			while(1);
// 		}
		
		if( height_ctrl_mode == 1)
		{
			//height_speed_ctrl(T,thr,0.8f*(thr-500),wz_speed_t);
			height_speed_ctrl(T,thr,my_deathzoom( 1.6f *(thr-500), 50 ),baro_speed);
			
		
			if(baro_ctrl_start==1)
			{
				baro_ctrl_start = 0;
				Baro_Ctrl(0.02,thr);
			}		
		}
		


		
		else if( height_ctrl_mode == 2)
		{
			height_speed_ctrl(T,thr,0.4f*ultra_ctrl_out,ultra_speed);
			
			if( ultra_start_f == 0 )
			{
				Ultra_Ctrl(0.1f,thr);//超声波周期100ms
				ultra_start_f = -1;
			}
		}
		
	////////////////////////////////////////////////////////////	
		if(height_ctrl_mode)
		{		
			height_ctrl_out = wz_speed_pid_v.pid_out;
		}
		else
		{
			height_ctrl_out = thr;
		}
		
		break; //case 1
		
		default: break;
		
	} //switch
}


void WZ_Speed_PID_Init()
{
	wz_speed_pid.kp = 0.3; 
	wz_speed_pid.kd = 1.4; 
	wz_speed_pid.ki = 0.12; 
}

float wz_speed;

float wz_acc_mms2;
void height_speed_ctrl(float T,float thr,float exp_z_speed,float h_speed)
{
	static float thr_lpf;
	float height_thr;
	static float lpf_tmp,hc_speed_i,hc_speed_i_2,wz_speed_0,wz_speed_1,wz_speed_2;
	
	height_thr = LIMIT( 2 * thr , 0, 600 );
	
	thr_lpf += ( 1 / ( 1 + 1 / ( 2.0f *3.14f *T ) ) ) *( height_thr - thr_lpf );
	
	wz_acc_mms2 = (wz_acc/4096.0f) *10000 ;//9800 *T;
	
	
	
	wz_speed_0 += wz_acc_mms2 *T;
	
	hc_speed_i += 0.4f *T *( h_speed - wz_speed_1 );
	hc_speed_i = LIMIT( hc_speed_i, -1500, 1500 );	
	
	wz_speed_0 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( h_speed - wz_speed_0  ) ;
	wz_speed_1 = wz_speed_0 + hc_speed_i;
	
	if( ABS( wz_speed_1 ) < 50 )
	{
		wz_speed_1 = 0;
	}
	
	wz_speed_2 += wz_acc_mms2 *T;
	

		lpf_tmp += 0.4f *T *( wz_speed_1 - wz_speed ); 
		lpf_tmp = LIMIT( lpf_tmp, -1500, 1500 ); 

	hc_speed_i_2 += 0.01f *T *( wz_speed_1 - wz_speed_2 ); 
	hc_speed_i_2 = LIMIT( hc_speed_i_2, -500, 500 );	
	
	wz_speed_2 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( wz_speed_1 - wz_speed_2 + hc_speed_i_2 ) ;//*(baro_speed - wz_speed);
	wz_speed = wz_speed_2 + lpf_tmp;
	
// 	if( wz_speed_0 > 2000 )
// 	{
// 		while(1);
// 	}
	
	wz_speed_pid_v.err = wz_speed_pid.kp *( exp_z_speed - wz_speed );
	wz_speed_pid_v.err_d = 10*wz_speed_pid.kd * (-wz_acc_mms2) *T;//(wz_speed_pid_v.err - wz_speed_pid_v.err_old);
	
	//wz_speed_pid_v.err_i += wz_speed_pid.ki *wz_speed_pid_v.err *T;
	wz_speed_pid_v.err_i += wz_speed_pid.ki *wz_speed_pid.kp *( exp_z_speed - h_speed ) *T;
	wz_speed_pid_v.err_i = LIMIT(wz_speed_pid_v.err_i,-Thr_Weight *300,Thr_Weight *300);
	
	wz_speed_pid_v.pid_out = thr_lpf + Thr_Weight *LIMIT((wz_speed_pid.kp *exp_z_speed + wz_speed_pid_v.err + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i),-300,300);

	wz_speed_pid_v.err_old = wz_speed_pid_v.err; 
}

u8 baro_ctrl_start;
float baro_height,baro_height_old;
void Baro_Ctrl(float T,float thr)
{
	static float start_height;
	static float measure;
  static float h_t,h_i;
	
	if( (s32)start_height == 0 )
	{
		start_height = baroAlt;
	}
	
	measure = 10 *( baroAlt - start_height );

	h_i += 0.5f *T *( measure - baro_height );
	h_t += wz_speed *T;
	h_t +=  0.02f *3.14f *T *( measure - baro_height );//( c*wz_speed + ( 1.0f - c ) *(measure - baro_height) ) *T;
	
	baro_height = h_t + h_i;
	
}

#define ULTRA_SPEED 		 200    // mm/s
#define ULTRA_MAX_HEIGHT 2000   // mm
#define ULTRA_INT        300    // 积分幅度

_st_height_pid_v ultra_ctrl;
_st_height_pid ultra_pid;

void Ultra_PID_Init()
{
	ultra_pid.kp = 2;//1.5;
	ultra_pid.kd = 2.5;
	ultra_pid.ki = 0;
}

float exp_height_speed,exp_height;
float ultra_speed;
float ultra_dis_lpf;
float ultra_ctrl_out;

void Ultra_Ctrl(float T,float thr)
{
	float ultra_sp_tmp,ultra_dis_tmp;	
	#define MID_THR 555
	exp_height_speed = ULTRA_SPEED *my_deathzoom_2(thr - MID_THR,50)/200.0f; //+-ULTRA_SPEEDmm / s
	exp_height_speed = LIMIT(exp_height_speed ,-ULTRA_SPEED,ULTRA_SPEED);
	
	if( exp_height > ULTRA_MAX_HEIGHT )
	{
		if( exp_height_speed > 0 )
		{
			exp_height_speed = 0;
		}
	}
	else if( exp_height < 20 )
	{
		if( exp_height_speed < 0 )
		{
			exp_height_speed = 0;
		}
	}
	
	exp_height += exp_height_speed *T;
	
	if( thr < 100 )
	{
		exp_height += ( 1 / ( 1 + 1 / ( 0.2f *3.14f *T ) ) ) *( -exp_height);
	}
	
	//ultra_sp_tmp = Moving_Median(0,5,ultra_delta/T); //ultra_delta/T;
	ultra_sp_tmp=Moving_Median(0,5,v_pred*100); 
	if( ultra_dis_tmp < 2000 )
	{
		if( ABS(ultra_sp_tmp) < 100 )
		{
			ultra_speed += ( 1 / ( 1 + 1 / ( 4 *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
		}
		else
		{
			ultra_speed += ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
		}
	}
	
	ultra_dis_tmp = Moving_Median(1,5,ultra_distance);
	
	if( ultra_dis_tmp < 2000 )
	{
		
		if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 100 )
		{
			
			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 4.0f *3.14f *T ) ) ) *(ultra_dis_tmp - ultra_dis_lpf) ;
		}
		else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 200 )
		{
			
			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 2.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
		}
		else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 400 )
		{
			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 1.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
		}
		else
		{
			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 0.6f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
		}
	}
	else
	{
		
	}

	ultra_ctrl.err = ( ultra_pid.kp*(exp_height - ultra_dis_lpf) );
	
	ultra_ctrl.err_i += ultra_pid.ki *ultra_ctrl.err *T;
	
	ultra_ctrl.err_i = LIMIT(ultra_ctrl.err_i,-Thr_Weight *ULTRA_INT,Thr_Weight *ULTRA_INT);
	
	ultra_ctrl.err_d = ultra_pid.kd *( 0.6f *(-wz_speed*T) + 0.4f *(ultra_ctrl.err - ultra_ctrl.err_old) );
	
	ultra_ctrl.pid_out = ultra_ctrl.err + ultra_ctrl.err_i + ultra_ctrl.err_d;
	
	ultra_ctrl.pid_out = LIMIT(ultra_ctrl.pid_out,-500,500);
		
	ultra_ctrl_out = ultra_ctrl.pid_out;
	
	ultra_ctrl.err_old = ultra_ctrl.err;
}

