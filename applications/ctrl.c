/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * ����   �������ƴ�
 * �ļ���  ��ctrl.c
 * ����    ���ɿؿ���
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/

#include "ctrl.h"
#include "height_ctrl.h"
#define LON 0
#define LAT 1
ctrl_t ctrl_1;
ctrl_t ctrl_2;

// Calculate nav_lat and nav_lon from the x and y error and the speed
#define NAV_BANK_MAX 15
float nav[2];
float GPS_angle[2];
extern void GPS_calc_poshold(void)
{
    int32_t p, i, d;
    int32_t output;
    int32_t target_speed;
	  static 	float  integrator[2],last_derivative[2];
	  float derivative[2];
	  static int32_t last_error[2];
    int axis;
   float error[axis], rate_error[axis];
	 float actual_speed[axis];
   float cos_yaw,sin_yaw;	
		 actual_speed[0]=imu_nav.flow.speed.x;//mm
	   actual_speed[1]=imu_nav.flow.speed.y;
	 
    for (axis = 0; axis < 2; axis++) {
			//loc p
			  error[axis]=0;//tar-now
        target_speed =  error[axis]*pid.nav.out.p;       // calculate desired speed from lon error
			
        rate_error[axis] = target_speed - actual_speed[axis];   // calc the speed error
       //-----------rad
        p = rate_error[axis]*pid.nav.in.p;
			integrator[axis] += ((float) rate_error[axis]* pid.nav.in.i);// *DT;
    if (integrator[axis]  < 	-NAV_BANK_MAX*0.3) {
        integrator[axis]  = - 	NAV_BANK_MAX*0.3;
    } else if ( integrator[axis] >  	NAV_BANK_MAX*0.3) {
        integrator[axis] =  	NAV_BANK_MAX*0.3;
    }
        i = integrator[axis] ;
		
		
		 derivative[axis] = (rate_error[axis] - last_error[axis]) ;/// DT;
    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
    // derivative[axis] = last_derivative[axis] + (DT / (AC_PID_FILTER + DT)) * (derivative[axis] - last_derivative[axis]);
		 derivative[axis]=0.3* derivative[axis]+ 0.7*last_derivative[axis];
    // update state
    last_error[axis] = rate_error[axis] ;
    last_derivative[axis]= derivative[axis];
    // add in derivative component
        d = derivative[axis]*pid.nav.in.d;//d
		
        //d = constrain_int16(d, -2000, 2000);
        // get rid of noise
        if (fabs(actual_speed[axis]) < 8)
            d = 0;
        output = p + i + d;

        GPS_angle[axis] = limit_mine(output,NAV_BANK_MAX);//�������ƽǶ�
     
    }
		cos_yaw=cos(YAW*0.017);
		sin_yaw=sin(YAW*0.017);
			nav[PIT] = (GPS_angle[LAT] * cos_yaw + GPS_angle[LON] * sin_yaw);// / 10;
			nav[ROL] = -(GPS_angle[LON] * cos_yaw - GPS_angle[LAT] * sin_yaw);/// 10;
		 limit_mine( (nav[PIT]) ,10 );
		 limit_mine( (nav[ROL]) ,10 );
		/*
		north    LAT=1     V_West+                             __________
		|   Y+  x                                              P- R- GPS-
		                                        
		|P+
	
    _____	R	+   y         LON=0  V_East+
	   
head  |    1 PIT x+
			| 
		   _____  0 ROL y-
	
		*/
}




void Ctrl_Para_Init()		//����Ĭ�ϲ���
{
//====================================
	ctrl_1.PID[PIDROLL].kdamp  = 1;
	ctrl_1.PID[PIDPITCH].kdamp = 1;
	ctrl_1.PID[PIDYAW].kdamp 	 = 1;
	
	ctrl_1.FB = 0.20;   //��  0<fb<1
}

xyz_f_t except_A = {0,0,0};

xyz_f_t ctrl_angle_offset = {0,0,0};

xyz_f_t compensation;
 float YAW_NO_HEAD;
void CTRL_2(float T)
{
	static xyz_f_t acc_no_g;
	static xyz_f_t acc_no_g_lpf;
	float nav_angle[2]={0,0};
	float except_A_SB[2];

  float cos1,sin1;
	float temp;
/*   head  |    1 PIT x+
	         | 
	         _____  0 ROL y-
	*/
//=========================== �����Ƕ� ========================================
	 except_A_SB[ROL] = MAX_CTRL_ANGLE  *( my_deathzoom( (CH_filter[ROL]) ,30 )/500.0f );  
	 except_A_SB[PIT] = MAX_CTRL_ANGLE  *( my_deathzoom( (CH_filter[PIT]) ,30 )/500.0f );  
	
		if(mode.test1&&((fabs(except_A_SB[PIT])<2)&&(fabs(except_A_SB[ROL])<2)))
	{
	nav_angle[PIT]=nav[PIT];
	nav_angle[ROL]=nav[ROL];	
	}
	/*YAW -180~180     -90  135
		-	 0  + 
			 |
			 |
		-	180 +
	*/
	if(mode.no_head)
	{ 
	temp=Yaw-YAW_NO_HEAD;
	cos1=cos(temp*0.017);
	sin1=sin(temp*0.017);
	except_A.x=except_A_SB[PIT]*cos1+except_A_SB[ROL]*sin1;
	except_A.y=except_A_SB[ROL]*cos1+except_A_SB[PIT]*sin1;
	}	
	else
	{
	YAW_NO_HEAD=Yaw;	
	except_A.x=except_A_SB[PIT];
	except_A.y=except_A_SB[ROL];
	}
	except_A.x=limit_mine(nav_angle[PIT]+except_A.x,MAX_CTRL_ANGLE);
	except_A.y=limit_mine(nav_angle[ROL]+except_A.y,MAX_CTRL_ANGLE);
	
	
	if( Thr_Low == 0 )
	{
		except_A.z += (s16)( MAX_CTRL_YAW_SPEED *( my_deathzoom_2( (CH_filter[YAW]) ,40 )/500.0f ) ) *T ;  //50
	}
	else
	{
		except_A.z += 1 *3.14 *T *( Yaw - except_A.z );
	}
	except_A.z = To_180_degrees(except_A.z);
//==============================================================================
	acc_no_g.x =  mpu6050.Acc.x - reference_v.x *4096;
	acc_no_g.y =  mpu6050.Acc.y - reference_v.y *4096;
	acc_no_g.z =  mpu6050.Acc.z - reference_v.z *4096;
	
	acc_no_g_lpf.x += 0.5f *T *3.14f * ( acc_no_g.x - acc_no_g_lpf.x );
	acc_no_g_lpf.y += 0.5f *T *3.14f * ( acc_no_g.y - acc_no_g_lpf.y );
	acc_no_g_lpf.z += 0.5f *T *3.14f * ( acc_no_g.z - acc_no_g_lpf.z );
	
	compensation.x = LIMIT( 0.003f *acc_no_g_lpf.x, -10,10 );
	compensation.y = LIMIT( 0.003f *acc_no_g_lpf.y, -10,10 );
	compensation.z = LIMIT( 0.003f *acc_no_g_lpf.z, -10,10 );
//==============================================================================	

  /* �õ��Ƕ���� */
	ctrl_2.err.x =  To_180_degrees( ctrl_angle_offset.x + except_A.x - Roll  );
	ctrl_2.err.y =  To_180_degrees( ctrl_angle_offset.y + except_A.y - Pitch );
	ctrl_2.err.z =  To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw	 );
	/* ����Ƕ����Ȩ�� */
	ctrl_2.err_weight.x = ABS(ctrl_2.err.x)/ANGLE_TO_MAX_AS;
	ctrl_2.err_weight.y = ABS(ctrl_2.err.y)/ANGLE_TO_MAX_AS;
	ctrl_2.err_weight.z = ABS(ctrl_2.err.z)/ANGLE_TO_MAX_AS;
	/* �Ƕ����΢�֣�����������߱仯��*/
	ctrl_2.err_d.x = 10 *ctrl_2.PID[PIDROLL].kd  *(ctrl_2.err.x - ctrl_2.err_old.x) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_2.err_weight.x );
	ctrl_2.err_d.y = 10 *ctrl_2.PID[PIDPITCH].kd *(ctrl_2.err.y - ctrl_2.err_old.y) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_2.err_weight.y );
	ctrl_2.err_d.z = 10 *ctrl_2.PID[PIDYAW].kd 	 *(ctrl_2.err.z - ctrl_2.err_old.z) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_2.err_weight.z );
	/* �Ƕ������� */
	ctrl_2.err_i.x += ctrl_2.PID[PIDROLL].ki  *ctrl_2.err.x *T;
	ctrl_2.err_i.y += ctrl_2.PID[PIDPITCH].ki *ctrl_2.err.y *T;
	ctrl_2.err_i.z += ctrl_2.PID[PIDYAW].ki 	*ctrl_2.err.z *T;
	/* �Ƕ������ַ��� */
	ctrl_2.eliminate_I.x = Thr_Weight *CTRL_2_INT_LIMIT;
	ctrl_2.eliminate_I.y = Thr_Weight *CTRL_2_INT_LIMIT;
	ctrl_2.eliminate_I.z = Thr_Weight *CTRL_2_INT_LIMIT;
	/* �Ƕ��������޷� */
	ctrl_2.err_i.x = LIMIT( ctrl_2.err_i.x, -ctrl_2.eliminate_I.x,ctrl_2.eliminate_I.x );
	ctrl_2.err_i.y = LIMIT( ctrl_2.err_i.y, -ctrl_2.eliminate_I.y,ctrl_2.eliminate_I.y );
	ctrl_2.err_i.z = LIMIT( ctrl_2.err_i.z, -ctrl_2.eliminate_I.z,ctrl_2.eliminate_I.z );
	/* �����ڼ������������ĽǶ�����޷� */
	ctrl_2.err.x = LIMIT( ctrl_2.err.x, -90, 90 );
	ctrl_2.err.y = LIMIT( ctrl_2.err.y, -90, 90 );
	ctrl_2.err.z = LIMIT( ctrl_2.err.z, -90, 90 );
	/* �Ƕ�PID��� */
	ctrl_2.out.x = ctrl_2.PID[PIDROLL].kp  *( ctrl_2.err.x + ctrl_2.err_d.x + ctrl_2.err_i.x ) + compensation.y;	//rol
	ctrl_2.out.y = ctrl_2.PID[PIDPITCH].kp *( ctrl_2.err.y + ctrl_2.err_d.y + ctrl_2.err_i.y ) + compensation.x;  //pit
	ctrl_2.out.z = ctrl_2.PID[PIDYAW].kp   *( ctrl_2.err.z + ctrl_2.err_d.z + ctrl_2.err_i.z );
	/* ��¼��ʷ���� */	
	ctrl_2.err_old.x = ctrl_2.err.x;
	ctrl_2.err_old.y = ctrl_2.err.y;
	ctrl_2.err_old.z = ctrl_2.err.z;

}

xyz_f_t except_AS;

float g_old[ITEMS];

void CTRL_1(float T)  //x roll,y pitch,z yaw
{
	xyz_f_t EXP_LPF_TMP;
	/* ��������Ŀ�꣩���ٶ� */
	EXP_LPF_TMP.x = MAX_CTRL_ASPEED *(ctrl_2.out.x/ANGLE_TO_MAX_AS);//*( (CH_filter[0])/500.0f );//
	EXP_LPF_TMP.y = MAX_CTRL_ASPEED *(ctrl_2.out.y/ANGLE_TO_MAX_AS);//*( (CH_filter[1])/500.0f );//
	EXP_LPF_TMP.z = MAX_CTRL_ASPEED *(ctrl_2.out.z/ANGLE_TO_MAX_AS);
	
	except_AS.x = EXP_LPF_TMP.x;//20 *3.14 *T *( EXP_LPF_TMP.x - except_AS.x );//
	except_AS.y = EXP_LPF_TMP.y;//20 *3.14 *T *( EXP_LPF_TMP.y - except_AS.y );//
	except_AS.z = EXP_LPF_TMP.z;//20 *3.14 *T *( EXP_LPF_TMP.z - except_AS.z );//
	/* �������ٶ��޷� */
	except_AS.x = LIMIT(except_AS.x, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );
	except_AS.y = LIMIT(except_AS.y, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );
	except_AS.z = LIMIT(except_AS.z, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );

	/* ���ٶ�ֱ��΢�֣��Ǽ��ٶȣ������������γɽ��ٶȵ����ᣨ�谭���ٶȵı仯��*/
	ctrl_1.damp.x = ( mpu6050.Gyro_deg.x - g_old[A_X]) *( 0.002f/T );//ctrl_1.PID[PIDROLL].kdamp
	ctrl_1.damp.y = (-mpu6050.Gyro_deg.y - g_old[A_Y]) *( 0.002f/T );//ctrl_1.PID[PIDPITCH].kdamp *
	ctrl_1.damp.z = (-mpu6050.Gyro_deg.z - g_old[A_Z]) *( 0.002f/T );//ctrl_1.PID[PIDYAW].kdamp	 *
	/* ���ٶ���� */
	ctrl_1.err.x =  ( except_AS.x - mpu6050.Gyro_deg.x ) *(300.0f/MAX_CTRL_ASPEED);
	ctrl_1.err.y =  ( except_AS.y + mpu6050.Gyro_deg.y ) *(300.0f/MAX_CTRL_ASPEED);  //-y
	ctrl_1.err.z =  ( except_AS.z + mpu6050.Gyro_deg.z ) *(300.0f/MAX_CTRL_ASPEED);	 //-z
	/* ���ٶ����Ȩ�� */
	ctrl_1.err_weight.x = ABS(ctrl_1.err.x)/MAX_CTRL_ASPEED;
	ctrl_1.err_weight.y = ABS(ctrl_1.err.y)/MAX_CTRL_ASPEED;
	ctrl_1.err_weight.z = ABS(ctrl_1.err.z)/MAX_CTRL_YAW_SPEED;
	/* ���ٶ�΢�� */
	ctrl_1.err_d.x = ( ctrl_1.PID[PIDROLL].kd  *( -10 *ctrl_1.damp.x) *( 0.002f/T ) );
	ctrl_1.err_d.y = ( ctrl_1.PID[PIDPITCH].kd *( -10 *ctrl_1.damp.y) *( 0.002f/T ) );
	ctrl_1.err_d.z = ( ctrl_1.PID[PIDYAW].kd   *( -10 *ctrl_1.damp.z) *( 0.002f/T ) );

//	ctrl_1.err_d.x += 40 *3.14 *0.002 *( 10 *ctrl_1.PID[PIDROLL].kd  *(ctrl_1.err.x - ctrl_1.err_old.x) *( 0.002f/T ) - ctrl_1.err_d.x);
//	ctrl_1.err_d.y += 40 *3.14 *0.002 *( 10 *ctrl_1.PID[PIDPITCH].kd *(ctrl_1.err.y - ctrl_1.err_old.y) *( 0.002f/T ) - ctrl_1.err_d.y);
//	ctrl_1.err_d.z += 40 *3.14 *0.002 *( 10 *ctrl_1.PID[PIDYAW].kd   *(ctrl_1.err.z - ctrl_1.err_old.z) *( 0.002f/T ) - ctrl_1.err_d.z);

	/* ���ٶ������� */
	ctrl_1.err_i.x += ctrl_1.PID[PIDROLL].ki  *(ctrl_1.err.x - ctrl_1.damp.x) *T;
	ctrl_1.err_i.y += ctrl_1.PID[PIDPITCH].ki *(ctrl_1.err.y - ctrl_1.damp.y) *T;
	ctrl_1.err_i.z += ctrl_1.PID[PIDYAW].ki 	*(ctrl_1.err.z - ctrl_1.damp.z) *T;
	/* ���ٶ������ַ��� */
	ctrl_1.eliminate_I.x = Thr_Weight *CTRL_1_INT_LIMIT ;
	ctrl_1.eliminate_I.y = Thr_Weight *CTRL_1_INT_LIMIT ;
	ctrl_1.eliminate_I.z = Thr_Weight *CTRL_1_INT_LIMIT ;
	/* ���ٶ��������޷� */
	ctrl_1.err_i.x = LIMIT( ctrl_1.err_i.x, -ctrl_1.eliminate_I.x,ctrl_1.eliminate_I.x );
	ctrl_1.err_i.y = LIMIT( ctrl_1.err_i.y, -ctrl_1.eliminate_I.y,ctrl_1.eliminate_I.y );
	ctrl_1.err_i.z = LIMIT( ctrl_1.err_i.z, -ctrl_1.eliminate_I.z,ctrl_1.eliminate_I.z );
	/* ���ٶ�PID��� */
	ctrl_1.out.x = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.x),0,1)*except_AS.x + ( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDROLL].kp  *( ctrl_1.err.x + ctrl_1.err_d.x + ctrl_1.err_i.x ) );
										//*(MAX_CTRL_ASPEED/300.0f);
	ctrl_1.out.y = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.y),0,1)*except_AS.y + ( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDPITCH].kp *( ctrl_1.err.y + ctrl_1.err_d.y + ctrl_1.err_i.y ) );
										//*(MAX_CTRL_ASPEED/300.0f);
	ctrl_1.out.z = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.z),0,1)*except_AS.z + ( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDYAW].kp   *( ctrl_1.err.z + ctrl_1.err_d.z + ctrl_1.err_i.z ) );
										//*(MAX_CTRL_ASPEED/300.0f);
	Thr_Ctrl(T);// ���ſ���
	
	All_Out(ctrl_1.out.x,ctrl_1.out.y,ctrl_1.out.z);


	ctrl_1.err_old.x = ctrl_1.err.x;
	ctrl_1.err_old.y = ctrl_1.err.y;
	ctrl_1.err_old.z = ctrl_1.err.z;

	g_old[A_X] =  mpu6050.Gyro_deg.x ;
	g_old[A_Y] = -mpu6050.Gyro_deg.y ;
	g_old[A_Z] = -mpu6050.Gyro_deg.z ;
}


float thr_value;
u8 Thr_Low;
float Thr_Weight;
float thr_test;
void Thr_Ctrl(float T)
{	float delta_thr;
	static float thr;
	static float Thr_tmp;
		static u8 cnt_thr_add;
	if(!fly_ready){
	   thr=0;
		 }
	else
	{
	
    thr = 500 + CH_filter[THR]; //����ֵ 0 ~ 1000
	
			
		}
	//thr = 500 + CH_filter[THR]; //����ֵ 0 ~ 1000
	thr_test=thr;
	Thr_tmp += 10 *3.14f *T *(thr/400.0f - Thr_tmp); //��ͨ�˲�
	Thr_Weight = LIMIT(Thr_tmp,0,1);    							//��߶ദ�������ݻ��õ����ֵ
	
	if( thr < 100 )
	{
		Thr_Low = 1;
	}
	else
	{
		Thr_Low = 0;
	}
	
	#if(CTRL_HEIGHT)
	Height_Ctrl(T,thr);
	
	thr_value = Thr_Weight *height_ctrl_out;   //ʵ��ʹ��ֵ
	#else
	thr_value = thr;   //ʵ��ʹ��ֵ
	#endif
	
	thr_value = LIMIT(thr_value,0,10 *MAX_THR *MAX_PWM/100);
}


float motor[MAXMOTORS];
float posture_value[MAXMOTORS];
float curve[MAXMOTORS];
void All_Out(float out_roll,float out_pitch,float out_yaw)
{
	s16 motor_out[MAXMOTORS];
	u8 i;
	float posture_value[MAXMOTORS];
  float curve[MAXMOTORS];
	static float motor_last[MAXMOTORS];

	out_yaw = LIMIT( out_yaw , -5*MAX_THR ,5*MAX_THR ); //50%
	
	posture_value[0] = - out_roll + out_pitch + out_yaw ;
	posture_value[1] = + out_roll + out_pitch - out_yaw ;
	posture_value[2] = + out_roll - out_pitch + out_yaw ;
	posture_value[3] = - out_roll - out_pitch - out_yaw ;
	
	for(i=0;i<4;i++)
	{
		posture_value[i] = LIMIT(posture_value[i], -1000,1000 );
	}
	
	curve[0] = (0.55f + 0.45f *ABS(posture_value[0])/1000.0f) *posture_value[0] ;
	curve[1] = (0.55f + 0.45f *ABS(posture_value[1])/1000.0f) *posture_value[1] ;
	curve[2] = (0.55f + 0.45f *ABS(posture_value[2])/1000.0f) *posture_value[2] ;
	curve[3] = (0.55f + 0.45f *ABS(posture_value[3])/1000.0f) *posture_value[3] ;
	
	
  motor[0] = thr_value + Thr_Weight *curve[0] ;
	motor[1] = thr_value + Thr_Weight *curve[1] ;
	motor[2] = thr_value + Thr_Weight *curve[2] ;
	motor[3] = thr_value + Thr_Weight *curve[3] ;
	
	  if(mode.en_moto_smooth){
     for(i=0;i<MAXMOTORS;i++){
        if(motor[i] > motor_last[i]) 
					motor[i] = (1 * (int16_t) motor_last[i] + motor[i]) / 2;  //mean of old and new
        else                                         
					motor[i] = motor[i] - (motor_last[i] - motor[i]) * 1; // 2 * new - old
			}
			 for(i=0;i<8;i++)
					motor_last[i] = motor[i];  //mean of old and new
     
	    }
		
			
			
	/* �Ƿ���� */
	if(fly_ready)
	{
		if( !Thr_Low )  //��������
		{
			for(i=0;i<4;i++)
			{
				motor[i] = LIMIT(motor[i], (10 *READY_SPEED),(10*MAX_PWM) );
			}
		}
		else						//���ŵ�
		{
			for(i=0;i<4;i++)
			{
				motor[i] = LIMIT(motor[i], 0,(10*MAX_PWM) );
			}
		}
	}
	else
	{
		for(i=0;i<4;i++)
		{
			motor[i] = 0;
		}
	}
	/* xxx */
	/*motor_out[0] = (s16)(motor[0]);  
	motor_out[1] = (s16)(motor[1]);	 
	motor_out[2] = (s16)(motor[2]);
	motor_out[3] = (s16)(motor[3]);*/
  motor_out[0] = (s16)(motor[0]);  
	motor_out[1] = (s16)(motor[1]);	 
	motor_out[2] = (s16)(motor[2]);
	motor_out[3] = (s16)(motor[3]);
	
	SetPwm(motor_out,0,1000); //1000
	
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/


