/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * ����   �������ƴ�
 * �ļ���  ��imu.c
 * ����    ����̬����
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/

#include "imu.h"
#include "include.h"
#include "ak8975.h"
#include "mymath.h"

#define Kp 0.6f                	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.1f                	// 0.001  integral gain governs rate of convergence of gyroscope biases

#define IMU_INTEGRAL_LIM  ( 2.0f *ANGLE_TO_RADIAN )
#define NORM_ACC_LPF_HZ 10  		//(Hz)
#define REF_ERR_LPF_HZ  1				//(Hz)

xyz_f_t reference_v;
ref_t 	ref;

//xyz_f_t Gravity_Vec;  				//�������������
	
float Roll,Pitch,Yaw;    				//��̬��

float ref_q[4] = {1,0,0,0};
float norm_acc,norm_q;
float norm_acc_lpf;

extern u8 fly_ready;
void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw) 
{		
	float ref_err_lpf_hz;
	float yaw_correct;
	float mag_norm_tmp;
	static float mag_norm ,mag_norm_xyz ,mag_tmp_x,mag_tmp_y,yaw_mag;
	
	mag_norm = my_sqrt(ak8975.Mag_Val.x * ak8975.Mag_Val.x + ak8975.Mag_Val.y * ak8975.Mag_Val.y);
	mag_norm_xyz = my_sqrt(ak8975.Mag_Val.x * ak8975.Mag_Val.x + ak8975.Mag_Val.y * ak8975.Mag_Val.y + ak8975.Mag_Val.z * ak8975.Mag_Val.z);
	
	mag_norm_tmp = LIMIT( 0.02f *( 50 - ABS(my_deathzoom((mag_norm_xyz - 100),10)) ) ,0.05f ,1 ) * 20 *(6.28f *half_T);
	
	mag_tmp_x += mag_norm_tmp *( (float)ak8975.Mag_Val.x - mag_tmp_x);
	mag_tmp_y += mag_norm_tmp *( (float)ak8975.Mag_Val.y - mag_tmp_y);	
	
	if(ak8975.Mag_Val.x != 0 && ak8975.Mag_Val.y != 0 )
	{
		yaw_mag = fast_atan2(mag_tmp_y/mag_norm , mag_tmp_x/mag_norm) *57.3f+63;
		
	}
	//=============================================================================
	// �����Ч��������
	reference_v.x = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	reference_v.y = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	reference_v.z = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3];

	
	//���ǰ���Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء�
	//�������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ�ء�
	//���������vx\y\z����ʵ���ǵ�ǰ��ŷ���ǣ�����Ԫ�����Ļ����������ϵ�ϣ����������������λ������       
	//=============================================================================


	// ������ٶ�������ģ
	norm_acc = my_sqrt(ax*ax + ay*ay + az*az);   
	norm_acc_lpf +=  NORM_ACC_LPF_HZ *(6.28f *half_T) *(norm_acc - norm_acc_lpf);  //10hz *3.14 * 2*0.001


	if(ABS(ax)<4400 && ABS(ay)<4400 && ABS(az)<4400 )
	{	
		//�ѼӼƵ���ά����ת�ɵ�λ������
		ax = ax / norm_acc;//4096.0f;
		ay = ay / norm_acc;//4096.0f;
		az = az / norm_acc;//4096.0f; 
		
		if( 3800 < norm_acc && norm_acc < 4400 )
		{
			/* ��˵õ���� */
			ref.err_tmp.x = ay*reference_v.z - az*reference_v.y;
			ref.err_tmp.y = az*reference_v.x - ax*reference_v.z;
	    //ref.err_tmp.z = ax*reference_v.y - ay*reference_v.x;
			
			/* ����ͨ */
			ref_err_lpf_hz = REF_ERR_LPF_HZ *(6.28f *half_T);
			ref.err_lpf.x += ref_err_lpf_hz *( ref.err_tmp.x  - ref.err_lpf.x );
			ref.err_lpf.y += ref_err_lpf_hz *( ref.err_tmp.y  - ref.err_lpf.y );
	//			 ref.err_lpf.z += ref_err_lpf_hz *( ref.err_tmp.z  - ref.err_lpf.z );
			
			ref.err.x = ref.err_lpf.x;//
			ref.err.y = ref.err_lpf.y;//
//				ref.err.z = ref.err_lpf.z ;
		}
	}
	else
	{
		ref.err.x = 0; 
		ref.err.y = 0  ;
//		ref.err.z = 0 ;
	}
	/* ������ */
	ref.err_Int.x += ref.err.x *Ki *2 *half_T ;
	ref.err_Int.y += ref.err.y *Ki *2 *half_T ;
	ref.err_Int.z += ref.err.z *Ki *2 *half_T ;
	
	/* �����޷� */
	ref.err_Int.x = LIMIT(ref.err_Int.x, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.y = LIMIT(ref.err_Int.y, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.z = LIMIT(ref.err_Int.z, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );

	if( fly_ready  )
	{
		yaw_correct = Kp *0.1f *LIMIT( my_deathzoom( To_180_degrees(yaw_mag - Yaw), 10),-20,20 );
	}
	else
	{
		yaw_correct = Kp *1.0f *To_180_degrees(yaw_mag - Yaw);
	}
	
	if( reference_v.z > 0.7f )
	{
		*yaw += 2*half_T *( -gx *reference_v.x - gy *reference_v.y - gz *reference_v.z + yaw_correct ); 

	}
	else
	{
		*yaw += 2*half_T *( -gx *reference_v.x - gy *reference_v.y - gz *reference_v.z ); 
	}	
	*yaw = To_180_degrees( *yaw );	
	
	ref.g.x = gx *ANGLE_TO_RADIAN + ( Kp*(ref.err.x + ref.err_Int.x) ) ;     //IN RADIAN
	ref.g.y = gy *ANGLE_TO_RADIAN + ( Kp*(ref.err.y + ref.err_Int.y) ) ;		  //IN RADIAN
	ref.g.z = gz *ANGLE_TO_RADIAN;
	
	/* �ò���������PI����������ƫ */

	// integrate quaternion rate and normalise
	ref_q[0] = ref_q[0] +(-ref_q[1]*ref.g.x - ref_q[2]*ref.g.y - ref_q[3]*ref.g.z)*half_T;
	ref_q[1] = ref_q[1] + (ref_q[0]*ref.g.x + ref_q[2]*ref.g.z - ref_q[3]*ref.g.y)*half_T;
	ref_q[2] = ref_q[2] + (ref_q[0]*ref.g.y - ref_q[1]*ref.g.z + ref_q[3]*ref.g.x)*half_T;
	ref_q[3] = ref_q[3] + (ref_q[0]*ref.g.z + ref_q[1]*ref.g.y - ref_q[2]*ref.g.x)*half_T;  

	/* ��Ԫ����һ�� normalise quaternion */
	norm_q = my_sqrt(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3]);
	ref_q[0] = ref_q[0] / norm_q;
	ref_q[1] = ref_q[1] / norm_q;
	ref_q[2] = ref_q[2] / norm_q;
	ref_q[3] = ref_q[3] / norm_q;
	

	*rol = fast_atan2(2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]),1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2])) *57.3f;
	*pit = asin(2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2])) *57.3f;
// 				//Yaw   = ( - fast_atan2(2*(ref_q[1]*ref_q[2] + ref_q[0]*ref_q[3]),ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] - ref_q[3]*ref_q[3]) )* 57.3;
	//*yaw = fast_atan2(2*(ref_q[1]*ref_q[2] - ref_q[0]*ref_q[3]), 2*(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1]) - 1) *57.3f;


}
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_AHRSupdate
*��������:	 ����AHRS ������Ԫ�� 
��������� ��ǰ�Ĳ���ֵ��
���������û��
*******************************************************************************/
float Kp_imu=5.0f;   // proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki_imu=0.1f;   // integral gain governs rate of convergence of gyroscope biases
volatile float exInt, eyInt, ezInt;  // ������
volatile float q0, q1, q2, q3,w1,w2,w3; // ȫ����Ԫ��
float angles[3];
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float norm;
  float hx, hy, hz, bx, bz;
  double vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float tempq0,tempq1,tempq2,tempq3;
float halfT;
	static u8 init;
	if(!init){
	
		q0 = 1.0f;  //��ʼ����Ԫ��
  	q1 = 0.0f;
  	q2 = 0.0f;
  	q3 = 0.0f;
  	exInt = 0.0;
  	eyInt = 0.0;
  	ezInt = 0.0;
		init=1;
	}
	
  // �Ȱ���Щ�õõ���ֵ���
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;          
  
  halfT = Get_Cycle_T(3)/2;		

norm = invSqrt(ax*ax + ay*ay + az*az); 
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;
  //�ѼӼƵ���ά����ת�ɵ�λ������

  norm = invSqrt(mx*mx + my*my + mz*mz);          
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  /*
  ���ǰ���Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء�
�������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ�ء�
���������vx\y\z����ʵ���ǵ�ǰ��ŷ���ǣ�����Ԫ�����Ļ����������ϵ�ϣ����������������λ������
  */
  // compute reference direction of flux
  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;     
  
  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
  
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

  /*
  axyz�ǻ����������ϵ�ϣ����ٶȼƲ����������������Ҳ����ʵ�ʲ����������������
axyz�ǲ����õ�������������vxyz�����ݻ��ֺ����̬����������������������Ƕ��ǻ����������ϵ�ϵ�����������
������֮�������������������ݻ��ֺ����̬�ͼӼƲ��������̬֮�����
������������������������Ҳ�������������ˣ�����ʾ��exyz�����������������Ĳ����
�����������Ծ���λ�ڻ�������ϵ�ϵģ������ݻ������Ҳ���ڻ�������ϵ�����Ҳ���Ĵ�С�����ݻ����������ȣ����������������ݡ���������Լ��ö�������һ�£����������ǶԻ���ֱ�ӻ��֣����Զ����ݵľ�������ֱ�������ڶԻ�������ϵ�ľ�����
  */
if(ex != 0.0f && ey != 0.0f && ez != 0.0f){
  exInt = exInt + ex * Ki_imu * halfT;
  eyInt = eyInt + ey * Ki_imu * halfT;	
  ezInt = ezInt + ez * Ki_imu * halfT;

  // �ò���������PI����������ƫ
  gx = gx + Kp_imu*ex + exInt;
  gy = gy + Kp_imu*ey + eyInt;
  gz = gz + Kp_imu*ez + ezInt;

  }

  // ��Ԫ��΢�ַ���
  tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  
  // ��Ԫ���淶��
  norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
  q0 = tempq0 * norm;
  q1 = tempq1 * norm;
  q2 = tempq2 * norm;
  q3 = tempq3 * norm;
// ekf_att.rollekf_att.pitch
   angles[0]= asin(-2 * q1* q3 + 2 * q0 * q2)* 180/3.14; // pitch
   angles[1]= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)* 180/3.14; // roll
   angles[2] =-atan2(2 * q1 * q2 + 2 * q0* q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 180/3.14+114+39+26+245; // yaw
	 angles[2]=To_180_degrees( angles[2] );	
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

