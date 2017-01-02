#ifndef _MultiRotor_rc_H_
#define _MultiRotor_rc_H_
#include "head.h"

void Nrf_Check_Event(void);
void NRF_Send_AF(void);
void NRF_Send_AE(void);
void NRF_Send_OFFSET(void);
void NRF_Send_PID(void);
void NRF_Send_ARMED(void);
void NRF_SEND_test(void);
extern unsigned int cnt_timer2;
extern void RC_Send_Task(void);
extern u8 key_rc[6];


extern u8 is_lock;
extern u8 tx_lock;
extern u8 EN_FIX_GPSF;
extern u8 EN_FIX_LOCKWF;
extern u8 EN_CONTROL_IMUF;
extern u8 EN_FIX_INSF;
extern u8 EN_FIX_HIGHF;

extern u8 EN_FIX_GPS;
extern u8 EN_FIX_LOCKW;
extern u8 EN_CONTROL_IMU;
extern u8 EN_FIX_INS;
extern u8 EN_FIX_HIGH;
extern u8 EN_TX_GX;
extern u8 EN_TX_AX;
extern u8 EN_TX_HM;
extern u8 EN_TX_YRP;
extern u8 EN_TX_GPS;
extern u8 EN_TX_HIGH;
extern u8	(up_load_set);
extern u8	(up_load_pid);
extern u16 yaw_sb;

extern u8 has_tx,gps_f_connect_flag;


extern double GPS_W,GPS_J;
extern float ypr[3];
extern u16 high_f,bat_fly;
extern u8 gps_mode,fly_mode,gps_no;

extern PID_STA HPIDt,SPIDt;		
extern  u16 ax,ay,az,gx,gy,gz,hx,hy,hz,YM,PWM1,PWM2,PWM3,PWM4,fix_pit,fix_rol;
extern u8 data_rate,fly_mode;
extern void CAL_CHECK(void);

extern int rc_rate_cnt[10];
extern int DEBUG[35];

extern int qr_pos[2],tar_pos[2],drone_pos[3];
extern int yaw_gimbal;
extern u8 state_drone;
#endif
