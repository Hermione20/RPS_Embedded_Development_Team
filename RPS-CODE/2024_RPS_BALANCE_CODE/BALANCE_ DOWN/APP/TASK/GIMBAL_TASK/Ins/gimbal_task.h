#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H
#include "public.h"



#define INFANTRY_PITCH_MAX 35.0f
#define INFANTRY_PITCH_MIN -25.0f

#define FIGHTER_PITCH_MAX 10.0f
#define FIGHTER_PITCH_MIN -30.0f

#define SECURITY_PITCH_MAX 35.0f
#define SECURITY_PITCH_MIN -25.0f

#define HERO_PITCH_MAX 8548
#define HERO_PITCH_MIN -1963
/************************buff*****************************/
//摄像头和枪管中心的距离
#define HEIGHT_BETWEEN_GUN_CAMERA 	4.89f
//相机焦距mm
#define FOCAL_LENGTH                6.0F
//靶面长mm
#define TARGET_SURFACE_LENGTH      3.45e-3F
//靶面宽mm
#define TARGET_SURFACE_WIDTH        3.45e-3F




typedef enum
{
  GIMBAL_RELAX         = 0,
  GIMBAL_INIT          = 1,
  GIMBAL_NO_ARTI_INPUT = 2,
  GIMBAL_FOLLOW_ZGYRO  = 3,
  GIMBAL_TRACK_ARMOR   = 4,
  GIMBAL_PATROL_MODE   = 5,
  GIMBAL_SHOOT_BUFF    = 6,
  GIMBAL_POSITION_MODE = 7,
  GIMBAL_AUTO_AIM	   = 8,
  GIMBAL_AUTO_SUP      = 9,
  GIMBAL_REVERSE       = 10,
  GIMBAL_AUTO_SMALL_BUFF   = 11,
  GIMBAL_AUTO_BIG_BUFF     = 12,
  GIMBAL_AUTO_ANGLE    = 13,
  GIMBAL_FOLLOW_CHASSIS=14,
	GIMBAL_CHANGE_DIRCTION  =15,
} gimbal_mode_e;

typedef struct
{
  /* position loop */
  float yaw_angle_ref;
  float pit_angle_ref;
  float yaw_angle_fdb;
  float pit_angle_fdb;
  /* speed loop */
  float yaw_speed_ref;
  float pit_speed_ref;
  float yaw_speed_fdb;
  float pit_speed_fdb;

  int16_t yaw_motor_input;
  int16_t pitch_motor_input;
} gim_ref_and_fdb_t;


typedef struct 
{
  float pitch_angle_dynamic_ref;
  float yaw_angle_dynamic_ref;
}gim_dynamic_ref_t;

typedef struct
{
  /* ctrl mode */
  gimbal_mode_e ctrl_mode;
  gimbal_mode_e last_ctrl_mode;
  
  gim_ref_and_fdb_t gim_ref_and_fdb;
  gim_dynamic_ref_t gim_dynamic_ref;
	
	u8 if_finish_Init;
	
  pid_t pid_init_yaw_Angle; 
  pid_t pid_init_pit_Angle; 
  pid_t pid_init_yaw_speed; 
  pid_t pid_init_pit_speed;

  pid_t pid_yaw_Angle; 
  pid_t pid_pit_Angle; 
  pid_t pid_yaw_speed; 
  pid_t pid_pit_speed;

  //英雄吊射模式下的参数
  pid_t pid_auto_yaw_Angle; 
  pid_t pid_auto_pit_Angle; 
  pid_t pid_auto_yaw_speed; 
  pid_t pid_auto_pit_speed; 

  // 小符下的PID参数
  pid_t pid_yaw_small_buff;
  pid_t pid_pit_small_buff;
  pid_t pid_pit_speed_small_buff;
  pid_t pid_yaw_speed_small_buff;

  // 自瞄模式外环的参数
  pid_t pid_yaw_follow;
  pid_t pid_pit_follow;
  pid_t pid_pit_speed_follow;
  pid_t pid_yaw_speed_follow;

  // 大符下的PID参数
  pid_t pid_yaw_big_buff;
  pid_t pid_pit_big_buff;
  pid_t pid_pit_speed_big_buff;
  pid_t pid_yaw_speed_big_buff;

} gimbal_t;


void gimbal_parameter_Init(void);
float gimbal_yaw_loop_task(pid_t *Outer_loop_pid,pid_t *Inner_loop_pid,float angle_ref,float angle_fdb,float speed_Feedforward);
float gimbal_pit_loop_task(pid_t *Outer_loop_pid,pid_t *Inner_loop_pid,float angle_ref,float angle_fdb,float speed_Feedforward);

void gimbal_task(void);
void gimbal_init_handle	( void );
void gimbal_follow_gyro_handle(void);
void auto_small_buff_handle(void);
void auto_big_buff_handle(void);
void security_gimbal_handle(void);
void gimbal_auto_angle_handle(void);
float raw_data_to_pitch_angle(float ecd_angle_pit);

extern gimbal_t gimbal_data;
extern float yaw_angle_ref_aim,pit_angle_ref_aim;
extern float pitch_max,pitch_min;






#endif

