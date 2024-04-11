#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H
#include "public.h"



/************************buff*****************************/
//摄像头和枪管中心的距离
#define HEIGHT_BETWEEN_GUN_CAMERA 	4.89f
//相机焦距mm
#define FOCAL_LENGTH                6.0F
//靶面长mm
#define TARGET_SURFACE_LENGTH      3.45e-3F
//靶面宽mm
#define TARGET_SURFACE_WIDTH        3.45e-3F



 /**
  ******************************************************************************
																云台模式枚举
	 =============================================================================
 **/
typedef enum
{
  GIMBAL_RELAX         = 0,			//关控
  GIMBAL_INIT          = 1,			//初始化
  GIMBAL_NO_ARTI_INPUT = 2,			//未实际意义
  GIMBAL_FOLLOW_ZGYRO  = 3,			//跟随陀螺仪
  GIMBAL_TRACK_ARMOR   = 4,			//未实际意义
  GIMBAL_PATROL_MODE   = 5,			//未实际意义
  GIMBAL_SHOOT_BUFF    = 6,			//未实际意义
  GIMBAL_POSITION_MODE = 7,			//未实际意义
  GIMBAL_AUTO_AIM	   = 8,				//哨兵自动云台
  GIMBAL_AUTO_SUP      = 9,			//未实际意义
  GIMBAL_REVERSE       = 10,		//未实际意义
  GIMBAL_AUTO_SMALL_BUFF   = 11,//小buff
  GIMBAL_AUTO_BIG_BUFF     = 12,//大buff
  GIMBAL_AUTO_ANGLE    = 13,		//英雄吊射
  GIMBAL_FOLLOW_CHASSIS=14,			//未实际意义
	GIMBAL_CHANGE_DIRCTION  =15,	//未实际意义
} gimbal_mode_e;

 /**
  ******************************************************************************
										云台输入与反馈结构体（输入，反馈，电机输出）
	 =============================================================================
 **/
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

 /**
  ******************************************************************************
													云台外部控制信号输入结构体
	 =============================================================================
 **/
typedef struct 
{
  float pitch_angle_dynamic_ref;
  float yaw_angle_dynamic_ref;
}gim_dynamic_ref_t;


 /**
  ******************************************************************************
													云台结构体
	 =============================================================================
 **/
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

  // 自瞄模式外环的参数
  pid_t pid_yaw_follow;
  pid_t pid_pit_follow;
  pid_t pid_pit_speed_follow;
  pid_t pid_yaw_speed_follow;


} gimbal_t;


void gimbal_parameter_Init(void);
float gimbal_yaw_loop_task(pid_t *Outer_loop_pid,pid_t *Inner_loop_pid,float angle_ref,float angle_fdb,float speed_Feedforward);
float gimbal_pit_loop_task(pid_t *Outer_loop_pid,pid_t *Inner_loop_pid,float angle_ref,float angle_fdb,float speed_Feedforward);

void gimbal_task(void);
void gimbal_init_handle	( void );
void gimbal_follow_gyro_handle(void);
void gimbal_auto_angle_handle(void);

extern gimbal_t gimbal_data;
extern float yaw_angle_ref_aim,pit_angle_ref_aim;
extern float pitch_max,pitch_min;






#endif

