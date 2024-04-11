#ifndef __42MM_SHOOT_TASK_H
#define __42MM_SHOOT_TASK_H
#include "public.h"




 /**
  ******************************************************************************
																摩擦轮状态枚举		
	 =============================================================================
 **/
typedef enum
{
    Stop = 0,
    START = 1,
    NORMAL= 2,
    BACK = 3,
    LOCK = 4
} friction_state_t; // 1正常 2堵转


 /**
  ******************************************************************************
																发射输入输出结构体		
	 =============================================================================
 **/
typedef struct 
{
   float up_poke_angle_ref;
   float up_poke_angle_fdb;
   float up_poke_speed_ref;
   float up_poke_speed_fdb;

   float down_poke_speed_ref;
   float down_poke_speed_fdb;
   float down_poke_angle_ref;
   float down_poke_angle_fdb;
   float down_poke_angle_dynamic_ref;
   float down_poke_speed_dynamic_ref;

   float left_friction_speed_ref;
   float left_friction_speed_fdb;

   float right_friction_speed_ref;
   float right_friction_speed_fdb;

   int16_t up_poke_motor_input;
   int16_t down_poke_motor_input;
   int16_t left_friction_motor_input;
   int16_t right_friction_motor_input;
}shoot_ref_and_fdb_t;

 /**
  ******************************************************************************
																发射结构体		
	 =============================================================================
 **/
typedef struct
{
    shoot_ref_and_fdb_t shoot_ref_and_fdb;
    friction_state_t friction_state;
    u8 shoot_flag; //开火标志位
	u8 inverse_flag;//拨盘反转标志位

    pid_t pid_uppoke_angle;
    pid_t pid_uppoke_speed;

    pid_t pid_downpoke_speed;
	pid_t pid_downpoke_angle;
    
    pid_t pid_left_friction_speed;
    pid_t pid_right_friction_speed;
} _42mm_shoot_t;




void shoot_task(void);
static void Shoot_42mm_speed_Select(uint16_t test_frictionSpeed_42);
void heat_limit_42mm(u8 ifignore);
void shoot_friction_handle_42(void);
void shoot_bullet_handle_42(void);
void shoot_param_init(void);


extern _42mm_shoot_t _42mm_shoot;
extern int bullet_lock_flag;
extern int bullet_locked_flag;











#endif

