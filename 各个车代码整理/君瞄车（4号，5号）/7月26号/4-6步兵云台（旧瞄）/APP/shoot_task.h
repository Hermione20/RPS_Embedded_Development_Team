
/****************************************************************************
 *  @file shoot_task.h
 *  @version 1.1
 *  @date June 2017
 *
 *  @brief shoot bullet task
 *
 ***************************************************************************/

#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__
#include "main.h"
/* shoot task control period time (ms) */

#if  STANDARD == 3

#define FRICTION_SPEED_15  (-540)      //弹速
#define FRICTION_SPEED_18  (-607)
#define FRICTION_SPEED_30  (-935)

#define residual_heat_normal_else_1 (40)
#define residual_heat_normal_else_2 (28)
#define residual_heat_normal_else_3 (28)
#define residual_heat_normal_else_4 (25)


#define residual_heat_normal_coling_1 (32)
#define residual_heat_normal_coling_2 (25)
#define residual_heat_normal_coling_3 (27)

#define residual_heat_normal_speed_1 (27)
#define residual_heat_normal_speed_2 (27)
#define residual_heat_normal_speed_3 (25)

#define residual_heat_normal_outburst_1 (30)
#define residual_heat_normal_outburst_2 (23)
#define residual_heat_normal_outburst_3 (40)

#define residual_heat_middle_coling_1 (100)
#define residual_heat_middle_coling_2 (35)
#define residual_heat_middle_coling_3 (34)


#define residual_heat_middle_speed_1 (100)
#define residual_heat_middle_speed_2 (45)
#define residual_heat_middle_speed_3 (43)


#define residual_heat_middle_outburst_1 (100)
#define residual_heat_middle_outburst_2 (35)
#define residual_heat_middle_outburst_3 (80)

//射频
#define PID_SHOOT_MOTOR_SPEED_1    (-300)//-400
#define PID_SHOOT_MOTOR_SPEED_2    (-400)//-250
#define PID_SHOOT_MOTOR_SPEED_3    (-450)
#define PID_SHOOT_MOTOR_SPEED_4    (-500)
#define PID_SHOOT_MOTOR_SPEED_5    (-600)

#elif STANDARD ==4
#define FRICTION_SPEED_15  (-560)      //弹速
#define FRICTION_SPEED_18  (-607)
#define FRICTION_SPEED_30  (-935)

#define residual_heat_normal_else_1 (40)
#define residual_heat_normal_else_2 (28)
#define residual_heat_normal_else_3 (28)
#define residual_heat_normal_else_4 (25)


#define residual_heat_normal_coling_1 (35)
#define residual_heat_normal_coling_2 (33)
#define residual_heat_normal_coling_3 (25)

#define residual_heat_normal_speed_1 (34)
#define residual_heat_normal_speed_2 (34)
#define residual_heat_normal_speed_3 (25)

#define residual_heat_normal_outburst_1 (34)
#define residual_heat_normal_outburst_2 (34)
#define residual_heat_normal_outburst_3 (35)

#define residual_heat_middle_coling_1 (100)
#define residual_heat_middle_coling_2 (42)
#define residual_heat_middle_coling_3 (42)


#define residual_heat_middle_speed_1 (100)
#define residual_heat_middle_speed_2 (38)
#define residual_heat_middle_speed_3 (47)


#define residual_heat_middle_outburst_1 (100)
#define residual_heat_middle_outburst_2 (60)
#define residual_heat_middle_outburst_3 (65)

#define PID_SHOOT_MOTOR_SPEED_1    (-340)//-400
#define PID_SHOOT_MOTOR_SPEED_2    (-390)//-250
#define PID_SHOOT_MOTOR_SPEED_3    (-440)
#define PID_SHOOT_MOTOR_SPEED_4    (-490)
#define PID_SHOOT_MOTOR_SPEED_5    (-590)


#elif STANDARD == 5
#define FRICTION_SPEED_15  (-550)//弹速
#define FRICTION_SPEED_18  (-630)
#define FRICTION_SPEED_30  (-895)

#define residual_heat_normal_else_1 (40)
#define residual_heat_normal_else_2 (30)
#define residual_heat_normal_else_3 (30)
#define residual_heat_normal_else_4 (30)


#define residual_heat_normal_coling_1 (28)
#define residual_heat_normal_coling_2 (28)
#define residual_heat_normal_coling_3 (25)

#define residual_heat_normal_speed_1 (27)
#define residual_heat_normal_speed_2 (27)
#define residual_heat_normal_speed_3 (25)

#define residual_heat_normal_outburst_1 (35)
#define residual_heat_normal_outburst_2 (40)
#define residual_heat_normal_outburst_3 (40)

#define residual_heat_middle_coling_1 (100)
#define residual_heat_middle_coling_2 (40)
#define residual_heat_middle_coling_3 (40)


#define residual_heat_middle_speed_1 (100)
#define residual_heat_middle_speed_2 (45)
#define residual_heat_middle_speed_3 (45)


#define residual_heat_middle_outburst_1 (100)
#define residual_heat_middle_outburst_2 (50)
#define residual_heat_middle_outburst_3 (50)

#define PID_SHOOT_MOTOR_SPEED_1    (-350)//-400
#define PID_SHOOT_MOTOR_SPEED_2    (-400)//-250
#define PID_SHOOT_MOTOR_SPEED_3    (-450)
#define PID_SHOOT_MOTOR_SPEED_4    (-500)
#define PID_SHOOT_MOTOR_SPEED_5    (-600)


#elif STANDARD == 6
#define FRICTION_SPEED_15  (-541)//弹速
#define FRICTION_SPEED_18  (-595)
#define FRICTION_SPEED_30  (-910)


#define residual_heat_normal_else_1 (44)
#define residual_heat_normal_else_2 (47)
#define residual_heat_normal_else_3 (41)
#define residual_heat_normal_else_4 (38)

#define residual_heat_normal_coling_1 (33)
#define residual_heat_normal_coling_2 (27)
#define residual_heat_normal_coling_3 (40)

#define residual_heat_normal_speed_1 (32)
#define residual_heat_normal_speed_2 (32)
#define residual_heat_normal_speed_3 (34)

#define residual_heat_normal_outburst_1 (33)
#define residual_heat_normal_outburst_2 (33)
#define residual_heat_normal_outburst_3 (33)

#define residual_heat_middle_coling_1 (100)
#define residual_heat_middle_coling_2 (44)
#define residual_heat_middle_coling_3 (44)


#define residual_heat_middle_speed_1 (100)
#define residual_heat_middle_speed_2 (58)
#define residual_heat_middle_speed_3 (35)
#define residual_heat_middle_outburst_1 (100)
#define residual_heat_middle_outburst_2 (50)
#define residual_heat_middle_outburst_3 (50)

#define residual_heat_middle_outbreak_1 (100)
#define residual_heat_middle_outbreak_2 (48)
#define residual_heat_middle_outbreak_3 (35)

#define PID_SHOOT_MOTOR_SPEED_1    (-350)//-400
#define PID_SHOOT_MOTOR_SPEED_2    (-400)//-250
#define PID_SHOOT_MOTOR_SPEED_3    (-450)
#define PID_SHOOT_MOTOR_SPEED_4    (-500)
#define PID_SHOOT_MOTOR_SPEED_5    (-600)

#endif

#define residual_heat_1 (40)
#define residual_heat_2 (35)
#define residual_heat_3 (30)
#define residual_heat_4 (25)
#define residual_heat_5 (20)
#define residual_heat_6 (20)

typedef enum
{
  outburst  = 0,
  cooling   = 1,
  speed     = 2,

} shoot_mode_selection_e;

typedef enum
{
  SHOT_DISABLE       = 0,
  REMOTE_CTRL_SHOT   = 1,
  KEYBOARD_CTRL_SHOT = 2,
  SEMIAUTO_CTRL_SHOT = 3,
  AUTO_CTRL_SHOT     = 4,

} shoot_mode_e;

typedef enum
{
  TRIG_INIT       = 0,
  TRIG_PRESS_DOWN = 1,
  TRIG_BOUNCE_UP  = 2,
  TRIG_ONE_DONE   = 3,
} trig_state_e;

typedef __packed struct
{
  /* shoot task relevant param */
  shoot_mode_e ctrl_mode;
  uint8_t      shoot_cmd;
  uint32_t     c_shoot_time;   //continuous
  uint8_t      c_shoot_cmd;
  uint8_t      fric_wheel_run; //run or not
  uint16_t     fric_wheel_spd;
  uint16_t     ref_shot_bullets;
  uint16_t     shot_bullets;
  uint16_t     remain_bullets;
  float        total_speed;
  float        limit_heart0;
  uint16_t     max_heart0;
  uint16_t     handle_timescouter;
  uint16_t     cooling_ratio;
  uint16_t     ShootMotorSpeed;
  uint16_t     NoLimitHeat;
  uint8_t			 Speed_Gear;
} shoot_t;

typedef __packed struct
{
  /* trigger motor param */
  int32_t   spd_ref;
  int32_t   pos_ref;
  int8_t    dir;
  uint8_t   key;
  uint8_t   key_last;
  uint32_t  one_time;
  int32_t   feed_bullet_spd;
  int32_t   c_shot_spd;
  trig_state_e one_sta;
} trigger_t;

typedef enum
{
  SHOOT_CMD,
  FRIC_CTRL,
} shoot_type_e;
void Mode_switch(void);
void System_performance(void);
void Speed_switch(void);
void Heat0_switch(void);
void shot_param_init(void);
void shot_task(void);
void heat0_limit(void);
static void shoot_bullet_handle(void);
static void fric_wheel_ctrl(void);
void shoot_friction_handle(void);
void shoot_buff_handle(void);
extern shoot_t   shot;
extern trigger_t trig;
extern u8 friction_rotor;
extern u8 Hove;
extern float Delta_Dect_Angle_Yaw;
extern float Delta_Dect_Angle_Pit;
extern float shoot_frequency;

extern int buff_shoot_flag_time;

extern float shoot_flag;
extern int BUFF_shoot_flag;
extern float FRICTION_SPEED_plan_ref;
extern FrictionWheelState_e friction_wheel_state;
extern shoot_mode_selection_e shoot_mode_selection;
#endif
