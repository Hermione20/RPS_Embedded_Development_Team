#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "public.h"

/********** pid结构体说明 ************
	单枪管发射机构，摩擦轮
		pid_friction_whell_speed[0]
		pid_friction_whell_speed[1]
	烧饼双枪管，摩擦轮
		pid_friction_whell_speed[2]
		pid_friction_whell_speed[3]
	英雄下拨盘
		pid_42mm_poke_speed
	英雄上拨盘
		pid_42mm_poke2_angle
		pid_42mm_poke2_speed
	步兵，飞机，烧饼左拨盘
		pid_17mm_poke_angle
		pid_17mm_poke_speed
**************************************/

/********** 电机编码器结构体说明 *********
	单枪管发射机构，摩擦轮
		general_friction.left_motor1
		general_friction.right_motor1
	烧饼双枪管，摩擦轮
		general_friction.left_motor2
		general_friction.right_motor2
	英雄下拨盘，哨兵左拨盘，步兵拨盘
		general_poke.left_poke
	英雄上波盘，哨兵右拨盘
		general_poke.right_poke
**************************************/
//发射机构种类选择
#define TYPE 1			//1：42mm		2：17mm		3:17mm x2
#define STANDARD 3
// <o> STANDARD  - 几号步兵
// <3=> NUM_3
// <4=> NUM_4
// <5=> NUM_5
#define STANDARD 			6
//选择步兵为3号 或者4号

//摩擦轮转速——42mm
#define FRICTION_SPEED_10 2000
//#define FRICTION_SPEED_12 2100
//#define FRICTION_SPEED_14 2575
#define FRICTION_SPEED_16 3200

#define POKE_SPEED 200			//英雄下拨盘转速
#define POKE_MAX_OUT 6000		//英雄下拨盘力度限制
#define ONE_POKE_ANGLE_42 90.0f	//42mm单个弹丸角度
#define ONE_POKE_ANGLE_17 30.0f	//17mm单个弹丸角度
#define k_speed	0.005


//摩擦轮转速——17mm
#define FRICTION_SPEED_15 2000
#define FRICTION_SPEED_18 2100
#define FRICTION_SPEED_30 2575

#define SINGLE_ANGLE  60

typedef enum
{
	Stop=0,
  NORMAL=1,
	BACK=2,
  LOCK=3,
} friction_state_t ;  //1正常 2堵转

typedef enum
{
	NOSHOOTING=0,
	SHOOTINT=1,
}	shoot_state_e;		//0不发射	1发射

typedef enum
{
  outburst  = 0,
  cooling   = 1,
  speed     = 2,
} shoot_mode_selection_e;		//步兵发射模式选择

void shoot_task(void);		//发射任务总成
void Mode_switch(void);		//步兵发射模式选择

static void Shoot_42mm_speed_Select(void);		//42mm弹速
static void Shoot_17mm_speed_Select(void);		//17mm弹速
static void Shoot_17mm_speed_Select_double(void);//17mm弹速,双枪管

void heat1_limit_42mm(void);								//热量限制,42mm
void heat1_limit_17mm(void);								//热量限制,17mm
void heat1_limit_17mm_double(void);					//热量限制,17mm,双枪管

void shoot_friction_handle_42(void);				//摩擦轮部分，42mm
void shoot_friction_handle_17(void);				//摩擦轮部分，17mm
void shoot_friction_handle_17_double(void);	//摩擦轮部分，17mm,双枪管

void shoot_bullet_handle_42(void);					//42mm拨盘部分

void friction_lock(void);										//摩擦轮堵转部分
void Heat0_switch(void);


void shot_param_init(void);									//发射机构PID参数初始化

#endif
