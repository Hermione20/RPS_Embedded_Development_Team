#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "public.h"

//摩擦轮转速——42mm
#define FRICTION_SPEED_10 2000
#define FRICTION_SPEED_12 2100
#define FRICTION_SPEED_14 2575
#define FRICTION_SPEED_16 3200

#define POKE_SPEED 200			//英雄下拨盘转速
#define POKE_MAX_OUT 6000		//英雄下拨盘力度限制
#define ONE_POKE_ANGLE_2 90.0f	//英雄上拨盘单个弹丸角度
#define k_speed	0.005


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

void shoot_task(void);											//发射任务总成
static void Shoot_42mm_speed_Select(void);	//42mm弹速
void heat1_limit(void);											//热量限制
void shoot_friction_handle(void);						//摩擦轮部分
void shoot_bullet_handle(void);							//拨盘部分

void friction_lock(void);										//摩擦轮堵转部分


void shot_param_init(void);									//发射机构PID参数初始化

#endif
