#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "public.h"

//摩擦轮转速——42mm
#define FRICTION_SPEED_10 2000
#define FRICTION_SPEED_12 2100
#define FRICTION_SPEED_14 2575
#define FRICTION_SPEED_16 3200

typedef enum
{
	STOP=0，
  NOMAL=1,
  LOCK=2,
} shoot_state_e;  //0正常 1堵转




void shoot_task(void);											//发射任务总成
static void Shoot_42mm_speed_Select(void);	//42mm弹速
void heat1_limit(void);											//热量限制
void shoot_bullet_handle(void);							//拨盘部分
void shoot_friction_handle(void);						//摩擦轮部分

void friction_lock(void);										//摩擦轮堵转部分


void shot_param_init(void);									//发射机构PID参数初始化


#endif
