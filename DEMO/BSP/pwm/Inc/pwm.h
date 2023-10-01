#ifndef __PWM_H__
#define __PWM_H__

#include "public.h"

#define PWM_STOP 0//待改
#define PWM_UP 1
#define PWM_DOWN 2

#define PWM_MAX 140//舵机机械限位0-270，在机械范围中间移动
#define PWM_MID 133
#define PWM_MIN 85

//#define PWM1  TIM3->CCR3
#define PWM2  TIM3->CCR4//图传俯仰舵机

extern int pwm_flag;
extern u32 pwm_mode;

void PWM_Configuration(void);
void PWM(void);


#endif
