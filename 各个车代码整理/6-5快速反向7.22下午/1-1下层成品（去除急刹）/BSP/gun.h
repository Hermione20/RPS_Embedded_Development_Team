#ifndef __GUN_H__
#define __GUN_H__
#include "config.h"
#ifndef FRICTION_WHEEL
#define FRICTION_WHEEL
#endif 

#if defined(FRICTION_WHEEL)

#define PWM3  TIM9->CCR1
#define PWM4  TIM3->CCR3
#define PWM5  TIM3->CCR4
#define PWM6  TIM5->CCR1
#define PWM7  TIM5->CCR2
#define InitFrictionWheel()     
#define SetFrictionWheelSpeed(x) \
        PWM1 = x;                \
        PWM2 = x;

#endif 
  


//标识3号步兵
#if STANDARD == 3
#define CloseDoor PWM4=73,PWM5=195;//4-left    40  200
#define OpenDoor  PWM4=180,PWM5=88;
#elif STANDARD ==4
////标识四号步兵
	#define CloseDoor PWM4=82,PWM5=157;//zuo PWM5  48  195
	#define OpenDoor PWM4=190,PWM5=65;
#elif STANDARD ==5
////标识四号步兵
//a0,a1,c8,c9,对应1，2，3，4，对应PWM6,PWM7,PWM4,PWM5
	#define CloseDoor PWM4=89,PWM5=103,PWM6=143,PWM7=124;//PWM4 ZUO
	
	#define OpenDoor PWM4=120,PWM5=70,PWM6=174,PWM7=155;
#elif STANDARD ==6
////标识四号步兵
	#define CloseDoor PWM4=200,PWM5=32;//zuo PWM5  198 34
	#define OpenDoor PWM4=85,PWM5=150;
#else
	"ERROR,please define STANDARD as 3 or 4 or 5"
#endif

extern int door_flag;
void PWM_Configuration(void);
void PWM_Configuration1(void);
 
#endif /* __GUN_H__*/

