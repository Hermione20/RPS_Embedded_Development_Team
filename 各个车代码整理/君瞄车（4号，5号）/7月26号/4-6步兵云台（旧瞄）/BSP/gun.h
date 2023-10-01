#ifndef __GUN_H__
#define __GUN_H__
#include "config.h"
#ifndef FRICTION_WHEEL
#define FRICTION_WHEEL
#endif 

#if defined(FRICTION_WHEEL)

#define PWM3  TIM9->CCR1
#define PWM4  TIM3->CCR3//弹仓
#define PWM5  TIM3->CCR4
#define InitFrictionWheel()     
#define SetFrictionWheelSpeed(x) \
        PWM1 = x;                \
        PWM2 = x;

#endif 
  


//标识3号步兵
#if STANDARD == 3
#define CloseDoor PWM4=75,PWM5=150;//4-left    40  200    //73 180   195  88    90 32
#define OpenDoor  PWM4=165,PWM5=65;
#elif STANDARD ==4
////标识四号步兵
////标识四号步兵
#define CloseDoor PWM4=174,PWM5=65;//右 PWM4  
#define OpenDoor PWM4=90,PWM5=155;
#elif STANDARD ==5
////标识四号步兵
#define CloseDoor PWM4=150,PWM5=50;//4-left    40  200    //73 180   195  88    90 32
#define OpenDoor  PWM4=70,PWM5=120;
//#define CloseDoor PWM4=50,PWM5=250;//4-left    40  200
//#define OpenDoor  PWM4=100,PWM5=170;
#elif STANDARD ==6
////标识四号步兵
	#define CloseDoor PWM4=55,PWM5=95;//zuo PWM5  198 34
	#define OpenDoor PWM4=110,PWM5=45;
#else
	"ERROR,please define STANDARD as 3 or 4 or 5"
#endif

extern int door_flag;
void PWM_Configuration(void);
 
#endif /* __GUN_H__*/

