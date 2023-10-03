#ifndef __MODE_SWITCH_TASK_H
#define __MODE_SWITCH_TASK_H
#include "public.h"



/*************************************************/
#define HIGH_SPEED 1600
#define NORMAL_SPEED 550 

#define HIGH_SPEED_MODE 1
#define NORMAL_SPEED_MODE 0



void mode_switch_task(void);


extern chassis_t chassis;




#endif
