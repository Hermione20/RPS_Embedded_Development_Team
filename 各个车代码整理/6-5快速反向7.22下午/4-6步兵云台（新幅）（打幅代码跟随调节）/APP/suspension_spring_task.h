#ifndef __spring_TASK_H__
#define __spring_TASK_H__
#include "main.h"



void spring_task(void);
void spring_handle(void);
void spring_param_init(void);

typedef enum
{
	SPRING_NORMAL_STATE = 0,
	SPRING_STATE_1 = 1,
	SPRING_STATE_2 = 2,
	SPRING_STATE_3 = 3,
	}spring_State_e;


#endif


