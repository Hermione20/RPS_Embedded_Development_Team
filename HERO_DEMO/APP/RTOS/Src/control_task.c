#include "control_task.h"


int time_tick = 0;

void control_task(void)
{
	time_tick++;
	if(time_tick%10 == 0)
	{
		chassis_task();
	}
		
	if(time_tick%2 == 0)
	{
		gimbal_task();
			can_bus_send_task();
	}
	
	
	
}


void control_task_Init(void)
{
	gimbal_parameter_Init();
	chassis_param_init();
	
}

