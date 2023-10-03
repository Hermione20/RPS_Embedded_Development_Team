#include "control_task.h"


int time_tick = 0;

void control_task(void)
{
	time_tick++;
	if(time_tick)
	{
		gimbal_parameter_Init();
	}
	
	buff_karman_filter_calc(&buff_kalman_filter,yaw_angle_ref_aim,pit_angle_ref_aim,&new_location.buff_kf_flag);
		
	if(time_tick%2 == 0)
	{
		gimbal_task();		
	}
	
	can_bus_send_task();
	
}
