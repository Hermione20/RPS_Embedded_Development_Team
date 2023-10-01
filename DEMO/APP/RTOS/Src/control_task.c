#include "control_task.h"


int time_tick = 0;

void control_task(void)
{
	time_tick++;
	buff_karman_filter_calc(&buff_kalman_filter,yaw_angle_ref_aim,pit_angle_ref_aim,&new_location.buff_kf_flag);
	if(time_tick%10 == 0)
		can_chassis_task(CAN2,chassis.follow_gimbal,chassis.chassis_speed_mode,chassis.ctrl_mode,120.0,200,chassis.ChassisSpeed_Ref.left_right_ref,chassis.ChassisSpeed_Ref.forward_back_ref,15,23,4);
	if(time_tick%2 == 0)
	{
		gimbal_task();
		mode_switch_task();
	}
	
}
