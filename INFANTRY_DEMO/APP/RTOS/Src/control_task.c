#include "control_task.h"


int time_tick = 0;

void control_task(void)
{
	time_tick++;

	buff_karman_filter_calc(&buff_kalman_filter,yaw_angle_ref_aim,pit_angle_ref_aim,&new_location.buff_kf_flag);
	
	if(time_tick%10==0)
	{
		can_chassis_task(CAN2,chassis.follow_gimbal,
							chassis.chassis_speed_mode ,
							chassis.ctrl_mode,yaw_Encoder.ecd_angle,
							yaw_Encoder.filter_rate,
							chassis.ChassisSpeed_Ref.left_right_ref,
							chassis.ChassisSpeed_Ref.forward_back_ref,
							550,
							judge_rece_mesg.power_heat_data.chassis_power,
							judge_rece_mesg.power_heat_data.chassis_power_buffer,
							judge_rece_mesg.game_robot_state.chassis_power_limit);
	}
		
	if(time_tick%2 == 0)
	{
		gimbal_task();
			can_bus_send_task();
	}
	
	
	
}
