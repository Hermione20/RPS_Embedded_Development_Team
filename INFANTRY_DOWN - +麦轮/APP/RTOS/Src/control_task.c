#include "control_task.h"

int time_tick=0;


void control_task(void)
{
	time_tick++;

	if(time_tick%8 == 0)
	{
		chassis_task();

	}	
	else if(time_tick%10==9)
		power_send_handle1(CAN1,can_chassis_data.chassis_power_limit);
	else if(time_tick%10==5)
		power_send_handle2(CAN1);
	

	if(time_tick%2 == 0)
	{
	Set_GM6020_IQ1(CAN2,chassis.voltage[0],chassis.voltage[1],chassis.voltage[2],chassis.voltage[3]);
	}
	
	if(time_tick%8==0)
	{
		Set_C620andC610_IQ1(CAN2,chassis.current[0],chassis.current[1],chassis.current[2],chassis.current[3]);
	}
	
		if(time_tick%3000 == 0)
	{ 
//		printf("\n\ryaw %d\n\r Heading:%d %d %d %d \n\rDriving:%d %d %d %d \n\r",can_chassis_data.yaw_Encoder_filter_rate,\
																																steering_wheel_chassis.Heading_Encoder[0].filter_rate,\
																																steering_wheel_chassis.Heading_Encoder[1].filter_rate,\
																																steering_wheel_chassis.Heading_Encoder[2].filter_rate,\
																																steering_wheel_chassis.Heading_Encoder[3].filter_rate,\
																																steering_wheel_chassis.Driving_Encoder[0].filter_rate,\
																																steering_wheel_chassis.Driving_Encoder[1].filter_rate,\
																																steering_wheel_chassis.Driving_Encoder[2].filter_rate,\
																																steering_wheel_chassis.Driving_Encoder[3].filter_rate);
	} 
}


void control_task_init()
{			
		 time_tick = 0;
		 chassis_param_init();
}