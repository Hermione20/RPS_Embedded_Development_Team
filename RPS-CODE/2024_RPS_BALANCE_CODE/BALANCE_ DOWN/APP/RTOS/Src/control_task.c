#include "control_task.h"


int time_tick = 0;
float lp_data = 0;

void control_task(void)
{
	time_tick++;
	IWDG_ReloadCounter();            //Î¹¹·
	lp_data = Lpf_1st_calcu(&ACC_LPF,chassis_gyro.x_Acc,15,0.001);
	Mileage_kalman_filter_calc(&Mileage_kalman_filter,
								((balance_chassis.Driving_Encoder[0].angle + (-balance_chassis.Driving_Encoder[1].angle))/2.0f) * WHEEL_R,
								((balance_chassis.Driving_Encoder[0].gyro + (-balance_chassis.Driving_Encoder[1].gyro))/2.0f) * WHEEL_R,
								lp_data);
	
	if(time_tick%2==0)
	{
		balance_chassis_task();
		can_bus_send_task();
		
	}
	
	if(time_tick%10==9)
	{
		power_data_set_handle(CAN2,(uint16_t)(b_chassis.Max_power_to_PM01));
	}
	if(time_tick%10==5)
	{
		power_data_read_handle(CAN2);
		usart_gimbal_send(capacitance_message.cap_voltage_filte);
	}
		
	if(time_tick%1000==0)
		{
			LED0_ON;
			HARD_WDG_ON;
		}
		if(time_tick%2000==0)
		{
			LED0_OFF;
			HARD_WDG_OFF;
		}
	
}

void control_task_Init(void)
{
	balance_param_init();
}
