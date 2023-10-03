#include "main.h"


int main()
{
   BSP_Init();
	PID_struct_init(&gimbal_data.pid_yaw_Angle,POSITION_PID,500,4,10,0.1,0);
    PID_struct_init(&gimbal_data.pid_yaw_speed, POSITION_PID, 29000, 10000, 30,0,0);
	 
	
	while(1)
	{
		

	}
}
