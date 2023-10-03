#include "main.h"
int i;

int main()
{
   BSP_Init();
	 PID_struct_init(&pid_yaw ,POSITION_PID ,15000,10000,0, 3, 2, 0);
	 
	
	while(1)
	{
		
			if(i%84000==0)
			{pid_calc(&pid_yaw,yaw_Encoder.filter_rate,100);
			Set_GM6020_IQ1(CAN2,0,0,pid_yaw.out,0);
			gimbal_task();
			mode_switch_task();
			
			}//		get_romote_set();
//			chassis_task();
			
			i++;
	}
}
