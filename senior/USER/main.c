#include "main.h"


int main()
{
		
   BSP_Init();

	while(1)
	{

		gimbal_task();
		mode_switch_task();
		
	}
}
