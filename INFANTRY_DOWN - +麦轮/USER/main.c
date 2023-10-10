#include "main.h"
int i;

int main()
{
   BSP_Init();
	 control_task_init();

	 
	
	while(1)
	{

		if(i%84000==0)
		{

		}
		i++;
	}
}
