#include "iwdg.h"
void IWDG_Configuration(void)	
{
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //IWDG->PR IWDG->RLR  
	IWDG_SetPrescaler(IWDG_Prescaler_4); 
	IWDG_SetReload(100);   //4ms   2*Reload
	IWDG_ReloadCounter(); 
	IWDG_Enable(); 
}	
 