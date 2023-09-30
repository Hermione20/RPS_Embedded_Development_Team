#ifndef __PM01_H
#define __PM01_H
#include "public.h"


/*******************超级电容控制模块*********************************/
typedef struct
{

	uint16_t mode;
	uint16_t mode_sure;
	
	uint16_t in_power;
	uint16_t in_v;
	uint16_t in_i;
	
	uint16_t out_power;
	uint16_t out_v;
	uint16_t out_i;

	uint16_t tempureture;
	uint16_t time;
	uint16_t this_time;
	
	uint16_t  cap_voltage_filte;
}capacitance_message_t;



void PM01_message_Process(volatile capacitance_message_t *v,CanRxMsg * msg);



extern volatile capacitance_message_t capacitance_message;






#endif
