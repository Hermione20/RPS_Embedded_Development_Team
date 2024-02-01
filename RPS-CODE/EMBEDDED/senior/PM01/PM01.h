#ifndef __PM01_H
#define __PM01_H
#include "public.h"


/*******************超级电容控制模块*********************************/
typedef struct
{

	uint16_t mode;
	uint16_t err_fdb;
	
	uint16_t in_power;
	uint16_t in_v;
	uint16_t in_i;
	
	uint16_t out_power;
	uint16_t out_v;
	uint16_t out_i;

	uint16_t tempureture;
	uint16_t time;
	uint16_t this_time;
	
	float cap_voltage_filte;  //电容输出电压
}capacitance_message_t;



void PM01_message_Process(volatile capacitance_message_t *v,CanRxMsg * msg);

void PM01_command_set(CAN_TypeDef *CANx ,uint16_t data,uint32_t StdId);
void PM01_data_read(CAN_TypeDef *CANx ,uint32_t StdId);
void power_data_read_handle(CAN_TypeDef *CANx);
void power_data_set_handle(CAN_TypeDef *CANx,u16 Max_Power);
void power_data_Init(CAN_TypeDef *CANx);

extern volatile capacitance_message_t capacitance_message;






#endif
