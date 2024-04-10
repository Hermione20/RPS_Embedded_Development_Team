#ifndef __CAN_CHASSIS_TRANSMIT_H
#define __CAN_CHASSIS_TRANSMIT_H
#include "public.h"




typedef struct
{
	u8 if_follow_gim;
	u8 speed_mode;
	u8 chassis_mode;
	int64_t yaw_Encoder_ecd_angle;
	int16_t cmd_leg_length;
	int16_t x;
	int16_t y;
	int16_t rotate_speed;
	int16_t chassis_power;
	uint16_t chassis_power_buffer;
	u8 chassis_power_limit;
} chassis_data_t;



void can_chassis_send1(CAN_TypeDef *CANx);
void can_chassis_send2(CAN_TypeDef *CANx);
void can_chassis_send3(CAN_TypeDef *CANx);
void can_chassis_task(CAN_TypeDef *CANx,u8 if_follow_gim,
										u8 speed_mode,
										u8 chassis_mode,
										double yaw_encoder_angle,
										int16_t cmd_leg_length,
										int16_t x,
										int16_t y,
										int16_t rotate_speed,
										int16_t chassis_power,
										uint16_t chassis_power_buffer,
										u8 chassis_power_limit);
void can_chassis_receive_task(CanRxMsg * msg);


extern chassis_data_t can_chassis_data;












#endif

