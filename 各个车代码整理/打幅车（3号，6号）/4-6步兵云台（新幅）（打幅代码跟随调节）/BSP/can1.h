#ifndef __CAN1_H__
#define __CAN1_H__
#include "main.h"

void CAN1_Configuration(void);
void GYRO_RST(void);
void Set_Poke_Current(CAN_TypeDef *CANx, int16_t Poke_iq);
void POWER_Control(u8* msg);
void sendcan1(CAN_TypeDef *CANx, int16_t romate_speed,int16_t romate_angle,int16_t get_speedw,int16_t start_angle);
void sendcan2(CAN_TypeDef *CANx, int16_t get_control_flag,int16_t get_mode_flag,int16_t die_flag,int16_t speed_yaw);
void sendcan3(CAN_TypeDef *CANx, int16_t chassis_power_buffer,int16_t mains_power_chassis_output,int16_t chassis_power_limit,int16_t chassis_power);
//void Set_ Friction_Current(CAN_TypeDef *CANx, int16_t CAN1_CM1_iq,int16_t CAN1_CM2_iq,int16_t CAN1_CM3_iq);


#endif 
