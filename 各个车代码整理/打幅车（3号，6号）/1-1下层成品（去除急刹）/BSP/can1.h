#ifndef __CAN1_H__
#define __CAN1_H__
#include "main.h"

void CAN1_Configuration(void);
void GYRO_RST(void);
void Set_Poke_Current(CAN_TypeDef *CANx, int16_t Poke_iq);
void POWER_Control(u8* msg);
//void Set_ Friction_Current(CAN_TypeDef *CANx, int16_t CAN1_CM1_iq,int16_t CAN1_CM2_iq,int16_t CAN1_CM3_iq);

#endif 
