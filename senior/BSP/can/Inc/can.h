#ifndef __CAN_H
#define __CAN_H	 
#include "public.h"


	
//接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    
#define CAN2_RX0_INT_ENABLE 1

#define CAN1_TX0_INT_ENABLE 1
#define CAN2_TX0_INT_ENABLE 1


#define CAN1_Data_Receive_Process  do{}while(0);
#define CAN2_Data_Receive_Process  do{}while(0);	





u8 CAN1_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化 
u8 CAN2_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化 
u8 CAN1_Send_Msg(u8* msg,u8 len);						//发送数据
u8 CAN1_Receive_Msg(u8 *buf);							//接收数据
u8 CAN2_Receive_Msg(u8 *buf);

void CAN1_Configuration(void);
void set_M3508_info(void);
void CAN2_Configuration(void);
void Set_Gimbal_Current1(CAN_TypeDef *CANx, int16_t ch_1_iq, int16_t ch_2_iq, int16_t ch_3_iq, int16_t ch_4_iq);
#endif

















