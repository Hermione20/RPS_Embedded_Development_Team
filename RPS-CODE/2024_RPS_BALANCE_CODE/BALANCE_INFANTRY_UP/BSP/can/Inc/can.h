#ifndef __CAN_H
#define __CAN_H	 
#include "public.h"	    


//接收RX0中断使能
#define EN_CAN1	1		//0,不使能;1,使能.								    
#define EN_CAN2 1


void CAN1_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN1初始化
 
void CAN2_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN2初始化 
 
#define CAN1_Data_Receive_Process do{Can1ReceiveMsgProcess(&rx_message);}while(0);
#define CAN2_Data_Receive_Process do{Can2ReceiveMsgProcess(&rx_message);}while(0);
	 
u8 CAN1_Send_Msg(u8* msg,u8 len);						//发送数据
u8 CAN1_Receive_Msg(u8 *buf);							  //接收数据
u8 CAN2_Receive_Msg(u8 *buf);

#endif

















