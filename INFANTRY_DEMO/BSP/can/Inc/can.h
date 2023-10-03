#ifndef __CAN_H
#define __CAN_H	 
#include "public.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//CAN驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/7
//版本：V1.0 
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

	
//接收RX0中断使能
#define EN_CAN1	1		//0,不使能;1,使能.								    
#define EN_CAN2 1


void CAN1_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
 
void CAN2_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化 
 
#define CAN1_Data_Receive_Process do{Can1ReceiveMsgProcess(&rx_message);}while(0);
#define CAN2_Data_Receive_Process do{Can2ReceiveMsgProcess(&rx_message);}while(0);
	 
u8 CAN1_Send_Msg(u8* msg,u8 len);						//发送数据

u8 CAN1_Receive_Msg(u8 *buf);							//接收数据

u8 CAN2_Receive_Msg(u8 *buf);

void CAN1_Configuration(void);
void set_M3508_info(void);
void CAN2_Configuration(void);
void Set_Gimbal_Current1(CAN_TypeDef *CANx, int16_t ch_1_iq, int16_t ch_2_iq, int16_t ch_3_iq, int16_t ch_4_iq);
#endif

















