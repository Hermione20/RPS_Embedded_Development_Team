#ifndef __CANBUS_H
#define __CANBUS_H
#include "main.h"


/**********************舵轮底盘电机id||麦轮底盘电机id**************************************/
#define GM1Encoder_MOTOR 0x00
#define GM2Encoder_MOTOR 0X00
#define GM3Encoder_MOTOR 0X00
#define GM4Encoder_MOTOR 0X00

#define CM1Encoder_MOTOR 0x201
#define CM2Encoder_MOTOR 0x202
#define CM3Encoder_MOTOR 0x203
#define CM4Encoder_MOTOR 0x204
/*************************云台电机id******************************/
#define GIMBAL_YAW_MOTOR 0x142
#define GIMBAL_PITCH_MOTOR 0x141
/****************************英雄小云台电机id***********************************/
#define SMALL_GIMBAL_MOTOR 0x209
#define SCOPE_MOTOR 0x208
/*********************************摩擦轮电机id**************************************/
#define LEFT_FRICTION1 0X202
#define RIGHT_FRICTION1 0X201
#define LEFT_FRICTION2 0X00
#define RIGHT_FRICTION2 0X00
/*********************************拨盘电机**********************************************/
#define POKE_1 0X205
#define POKE_2 0X205

#define POKE_3 0x203

/*********************************舵轮上下板通信id*********************************/
#define UP_CAN2_TO_DOWN_CAN1_1 0X407
#define UP_CAN2_TO_DOWN_CAN1_2 0X408
#define UP_CAN2_TO_DOWN_CAN1_3 0X409



void Can1ReceiveMsgProcess(CanRxMsg * msg);
void Can2ReceiveMsgProcess(CanRxMsg * msg);




										
void can_bus_send_task(void);

#endif
