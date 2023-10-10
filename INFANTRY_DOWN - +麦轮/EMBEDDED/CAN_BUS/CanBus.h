#ifndef __CANBUS_H
#define __CANBUS_H
#include "main.h"


/**********************麦轮底盘电机id||舵轮底盘电机id**************************************/
#define CM1Encoder_MOTOR 0x201
#define CM2Encoder_MOTOR 0x202
#define CM3Encoder_MOTOR 0x203
#define CM4Encoder_MOTOR 0x204

#define GM1Encoder_MOTOR 0x205
#define GM2Encoder_MOTOR 0X206
#define GM3Encoder_MOTOR 0X207
#define GM4Encoder_MOTOR 0X208
/*************************云台电机id******************************/
#define GIMBAL_YAW_MOTOR 0X205
#define GIMBAL_PITCH_MOTOR 0X00
/****************************英雄小云台电机id***********************************/
#define SMALL_GIMBAL_MOTOR 0X00
#define SCOPE_MOTOR 0X00
/*********************************摩擦轮电机id**************************************/
#define LEFT_FRICTION1 0X00
#define RIGHT_FRICTION1 0X00
#define LEFT_FRIICTION2 0X00
#define RIGHT_FRICTION2 0X00
/*********************************舵轮上下板通信id*********************************/
#define UP_CAN2_TO_DOWN_CAN1_1 0X407
#define UP_CAN2_TO_DOWN_CAN1_2 0X408
#define UP_CAN2_TO_DOWN_CAN1_3 0x409



void Can1ReceiveMsgProcess(CanRxMsg * msg);
void Can2ReceiveMsgProcess(CanRxMsg * msg);

//void Set_GM6020_IQ1(CAN_TypeDef *CANx, int16_t motor1_iq, int16_t motor2_iq, int16_t motor3_iq, int16_t motor4_iq);
//void Set_GM6020_IQ2(CAN_TypeDef *CANx, int16_t motor5_iq, int16_t motor6iq, int16_t motor7_iq, int16_t motor8_iq);
//void Set_C620andC610_IQ1(CAN_TypeDef *CANx, int16_t motor1_iq, int16_t motor2_iq, int16_t motor3_iq, int16_t motor4_iq);
//void Set_C620andC610_IQ2(CAN_TypeDef *CANx, int16_t motor5_iq, int16_t motor6_iq, int16_t motor7_iq, int16_t motor8_iq);
//void CAN_9015Command(CAN_TypeDef *CANx ,uint8_t command,uint8_t id);
//void CAN_9015setpidCommand(CAN_TypeDef *CANx, float akp,
//                           float aki,
//                           float skp,
//                           float ski,
//                           float iqkp,
//                           float iqki, uint8_t id);
//void CAN_9015angleControl(CAN_TypeDef *CANx ,int16_t maxSpeed ,uint32_t angleControl,uint8_t id);
//void CAN_9015speedControl(CAN_TypeDef *CANx ,uint32_t speedControl,uint8_t id);
//void CAN_9015torsionControl(CAN_TypeDef *CANx ,int16_t iqcontrol,uint8_t id);
//void POWER_Control1(CAN_TypeDef *CANx ,uint16_t Power,uint16_t StdId);
//void POWER_Control1l(CAN_TypeDef *CANx ,uint16_t StdId);
//void power_send_handle2(CAN_TypeDef *CANx);
//void power_send_handle1(CAN_TypeDef *CANx,u16 Max_Power);
//void HT_430_Encoder_Calibration(CAN_TypeDef *CANx,int ID);
//void HT_430_Encoder_Origin(CAN_TypeDef *CANx,int ID);
//void Motor_Information_Request(CAN_TypeDef *CANx,int ID);
//void HT_430_Fault_Clear(CAN_TypeDef *CANx,int ID);
//void HT_430_Tuen_Off(CAN_TypeDef *CANx,int ID);
//void HT_430_Origin_Total(CAN_TypeDef *CANx,int ID);
//void HT_430_Back(CAN_TypeDef *CANx,int ID);
//void HT_430_Power_Open_Loop(CAN_TypeDef *CANx,int ID,int16_t Pow);
//void HT_430_V_Clossed_Loop(CAN_TypeDef *CANx,int ID,int16_t V);
//void HT_430_Absolute_Position_closed_Loop(CAN_TypeDef *CANx,int ID,uint32_t Angle);
//void HT_430_Relative_Position_closed_Loop(CAN_TypeDef *CANx,int ID,uint32_t Angle);
//void HT_430_Position_closed_Loop_T_R_OR_W(CAN_TypeDef *CANx,int ID,int16_t V,int Flag_RW);


#endif
