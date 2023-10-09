#include "senior.h"

/**
  ******************************************************************************
  * @file    senior.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   此文件为本框架嵌入层的导读文件，这里存有rps电控组所用所有传感器
						 的数据接收与解算，所有现役电机或模块的发送函数，本文件还定义了通用
						 结构体作为接口和上层算法层对接。具体见下注释，同时在本文件的头文件
						 里定义了通用电机的初始编码器，可到senior.h中找寻定义并进行修改,
						 
	* @notice  有关can的模块的接收函数的使用请移步至can_bus.c文件中并在其中的接受
						 函数里选择模块id并调用解算函数，也可在can_bus.h文件中修改模块id
						 有关can的发送函数的使用也移步至can_bus.c文件，并在总发送任务函数中
						 配置要发送的函数，请务必将can_bus_send_task函数的调用放在controltask
						 中。
						 
	* @notice  有关串口的模块接收函数的使用可调用至通用串口模块的头文件中，接收函数
						 请放置在do{}while内
@verbatim
 ===============================================================================
 **/
 
 
/**************general_gyro define**********************/
general_gyro_t gimbal_gyro;
general_gyro_t chassis_gyro;
/*********************general_chassis define***************************/
steering_wheel_t steering_wheel_chassis = {0};
Mecanum_wheel_t Mecanum_chassis  = {0};
/*************************general_gimbal_define****************************************/
volatile Encoder Pitch_Encoder = {0};
volatile Encoder yaw_Encoder = {0};
hero_small_gimbal_t hero_small_gimbal = {0};
/***********************************friction_encoder*****************************************************/
friction_t general_friction = {0};
/************************************poke_encoder*********************************************/
poke_t general_poke = {0};





/******************************capacitance_define*************************************/
//volatile capacitance_message_t capacitance_message;
/*************************************judge define********************************************/
//receive_judge_t judge_rece_mesg;
/******************************************auto_shoot_define***************************************/
//location new_location;
/**********************************************remote_define***************************************/
//RC_Ctl_t RC_CtrlData;
/**************************************************chassis_data************************************/
//chassis_data_t can_chassis_data;




///***************************senior receive function*************************************/
//void CH100_getDATA(uint8_t *DataAddress,general_gyro_t *GYRO);

//void HI220_getDATA(uint8_t *DataAddress,general_gyro_t *GYRO,uint8_t length);

//void M3508orM2006EncoderTask(uint32_t can_count,volatile Encoder *v, CanRxMsg * msg);

//void GM6020EncoderTask(uint32_t can_count,volatile Encoder *v, CanRxMsg * msg,int offset);

//void MF_EncoderTask(uint32_t can_count,volatile Encoder *v, CanRxMsg * msg,int offset);

//void PM01_message_Process(volatile capacitance_message_t *v,CanRxMsg * msg);

//void HT_430_Information_Receive(CanRxMsg * msg,HT430_J10_t *HT430_J10_t,volatile Encoder *v);

//void judgement_data_handle(uint8_t *p_frame,u16 rec_len);

//void vision_process_general_message(unsigned char* address, unsigned int length);

//void send_protocol(float x, float y, float r, int id, float ammo_speed, int gimbal_mode, u8 *data);

//void RemoteDataPrcess(uint8_t *pData);

//void can_chassis_receive_task(CanRxMsg * msg);


/*******************************Module send function***************************************/

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


//void can_chassis_task(CAN_TypeDef *CANx,
//										u8 if_follow_gim,
//										u8 speed_mode,
//										u8 chassis_mode,
//										double yaw_encoder_angle,
//										int16_t yaw_encoder_filter_rate,
//										int16_t x,
//										int16_t y,
//										int16_t rotate_speed,
//										int16_t chassis_power,
//										uint16_t chassis_power_buffer,
//										u8 chassis_power_limit)





