#ifndef __SENIOR_H
#define __SENIOR_H
#include "main.h"


#define  GMPitchEncoder_Offset 0
//yaw轴电机初始位置
#define  GMYawEncoder_Offset   4758
//底盘航向轴电机初始位置
#define  GM1Encoder_Offset   1437
#define  GM2Encoder_Offset   8042
#define  GM3Encoder_Offset   4141
#define  GM4Encoder_Offset   6732



/********************general chassis encoder********************************/


typedef struct 
{
	volatile Encoder right_front_GM6020;
	volatile Encoder left_front_GM6020;
	volatile Encoder left_behind_GM6020;
	volatile Encoder right_behind_GM6020;

	volatile Encoder right_front_motor;
	volatile Encoder left_front_motor;
	volatile Encoder left_behind_motor;
	volatile Encoder right_behind_motor;
}steering_wheel_t;

typedef struct
{
	volatile Encoder right_front_motor;
	volatile Encoder left_front_motor;
	volatile Encoder left_behind_motor;
	volatile Encoder right_behind_motor;
}Mecanum_wheel_t;

/***************************general friction encoder********************************************/
typedef struct 
{
	volatile Encoder right_motor1;
	volatile Encoder left_motor1;
	volatile Encoder left_motor2;
	volatile Encoder right_motor2;
}friction_t;

/************************************general poke encoder******************************************************/

typedef struct 
{
	volatile Encoder right_poke;
	volatile Encoder left_poke;
}poke_t;

/****************************************hero small gimbal encoder*****************************************************************/
typedef struct 
{
	volatile Encoder scope_encoder;
	volatile Encoder small_gimbal_encoder;
}hero_small_gimbal_t;
/*********************************************chassis_data*****************************************/





///***************************senior function*************************************/
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
/**************general_gyro define**********************/
extern general_gyro_t gimbal_gyro;
extern general_gyro_t chassis_gyro;
extern steering_wheel_t steering_wheel_chassis;
extern Mecanum_wheel_t Mecanum_chassis;
extern volatile Encoder Pitch_Encoder;
extern volatile Encoder yaw_Encoder;
extern hero_small_gimbal_t hero_small_gimbal;
extern friction_t general_friction;
extern poke_t general_poke;


//extern volatile capacitance_message_t capacitance_message;
//extern receive_judge_t judge_rece_mesg;
//extern location new_location;
//extern RC_Ctl_t RC_CtrlData;
#endif

