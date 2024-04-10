#ifndef __SENIOR_H
#define __SENIOR_H
#include "public.h"


#define  GMPitchEncoder_Offset 6165
//yaw轴电机初始位置
#define  GMYawEncoder_Offset   6344
//底盘航向轴电机初始位置
#define  GM1Encoder_Offset   1370-1024
#define  GM2Encoder_Offset   1298+1024
#define  GM3Encoder_Offset   1231-1024
#define  GM4Encoder_Offset   3964+1024


#ifndef STRUCT_MOTOR
#define STRUCT_MOTOR

#define RATE_BUF_SIZE 6
typedef struct{
	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;								//上一次的编码器原始值
	int32_t ecd_value;                       //经过处理后连续的编码器值
	int32_t diff;													//两次编码器之间的差值
	int32_t temp_count;                   //计数用
	uint8_t buf_count;								//滤波更新buf用
	int32_t ecd_bias;											//初始编码器值	
	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
	int32_t rate_buf[RATE_BUF_SIZE];	//buf，for filter
	int32_t round_cnt;										//圈数
	int32_t can_cnt;					//记录函数的使用次数，在电机初始完成部分任务

	int32_t filter_rate;											//速度
	double ecd_angle;											//角度
	u32 temperature;
	int16_t rate_rpm;
	
}Encoder;
	





#endif

#ifndef GENERAL_GYRO_T
/**************************general gyro*********************************/
#define GENERAL_GYRO_T
typedef struct 
{
	float pitch_Angle;
	float yaw_Angle;
	float roll_Angle;
	float pitch_Gyro;
	float yaw_Gyro;
	float roll_Gyro;
	float x_Acc;
	float y_Acc;
	float z_Acc;
}general_gyro_t;



#endif

//以下底盘结构体的数组方位表示为象限表示

/********************general chassis encoder********************************/

//名字整改
//定义中间结构体
typedef struct 
{
	volatile Encoder Heading_Encoder[4];

	volatile Encoder Driving_Encoder[4];
	
}steering_wheel_t;

typedef struct
{
	volatile Encoder Driving_Encoder[4];
}Mecanum_wheel_t;

/***************************general friction encoder********************************************/
typedef struct 
{
	volatile Encoder right_up_motor;
	volatile Encoder left_up_motor;
	volatile Encoder left_down_motor;
	volatile Encoder right_down_motor;

	volatile Encoder left_motor;
	volatile Encoder right_motor;
}friction_t;

/************************************general poke encoder******************************************************/

typedef struct 
{
	//哨兵
	volatile Encoder right_poke;
	volatile Encoder left_poke;
	//英雄
	volatile Encoder up_poke;
	volatile Encoder down_poke;
	//其他
	volatile Encoder poke;
}poke_t;

/****************************************hero small gimbal encoder*****************************************************************/
typedef struct 
{
	volatile Encoder scope_encoder;
	volatile Encoder small_gimbal_encoder;
}hero_small_gimbal_t;







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



#endif

