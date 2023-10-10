#ifndef __SENIOR_H
#define __SENIOR_H
#include "main.h"


#define  GMPitchEncoder_Offset 6165
//yaw轴电机初始位置
#define  GMYawEncoder_Offset   6013
//底盘航向轴电机初始位置
//电机倒放，顺时针为正
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
	
}Encoder;




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

