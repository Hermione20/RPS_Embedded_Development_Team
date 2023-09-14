#ifndef __SENIOR_H
#define __SENIOR_H
#include "public.h"

#define  GMPitchEncoder_Offset 0
//yaw轴电机初始位置
#define  GMYawEncoder_Offset   4758
//底盘航向轴电机初始位置
#define  GM1Encoder_Offset   1437
#define  GM2Encoder_Offset   8042
#define  GM3Encoder_Offset   4141
#define  GM4Encoder_Offset   6732

/********************DJI Encoder******************************/
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
	int32_t filter_rate;											//速度
	double ecd_angle;											//角度
	u32 temperature;
	int16_t rate_rpm;
	
}Encoder;

/***************************CH100********************************/

__packed typedef struct
{
uint8_t tag; /* ????:0x91 */
uint8_t id; /* ??ID */
uint8_t rev[2];
float prs; /* ?? */
uint32_t ts; /* ??? */
float acc[3]; /* ??? */
float gyr[3]; /* ??? */
float mag[3]; /* ?? */
float eul[3]; /* ???:
Roll,Pitch,Yaw */
float quat[4]; /* ??? */
}id0x91_t;



/****************************hi226********************************/
#define LENGTH_USER_ID_0x90 			1
#define LENGTH_ACC_0xa0 					6
#define LENGTH_LINEAR_ACC_0xa5 		6
#define LENGTH_ANG_VEL_0xb0 			6
#define LENGTH_MAG_0xc0 					6
#define LENGTH_EULER_ANG_s16_0xd0 6
#define LENGTH_EULER_ANG_f_0xd9 	12
#define LENGTH_QUATERNION_0xd1 		16
#define LENGTH_AIR_PRESS_0xf0 		4
#define USART6_RX_BUF_LENGTH 100

typedef struct {
					u8 User_ID;
					s16 Acc_X;
					s16 Acc_Y;
					s16 Acc_Z;
					s16 Linear_Acc_X;
					s16 Linear_Acc_Y;
					s16 Linear_Acc_Z;
					s16 Ang_Velocity_X;
					s16 Ang_Velocity_Y;
					s16 Ang_Velocity_Z;
					s16 Mag_X;
					s16 Mag_Y;
					s16 Mag_Z;
					s16 Euler_Angle_Pitch_s16;
					s16 Euler_Angle_Roll_s16;
					s16 Euler_Angle_Yaw_s16;
					
					union
					{
						float Euler_Angle_Pitch_f;
						u8 Euler_Angle_Pitch_u8[4];
					}Euler_Angle_Pitch;
					
					union
					{
						float Euler_Angle_Roll_f;
						u8 Euler_Angle_Roll_u8[4];
					}Euler_Angle_Roll;

					union
					{
						float Euler_Angle_Yaw_f;
						u8 Euler_Angle_Yaw_u8[4];
					}Euler_Angle_Yaw;
					
					union
					{
						float Quaternion_W_f;
						u8 Quaternion_W_u8[4];
					}Quaternion_W;		

					union
					{
						float Quaternion_X_f;
						u8 Quaternion_X_u8[4];
					}Quaternion_X;	
						union
					{
						float Quaternion_Y_f;
						u8 Quaternion_Y_u8[4];
					}Quaternion_Y;	
						union
					{
						float Quaternion_Z_f;
						u8 Quaternion_Z_u8[4];
					}Quaternion_Z;	
					float Euler_Angle_Pitch_s16_2_f;
					float Euler_Angle_Roll_s16_2_f;
					float Euler_Angle_Yaw_s16_2_f;	
} HI220_Stucture;

typedef struct
{
	union
	{
		u8 Flag_Configured;
		struct
		{
			u8 Eout_Configured 	: 1;
			u8 ODR_Configured 	: 1;
			u8 BAUD_Configured 	: 1;
			u8 SETPTL_Configured: 1;
			u8 MODE_Configured 	: 1;
			u8 MCAL_Configured 	: 1;
			u8 Reserve 					: 2;
			
		}Bits;
	}Hi220_Flag_Configured;
	union
	{
		u8 Flag_Reconfig;
		struct
		{
			u8 Eout_Reconfig 	: 1;
			u8 ODR_Reconfig 	: 1;
			u8 BAUD_Reconfig 	: 1;
			u8 SETPTL_Reconfig: 1;
			u8 MODE_Reconfig 	: 1;
			u8 MCAL_Reconfig 	: 1;
			u8 Reserve 				: 2;
			
		}Bits;
	}Hi220_Flag_Reconfig;
}Hi220_Flags_t;

/**************************general gyro*********************************/
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

/****************************ddt motor**********************************************/

typedef struct
{
	uint16_t ID;						//ID
	uint16_t Mode;					//模式
	int16_t current;				//转矩电流
	int16_t rate_rpm;				//转速
	int16_t ecd_value;			//编码器位置
	uint16_t error_gate;		//故障码
	uint16_t crc_check;			//CRC校验

	int16_t ecd_bias;			//初始值
	int rount_count;		//圈数
	int16_t last_value;	//上一次的编码器位置
	float angle;					//角度(累计)
	
}ddtEncoder_t;

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

/***************************senior function*************************************/
void CH100_getDATA(uint8_t *DataAddress,general_gyro_t *GYRO);
static void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes);
void HI220_getDATA(uint8_t *DataAddress,general_gyro_t *GYRO,uint8_t length);
void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg);
void EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void GM6020EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void M3508orM2006EncoderTask(uint32_t can_count,volatile Encoder *v, CanRxMsg * msg);
void GM6020EncoderTask(uint32_t can_count,volatile Encoder *v, CanRxMsg * msg,int offset);
/**************general_gyro define**********************/
extern general_gyro_t gimbal_gyro;
extern general_gyro_t chassis_gyro;
extern steering_wheel_t steering_wheel_chassis;
extern Mecanum_wheel_t Mecanum_chassis;

#endif

