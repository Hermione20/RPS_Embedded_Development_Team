#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H
#include "public.h"

/********************DJI Encoder******************************/
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





void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg);
void EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void GM6020EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void M3508orM2006EncoderTask(volatile Encoder *v, CanRxMsg * msg);
void GM6020EncoderTask(volatile Encoder *v, CanRxMsg * msg,int offset);


void Set_GM6020_IQ1(CAN_TypeDef *CANx, int16_t motor1_iq, int16_t motor2_iq, int16_t motor3_iq, int16_t motor4_iq);
void Set_GM6020_IQ2(CAN_TypeDef *CANx, int16_t motor5_iq, int16_t motor6iq, int16_t motor7_iq, int16_t motor8_iq);
void Set_C620andC610_IQ1(CAN_TypeDef *CANx, int16_t motor1_iq, int16_t motor2_iq, int16_t motor3_iq, int16_t motor4_iq);
void Set_C620andC610_IQ2(CAN_TypeDef *CANx, int16_t motor5_iq, int16_t motor6_iq, int16_t motor7_iq, int16_t motor8_iq);






#endif