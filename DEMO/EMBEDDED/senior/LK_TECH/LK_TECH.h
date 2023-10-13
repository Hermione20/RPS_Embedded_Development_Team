#ifndef __LK_TECH_H
#define __LK_TECH_H
#include "public.h"

typedef struct{
	
	uint8_t anglekp;
	uint8_t angleki;
	uint8_t speedkp;
	uint8_t speedki;
	uint8_t torquekp;
	uint8_t torqueki;
	
}PID9015Typedefine;

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

void MF_EncoderProcess(volatile Encoder *v, CanRxMsg * msg);//云台yaw，pitch共用
void MF_EncoderTask(volatile Encoder *v, CanRxMsg * msg,int offset);


void CAN_9015Command(CAN_TypeDef *CANx ,uint8_t command,uint32_t id);
void CAN_9015setpidCommand(CAN_TypeDef *CANx, float akp,
                           float aki,
                           float skp,
                           float ski,
                           float iqkp,
                           float iqki, uint32_t id);
void CAN_9015angleControl(CAN_TypeDef *CANx ,int16_t maxSpeed ,uint32_t angleControl,uint32_t id);
void CAN_9015speedControl(CAN_TypeDef *CANx ,uint32_t speedControl,uint32_t id);
void CAN_9015torsionControl(CAN_TypeDef *CANx ,int16_t iqcontrol,uint32_t id);





#endif
