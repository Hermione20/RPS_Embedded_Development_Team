#ifndef __DDT_MOTOR_H
#define __DDT_MOTOR_H

#include "main.h"

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

extern int16_t speed_ddt;
extern volatile ddtEncoder_t ddt_Encoder;

extern void ddtEncoderProcess(u8* msg);
extern void ddt_SetMotor(int16_t val);
extern void ddt_SetMode(u16 mode);
extern void ddt_SetID(u16 ID);
extern void ddt_Stop(u16 stop);


#endif
