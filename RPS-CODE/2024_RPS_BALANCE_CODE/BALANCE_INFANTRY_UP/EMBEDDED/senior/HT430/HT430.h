#ifndef __HT430_H
#define __HT430_H
#include "public.h"


/**********************************HT430**********************************/
typedef enum
{
    OFF_STATE=0,
    OPEN_LOOP=1,
    SPEED_MODE=3,
    ANGLE_MODE=5,
} Operating_State_t;
typedef struct{
	uint16_t Angle;//单圈绝对值角度
	int32_t Total_Angle;//多圈绝对值角度
	int16_t V;//电机转速
	Operating_State_t Operating_State;//运行状态
	uint8_t Voltage;//电源电压
	uint8_t Currents;//电流
	uint8_t Temperature;//温度
	uint8_t DTC;//故障码
}HT430_J10_t;

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




void HT_430_Information_Receive(CanRxMsg * msg,HT430_J10_t *HT430_J10_t,volatile Encoder *v);


void HT_430_Encoder_Calibration(CAN_TypeDef *CANx,int ID);
void HT_430_Encoder_Origin(CAN_TypeDef *CANx,int ID);
void Motor_Information_Request(CAN_TypeDef *CANx,int ID);
void HT_430_Fault_Clear(CAN_TypeDef *CANx,int ID);
void HT_430_Tuen_Off(CAN_TypeDef *CANx,int ID);
void HT_430_Origin_Total(CAN_TypeDef *CANx,int ID);
void HT_430_Back(CAN_TypeDef *CANx,int ID);
void HT_430_Power_Open_Loop(CAN_TypeDef *CANx,int ID,int16_t Pow);
void HT_430_V_Clossed_Loop(CAN_TypeDef *CANx,int ID,int16_t V);
void HT_430_Absolute_Position_closed_Loop(CAN_TypeDef *CANx,int ID,uint32_t Angle);
void HT_430_Relative_Position_closed_Loop(CAN_TypeDef *CANx,int ID,uint32_t Angle);
void HT_430_Position_closed_Loop_T_R_OR_W(CAN_TypeDef *CANx,int ID,int16_t V,int Flag_RW);



extern HT430_J10_t HT430_J10;














#endif
