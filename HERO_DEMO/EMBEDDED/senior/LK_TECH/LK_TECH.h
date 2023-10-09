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
	int32_t raw_value;   									//����������������ԭʼֵ
	int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
	int32_t ecd_value;                       //���������������ı�����ֵ
	int32_t diff;													//���α�����֮��Ĳ�ֵ
	int32_t temp_count;                   //������
	uint8_t buf_count;								//�˲�����buf��
	int32_t ecd_bias;											//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];	//buf��for filter
	int32_t round_cnt;										//Ȧ��
	int32_t filter_rate;											//�ٶ�
	double ecd_angle;											//�Ƕ�
	u32 temperature;
	int16_t rate_rpm;
	
}Encoder;





#endif

void MF_EncoderProcess(volatile Encoder *v, CanRxMsg * msg);//��̨yaw��pitch����
void MF_EncoderTask(uint32_t can_count,volatile Encoder *v, CanRxMsg * msg,int offset);


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