#include "LK_TECH.h"


/**
  ******************************************************************************
  * @file    LK_TECH.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   此文件编写LK_TECH各型号电机的解算,入口参数包含通用
						 编码器结构体，can总线的计数，can结构体，编码器初始值
						 设定.发送任务函数见senior文件
						 
@verbatim
 ===============================================================================
 **/







/*******************************LK_Tech电机***********************************/

void MF_EncoderProcess(volatile Encoder *v, CanRxMsg * msg)//云台yaw，pitch共用
{
	int i=0;
	int32_t temp_sum = 0;
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[7]<<8)|msg->Data[6];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -32768)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 65536;
	}
	else if(v->diff>32768)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 65536;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	v->ecd_value = v->raw_value + v->round_cnt * 65536;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*0.0054931641f  + v->round_cnt * 360;
	//从电机编码器读取的速度
	v->filter_rate = (msg->Data[5]<<8)|msg->Data[4];
	v->temperature = msg->Data[1];
}

void MF_EncoderTask(volatile Encoder *v, CanRxMsg * msg,int offset)
{
	v->can_cnt++;
	if(v->can_cnt<=2){v->ecd_bias = offset;}
	MF_EncoderProcess(v, msg);
	// 码盘中间值设定也需要修改
	if (v->can_cnt <= 10)
	{
		if ((v->ecd_bias - v->ecd_value) < -32700)
		{
				v->ecd_bias = offset + 65536;
		}
		else if ((v->ecd_bias - v->ecd_value) > 32700)
		{
				v->ecd_bias = offset - 65536;
		}
	}
}

/********************FM9025命令发送函数*********************/
void CAN_9015Command(CAN_TypeDef *CANx ,uint8_t command,uint32_t id)
{
	CanTxMsg txmsg;
	txmsg.StdId = id;
	txmsg.DLC = 0x08;
	txmsg.IDE = CAN_Id_Standard;
	txmsg.RTR = CAN_RTR_Data;
	txmsg.Data[0] = command;
	txmsg.Data[1] = 0;
	txmsg.Data[2] = 0;
	txmsg.Data[3] = 0;
	txmsg.Data[4] = 0;
	txmsg.Data[5] = 0;
	txmsg.Data[6] = 0;
	txmsg.Data[7] = 0;
	
	
	CAN_Transmit(CANx,&txmsg);
	
}

void CAN_9015setpidCommand(CAN_TypeDef *CANx, float akp,
                           float aki,
                           float skp,
                           float ski,
                           float iqkp,
                           float iqki, uint32_t id)
{
    CanTxMsg txmsg;
    txmsg.StdId = id;
    txmsg.DLC = 0x08;
    txmsg.IDE = CAN_Id_Standard;
    txmsg.RTR = CAN_RTR_Data;
    txmsg.Data[0] = 0x32;
    txmsg.Data[1] = 0x00;
    txmsg.Data[2] = akp;
    txmsg.Data[3] = aki;
    txmsg.Data[4] = skp;
    txmsg.Data[5] = ski;
    txmsg.Data[6] = iqkp;
    txmsg.Data[7] = iqki;

    CAN_Transmit(CANx, &txmsg);
}

void CAN_9015angleControl(CAN_TypeDef *CANx ,int16_t maxSpeed ,uint32_t angleControl,uint32_t id)
{
	CanTxMsg txmsg;
	txmsg.StdId = id;
	txmsg.DLC = 0x08;
	txmsg.IDE = CAN_Id_Standard;
	txmsg.RTR = CAN_RTR_Data;
	txmsg.Data[0] = 0xA4;
	txmsg.Data[1] = 0x00;
	txmsg.Data[2] = (uint8_t)maxSpeed;
	txmsg.Data[3] = (uint8_t)(maxSpeed >> 8);
	txmsg.Data[4] = (uint8_t)angleControl;
	txmsg.Data[5] = (uint8_t)(angleControl >> 8);
	txmsg.Data[6] = (uint8_t)(angleControl >> 16);
	txmsg.Data[7] = (uint8_t)(angleControl >> 24);
	
	CAN_Transmit(CANx,&txmsg);
}

void CAN_9015speedControl(CAN_TypeDef *CANx ,uint32_t speedControl,uint32_t id)
{
	CanTxMsg txmsg;
	txmsg.StdId = id;
	txmsg.DLC = 0x08;
	txmsg.IDE = CAN_Id_Standard;
	txmsg.RTR = CAN_RTR_Data;
	txmsg.Data[0] = 0xA2;
	txmsg.Data[1] = 0x00;
	txmsg.Data[2] = 0x00;
	txmsg.Data[3] = 0x00;
	txmsg.Data[4] = (uint8_t)speedControl;
	txmsg.Data[5] = (uint8_t)(speedControl >> 8);
	txmsg.Data[6] = (uint8_t)(speedControl >> 16);
	txmsg.Data[7] = (uint8_t)(speedControl >> 24);
	
	CAN_Transmit(CANx,&txmsg);
}

void CAN_9015torsionControl(CAN_TypeDef *CANx ,int16_t iqcontrol,uint32_t id)
{
	CanTxMsg txmsg;
	txmsg.StdId = id;
	txmsg.DLC = 0x08;
	txmsg.IDE = CAN_Id_Standard;
	txmsg.RTR = CAN_RTR_Data;
	txmsg.Data[0] = 0xA1;
	txmsg.Data[1] = 0x00;
	txmsg.Data[2] = 0x00;
	txmsg.Data[3] = 0x00;
	txmsg.Data[4] = (uint8_t)iqcontrol;
	txmsg.Data[5] = (uint8_t)(iqcontrol >> 8);
	txmsg.Data[6] = 0x00;
	txmsg.Data[7] = 0x00;
	
	CAN_Transmit(CANx,&txmsg);

	
}
