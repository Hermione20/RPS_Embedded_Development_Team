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
						 
						 MF9025转矩常数：0.002597741   最大转矩：5NM
						 MG8016转矩常数：0.017368678		最大转矩：34NM
							单位：NM/转矩电流值
							转矩电流最大值均为2000
						 
@verbatim
 ===============================================================================
 **/







/*******************************LK_Tech电机MF18bit系列***********************************/


void CAN_MF_single_torsionControl(CAN_TypeDef *CANx ,float torque,uint32_t id,float Torque_Constant)
{
	CanTxMsg txmsg;
	
	float iq = (torque/Torque_Constant);
	int16_t iqcontrol = iq;
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

void CAN_MF_multiy_torsionControl(CAN_TypeDef *CANx ,float Torque_Constant,float torque1,float torque2,float torque3,float torque4)
{
	CanTxMsg txmsg;
	float torque[4];
	torque[0] = torque1;
	torque[1] = torque2;
	torque[2] = torque3;
	torque[3] = torque4;
	float iq[4];
	int16_t iqcontrol[4];
	for(int i = 0;i < 4;i++)
	{
		iq[i] = (torque[i]/Torque_Constant);
		iqcontrol[i] = iq[i];
	}
	
	txmsg.StdId = 0x280;
	txmsg.DLC = 0x08;
	txmsg.IDE = CAN_Id_Standard;
	txmsg.RTR = CAN_RTR_Data;
	txmsg.Data[0] = (uint8_t)iqcontrol[0];
	txmsg.Data[1] = (uint8_t)(iqcontrol[0] >> 8);
	txmsg.Data[2] = (uint8_t)iqcontrol[1];
	txmsg.Data[3] = (uint8_t)(iqcontrol[1] >> 8);
	txmsg.Data[4] = (uint8_t)iqcontrol[2];
	txmsg.Data[5] = (uint8_t)(iqcontrol[2] >> 8);
	txmsg.Data[6] = (uint8_t)iqcontrol[3];
	txmsg.Data[7] = (uint8_t)(iqcontrol[3] >> 8);
	
	CAN_Transmit(CANx,&txmsg);
}


void MF_18bit_EncoderProcess(volatile Encoder *v, CanRxMsg * msg,float Torque_Constant)//云台yaw，pitch共用
{
	int i=0;
	int32_t temp_sum = 0;
	v->cal_data.last_raw_value = v->cal_data.raw_value;
	v->cal_data.raw_value = (msg->Data[7]<<8)|msg->Data[6];
	v->cal_data.diff = v->cal_data.raw_value - v->cal_data.last_raw_value;
	if(v->cal_data.diff < -32768)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->cal_data.round_cnt++;
		v->cal_data.ecd_raw_rate = v->cal_data.diff + 65536;
	}
	else if(v->cal_data.diff>32768)
	{
		v->cal_data.round_cnt--;
		v->cal_data.ecd_raw_rate = v->cal_data.diff- 65536;
	}		
	else
	{
		v->cal_data.ecd_raw_rate = v->cal_data.diff;
	}
	v->cal_data.ecd_value = v->cal_data.raw_value + v->cal_data.round_cnt * 65536;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->cal_data.raw_value - v->cal_data.ecd_bias)*0.0054931641f  + v->cal_data.round_cnt * 360;
	//从电机编码器读取的速度
	v->filter_rate = ((int16_t)((msg->Data[5]<<8)|msg->Data[4]));
	v->temperature = msg->Data[1];
	int16_t iq = (msg->Data[3]<<8)|msg->Data[2];
	v->Torque = iq*Torque_Constant;
	v->rate_rpm = v->filter_rate*60/360;
	v->gyro = v->filter_rate*PI/180.0f;
	v->angle = v->ecd_angle*PI/180.0f;
}

void MF_18bit_EncoderTask(volatile Encoder *v, CanRxMsg * msg,int offset,float Torque_Constant)
{
	v->cal_data.can_cnt++;
	if(v->cal_data.can_cnt<=2){v->cal_data.ecd_bias = offset;}
	MF_18bit_EncoderProcess(v, msg,Torque_Constant);
	// 码盘中间值设定也需要修改
	if (v->cal_data.can_cnt <= 10)
	{
		if ((v->cal_data.ecd_bias - v->cal_data.ecd_value) < -32700)
		{
				v->cal_data.ecd_bias = offset + 65536;
		}
		else if ((v->cal_data.ecd_bias - v->cal_data.ecd_value) > 32700)
		{
				v->cal_data.ecd_bias = offset - 65536;
		}
	}
}


/*******************************LK_Tech电机MG18bit系列***********************************/


void CAN_MG_single_torsionControl(CAN_TypeDef *CANx ,float torque,uint32_t id,float Torque_Constant)
{
	CanTxMsg txmsg;
	
	float iq = (torque/Torque_Constant);
	int16_t iqcontrol = (int16_t)iq;
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

void CAN_MG_multiy_torsionControl(CAN_TypeDef *CANx ,float Torque_Constant,float torque1,float torque2,float torque3,float torque4)
{
	CanTxMsg txmsg;
	float torque[4];
	torque[0] = torque1;
	torque[1] = torque2;
	torque[2] = torque3;
	torque[3] = torque4;
	float iq[4];
	int16_t iqcontrol[4];
	for(int i = 0;i < 4;i++)
	{
		iq[i] = (torque[i]/Torque_Constant);
		iqcontrol[i] = iq[i];
	}
	
	txmsg.StdId = 0x280;
	txmsg.DLC = 0x08;
	txmsg.IDE = CAN_Id_Standard;
	txmsg.RTR = CAN_RTR_Data;
	txmsg.Data[0] = (uint8_t)iqcontrol[0];
	txmsg.Data[1] = (uint8_t)(iqcontrol[0] >> 8);
	txmsg.Data[2] = (uint8_t)iqcontrol[1];
	txmsg.Data[3] = (uint8_t)(iqcontrol[1] >> 8);
	txmsg.Data[4] = (uint8_t)iqcontrol[2];
	txmsg.Data[5] = (uint8_t)(iqcontrol[2] >> 8);
	txmsg.Data[6] = (uint8_t)iqcontrol[3];
	txmsg.Data[7] = (uint8_t)(iqcontrol[3] >> 8);
	
	CAN_Transmit(CANx,&txmsg);
}


void MG_18bit_EncoderProcess(volatile Encoder *v, CanRxMsg * msg,float Torque_Constant)//云台yaw，pitch共用
{
	int i=0;
	int32_t temp_sum = 0;
	v->cal_data.last_raw_value = v->cal_data.raw_value;
	v->cal_data.raw_value = (msg->Data[7]<<8)|msg->Data[6];
	v->cal_data.diff = v->cal_data.raw_value - v->cal_data.last_raw_value;
	if(v->cal_data.diff < -32768)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->cal_data.round_cnt++;
		v->cal_data.ecd_raw_rate = v->cal_data.diff + 65536;
	}
	else if(v->cal_data.diff>32768)
	{
		v->cal_data.round_cnt--;
		v->cal_data.ecd_raw_rate = v->cal_data.diff- 65536;
	}		
	else
	{
		v->cal_data.ecd_raw_rate = v->cal_data.diff;
	}
	v->cal_data.ecd_value = v->cal_data.raw_value + v->cal_data.round_cnt * 65536;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)((v->cal_data.raw_value - v->cal_data.ecd_bias)*0.0054931641f  + v->cal_data.round_cnt * 360);
	//从电机编码器读取的速度
	int16_t raw_rate = (msg->Data[5]<<8)|msg->Data[4];
	v->filter_rate = raw_rate/6.0f;
	v->rate_rpm = v->filter_rate*60/360;
	v->temperature = msg->Data[1];
	int16_t iq = (msg->Data[3]<<8)|msg->Data[2];
	v->Torque = iq*Torque_Constant;
	v->gyro = v->filter_rate*PI/180.0f;
	v->angle = v->ecd_angle*PI/180.0f;
}

void MG_18bit_EncoderTask(volatile Encoder *v, CanRxMsg * msg,int offset,float Torque_Constant)
{
	v->cal_data.can_cnt++;
	if(v->cal_data.can_cnt<=2){v->cal_data.ecd_bias = offset;}
	MG_18bit_EncoderProcess(v, msg,Torque_Constant);
	// 码盘中间值设定也需要修改
	if (v->cal_data.can_cnt <= 10)
	{
		if ((v->cal_data.ecd_bias - v->cal_data.ecd_value) < -32700)
		{
				v->cal_data.ecd_bias = offset + 65536;
		}
		else if ((v->cal_data.ecd_bias - v->cal_data.ecd_value) > 32700)
		{
				v->cal_data.ecd_bias = offset - 65536;
		}
	}
}


/********************LK_Tech命令发送函数*********************/
void CAN_LK_TechCommand(CAN_TypeDef *CANx ,uint8_t command,uint32_t id)
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

void CAN_LK_TechsetpidCommand(CAN_TypeDef *CANx, float akp,
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

void CAN_LK_TechangleControl(CAN_TypeDef *CANx ,int16_t maxSpeed ,uint32_t angleControl,uint32_t id)
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

void CAN_LK_TechspeedControl(CAN_TypeDef *CANx ,uint32_t speedControl,uint32_t id)
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


