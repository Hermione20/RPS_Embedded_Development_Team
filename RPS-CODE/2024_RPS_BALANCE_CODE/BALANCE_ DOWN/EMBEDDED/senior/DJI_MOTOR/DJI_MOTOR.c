#include "DJI_MOTOR.h"


/**
  ******************************************************************************
  * @file    DJI_MOTOR.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   此文件编写了DJI各型号电机的解算，包含c610电调
						 c620电调与GM6020电机的解算，入口参数包含通用
						 编码器结构体，can总线的计数，can结构体，（GM
						 6020还包含编码器初始值设定）
						 
@verbatim
 ===============================================================================
 **/
 
/*****************************dji encoder*************************************/

void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg)
{

            v->cal_data.ecd_bias = (msg->Data[0]<<8)|msg->Data[1];  //保存初始编码器值作为偏差  
            v->cal_data.ecd_value = v->cal_data.ecd_bias;
            v->cal_data.last_raw_value = v->cal_data.ecd_bias;
            v->cal_data.temp_count++;
}

void EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->cal_data.last_raw_value = v->cal_data.raw_value;
	v->cal_data.raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->cal_data.diff = v->cal_data.raw_value - v->cal_data.last_raw_value;
	if(v->cal_data.diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->cal_data.round_cnt++;
		v->cal_data.ecd_raw_rate = v->cal_data.diff + 8192;
	}
	else if(v->cal_data.diff>4096)
	{
		v->cal_data.round_cnt--;
		v->cal_data.ecd_raw_rate = v->cal_data.diff- 8192;
	}		
	else
	{
		v->cal_data.ecd_raw_rate = v->cal_data.diff;
	}
	//计算得到连续的编码器输出值
	v->cal_data.ecd_value = v->cal_data.raw_value + v->cal_data.round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->cal_data.raw_value - v->cal_data.ecd_bias)*0.04394531f + v->cal_data.round_cnt * 360;
	v->cal_data.rate_buf[v->cal_data.buf_count++] = v->cal_data.ecd_raw_rate;
	if(v->cal_data.buf_count == RATE_BUF_SIZE)
	{
		v->cal_data.buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->cal_data.rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);		
	v->rate_rpm = (msg->Data[2]<<8)|msg->Data[3];
}

void GM6020EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
	v->cal_data.last_raw_value = v->cal_data.raw_value;
	v->cal_data.raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->cal_data.diff = v->cal_data.raw_value - v->cal_data.last_raw_value;
	if(v->cal_data.diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->cal_data.round_cnt++;
		v->cal_data.ecd_raw_rate = v->cal_data.diff + 8192;
	}
	else if(v->cal_data.diff>4096)
	{
		v->cal_data.round_cnt--;
		v->cal_data.ecd_raw_rate = v->cal_data.diff- 8192;
	}		
	else
	{
		v->cal_data.ecd_raw_rate = v->cal_data.diff;
	}
	v->cal_data.ecd_value = v->cal_data.raw_value + v->cal_data.round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->cal_data.raw_value - v->cal_data.ecd_bias)*0.0439453125f  + v->cal_data.round_cnt * 360;
	v->filter_rate = (msg->Data[2]<<8)|msg->Data[3];
	if(v->filter_rate>=1000)
		v->filter_rate = v->filter_rate - 65535;
	v->temperature = msg->Data[6];
}


void M3508orM2006EncoderTask(volatile Encoder *v, CanRxMsg * msg)
{
	v->cal_data.can_cnt++;
	(v->cal_data.can_cnt<=5)?GetEncoderBias(v,msg):EncoderProcess(v,msg);
}


void GM6020EncoderTask(volatile Encoder *v, CanRxMsg * msg,int offset)
{
	v->cal_data.can_cnt++;
	if(v->cal_data.can_cnt<=2){v->cal_data.ecd_bias = offset;}
	GM6020EncoderProcess(v, msg);
	// 码盘中间值设定也需要修改
	if (v->cal_data.can_cnt <= 10)
	{
		if ((v->cal_data.ecd_bias - v->cal_data.ecd_value) < -4000)
		{
				v->cal_data.ecd_bias = offset + 8192;
		}
		else if ((v->cal_data.ecd_bias - v->cal_data.ecd_value) > 4000)
		{
				v->cal_data.ecd_bias = offset - 8192;
		}
	}
}

void Set_GM6020_IQ1(CAN_TypeDef *CANx, int16_t motor1_iq, int16_t motor2_iq, int16_t motor3_iq, int16_t motor4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (uint8_t)(motor1_iq >> 8);
    tx_message.Data[1] = (uint8_t)motor1_iq;
    tx_message.Data[2] = (uint8_t)(motor2_iq >> 8);
    tx_message.Data[3] = (uint8_t)motor2_iq;
    tx_message.Data[4] = (uint8_t)(motor3_iq >> 8);
    tx_message.Data[5] = (uint8_t)motor3_iq;
    tx_message.Data[6] = (uint8_t)(motor4_iq >> 8);
    tx_message.Data[7] = (uint8_t)motor4_iq;
    CAN_Transmit(CANx,&tx_message);
}

void Set_GM6020_IQ2(CAN_TypeDef *CANx, int16_t motor5_iq, int16_t motor6iq, int16_t motor7_iq, int16_t motor8_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x2FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (uint8_t)(motor5_iq >> 8);
    tx_message.Data[1] = (uint8_t)motor5_iq;
    tx_message.Data[2] = (uint8_t)(motor6iq >> 8);
    tx_message.Data[3] = (uint8_t)motor6iq;
    tx_message.Data[4] = (uint8_t)(motor7_iq >> 8);
    tx_message.Data[5] = (uint8_t)motor7_iq;
    tx_message.Data[6] = (uint8_t)(motor8_iq >> 8);
    tx_message.Data[7] = (uint8_t)motor8_iq;
    CAN_Transmit(CANx,&tx_message);
}

void Set_C620andC610_IQ1(CAN_TypeDef *CANx, int16_t motor1_iq, int16_t motor2_iq, int16_t motor3_iq, int16_t motor4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (uint8_t)(motor1_iq >> 8);
    tx_message.Data[1] = (uint8_t)motor1_iq;
    tx_message.Data[2] = (uint8_t)(motor2_iq >> 8);
    tx_message.Data[3] = (uint8_t)motor2_iq;
    tx_message.Data[4] = (uint8_t)(motor3_iq >> 8);
    tx_message.Data[5] = (uint8_t)motor3_iq;
    tx_message.Data[6] = (uint8_t)(motor4_iq >> 8);
    tx_message.Data[7] = (uint8_t)motor4_iq;
    CAN_Transmit(CANx,&tx_message);
}

void Set_C620andC610_IQ2(CAN_TypeDef *CANx, int16_t motor5_iq, int16_t motor6_iq, int16_t motor7_iq, int16_t motor8_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (uint8_t)(motor5_iq >> 8);
    tx_message.Data[1] = (uint8_t)motor5_iq;
    tx_message.Data[2] = (uint8_t)(motor6_iq >> 8);
    tx_message.Data[3] = (uint8_t)motor6_iq;
    tx_message.Data[4] = (uint8_t)(motor7_iq >> 8);
    tx_message.Data[5] = (uint8_t)motor7_iq;
    tx_message.Data[6] = (uint8_t)(motor8_iq >> 8);
    tx_message.Data[7] = (uint8_t)motor8_iq;
    CAN_Transmit(CANx,&tx_message);
}
