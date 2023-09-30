#include "LK_TECH.h"










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

void MF_EncoderTask(uint32_t can_count,volatile Encoder *v, CanRxMsg * msg,int offset)
{

	MF_EncoderProcess(v, msg);
	// 码盘中间值设定也需要修改
	if (can_count <= 100)
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

