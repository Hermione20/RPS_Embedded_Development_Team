#include "PM01.h"


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
 
/******************************capacitance_define*************************************/
volatile capacitance_message_t capacitance_message;

void PM01_message_Process(volatile capacitance_message_t *v,CanRxMsg * msg)
{
	switch (msg->StdId)
	{
	case 0x610:
	{
		v->mode = (msg->Data[0] << 8) | msg->Data[1];
		v->err_fdb = (msg->Data[2] << 8) | msg->Data[3];
	}
	break;
	case 0x611:
	{
		v->in_power = (msg->Data[0] << 8) | msg->Data[1];
		v->in_v = (msg->Data[2] << 8) | msg->Data[3];
		v->in_i = (msg->Data[4] << 8) | msg->Data[5];
	}
	break;
	case 0x612:
	{
		v->out_power = (msg->Data[0] << 8) | msg->Data[1];
		v->out_v = (msg->Data[2] << 8) | msg->Data[3];
		v->out_i = (msg->Data[4] << 8) | msg->Data[5];
    v->cap_voltage_filte = v->out_v/100.0f;
	}
	break;
	case 0x613:
	{
		v->tempureture=(msg->Data[0]<<8)|msg->Data[1];
		v->time=(msg->Data[2]<<8)|msg->Data[3];
        v->this_time=(msg->Data[4]<<8)|msg->Data[5];
	}break;

	default:
		break;
	}
}



/**********************超级电容控制板command**************************/

void PM01_command_set(CAN_TypeDef *CANx ,uint16_t data,uint32_t StdId) //设置参数使用数据帧，设置成功返回设置，设置失败返回 0x00 00
{
    CanTxMsg tx_message;    
    tx_message.StdId = StdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (data >> 8);
    tx_message.Data[1] = data;
    tx_message.Data[2] = (0 >> 8);
    tx_message.Data[3] = 0;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}

void PM01_data_read(CAN_TypeDef *CANx ,uint32_t StdId)//读取数据采用远程帧访问，模块反馈回来是数据帧
{
    CanTxMsg tx_message;    
    tx_message.StdId = StdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Remote;//CAN_RTR_Data;
    tx_message.DLC = 0x06;
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x00;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;

    CAN_Transmit(CANx,&tx_message);
}

void power_data_read_handle(CAN_TypeDef *CANx)
{
//    PM01_data_read(CANx,0x610); //读取模块状态
//    PM01_data_read(CANx,0x611); //读取输入电压与电流
    PM01_data_read(CANx,0x612); //读取输出功率，电压，电流
//    PM01_data_read(CANx,0x613); //温度，累计运行时间与本次时间
}

void power_data_set_handle(CAN_TypeDef *CANx,u16 Max_Power)
{
    PM01_command_set(CANx,2, 0x600); //超电控制板开机
    PM01_command_set(CANx,Max_Power * 100, 0x601); //最大输入功率设置
    PM01_command_set(CANx,2400, 0x602); //最大输出电压设置
    PM01_command_set(CANx,7 * 100, 0x603); //最大输出电流设置
}

void power_data_Init(CAN_TypeDef *CANx)
{
	PM01_command_set(CANx,2, 0x600); //超电控制板开机
	PM01_command_set(CANx,2400, 0x602); //最大输出电压设置
  PM01_command_set(CANx,7 * 100, 0x603); //最大输出电流设置
}
