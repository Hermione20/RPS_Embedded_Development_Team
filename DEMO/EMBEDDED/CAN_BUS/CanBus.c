#include "CanBus.h"

static uint32_t can1_count = 0;
static uint32_t can2_count = 0;

void Can1ReceiveMsgProcess(CanRxMsg * msg)
{
    can1_count++;
    switch (msg->StdId)
    {
    case GIMBAL_YAW_MOTOR:
		{
//			GM6020EncoderTask(can2_count,&yaw_Encoder,msg,GMYawEncoder_Offset);
		}break;
    default:
        break;
    }
}

void Can2ReceiveMsgProcess(CanRxMsg * msg)
{
    can2_count++;
    switch (msg->StdId)
    {
    case GIMBAL_YAW_MOTOR:
			GM6020EncoderTask(can2_count,&yaw_Encoder,msg,GMYawEncoder_Offset);
        /* code */
        break;

    default:
        break;
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

/********************FM9025命令发送函数*********************/
void CAN_9015Command(CAN_TypeDef *CANx ,uint8_t command,uint8_t id)
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
                           float iqki, uint8_t id)
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

void CAN_9015angleControl(CAN_TypeDef *CANx ,int16_t maxSpeed ,uint32_t angleControl,uint8_t id)
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

void CAN_9015speedControl(CAN_TypeDef *CANx ,uint32_t speedControl,uint8_t id)
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

void CAN_9015torsionControl(CAN_TypeDef *CANx ,int16_t iqcontrol,uint8_t id)
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

/**********************超级电容**************************/
void POWER_Control1(CAN_TypeDef *CANx ,uint16_t Power,uint16_t StdId) //设置参数使用数据帧，设置成功返回设置，设置失败返回 0x00 00
{
    CanTxMsg tx_message;    
    tx_message.StdId = StdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (Power >> 8);
    tx_message.Data[1] = Power;
    tx_message.Data[2] = (0 >> 8);
    tx_message.Data[3] = 0;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}
void POWER_Control1l(CAN_TypeDef *CANx ,uint16_t StdId)//读取数据采用远程帧访问，模块反馈回来是数据帧
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

void power_send_handle2(CAN_TypeDef *CANx)
{
    POWER_Control1l(CANx,0x610);
    POWER_Control1l(CANx,0x611);
    POWER_Control1l(CANx,0x612);
    POWER_Control1l(CANx,0x613);
}

void power_send_handle1(CAN_TypeDef *CANx,u16 Max_Power)
{
    POWER_Control1(CANx,2, 0x600);
    POWER_Control1(CANx,Max_Power * 100, 0x601);
    POWER_Control1(CANx,2500, 0x602);
    POWER_Control1(CANx,7 * 100, 0x603);
}

/************************HT430**************************************/

/*电机编码器校准。电机出厂前已经对编码器进行了校准；用户如有拆卸电机驱动板，需
执行该命令对电机编码器重新校准。注意：进行电机编码器校准时，请确保电机处于空
载状态，同时，在校准过程中请勿干扰电机转动【0x20】*/
void HT_430_Encoder_Calibration(CAN_TypeDef *CANx,int ID)
{
	CanTxMsg Motor_HT430_CanTxMsg;

	Motor_HT430_CanTxMsg.StdId=(0x20<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x00;
	
	CAN_Transmit(CANx,&Motor_HT430_CanTxMsg);
}

/*设置电机当前位置为原点；电机收到该命令后，设置电机当前位置为原点并将电机运行
模式切换为关闭模式；【0x21】*/

void HT_430_Encoder_Origin(CAN_TypeDef *CANx,int ID)
{
	CanTxMsg Motor_HT430_CanTxMsg;

	Motor_HT430_CanTxMsg.StdId=(0x21<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x00;
	
	CAN_Transmit(CANx,&Motor_HT430_CanTxMsg);
}


void Motor_Information_Request(CAN_TypeDef *CANx,int ID)
{
	CanTxMsg Motor_HT430_CanTxMsg;
//	
//	Motor_HT430_CanTxMsg.StdId = (0x2f<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x00;
//	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_HT430_CanTxMsg));
	
	Motor_HT430_CanTxMsg.StdId=(0x40<<4|ID);
	CAN_Transmit(CANx,&Motor_HT430_CanTxMsg);
}

/*清除系统当前故障（电压故障、电流故障、温度故障）；【0x41】*/
void HT_430_Fault_Clear(CAN_TypeDef *CANx,int ID)
{
	CanTxMsg Motor_HT430_CanTxMsg;

	Motor_HT430_CanTxMsg.StdId=(0x41<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x00;
	
	CAN_Transmit(CANx,&Motor_HT430_CanTxMsg);
}

/*关闭电机，电机进入关闭模式，并处于自由态不受控制；电机上电后为该模式。【0x50】*/
void HT_430_Tuen_Off(CAN_TypeDef *CANx,int ID)
{
	CanTxMsg Motor_HT430_CanTxMsg;

	Motor_HT430_CanTxMsg.StdId=(0x50<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x00;
	
	CAN_Transmit(CANx,&Motor_HT430_CanTxMsg);
}

/*电机根据当前多圈绝对值角度，回到设定的原点；【0x51】*/
void HT_430_Origin_Total(CAN_TypeDef *CANx,int ID)
{
	CanTxMsg Motor_HT430_CanTxMsg;

	Motor_HT430_CanTxMsg.StdId=(0x51<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x00;
	
	CAN_Transmit(CANx,&Motor_HT430_CanTxMsg);
}
/*电机按照最短的距离回到设定的原点，旋转的角度不大于 180 度；【0x52】*/

void HT_430_Back(CAN_TypeDef *CANx,int ID)
{
	CanTxMsg Motor_HT430_CanTxMsg;

	Motor_HT430_CanTxMsg.StdId=(0x52<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x00;
	
	CAN_Transmit(CANx,&Motor_HT430_CanTxMsg);
}
//功率开环控制
void HT_430_Power_Open_Loop(CAN_TypeDef *CANx,int ID,int16_t Pow)
{
	CanTxMsg Motor_HT430_CanTxMsg;
	
	Motor_HT430_CanTxMsg.StdId = (0x53<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x02;
	
	Motor_HT430_CanTxMsg.Data[0]=Pow&0x00ff;
	Motor_HT430_CanTxMsg.Data[1]=(Pow&0xff00)>>8;
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_HT430_CanTxMsg));
}
//速度闭环控制目标速度，单位为 0.1RPM；数据类型 int16_t 类型
void HT_430_V_Clossed_Loop(CAN_TypeDef *CANx,int ID,int16_t V)
{
	CanTxMsg Motor_HT430_CanTxMsg;
	
	Motor_HT430_CanTxMsg.StdId = (0x54<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x02;
	
	Motor_HT430_CanTxMsg.Data[0]=V&0x00ff;
	Motor_HT430_CanTxMsg.Data[1]=(V&0xff00)>>8;
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_HT430_CanTxMsg));
}
//功率开环控制
/*目标绝对值位置 Count 值
数据类型 uint32_t*/
void HT_430_Absolute_Position_closed_Loop(CAN_TypeDef *CANx,int ID,uint32_t Angle)
{
	CanTxMsg Motor_HT430_CanTxMsg;
	
	uint32_t Count;
	
	Motor_HT430_CanTxMsg.StdId = (0x55<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x04;
	
	Count=Angle*16384/360;
	
	Motor_HT430_CanTxMsg.Data[0]=Count&0xff;
	Motor_HT430_CanTxMsg.Data[1]=(Count&0xff00)>>8;
	Motor_HT430_CanTxMsg.Data[2]=(Count&0xff0000)>>16;
	Motor_HT430_CanTxMsg.Data[3]=(Count&0xff000000)>>24;
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_HT430_CanTxMsg));
}
/*电机相对位置闭环控制；电机基于当前位置相对运动的角度。输入参数的数据类型为
int32_t（兼容低版本协议中的 int16_t 数据类型），当参数值为负数时，表示电机反转；
电机旋转一圈为 16384 个 Count；【0x56】*/
void HT_430_Relative_Position_closed_Loop(CAN_TypeDef *CANx,int ID,uint32_t Angle)
{
	CanTxMsg Motor_HT430_CanTxMsg;
	
	uint32_t Count;
	
	Motor_HT430_CanTxMsg.StdId = (0x56<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x04;
	
	Count=Angle*16384/360;
	
	Motor_HT430_CanTxMsg.Data[0]=Count&0xff;
	Motor_HT430_CanTxMsg.Data[1]=(Count&0xff00)>>8;
	Motor_HT430_CanTxMsg.Data[2]=(Count&0xff0000)>>16;
	Motor_HT430_CanTxMsg.Data[3]=(Count&0xff000000)>>24;
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_HT430_CanTxMsg));
}

/*位置闭环目标速度读取和配置；读取电机当前配置的位置闭环目标速度，或配置电机位
置闭环目标速度参数到电机。电机上电后位置闭环目标速度的默认值为，通过 0x0E 命
令保存到电机的值。当前命令写入的位置闭环目标速度只是写入到电机，但断电不保存。
写入成功后，电机在绝对值位置或相对位置闭环模式下将按照配置的速度运动。【0x57】*
0x00：读取位置闭环目标速度
0x01：配置位置闭环目标速度*/
void HT_430_Position_closed_Loop_T_R_OR_W(CAN_TypeDef *CANx,int ID,int16_t V,int Flag_RW)
{
	CanTxMsg Motor_HT430_CanTxMsg;
	
	Motor_HT430_CanTxMsg.StdId = (0x57<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x03;
	
	Motor_HT430_CanTxMsg.Data[0]=Flag_RW;
	*(int16_t*)Motor_HT430_CanTxMsg.Data[1]=V;
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_HT430_CanTxMsg));
}


