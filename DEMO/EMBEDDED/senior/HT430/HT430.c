#include "HT430.h"

/**
  ******************************************************************************
  * @file    HT430.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   此文件编写了HT430型号电机的解算，入口参数包含通用
						 编码器结构体，can结构体，自带的HT430_J10，这里的
						 结构体需要输入，但是在算法层调用的时候仅调用通用
						 编码器结构体
						 
@verbatim
 ===============================================================================
 **/

HT430_J10_t HT430_J10;



/********************************HT430_J10************************************/

void HT_430_Information_Receive(CanRxMsg * msg,HT430_J10_t *HT430_J10_t,volatile Encoder *v)
{
  switch ((msg->StdId&0xfffe)>>4)
	{
		case 0x2f:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=msg->Data[7]<<8|msg->Data[6];
		}break;
		
		case 0x40:
		{
			HT430_J10_t->Voltage=msg->Data[0]*0.2;
			HT430_J10_t->Currents=msg->Data[1]*0.03;
			HT430_J10_t->Temperature=msg->Data[2]*0.4;
			HT430_J10_t->DTC=msg->Data[3];
			HT430_J10_t->Operating_State=msg->Data[4];
		}break;
		
		case 0x53:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=msg->Data[7]<<8|msg->Data[6];
		}break;
		
		case 0x54:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=(msg->Data[7]<<8|msg->Data[6]);
		}break;
		
		case 0x55:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=msg->Data[7]<<8|msg->Data[6];
		}break;
		
		case 0x56:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=msg->Data[7]<<8|msg->Data[6];
		}break;
		
		case 0x57:
		{
//			HT430_J10_t->V=(msg->Data[1]<<8|msg->Data[0])*0.1;
		}break;
		
		default:
		{
		}break;
	}
	HT430_J10_t->V=HT430_J10_t->V*360/16384/6;
	v->ecd_angle = HT430_J10_t->Total_Angle;
	v->filter_rate = HT430_J10_t->V;
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
