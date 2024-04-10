#include "CanBus.h"


/**
  ******************************************************************************
  * @file    CanBus.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   此文件用于配置can总线的发送与接收任务，若设置模块id可去头文件设置
						 
	* @notice  有关can的模块的接收函数的使用请移步至can_bus.c文件中并在其中的接受
						 函数里选择模块id并调用解算函数，也可在can_bus.h文件中修改模块id
						 有关can的发送函数的使用也移步至can_bus.c文件，并在总发送任务函数中
						 配置要发送的函数，请务必将can_bus_send_task函数的调用放在controltask
						 中。
@verbatim
 ===============================================================================
 **/
 


void Can1ReceiveMsgProcess(CanRxMsg * msg)
{
	can_chassis_receive_task(msg);
    switch (msg->StdId)
    {
			case JM1Encoder_MOTOR:
		{

			MG_18bit_EncoderTask(&balance_chassis.joint_Encoder[0],msg,JM1Encoder_Offset,0.017368678);
			
		}
        break;
    
			case JM2Encoder_MOTOR:
		{

			MG_18bit_EncoderTask(&balance_chassis.joint_Encoder[1],msg,JM2Encoder_Offset,0.017368678);
			
		}
        break;
			case JM3Encoder_MOTOR:
		{

			MG_18bit_EncoderTask(&balance_chassis.joint_Encoder[2],msg,JM3Encoder_Offset,0.017368678);
			
		}
        break;
   
    case JM4Encoder_MOTOR:
		{

			MG_18bit_EncoderTask(&balance_chassis.joint_Encoder[3],msg,JM4Encoder_Offset,0.017368678);
			
		}
        break;
    default:
        break;
    }
		
}

void Can2ReceiveMsgProcess(CanRxMsg * msg)
{
	
    switch (msg->StdId)
    {
    
		case TM1Encoder_MOTOR:
		{
			MF_18bit_EncoderTask(&balance_chassis.Driving_Encoder[0],msg,TM1Encoder_Offset,0.002597741);
			
		}break;
		case TM2Encoder_MOTOR:
		{
			MF_18bit_EncoderTask(&balance_chassis.Driving_Encoder[1],msg,TM2Encoder_Offset,0.002597741);
			
		}break;
		

    default:
        break;
    }
		PM01_message_Process(&capacitance_message,msg);
	
}








void can_bus_send_task(void)
{
//	CAN_MG_single_torsionControl(CAN2,b_chassis.joint_T[0],0x141,0.017368678);
//	CAN_MG_single_torsionControl(CAN1,-b_chassis.joint_T[1],0x142,0.017368678);
//	CAN_MG_single_torsionControl(CAN1,-b_chassis.joint_T[2],0x143,0.017368678);
//	CAN_MG_single_torsionControl(CAN2,b_chassis.joint_T[3],0x144,0.017368678);
//	
//	CAN_MF_single_torsionControl(CAN2,b_chassis.driving_T[0],0x141,0.002597741);
//	CAN_MF_single_torsionControl(CAN2,-b_chassis.driving_T[1],0x142,0.002597741);

		CAN_MF_multiy_torsionControl(CAN2,0.002597741,-b_chassis.driving_T[1],b_chassis.driving_T[0],0,0);
	

		CAN_MG_multiy_torsionControl(CAN1,0.017368678,b_chassis.joint_T[0],-b_chassis.joint_T[1],-b_chassis.joint_T[2],b_chassis.joint_T[3]);
	
//	CAN_MF_multiy_torsionControl(CAN2,0.002597741,0,0,0,0);
//	

//		CAN_MG_multiy_torsionControl(CAN1,0.017368678,0,0,0,0);
//	
		
	
	
}

