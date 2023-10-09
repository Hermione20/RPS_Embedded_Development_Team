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
 
 
uint32_t can1_count = 0;
uint32_t can2_count = 0;

void Can1ReceiveMsgProcess(CanRxMsg * msg)
{
    can1_count++;
    switch (msg->StdId)
    {
    case GIMBAL_PITCH_MOTOR:
		{
			MF_EncoderTask(can1_count,&Pitch_Encoder,msg,GMPitchEncoder_Offset);
		}break;
    default:
        break;
    }
}

void Can2ReceiveMsgProcess(CanRxMsg *msg)
{
    can2_count++;
    switch (msg->StdId)
    {
    case GIMBAL_YAW_MOTOR:
    {
        MF_EncoderTask(can2_count, &yaw_Encoder, msg, GMYawEncoder_Offset);
    }break;
    case CM1Encoder_MOTOR:
    {
        M3508orM2006EncoderTask(can2_count,&Mecanum_chassis.Driving_Encoder[0],msg);
    }break;
    case CM2Encoder_MOTOR:
    {
        M3508orM2006EncoderTask(can2_count,&Mecanum_chassis.Driving_Encoder[1],msg);
    }break;
    case CM3Encoder_MOTOR:
    {
        M3508orM2006EncoderTask(can2_count,&Mecanum_chassis.Driving_Encoder[2],msg);
    }break;
    case CM4Encoder_MOTOR:
    {
        M3508orM2006EncoderTask(can2_count,&Mecanum_chassis.Driving_Encoder[3],msg);
    }break;
    default:
        break;
    }
}

void can_bus_send_task(void)
{
	CAN_9015torsionControl(CAN2,gimbal_data.gim_ref_and_fdb.yaw_motor_input,GIMBAL_YAW_MOTOR);
	CAN_9015torsionControl(CAN1,gimbal_data.gim_ref_and_fdb.pitch_motor_input,GIMBAL_PITCH_MOTOR);
	
	Set_C620andC610_IQ1(CAN2,chassis.current[0],chassis.current[1],chassis.current[2],chassis.current[3]);
	
}

