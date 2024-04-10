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
    switch (msg->StdId)
    {
    case LEFT_FRICTION:
		{
			M3508orM2006EncoderTask(&general_friction.left_motor,msg);
		}break;
		case RIGHT_FRICTION:
		{
			M3508orM2006EncoderTask(&general_friction.right_motor,msg);
		}break;
		case POKE:
		{
			M3508orM2006EncoderTask(&general_poke.poke,msg);
		}break;
    default:
        break;
    }
}

void Can2ReceiveMsgProcess(CanRxMsg * msg)
{
    switch (msg->StdId)
    {
    case GIMBAL_YAW_MOTOR:
			GM6020EncoderTask(&yaw_Encoder,msg,GMYawEncoder_Offset);
        /* code */
        break;
		case GIMBAL_PITCH_MOTOR:
			GM6020EncoderTask(&Pitch_Encoder,msg,GMPitchEncoder_Offset);

    default:
        break;
    }
}








void can_bus_send_task(void)
{
	
	Set_GM6020_IQ1(CAN2,0,0,0,gimbal_data.gim_ref_and_fdb.pitch_motor_input);
	Set_GM6020_IQ2(CAN2,gimbal_data.gim_ref_and_fdb.yaw_motor_input,0,0,0);
	Set_C620andC610_IQ1(CAN1,shoot.fric_current[1],shoot.fric_current[0],shoot.poke_current[0],0);
}

