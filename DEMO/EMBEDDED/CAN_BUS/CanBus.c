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
			case LEFT_FRICTION1:
			{
				M3508orM2006EncoderTask(&general_friction.left_motor1,msg);
			}break;
			case RIGHT_FRICTION1:
			{
				M3508orM2006EncoderTask(&general_friction.right_motor1,msg);
			}break;
		#if TYPE==1
			case POKE_2:
			{
				M3508orM2006EncoderTask(&general_poke.right_poke,msg);
			}break;
		#endif
		#if TYPE==2
			case POKE_3:
			{
				M3508orM2006EncoderTask(&general_poke.left_poke,msg);
			}break;
		#endif
    default:
        break;
    }
}

void Can2ReceiveMsgProcess(CanRxMsg *msg)
{
    switch (msg->StdId)
    {
		#if TYPE==1
			case POKE_1:
			{
				M3508orM2006EncoderTask(&general_poke.left_poke,msg);
			}break;
		#endif


			default:
					break;
    }
}

void can_bus_send_task(void)
{
//	CAN_9015torsionControl(CAN2,gimbal_data.gim_ref_and_fdb.yaw_motor_input,GIMBAL_YAW_MOTOR);
//	CAN_9015torsionControl(CAN1,gimbal_data.gim_ref_and_fdb.pitch_motor_input,GIMBAL_PITCH_MOTOR);
	
//	Set_C620andC610_IQ1(CAN2,chassis.current[0],chassis.current[1],chassis.current[2],chassis.current[3]);
#if TYPE==1
		Set_C620andC610_IQ1(CAN1,pid_friction_whell_speed[0].out,pid_friction_whell_speed[1].out,0,0);
		Set_C620andC610_IQ2(CAN1,pid_42mm_poke2_speed.out,0,0,0);
		Set_C620andC610_IQ2(CAN2,pid_42mm_poke_speed.out,0,0,0);
#endif
#if TYPE==2
	Set_C620andC610_IQ1(CAN1,pid_friction_whell_speed[0].out,pid_friction_whell_speed[1].out,pid_17mm_poke_speed.out,0);
#endif
}

