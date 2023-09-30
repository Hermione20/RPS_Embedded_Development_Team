#include "mode_switch_task.h"

chassis_t chassis;




void mode_switch_task(void)
{
    static int value_cnt = 0;
    switch (RC_CtrlData.inputmode)
    {
    case REMOTE_INPUT:
    {
        /*******************************底盘云台遥感数据接收******************************************/
        if(gimbal_data.ctrl_mode != GIMBAL_INIT)
        {
            chassis.ChassisSpeed_Ref.forward_back_ref = (RC_CtrlData.rc.ch1- (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;
            chassis.ChassisSpeed_Ref.left_right_ref   = (RC_CtrlData.rc.ch0- (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;
        }
        if(gimbal_data.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)
        {
            gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref += (RC_CtrlData.rc.ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
            gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref   += (RC_CtrlData.rc.ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT;
        }
        if(RC_CtrlData.RemoteSwitch.s3to2)
        {            
                          
                chassis.ctrl_mode = CHASSIS_ROTATE;
            
        }else
				{
								chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
				}
        /****************************底盘默认状态设置**********************************************/
        if(gimbal_data.ctrl_mode == GIMBAL_INIT||gimbal_data.ctrl_mode == GIMBAL_AUTO_BIG_BUFF||gimbal_data.ctrl_mode == GIMBAL_AUTO_SMALL_BUFF)
        {
            chassis.ctrl_mode = CHASSIS_STOP;
        }else if (chassis.ctrl_mode != CHASSIS_ROTATE)
        {
            chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
        }
        /***************************云台默认状态设置**********************************************/
        if(gimbal_data.ctrl_mode != GIMBAL_INIT&&RC_CtrlData.inputmode != STOP&&gimbal_data.last_ctrl_mode == GIMBAL_RELAX)
        {
            gimbal_data.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
        }
        if (gimbal_data.last_ctrl_mode == GIMBAL_RELAX && gimbal_data.ctrl_mode != GIMBAL_RELAX)
	    {
		    gimbal_data.ctrl_mode = GIMBAL_INIT;
	    }
			if( gimbal_data.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)
				can_chassis_data.if_follow_gim = 1;
			/*****************************************************************************************/
			
			gimbal_data.last_ctrl_mode = gimbal_data.ctrl_mode;
			chassis.last_ctrl_mode = chassis.ctrl_mode;
    }
        break;
    case KEY_MOUSE_INPUT:
    {

    }
        break;
    case STOP:
    {

    }
        break;
    default:
        break;
    }
}












