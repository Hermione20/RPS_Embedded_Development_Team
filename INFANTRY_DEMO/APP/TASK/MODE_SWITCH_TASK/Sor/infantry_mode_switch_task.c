#include "infantry_mode_switch_task.h"

chassis_t chassis;
int16_t chassis_speed = 0;
u8 steering_P_mode = 0;

void mode_switch_task(void)
{
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
					VAL_LIMIT(gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref, pitch_min, pitch_max);
        }
        if (RC_CtrlData.RemoteSwitch.s3to2)
        {

            chassis.ctrl_mode = CHASSIS_ROTATE;
        }
        else
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
			{
				
				chassis.follow_gimbal = 1;
			}
			/*****************************************************************************************/
			
			
    }
        break;
    case KEY_MOUSE_INPUT:
    {
        /*******************************底盘云台键鼠数据接收******************************************/
         if(gimbal_data.ctrl_mode != GIMBAL_INIT)
         {
            /************************底盘键位赋值******************************/
            if (RC_CtrlData.Key_Flag.Key_SHIFT_Flag)
            {
                chassis.chassis_speed_mode = HIGH_SPEED_MODE;
                chassis_speed = HIGH_SPEED;
            }else
            {
                chassis.chassis_speed_mode = NORMAL_SPEED_MODE;
                chassis_speed = NORMAL_SPEED;
            }
            if(RC_CtrlData.Key_Flag.Key_W_Flag)
            {
                chassis.ChassisSpeed_Ref.forward_back_ref = chassis_speed;
            }else if(RC_CtrlData.Key_Flag.Key_S_Flag)
            {
                chassis.ChassisSpeed_Ref.forward_back_ref = -chassis_speed;
            }else
            {
                chassis.ChassisSpeed_Ref.forward_back_ref = 0;
            }

            if(RC_CtrlData.Key_Flag.Key_A_Flag)
            {
                if (chassis.chassis_speed_mode == HIGH_SPEED_MODE)
                {
                    chassis.ChassisSpeed_Ref.left_right_ref = -chassis_speed/2;
                }else
                {
                    chassis.ChassisSpeed_Ref.left_right_ref = -chassis_speed;
                }
            }else if(RC_CtrlData.Key_Flag.Key_D_Flag)
            {
                if (chassis.chassis_speed_mode == HIGH_SPEED_MODE)
                {
                    chassis.ChassisSpeed_Ref.left_right_ref = chassis_speed/2;
                }else
                {
                    chassis.ChassisSpeed_Ref.left_right_ref = chassis_speed;
                }
            }else
            {
                chassis.ChassisSpeed_Ref.left_right_ref = 0;
            }

            if (RC_CtrlData.Key_Flag.Key_CTRL_TFlag)
            {

                chassis.ctrl_mode = CHASSIS_ROTATE;
            }
            else
            {
                chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
            }
					 
           /*******************************键鼠云台赋值****************************************/
            if (gimbal_data.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)
            {
                VAL_LIMIT(RC_CtrlData.mouse.x, -100, 100);
                VAL_LIMIT(RC_CtrlData.mouse.y, -100, 100);
                gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref += RC_CtrlData.mouse.x * MOUSE_TO_YAW_ANGLE_INC_FACT;
                gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref += RC_CtrlData.mouse.y * MOUSE_TO_PITCH_ANGLE_INC_FACT;
                VAL_LIMIT(gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref, pitch_min, pitch_max);
            }
            else
            {
                if (RC_CtrlData.Key_Flag.Key_V_TFlag)
                {
                    gimbal_data.ctrl_mode = GIMBAL_AUTO_BIG_BUFF;
                }
                else if (RC_CtrlData.Key_Flag.Key_Z_TFlag)
                {
                    gimbal_data.ctrl_mode = GIMBAL_AUTO_SMALL_BUFF;
                }
                else
                {
                    gimbal_data.ctrl_mode = GIMBAL_INIT;
                    chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
                }
            }
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
				chassis.follow_gimbal = 1;
        
        
			/*****************************************************************************************/
    }
        break;
    case STOP:
    {
            gimbal_data.ctrl_mode = GIMBAL_RELAX;
            chassis.ctrl_mode = CHASSIS_RELAX;
    }
        break;
    default:
        break;
    }
    gimbal_data.last_ctrl_mode = gimbal_data.ctrl_mode;
	chassis.last_ctrl_mode = chassis.ctrl_mode;
}













