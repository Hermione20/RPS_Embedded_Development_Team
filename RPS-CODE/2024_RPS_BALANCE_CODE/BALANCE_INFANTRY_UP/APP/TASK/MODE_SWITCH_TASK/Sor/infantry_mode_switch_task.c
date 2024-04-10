#include "infantry_mode_switch_task.h"

chassis_t chassis;
int16_t chassis_speed = 0;
int16_t leg_length;
u8 this_input_mode = 0;
u8 last_input_mode = 0;

u8 reserve_flag = 0;
void infantry_mode_switch_task(void)
{
		//切换遥控模式的时候所有任务归位重新开始
		last_input_mode = this_input_mode;
		this_input_mode = RC_CtrlData.inputmode;
		if(this_input_mode != last_input_mode)
		{
			gimbal_data.ctrl_mode = GIMBAL_RELAX;
			gimbal_data.last_ctrl_mode = GIMBAL_RELAX;
      chassis.ctrl_mode = CHASSIS_RELAX;
			chassis.last_ctrl_mode = CHASSIS_RELAX;
		}
    switch (RC_CtrlData.inputmode)
    {
    case REMOTE_INPUT:
    {
        /*******************************底盘云台遥感数据接收******************************************/
        if(gimbal_data.ctrl_mode != GIMBAL_INIT)
        {
            chassis.ChassisSpeed_Ref.forward_back_ref = (RC_CtrlData.rc.ch1- (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * 0.4f;
            chassis.ChassisSpeed_Ref.left_right_ref   = (RC_CtrlData.rc.ch0- (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * 0.4f;
        }
        if(gimbal_data.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)
        {
            gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref += (RC_CtrlData.rc.ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
            gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref   += (RC_CtrlData.rc.ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT;
					VAL_LIMIT(gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref, pitch_min, pitch_max);
        }
        
        /****************************底盘默认状态设置**********************************************/
//        if(gimbal_data.ctrl_mode == GIMBAL_INIT||gimbal_data.ctrl_mode == GIMBAL_AUTO_BIG_BUFF||gimbal_data.ctrl_mode == GIMBAL_AUTO_SMALL_BUFF)
//        {
//            chassis.ctrl_mode = CHASSIS_STOP;
//        }
        /***************************云台默认状态设置**********************************************/
        if(gimbal_data.ctrl_mode != GIMBAL_INIT&&RC_CtrlData.inputmode != STOP&&gimbal_data.last_ctrl_mode == GIMBAL_RELAX)
        {
            gimbal_data.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
        }
        if (gimbal_data.last_ctrl_mode == GIMBAL_RELAX && gimbal_data.ctrl_mode != GIMBAL_RELAX)
	    {
		    gimbal_data.ctrl_mode = GIMBAL_INIT;
				gimbal_data.if_finish_Init = 0;
	    }
			if( gimbal_data.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)
			{
				
				chassis.follow_gimbal = 1;
			}
			//遥控器模式的模式选择从这里开始
			if(gimbal_data.if_finish_Init == 1)
			{
				gimbal_data.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
				
				if (RC_CtrlData.RemoteSwitch.s3to2)
        {

            chassis.ctrl_mode = CHASSIS_ROTATE;
					chassis.ChassisSpeed_Ref.rotate_ref = 10;
				
        }
        else
        {
            chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
					chassis.ChassisSpeed_Ref.rotate_ref = 0;
			
        }
			}
			leg_length = 16;
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
//                chassis.chassis_speed_mode = HIGH_SPEED_MODE;
                chassis_speed = HIGH_SPEED*100;
            }else
            {
//                chassis.chassis_speed_mode = NORMAL_SPEED_MODE;
                chassis_speed = NORMAL_SPEED*100;
            }
						
						if(RC_CtrlData.Key_Flag.Key_CTRL_Flag)
						{
							leg_length = 21;
						}else
						{
							leg_length = 16;
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

            
					 
           /*******************************键鼠云台赋值****************************************/
            if (gimbal_data.ctrl_mode == GIMBAL_FOLLOW_ZGYRO&&RC_CtrlData.mouse.press_r == 0)
            {
                VAL_LIMIT(RC_CtrlData.mouse.x, -100, 100);
                VAL_LIMIT(RC_CtrlData.mouse.y, -100, 100);
                gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref += RC_CtrlData.mouse.x * MOUSE_TO_YAW_ANGLE_INC_FACT;
                gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref -= RC_CtrlData.mouse.y * MOUSE_TO_PITCH_ANGLE_INC_FACT;
                VAL_LIMIT(gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref, pitch_min, pitch_max);
							//一键反向
							if(RC_CtrlData.Key_Flag.Key_X_Flag)
							{
								if(reserve_flag==0)
								{
									gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref +=180.0f;
									reserve_flag++;
								}
							}else
							{
								reserve_flag = 0;
							}
            }
            
         }

         /****************************底盘默认状态设置**********************************************/
        if(gimbal_data.ctrl_mode == GIMBAL_INIT||gimbal_data.ctrl_mode == GIMBAL_AUTO_BIG_BUFF||gimbal_data.ctrl_mode == GIMBAL_AUTO_SMALL_BUFF)
        {
            chassis.ctrl_mode = CHASSIS_STOP;
        }
        /***************************云台默认状态设置**********************************************/
        if(gimbal_data.ctrl_mode != GIMBAL_INIT&&RC_CtrlData.inputmode != STOP&&gimbal_data.last_ctrl_mode == GIMBAL_RELAX)
        {
            gimbal_data.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
        }
        if (gimbal_data.last_ctrl_mode == GIMBAL_RELAX && gimbal_data.ctrl_mode != GIMBAL_RELAX)
	    {
		    gimbal_data.ctrl_mode = GIMBAL_INIT;
				gimbal_data.if_finish_Init = 0;
	    }
			if( gimbal_data.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)
				chassis.follow_gimbal = 1;
      //键鼠模式的模式选择从这里开始
			if(gimbal_data.if_finish_Init == 1)
			{
//				if (RC_CtrlData.Key_Flag.Key_V_TFlag)
//                {
//                    gimbal_data.ctrl_mode = GIMBAL_AUTO_BIG_BUFF;
//                }
//                else if (RC_CtrlData.Key_Flag.Key_Z_TFlag)
//                {
//                    gimbal_data.ctrl_mode = GIMBAL_AUTO_SMALL_BUFF;
//                }
//                else
                {
                    gimbal_data.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
                    chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
                }
								
								if (RC_CtrlData.Key_Flag.Key_B_TFlag)
            {

                chassis.ctrl_mode = CHASSIS_ROTATE;
							chassis.ChassisSpeed_Ref.rotate_ref = 10;
            }
            else if(RC_CtrlData.Key_Flag.Key_F_TFlag)
            {
                chassis.ctrl_mode = CHASSIS_REVERSE;
							chassis.ChassisSpeed_Ref.rotate_ref = 0;
            }else
						{
							chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
							chassis.ChassisSpeed_Ref.rotate_ref = 0;
						}
						
						

						}
        
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
	chassis.last_ctrl_mode = chassis.ctrl_mode;
}













