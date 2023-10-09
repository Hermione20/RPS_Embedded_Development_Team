#include "hero_mode_switch_task.h"


int16_t chassis_speed = 0;


u8 this_input_mode = 0;
u8 last_input_mode = 0;

//英雄吊射云台微调变量
u8 auto_w_flag=1;
u32 auto_w_time =0;
u8 auto_s_flag=1;
u32 auto_s_time =0;
u8 auto_a_flag=1;
u32 auto_a_time =0;
u8 auto_d_flag=1;
u32 auto_d_time =0;

void hero_mode_switch_task(void)
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
	//--------------------------------------
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
            gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref += (RC_CtrlData.rc.ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * 0.1f;
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
            if (gimbal_data.ctrl_mode == GIMBAL_FOLLOW_ZGYRO&&RC_CtrlData.mouse.press_r == 0)
            {
                VAL_LIMIT(RC_CtrlData.mouse.x, -100, 100);
                VAL_LIMIT(RC_CtrlData.mouse.y, -100, 100);
                gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref += RC_CtrlData.mouse.x * MOUSE_TO_YAW_ANGLE_INC_FACT;
                gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref -= RC_CtrlData.mouse.y * MOUSE_TO_PITCH_ANGLE_INC_FACT;
                VAL_LIMIT(gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref, pitch_min, pitch_max);
            }
            /*****************************吊射模式下云台赋值***********************************/
            else if (gimbal_data.ctrl_mode == GIMBAL_AUTO_ANGLE&&RC_CtrlData.mouse.press_r == 0)
            {
							//如果第一次进入吊射模式，请将ref值重新初始化以防止突变
							
                if(RC_CtrlData.Key_Flag.Key_W_Flag&&auto_w_flag)
                {
                    if(RC_CtrlData.Key_Flag.Key_SHIFT_Flag)
                    {
                        gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref += 50;
                    }else
                    {
                        gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref += 15;
                    }
                    auto_w_flag = 0;
                }

                if(RC_CtrlData.Key_Flag.Key_S_Flag&&auto_s_flag)
                {
                    if(RC_CtrlData.Key_Flag.Key_SHIFT_Flag)
                    {
                        gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref -= 50;
                    }else
                    {
                        gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref -= 15;
                    }
                    auto_s_flag = 0;
                }

                if(RC_CtrlData.Key_Flag.Key_A_Flag&&auto_a_flag)
                {
                    if(RC_CtrlData.Key_Flag.Key_SHIFT_Flag)
                    {
                        gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref -= 0.5;
                    }else
                    {
                        gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref -= 0.2;
                    }
                    auto_a_flag = 0;
                }

                if(RC_CtrlData.Key_Flag.Key_D_Flag&&auto_d_flag)
                {
                    if(RC_CtrlData.Key_Flag.Key_SHIFT_Flag)
                    {
                        gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref += 0.5;
                    }else
                    {
                        gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref += 0.2;
                    }
                    auto_d_flag = 0;
                }

                if(auto_w_flag == 0)
					auto_w_time++;
				else
					auto_w_time = 0;
				if(auto_w_time == 20)
				{
					auto_w_flag = 1;
					auto_w_time = 0;
				}
				
				if(auto_s_flag == 0)
					auto_s_time++;
				else
					auto_s_time = 0;
				if(auto_s_time == 20)
				{
					auto_s_flag = 1;
					auto_s_time = 0;
				}
				
				if(auto_a_flag == 0)
					auto_a_time++;
				else
					auto_a_time = 0;
				if(auto_a_time == 20)
				{
					auto_a_flag = 1;
					auto_a_time = 0;
				}
				
				if(auto_d_flag == 0)
					auto_d_time++;
				else
					auto_d_time = 0;
				if(auto_d_time == 20)
				{
					auto_d_flag = 1;
					auto_d_time = 0;
				}
                VAL_LIMIT(gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref, pitch_min, pitch_max);
            }
        /*************************************************************************************/
            
          
         }

         /****************************底盘默认状态设置**********************************************/
        if(gimbal_data.ctrl_mode == GIMBAL_INIT||gimbal_data.ctrl_mode == GIMBAL_AUTO_ANGLE)
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
				gimbal_data.if_finish_Init = 0;
	    }
			if( gimbal_data.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)
				chassis.follow_gimbal = 1;

			//真正的模式选择从现在开始
				if(gimbal_data.if_finish_Init == 1)
				{
            if(RC_CtrlData.Key_Flag.Key_Q_TFlag)
            {
                gimbal_data.ctrl_mode = GIMBAL_AUTO_ANGLE;
            }else
            {
                gimbal_data.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
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













