#include "balance_task.h"



Balance_chassis_t b_chassis = { 0 };
    int Init_cnt;
    float V_T_gain;
    float V_Tp_gain;
    float balance_Tgain;
    float balance_Tpgain;

    float V_T_outlandgain ;
    float V_Tp_outlandgain ;
    float balance_Toutlandgain ;
    float balance_Tpoutlandgain ;


		
/**
************************************************************************************************************************
* @Name     : balance_param_init
* @brief    : 平衡底盘参数初始化
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void balance_param_init(void)
{
    memset(&b_chassis, 0, sizeof(Balance_chassis_t));
    b_chassis.chassis_dynemic_ref.leglength = 0.21;
    PID_struct_init(&b_chassis.left_leg.leglengthpid, POSITION_PID,20000,20000,1200,0,40000);
    PID_struct_init(&b_chassis.right_leg.leglengthpid, POSITION_PID, 20000, 20000, 1200, 0, 40000);
    PID_struct_init(&b_chassis.leg_harmonize_pid, POSITION_PID, 2000, 2000, 150, 0, 3000);
    PID_struct_init(&b_chassis.vw_pid, POSITION_PID,5,5,2,0,0);
    PID_struct_init(&b_chassis.roll_pid, POSITION_PID, 50000, 20000, 800, 0, 12000);
	
		PID_struct_init(&b_chassis.pid_follow_gim, POSITION_PID, 500, 200, 7, 0, 10);
	
	PID_struct_init(&b_chassis.Init_Tp_pid, POSITION_PID, 500, 200, 40, 0, 60);
	
}

/**
************************************************************************************************************************
* @Name     : balance_chassis_task
* @brief    : 平衡底盘总控制任务
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void balance_chassis_task(void)
{
    balance_cmd_select();
    switch (b_chassis.ctrl_mode)
    {
    case CHASSIS_RELAX:
    {
			
        b_chassis.joint_T[0] = 0;
			  b_chassis.joint_T[1] = 0;
			  b_chassis.joint_T[2] = 0;
				b_chassis.joint_T[3] = 0;
				b_chassis.driving_T[0] = 0;
			  b_chassis.driving_T[1] = 0;
			Init_cnt = 0;
			balance_Tpgain = 0;
			balance_Tpoutlandgain = 0;
			b_chassis.chassis_ref.pitch = 0;
			b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
			b_chassis.normal_Y_erroffset = NORMAL_Y_ERROFFSET;
			b_chassis.roll_pid.iout = 0;

    }
    break;
		case CHASSIS_STOP:
		{
			chassis_stop_handle();
			balance_task();
		}break;
    case CHASSIS_INIT:
    {
        chassis_Init_handle();

    }
    break;
		case CHASSIS_STAND_MODE:
		{
			chassis_standup_handle();
			balance_task();
		}break;
    case CHASSIS_SEPARATE:
    {
				chassis_seperate_handle();
        balance_task();
    }
    break;
    case MANUAL_FOLLOW_GIMBAL:
    {
			if(b_chassis.jump_flag==1)
			{
				balance_jump_handle();
				balance_task();
			}else
			{
				follow_gimbal_handle();
				Software_power_limit_handle();
        balance_task();
			}
    }
    break;
    case CHASSIS_ROTATE:
    {
			chassis_rotate_handle();
			Software_power_limit_handle();
			balance_task();
    }break;
		case CHASSIS_REVERSE:
		{
			chassis_side_handle();
			Software_power_limit_handle();
			balance_task();
		}break;
    default:
        break;
    }
#if POWER_LIMIT == 1
    power_limit_handle();
#else
    b_chassis.max_speed = 2.2;
    b_chassis.min_speed = -2.2;
		b_chassis.Max_power_to_PM01 = input_power_cal();
//		b_chassis.predict_power = all_power_cal(balance_chassis.Driving_Encoder[0].Torque,-2.528,0.000494,1,balance_chassis.Driving_Encoder[0].rate_rpm) + all_power_cal(balance_chassis.Driving_Encoder[1].Torque,-2.528,0.000494,1,balance_chassis.Driving_Encoder[1].rate_rpm);

#endif 
}


/**
************************************************************************************************************************
* @Name     : balance_cmd_select
* @brief    : 平衡底盘模式与命令处理
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void balance_cmd_select(void)
{
    b_chassis.last_ctrl_mode = b_chassis.ctrl_mode;
	if((b_chassis.ctrl_mode != CHASSIS_INIT&&b_chassis.ctrl_mode != CHASSIS_STAND_MODE)||usart_chassis_data.chassis_mode == 0)
    b_chassis.ctrl_mode = usart_chassis_data.chassis_mode;

    if (b_chassis.ctrl_mode != CHASSIS_INIT)
        {
            b_chassis.chassis_dynemic_ref.vy = usart_chassis_data.y/100.0f;
            b_chassis.chassis_dynemic_ref.vx = usart_chassis_data.x/100.0f;
            VAL_LIMIT(b_chassis.chassis_dynemic_ref.vy,b_chassis.min_speed,b_chassis.max_speed);
            VAL_LIMIT(b_chassis.chassis_dynemic_ref.vx,b_chassis.min_speed,b_chassis.max_speed);
        }
        if(b_chassis.ctrl_mode != CHASSIS_INIT&&usart_chassis_data.chassis_mode != CHASSIS_RELAX&&b_chassis.last_ctrl_mode == CHASSIS_RELAX&&usart_chassis_data.if_follow_gim)
        {
            b_chassis.ctrl_mode = usart_chassis_data.chassis_mode;
        }
        if (b_chassis.last_ctrl_mode == CHASSIS_RELAX && b_chassis.ctrl_mode != CHASSIS_RELAX)
	    {
		    b_chassis.ctrl_mode = CHASSIS_INIT;
	    }
        if (b_chassis.ctrl_mode == usart_chassis_data.chassis_mode && fabs(chassis_gyro.pitch_Angle) > 15 &&usart_chassis_data.chassis_mode != CHASSIS_RELAX&&usart_chassis_data.ctrl_mode!=1)
        {
            b_chassis.ctrl_mode = CHASSIS_INIT;
        }
				if(usart_chassis_data.jump_cmd)
				{
					b_chassis.jump_flag = usart_chassis_data.jump_cmd;
				}
				b_chassis.chassis_dynemic_ref.leglength = usart_chassis_data.cmd_leg_length/100.0f;
				
				get_remote_angle();
				VAL_LIMIT(b_chassis.chassis_ref.remote_speed,b_chassis.min_speed,b_chassis.max_speed);
}

/**
************************************************************************************************************************
* @Name     : get_remote_angle
* @brief    : 获取底盘转角
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void get_remote_angle(void)
{
	float vy;
	float vx;
	float temp_angle;
	
	b_chassis.yaw_angle_0_2pi = usart_chassis_data.yaw_Encoder_ecd_angle;
		if(b_chassis.yaw_angle_0_2pi>PI)
		{b_chassis.yaw_angle__pi_pi=b_chassis.yaw_angle_0_2pi-(2*PI);}
		else
		{b_chassis.yaw_angle__pi_pi=b_chassis.yaw_angle_0_2pi;}
		
	vy = b_chassis.chassis_dynemic_ref.vy;
	vx = b_chassis.chassis_dynemic_ref.vx;
	VAL_LIMIT(vx,-1.5,1.5);
	
	if(vy==0&&vx==0)
	{
		b_chassis.chassis_ref.remote_angle = 0;
		b_chassis.chassis_ref.remote_speed = 0;
	}else
	{
		b_chassis.chassis_ref.remote_speed = sqrt((vx*vx)+(vy*vy));
		temp_angle=atan2(vy,vx) - PI/2;
		if(temp_angle < -PI)
		{
			b_chassis.chassis_ref.remote_angle = temp_angle+2*PI;
		}else
		{
			b_chassis.chassis_ref.remote_angle = temp_angle;
		}
	}
	
	
}

/**
************************************************************************************************************************
* @Name     : chassis_standup_handle
* @brief    : 起立模式
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void chassis_standup_handle(void)
{
	b_chassis.chassis_ref.leglength = 0.15f;
    b_chassis.chassis_ref.vy = 0;
    b_chassis.chassis_ref.vx = 0;
    b_chassis.chassis_ref.vw = 0;
	b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
	
		if(fabs(b_chassis.balance_loop.state_err[4])<2*PI/180.0)
		b_chassis.ctrl_mode = usart_chassis_data.chassis_mode;
	
	
}


/**
************************************************************************************************************************
* @Name     : chassis_Init_handle
* @brief    : 初始化收腿
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void chassis_Init_handle(void)
{
		Init_cnt++;
    b_chassis.chassis_ref.leglength = 0.14f;
    b_chassis.chassis_ref.vy = 0;
    b_chassis.chassis_ref.vx = 0;
    b_chassis.chassis_ref.vw = 0;
	VMC_data_get(&b_chassis.left_leg,-balance_chassis.joint_Encoder[1].angle,
                                        -balance_chassis.joint_Encoder[1].gyro,
                                        -balance_chassis.joint_Encoder[2].angle+PI,
                                        -balance_chassis.joint_Encoder[2].gyro);

        VMC_data_get(&b_chassis.right_leg,balance_chassis.joint_Encoder[0].angle,
                                        balance_chassis.joint_Encoder[0].gyro,
                                        balance_chassis.joint_Encoder[3].angle+PI,
                                        balance_chassis.joint_Encoder[3].gyro);
	float phi0 = ((b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0)/2.0f) - 1.57f;
	if((fabs(phi0) >= 4*PI/180))
	{
		float Init_Tp = pid_calc(&b_chassis.Init_Tp_pid,phi0,0);
		//双腿协调pid
    float harmonize_output = pid_calc(&b_chassis.leg_harmonize_pid, (b_chassis.right_leg.phi0 - b_chassis.left_leg.phi0), 0);
		
		//腿部竖直力F的计算
    b_chassis.left_leg.leg_F = pid_calc(&b_chassis.left_leg.leglengthpid, b_chassis.left_leg.l0, b_chassis.chassis_ref.leglength);
    b_chassis.right_leg.leg_F = pid_calc(&b_chassis.right_leg.leglengthpid, b_chassis.right_leg.l0, b_chassis.chassis_ref.leglength);
		
		leg_conv(b_chassis.left_leg.leg_F, Init_Tp-harmonize_output, b_chassis.left_leg.phi1, b_chassis.left_leg.phi4, b_chassis.left_leg.T);
        b_chassis.joint_T[1] = b_chassis.left_leg.T[1];
        b_chassis.joint_T[2] = b_chassis.left_leg.T[0];
        b_chassis.driving_T[0] = 0 ;
		
		leg_conv(b_chassis.right_leg.leg_F, Init_Tp+ harmonize_output, b_chassis.right_leg.phi1, b_chassis.right_leg.phi4, b_chassis.right_leg.T);
        b_chassis.joint_T[0] = b_chassis.right_leg.T[1];
        b_chassis.joint_T[3] = b_chassis.right_leg.T[0];
        b_chassis.driving_T[1] = 0;
		
		//电机输出限幅
    VAL_LIMIT(b_chassis.joint_T[1], -34 , 34);
    VAL_LIMIT(b_chassis.joint_T[2], -34, 34);
    VAL_LIMIT(b_chassis.driving_T[0], -5, 5);

    VAL_LIMIT(b_chassis.joint_T[0], -34, 34);
    VAL_LIMIT(b_chassis.joint_T[3], -34, 34);
    VAL_LIMIT(b_chassis.driving_T[1], -5, 5);
	}else
	{

		b_chassis.ctrl_mode = CHASSIS_STAND_MODE;
		Init_cnt = 0;
		
	}

    
    

}


/**
************************************************************************************************************************
* @Name     : chassis_seperate_handle
* @brief    : 单个底盘模式，测试用
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void chassis_seperate_handle(void)
{
    b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;
    b_chassis.chassis_ref.vy = b_chassis.chassis_dynemic_ref.vy;
    b_chassis.chassis_ref.vx = b_chassis.chassis_dynemic_ref.vx;
    b_chassis.chassis_ref.vw = b_chassis.chassis_dynemic_ref.vw;
	b_chassis.chassis_ref.y_position += b_chassis.chassis_ref.vy*0.001*TIME_STEP;
}


/**
************************************************************************************************************************
* @Name     : follow_gimbal_handle
* @brief    : 底盘跟随云台
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
float target_angle;
void follow_gimbal_handle(void)
{
		
		   PID_struct_init(&b_chassis.roll_pid, POSITION_PID, 50000, 20000, 800, 0, 12000);
	     b_chassis.roll_pid.iout = 0;
    b_chassis.chassis_ref.vx = b_chassis.chassis_dynemic_ref.vx;
//		b_chassis.chassis_ref.y_position += b_chassis.chassis_ref.vy*0.001*TIME_STEP;
		if(fabs(b_chassis.balance_loop.dx) > 0.1||b_chassis.chassis_ref.vy != 0||usart_chassis_data.ctrl_mode==1)
			b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
		else
			b_chassis.normal_Y_erroffset-=b_chassis.balance_loop.dx*0.001*TIME_STEP;
		
		if(fabs(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi)<PI/2)
		{
			target_angle = b_chassis.chassis_ref.remote_angle;
			b_chassis.chassis_ref.vy = b_chassis.chassis_ref.remote_speed;
		}else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi > 3*PI/2)
		{
			target_angle = b_chassis.chassis_ref.remote_angle-2*PI;
			b_chassis.chassis_ref.vy = b_chassis.chassis_ref.remote_speed;
		}else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi < -3*PI/2)
		{
			target_angle = b_chassis.chassis_ref.remote_angle+2*PI;
			b_chassis.chassis_ref.vy = b_chassis.chassis_ref.remote_speed;
		}
		else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi>0)
		{
			target_angle = b_chassis.chassis_ref.remote_angle - PI;
			b_chassis.chassis_ref.vy = -b_chassis.chassis_ref.remote_speed;
		}else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi<0)
		{
			target_angle = b_chassis.chassis_ref.remote_angle + PI;
			b_chassis.chassis_ref.vy = -b_chassis.chassis_ref.remote_speed;
		}
		
		b_chassis.chassis_ref.vw = -pid_calc(&b_chassis.pid_follow_gim,b_chassis.yaw_angle__pi_pi,target_angle); 
		VAL_LIMIT(b_chassis.chassis_ref.vw,-5,5);
		
		b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;
		
		
}

/**
************************************************************************************************************************
* @Name     : balance_jump_handle
* @brief    : 跳跃任务
* @param		: jump_state
* @retval   : void
* @Note     :	jump_state用于记录跳跃状态
************************************************************************************************************************
**/
u8 jump_state = 0;
void balance_jump_handle(void)
{
	b_chassis.chassis_ref.vy = b_chassis.balance_loop.dx;
	b_chassis.chassis_ref.vx = 0;
	b_chassis.chassis_ref.vw = 0;
	b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
	if(jump_state == 0)
	{
		b_chassis.chassis_ref.leglength = 0.14f;
		if(fabs(b_chassis.balance_loop.L0 - b_chassis.chassis_ref.leglength)<=0.02)
		{
			jump_state++;
		}
	}else if(jump_state == 1)
	{
		PID_struct_init(&b_chassis.left_leg.leglengthpid, POSITION_PID,20000,20000,4000,0,15000);
    PID_struct_init(&b_chassis.right_leg.leglengthpid, POSITION_PID, 20000, 20000, 4000, 0, 15000);
		b_chassis.chassis_ref.leglength = 0.34f;
		if((fabs(b_chassis.balance_loop.L0 - b_chassis.chassis_ref.leglength)<=0.01))
			jump_state++;
		
	}else if(jump_state == 2)
	{
		b_chassis.chassis_ref.leglength = 0.14f;
		if(fabs(b_chassis.balance_loop.L0 - b_chassis.chassis_ref.leglength)<=0.01)
		{
			PID_struct_init(&b_chassis.left_leg.leglengthpid, POSITION_PID,20000,20000,1200,0,40000);
			PID_struct_init(&b_chassis.right_leg.leglengthpid, POSITION_PID, 20000, 20000, 1200, 0, 40000);
			jump_state=0;
			b_chassis.jump_flag = 0;
		}
	}
}


/**
************************************************************************************************************************
* @Name     : chassis_rotate_handle
* @brief    : 小陀螺
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void chassis_rotate_handle(void)
{
	PID_struct_init(&b_chassis.roll_pid, POSITION_PID, 50000, 30, 2000, 10, 12000);
	if(b_chassis.yaw_angle_0_2pi>=PI)
		{b_chassis.yaw_angle__pi_pi=b_chassis.yaw_angle_0_2pi-(2*PI);}
		else
		{b_chassis.yaw_angle__pi_pi=b_chassis.yaw_angle_0_2pi;}
		
		b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;
    b_chassis.chassis_ref.vy = b_chassis.balance_loop.dx;//b_chassis.chassis_dynemic_ref.vy*sinf(b_chassis.yaw_angle__pi_pi)+b_chassis.chassis_dynemic_ref.vx*cosf(b_chassis.yaw_angle__pi_pi);
    b_chassis.chassis_ref.vx = 0;
		
		//b_chassis.chassis_ref.y_position += b_chassis.chassis_ref.vy*0.001*TIME_STEP;
		
		b_chassis.chassis_ref.vw = usart_chassis_data.rotate_speed; 
		VAL_LIMIT(b_chassis.chassis_ref.vw,-15,15);
		VAL_LIMIT(b_chassis.chassis_ref.vy,-0.5,0.5);
}


/**
************************************************************************************************************************
* @Name     : chassis_side_handle
* @brief    : 侧向对敌
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void chassis_side_handle(void)
{
	 float side_angle;
	if((b_chassis.yaw_angle_0_2pi>=0)&&((b_chassis.yaw_angle_0_2pi<=PI)))
		{
			side_angle = PI/2;
			b_chassis.chassis_ref.vy = -b_chassis.chassis_dynemic_ref.vx;
		}
		else
		{
			side_angle = 3*PI/2;
			b_chassis.chassis_ref.vy = b_chassis.chassis_dynemic_ref.vx;
		}
		
		b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;
    

		if(fabs(b_chassis.balance_loop.dx) > 0.1||b_chassis.chassis_ref.vy != 0)
			b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
		else
			b_chassis.normal_Y_erroffset-=b_chassis.balance_loop.dx*0.001*TIME_STEP;
		
		b_chassis.chassis_ref.vw = -pid_calc(&b_chassis.pid_follow_gim,b_chassis.yaw_angle_0_2pi,side_angle); 
		VAL_LIMIT(b_chassis.chassis_ref.vw,-5,5);
		VAL_LIMIT(b_chassis.chassis_ref.vy,-1.4,1.4);
}

/**
************************************************************************************************************************
* @Name     : chassis_stop_handle
* @brief    : 停止模式，大幅用
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void chassis_stop_handle(void)
{
    b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;
    b_chassis.chassis_ref.vy = 0;
    b_chassis.chassis_ref.vx = 0;
    b_chassis.chassis_ref.vw = 0;
	b_chassis.chassis_ref.y_position += b_chassis.chassis_ref.vy*0.001*TIME_STEP;
}

/**
************************************************************************************************************************
* @Name     : balance_task
* @brief    : 平衡底盘解算
* @param		: void
* @retval   : void
* @Note     :	一定要注意弧度制的转化啊
							电机极性要仔细检查
							检查好各个传感器的单位与性能
************************************************************************************************************************
**/
void balance_task(void)
{
    /********************各个计算量的更新************************************/
        VMC_data_get(&b_chassis.left_leg,-balance_chassis.joint_Encoder[1].angle,
                                        -balance_chassis.joint_Encoder[1].gyro,
                                        -balance_chassis.joint_Encoder[2].angle+PI,
                                        -balance_chassis.joint_Encoder[2].gyro);

        VMC_data_get(&b_chassis.right_leg,balance_chassis.joint_Encoder[0].angle,
                                        balance_chassis.joint_Encoder[0].gyro,
                                        balance_chassis.joint_Encoder[3].angle+PI,
                                        balance_chassis.joint_Encoder[3].gyro);
   
    //计算状态变量    
    b_chassis.balance_loop.phi = chassis_gyro.pitch_Angle*PI/180.0f;
    b_chassis.balance_loop.dphi = chassis_gyro.pitch_Gyro*PI/180.0f;
    b_chassis.balance_loop.x = ((balance_chassis.Driving_Encoder[0].angle + (-balance_chassis.Driving_Encoder[1].angle))/2.0f) * WHEEL_R;
    b_chassis.balance_loop.dx = Mileage_kalman_filter.velocity;
    b_chassis.balance_loop.theta = ((((b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0)/2.0f) - 1.57f) - chassis_gyro.pitch_Angle*PI/180.0f);
    b_chassis.balance_loop.dtheta = (((b_chassis.left_leg.dphi0 + b_chassis.right_leg.dphi0)/2.0f) - chassis_gyro.pitch_Gyro*PI/180.0f);
    b_chassis.balance_loop.ddz = chassis_gyro.z_Acc*cos(chassis_gyro.pitch_Angle*PI/180.0f);
	
	  b_chassis.balance_loop.wheel_dx = ((balance_chassis.Driving_Encoder[0].gyro + (-balance_chassis.Driving_Encoder[1].gyro))/2.0f) * WHEEL_R;

    b_chassis.balance_loop.L0 = (b_chassis.left_leg.l0 + b_chassis.right_leg.l0)/2.0f;
		
		
		b_chassis.balance_loop.Fm = b_chassis.chassis_ref.vw*b_chassis.chassis_ref.vy*BODY_MASS;
    
    //计算支持力
    FN_calculate(&b_chassis.left_leg,-balance_chassis.joint_Encoder[2].Torque,-balance_chassis.joint_Encoder[1].Torque);//capacitance_message.out_power
    FN_calculate(&b_chassis.right_leg,balance_chassis.joint_Encoder[3].Torque,balance_chassis.joint_Encoder[0].Torque);//balance_chassis.Driving_Encoder[0].rate_rpm
																																																											 //balance_chassis.Driving_Encoder[0].Torque

    /*****************************************************************/

    //lqr参数根据腿长的变化获取
    lqr_k(b_chassis.balance_loop.L0, b_chassis.balance_loop.K);
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            
                b_chassis.balance_loop.k[j][i] = b_chassis.balance_loop.K[i * 2 + j];
            
                                                                                      
        }
            
    }
   
   //误差计算
    b_chassis.balance_loop.state_err[0] = -(b_chassis.balance_loop.theta);
	b_chassis.balance_loop.state_err[1] = -(b_chassis.balance_loop.dtheta);
	b_chassis.balance_loop.state_err[2] = -(b_chassis.balance_loop.x -b_chassis.chassis_ref.y_position);
	b_chassis.balance_loop.state_err[3] = -(b_chassis.balance_loop.dx - b_chassis.chassis_ref.vy);
	b_chassis.balance_loop.state_err[4] = b_chassis.chassis_ref.pitch-(b_chassis.balance_loop.phi);
	b_chassis.balance_loop.state_err[5] = -(b_chassis.balance_loop.dphi);
	
    //对腿变化加速度的限制
	if(b_chassis.chassis_dynemic_ref.vy == 1.6)
	{
    VAL_LIMIT(b_chassis.balance_loop.state_err[3], -1.2, 1.2);
	}else if(b_chassis.chassis_dynemic_ref.vy == 2.2)
	{
		VAL_LIMIT(b_chassis.balance_loop.state_err[3], -1.0, 1.0);
	}else
	{
		VAL_LIMIT(b_chassis.balance_loop.state_err[3], -1.2, 1.2);
	}

    //lqr未离地增益计算
		
    V_T_gain = b_chassis.balance_loop.k[0][3] * b_chassis.balance_loop.state_err[3];
    V_Tp_gain = b_chassis.balance_loop.k[1][3] * b_chassis.balance_loop.state_err[3] ;
    balance_Tgain = b_chassis.balance_loop.k[0][0] * b_chassis.balance_loop.state_err[0] + b_chassis.balance_loop.k[0][1] * b_chassis.balance_loop.state_err[1] + b_chassis.balance_loop.k[0][2] * (b_chassis.balance_loop.state_err[2]+b_chassis.normal_Y_erroffset) + b_chassis.balance_loop.k[0][4] * b_chassis.balance_loop.state_err[4] + b_chassis.balance_loop.k[0][5] * b_chassis.balance_loop.state_err[5];
    balance_Tpgain = b_chassis.balance_loop.k[1][0] * b_chassis.balance_loop.state_err[0] + b_chassis.balance_loop.k[1][1] * b_chassis.balance_loop.state_err[1] + b_chassis.balance_loop.k[1][2] * (b_chassis.balance_loop.state_err[2]+b_chassis.normal_Y_erroffset) + b_chassis.balance_loop.k[1][4] * b_chassis.balance_loop.state_err[4] + b_chassis.balance_loop.k[1][5] * b_chassis.balance_loop.state_err[5];
		
			
    //lqr离地增益计算
    V_T_outlandgain = 0;
    V_Tp_outlandgain = 0;
    balance_Toutlandgain = 0;
    balance_Tpoutlandgain = b_chassis.balance_loop.k[1][0] * b_chassis.balance_loop.state_err[0] + b_chassis.balance_loop.k[1][1] * b_chassis.balance_loop.state_err[1] ;
		
    
		
		
    //双腿协调pid
    float harmonize_output = pid_calc(&b_chassis.leg_harmonize_pid, (b_chassis.right_leg.phi0 - b_chassis.left_leg.phi0), 0);
    //转向pid
    float vw_torque = pid_calc(&b_chassis.vw_pid, chassis_gyro.yaw_Gyro*PI/180.0f, b_chassis.chassis_ref.vw);
    //roll平衡pid
    float roll_F_output = pid_calc(&b_chassis.roll_pid,chassis_gyro.roll_Angle*PI/180.0f,0);
    
    
    //腿部竖直力F的计算
    b_chassis.left_leg.leg_F = pid_calc(&b_chassis.left_leg.leglengthpid, b_chassis.left_leg.l0, b_chassis.chassis_ref.leglength)+ (BODY_MASS/2) * 9.8 + roll_F_output + b_chassis.balance_loop.Fm*0.5;
    b_chassis.right_leg.leg_F = pid_calc(&b_chassis.right_leg.leglengthpid, b_chassis.right_leg.l0, b_chassis.chassis_ref.leglength) + (BODY_MASS/2)*9.8 - roll_F_output - b_chassis.balance_loop.Fm*0.5;
    
   
		if(usart_chassis_data.ctrl_mode==1)
		{
			V_Tp_gain = 0;
			balance_Tgain = 0;
			balance_Tpgain = 0;
			b_chassis.left_leg.leg_F = 0;
			b_chassis.right_leg.leg_F = 0;
			V_T_gain = b_chassis.balance_loop.state_err[3]*2;
		}
		//lqr输出
    b_chassis.balance_loop.lqrOutT = balance_Tgain + V_T_gain;
    b_chassis.balance_loop.lqrOutTp = balance_Tpgain + V_Tp_gain;
    //此处的T0为phi1电机的扭矩，另一个是phi4的
    if (wheel_state_estimate(&b_chassis.left_leg)||(b_chassis.ctrl_mode==CHASSIS_INIT))
    {
        leg_conv(b_chassis.left_leg.leg_F, b_chassis.balance_loop.lqrOutTp-harmonize_output, b_chassis.left_leg.phi1, b_chassis.left_leg.phi4, b_chassis.left_leg.T);
        b_chassis.joint_T[1] = b_chassis.left_leg.T[1];
        b_chassis.joint_T[2] = b_chassis.left_leg.T[0];
        b_chassis.driving_T[0] = b_chassis.balance_loop.lqrOutT / 2.0f + vw_torque;
			
    }
    else
    {
			b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
        leg_conv(b_chassis.left_leg.leg_F, balance_Tpoutlandgain - harmonize_output, b_chassis.left_leg.phi1, b_chassis.left_leg.phi4, b_chassis.left_leg.T);
        b_chassis.joint_T[1] = b_chassis.left_leg.T[1];
        b_chassis.joint_T[2] = b_chassis.left_leg.T[0];
        b_chassis.driving_T[0] = 0;
			
			
    }

    if (wheel_state_estimate(&b_chassis.right_leg)||(b_chassis.ctrl_mode==CHASSIS_INIT))
    {
        leg_conv(b_chassis.right_leg.leg_F, b_chassis.balance_loop.lqrOutTp+ harmonize_output, b_chassis.right_leg.phi1, b_chassis.right_leg.phi4, b_chassis.right_leg.T);
        b_chassis.joint_T[0] = b_chassis.right_leg.T[1];
        b_chassis.joint_T[3] = b_chassis.right_leg.T[0];
        b_chassis.driving_T[1] = b_chassis.balance_loop.lqrOutT / 2.0f - vw_torque;
    }
    else
    {
			b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
        leg_conv(b_chassis.right_leg.leg_F, balance_Tpoutlandgain + harmonize_output, b_chassis.right_leg.phi1, b_chassis.right_leg.phi4, b_chassis.right_leg.T);
        b_chassis.joint_T[0] = b_chassis.right_leg.T[1];
        b_chassis.joint_T[3] = b_chassis.right_leg.T[0];
        b_chassis.driving_T[1] = 0;
    }


    
    
    //电机输出限幅
    VAL_LIMIT(b_chassis.joint_T[1], -34 , 34);
    VAL_LIMIT(b_chassis.joint_T[2], -34, 34);
    VAL_LIMIT(b_chassis.driving_T[0], -5, 5);

    VAL_LIMIT(b_chassis.joint_T[0], -34, 34);
    VAL_LIMIT(b_chassis.joint_T[3], -34, 34);
    VAL_LIMIT(b_chassis.driving_T[1], -5, 5);

    
}


/**
************************************************************************************************************************
* @Name     : middle_angle_adjust_handle
* @brief    : 重心自适应算法
* @param		: None
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/

void middle_angle_adjust_handle(void)
{
	if(fabs(b_chassis.chassis_ref.vy)==0.0f && fabs(b_chassis.balance_loop.wheel_dx) < 0.20f && fabs(chassis_gyro.yaw_Gyro*PI/180.0f)<=0.04f)
	{
		if(b_chassis.balance_loop.wheel_dx > 0.03f)
		{
			b_chassis.chassis_ref.pitch +=  b_chassis.balance_loop.wheel_dx*0.001;
		}else if(b_chassis.balance_loop.wheel_dx < -0.03f)
		{
			b_chassis.chassis_ref.pitch +=  b_chassis.balance_loop.wheel_dx*0.001;
		}
	}
	   
}

/**
************************************************************************************************************************
* @Name     : Software_power_limit_handle
* @brief    : 功率控制的主函数
* @param		: None
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/
	float limit_vw,limit_vy;
void Software_power_limit_handle(void)
{

	if(capacitance_message.cap_voltage_filte < 20)
	{
		switch(usart_chassis_data.chassis_power_limit)
		{
			case 45:
			{
					limit_vw = 7;
					limit_vy = 1.5;
			}break;
			case 50:
			{
					limit_vw = 7;
					limit_vy = 1.5;
			}break;
			case 60:
			{
				limit_vw = 8;
					limit_vy = 1.6;
			}break;
			case 70:
			{
				limit_vw = 9;
					limit_vy = 1.8;
			}break;
			case 80:
			{
					limit_vw = 9;
					limit_vy = 2.0;
			}break;
			case 100:
			{
				
			}break;
			}
					
				}else if(capacitance_message.cap_voltage_filte > 22)
				{
					limit_vw = 10;
					limit_vy = b_chassis.max_speed;
				}
		if(capacitance_message.cap_voltage_filte < 17)
		{
			PID_struct_init(&b_chassis.vw_pid, POSITION_PID,2,2,2,0,0);
		}else
		{
			PID_struct_init(&b_chassis.vw_pid, POSITION_PID,5,5,2,0,0);
		}
		
		VAL_LIMIT(b_chassis.chassis_ref.vw,-limit_vw,limit_vw);
		VAL_LIMIT(b_chassis.chassis_ref.vy,-limit_vy,limit_vy);
}
/**
************************************************************************************************************************
* @Name     : power_limit_handle
* @brief    : 功率控制的主函数
* @param		: None
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/

void power_limit_handle(void)
{
    b_chassis.Max_power_to_PM01 = input_power_cal();
    get_speed_err_limite_rate(output_power_cal(capacitance_message.cap_voltage_filte));
}

/**
************************************************************************************************************************
* @Name     : input_power_cal
* @brief    : 计算发送给功率控制板的最大功率值
* @retval   : Max_Power
* @Note     : 在此处处理缓冲功率
************************************************************************************************************************
**/

float input_power_cal(void)
{
	  float judge_power = usart_chassis_data.chassis_power_limit +
                      (usart_chassis_data.chassis_power_buffer - 5) * 2;
//		float judge_power = 80;
    float Max_Power = judge_power;

    if (capacitance_message.cap_voltage_filte >= 23.0)
    {
        Max_Power = (23.7 - capacitance_message.cap_voltage_filte) * 150;
        VAL_LIMIT(Max_Power, 0, judge_power);
    }
    if (capacitance_message.cap_voltage_filte >= 23.7)
    {
        Max_Power = 0;
    }

    VAL_LIMIT(Max_Power, 0, 150);
    return Max_Power;
}

/**
************************************************************************************************************************
* @Name     : output_power_cal
* @brief    : 计算功率控制板输出的最大功率值
* @retval   : Max_Power
* @Note     : 在此处进行软件限制
************************************************************************************************************************
**/
float output_power_cal(float voltage)//限制电压防止电压过低导致电机复位
{ 
	int max_power=0;
  if(voltage>WARNING_VOLTAGE+3)
    max_power=150;
  else
    max_power=b_chassis.Max_power_to_PM01;
  VAL_LIMIT(max_power,0,150);
  return max_power;
//  return 80;
}


/**
************************************************************************************************************************
* @Name     : get_speed_err_limite_rate
* @brief    : 根据给定最大功率求出最优功率限制系数
* @param		: max_power
* @retval   : 二次方程的根（可以优化！10.13）
* @Note     :
************************************************************************************************************************
**/

void get_speed_err_limite_rate(float max_power)
{
    static float K1 = 0.0f;
    static float K2 = 0.0f;
    static float K3 = 0.0f;
    float w = ((balance_chassis.Driving_Encoder[0].rate_rpm + (-balance_chassis.Driving_Encoder[1].rate_rpm))/2.0f);
    float Vmax[2];
    b_chassis.predict_power = all_power_cal(b_chassis.balance_loop.lqrOutT,K1,K2,K3,w);
    Vmax_cal(b_chassis.balance_loop.k[0][3],max_power,balance_Tgain,K1,K2,K3,w,Vmax);
    if (Vmax[0]>0)
    {
        b_chassis.max_speed = Vmax[0];
        b_chassis.min_speed = Vmax[1];
    }else
    {
        b_chassis.max_speed = Vmax[1];
        b_chassis.min_speed = Vmax[0];
    }
    
}

/* Function Definitions */
/*
 * ALL_POWER_CAL
 *     P = ALL_POWER_CAL(T,K1,K2,K3,W)
 *
 * Arguments    : float T
 *                float k1
 *                float k2
 *                float k3
 *                float w
 * Return Type  : float
 */
float all_power_cal(float T, float k1, float k2, float k3, float w)
{
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     2023-12-31 00:57:20 */
  return ((k3 + T * w * 0.10471204188481675) + T * T * k1) + k2 * (w * w);
}

/*
 * File trailer for all_power_cal.c
 *

/* Function Definitions */
/*
 * Vmax_cal
 *     Vmax = Vmax_cal(Kv,Pmax,bT_gain,K1,K2,K3,W)
 *
 * Arguments    : float Kv
 *                float Pmax
 *                float bT_gain
 *                float k1
 *                float k2
 *                float k3
 *                float w
 *                float Vmax[2]
 * Return Type  : void
 */
void Vmax_cal(float Kv, float Pmax, float bT_gain, float k1, float k2,
              float k3, float w, float Vmax[2])
{
  float Vmax_tmp;
  float b_Vmax_tmp;
  float t2;
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     2023-12-31 01:37:29 */
  t2 = w * w;
  t2 = sqrt(((Pmax * k1 * 16.0 - k1 * k3 * 16.0) - k1 * k2 * t2 * 16.0) +
            t2 * 0.043858446862750471) *
       191.0;
  Vmax_tmp = 1.0 / Kv * (1.0 / k1);
  b_Vmax_tmp = w * 40.0 + bT_gain * k1 * 764.0;
  Vmax[0] = Vmax_tmp * (b_Vmax_tmp - t2) * -0.00065445026178010475;
  Vmax[1] = Vmax_tmp * (b_Vmax_tmp + t2) * -0.00065445026178010475;
}

/*
 * File trailer for Vmax_cal.c
 *
 * [EOF]
 */

/**
************************************************************************************************************************
* @Name     : wheel_state_estimate
* @brief    : 底盘离地检测函数
* @param		: leg
* @retval   : wheel_state
* @Note     :
************************************************************************************************************************
**/

uint8_t wheel_state_estimate(leg_state_t *leg)
{
    if (leg->leg_FN < 15)
    {
			
        leg->wheel_state = 0;
        return 0;
    }
    else
    {
        leg->wheel_state = 1;
        return 1;
    }
}


/**
************************************************************************************************************************
* @Name     : lqr_k
* @brief    : 底盘倒立摆lqr计算函数
* @param		: double L0, double K[12]
* @retval   : void
* @Note     : This function was generated by the Symbolic Math Toolbox version 23.2.
************************************************************************************************************************
**/

void lqr_k(double L0, double K[12])
{
    double t2;
    double t3;
    /*     This function was generated by the Symbolic Math Toolbox version 23.2.
     */
     /*     2023-11-17 17:36:04 */
    t2 = L0 * L0;
    t3 = L0 * L0 * L0;


//		K[0] = ((L0 * -224.8656362888122 + t2 * 287.28601354985511) -
//          t3 * 167.080125914703) -
//         2.2708328063653309;
//  K[1] = ((L0 * 799.62215955328907 - t2 * 2421.0071871579148) +
//          t3 * 2369.2356209053869) -
//         18.715545190698261;
//  K[2] = ((L0 * -40.38018715277537 + t2 * 32.521969964233293) -
//          t3 * 21.552686854980259) +
//         0.78425211175398291;
//  K[3] = ((L0 * 141.5900544765947 - t2 * 419.48263098475292) +
//          t3 * 411.93317206269018) -
//         0.46043751213191358;
//  K[4] = ((L0 * -17.15704148443394 + t2 * 10.446769679445049) +
//          t3 * 12.03299467720135) -
//         2.279837861427144;
//  K[5] = ((L0 * 161.6106764263622 - t2 * 545.89886715268119) +
//          t3 * 568.97639790267169) -
//         4.3911422530775761;
//  K[6] = ((L0 * -23.559263858644151 - t2 * 0.25207525679081089) +
//          t3 * 36.613891289564663) -
//         5.0130786727043821;
//  K[7] = ((L0 * 271.99755480600669 - t2 * 928.21025842917868) +
//          t3 * 975.53282181648024) -
//         6.4120436957134306;
//  K[8] = ((L0 * -72.738997754922465 + t2 * 71.495363483826637) -
//          t3 * 27.011943732562759) +
//         37.21816652483988;
//  K[9] = ((L0 * 248.84544086263139 - t2 * 351.82171741912708) +
//          t3 * 163.43913537877319) +
//         34.99279463955817;
//  K[10] = ((L0 * -15.502668700859459 + t2 * 33.523719735351087) -
//           t3 * 31.186727381276171) +
//          5.6619913186474813;
//  K[11] = ((L0 * -10.663987348138379 + t2 * 80.881742458802307) -
//           t3 * 106.2776716888919) +
//          5.1536359190828316;

  
				
 if(b_chassis.ctrl_mode == CHASSIS_REVERSE||b_chassis.chassis_ref.vx != 0)
 {
	K[0] = ((L0 * -166.700655487403 + t2 * 310.40496987752272) -
          t3 * 266.253098888401) -
         5.1659900740138527;
  K[1] = ((L0 * 81.818861351604454 - t2 * 325.58297699603628) +
          t3 * 363.94896624414048) +
         12.20307867941224;
  K[2] = ((L0 * -14.267427353745539 + t2 * 3.7241741648246061) -
          t3 * 4.9128674498726879) -
         0.66270337619891273;
  K[3] = ((L0 * 4.2691484838940381 - t2 * 18.271848489835989) +
          t3 * 20.07906017097195) +
         2.0771651138383529;
  K[4] = ((L0 * -33.197238930809377 + t2 * 82.500934886222979) -
          t3 * 74.510133215860961) -
         3.6260383545289989;
  K[5] = ((L0 * -15.421484000669819 - t2 * 25.135627915412819) +
          t3 * 59.735923567667669) +
         10.17334896132809;
  K[6] = ((L0 * -19.271005148269111 + t2 * 38.05563556377173) -
          t3 * 31.687940514421989) -
         4.4878263601997563;
  K[7] = ((L0 * -21.50885842237216 + t2 * 9.7047654367477048) +
          t3 * 15.767316148188669) +
         9.2267510015333478;
  K[8] = ((L0 * -85.192936703491156 + t2 * 148.2423222620086) -
          t3 * 104.9760724587584) +
         25.12413278369683;
  K[9] = ((L0 * 145.012692339202 - t2 * 356.10775862585882) +
          t3 * 319.91533093350989) +
         14.928284932628261;
  K[10] = ((L0 * -17.349495174302621 + t2 * 32.807077769741987) -
           t3 * 25.516326693488381) +
          5.2967107669759166;
  K[11] = ((L0 * 28.833930841075919 - t2 * 68.035599136499442) +
           t3 * 59.619025767661952) +
          1.517413529554708;
				}else
 {
	 K[0] = ((L0 * -188.0979417294775 + t2 * 196.2711201387192) -
          t3 * 102.1898825273429) -
         4.0444121665275814;
  K[1] = ((L0 * 889.395127069674 - t2 * 2615.6322745719) +
          t3 * 2535.188316302409) -
         14.508601442326089;
  K[2] = ((L0 * -27.238914598312 + t2 * 3.4206070806111648) -
          t3 * 1.3307700052804621) +
         0.32095098946980122;
  K[3] = ((L0 * 137.7749599043191 - t2 * 372.64676999781028) +
          t3 * 356.54333514254972) +
         0.45381509174984069;
  K[4] = ((L0 * -17.572099196096321 + t2 * 6.94704206930822) +
          t3 * 16.713686389144168) -
         1.803965837261339;
  K[5] = ((L0 * 197.03480757629529 - t2 * 691.11777594285388) +
          t3 * 729.93905142376923) -
         0.26797893632203129;
  K[6] = ((L0 * -21.508085724457629 - t2 * 10.745747937731959) +
          t3 * 46.309977855738957) -
         4.2054198883019076;
  K[7] = ((L0 * 306.74254739534513 - t2 * 1077.6546512317209) +
          t3 * 1142.8529673953) +
         0.67019773680314454;
  K[8] = ((L0 * -49.600450813044468 + t2 * 46.862666981597933) -
          t3 * 23.6227186268123) +
         29.91830985525225;
  K[9] = ((L0 * 146.84610685892849 + t2 * 17.732739412644261) -
          t3 * 237.79628209031921) +
         22.056563271908619;
  K[10] = ((L0 * -15.891732670145769 + t2 * 35.62274317381668) -
           t3 * 34.3525933185872) +
          5.9500628114089684;
  K[11] = ((L0 * -15.9840850382652 + t2 * 124.6211686324778) -
           t3 * 163.97155244350691) +
          3.2052607282776351;
 }
}

/*
 * File trailer for lqr_k.c
 *
 * [EOF]
 */


/**
************************************************************************************************************************
* @Name     : convert_ecd_angle_to_0_2pi
* @brief    : 将电机编码器的机械角度值（范围正负无穷大）解算为范围在0~2pi的角度值      
* @param		: ecd_angle 电机编码器的机械角度值  类型  double
* @param		: _0_2pi_angle 范围在0~2pi的角度值  类型  float
* @retval   : _0_2pi_angle 范围在0~2pi的角度值  类型  float
* @Note     : 
************************************************************************************************************************
**/
double convert_ecd_angle_to_0_2pi(double ecd_angle,float _0_2pi_angle)
{
	_0_2pi_angle=fmod(YAW_POLARITY*ecd_angle*ANGLE_TO_RAD,2*PI);	
	if(_0_2pi_angle<0)
		 _0_2pi_angle+=2*PI;

	return _0_2pi_angle;
}

