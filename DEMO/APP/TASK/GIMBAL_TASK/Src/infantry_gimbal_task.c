#include "infantry_gimbal_task.h"

gimbal_t gimbal_data;

/****************************big_or_small_buff_var************************************/
float last_pitch_angle;
float last_yaw_angle;
float yaw_angle_ref_aim,pit_angle_ref_aim;
float last_yaw,last_pit;
uint8_t first_flag = 0;
uint8_t ved = 0;
float Delta_Dect_Angle_Yaw,Delta_Dect_Angle_Pit;
/*************************************************************************************/

void gimbal_parameter_Init(void)
{
    memset(&gimbal_data, 0, sizeof(gimbal_t));
    gimbal_data.ctrl_mode = GIMBAL_RELAX;
    gimbal_data.last_ctrl_mode = GIMBAL_RELAX;

    /*******************************pid_Init*********************************/

    /************************************************************************/
}

void gimbal_task(void)
{
    switch (gimbal_data.ctrl_mode)
    {
    case GIMBAL_RELAX:
    {
        gimbal_data.gim_ref_and_fdb.yaw_motor_input = 0;
        gimbal_data.gim_ref_and_fdb.pitch_motor_input = 0;
    }
        break;
    case GIMBAL_INIT:
    {
        gimbal_init_handle();
    }
        break;
    case GIMBAL_FOLLOW_ZGYRO:
    {
        gimbal_follow_gyro_handle();
    }
        break;
    case GIMBAL_AUTO_SMALL_BUFF:
    {
        auto_small_buff_handle();
    }
        break;
    case GIMBAL_AUTO_BIG_BUFF:
    {
        auto_big_buff_handle();
    }
        break;
    default:
        break;
    }
    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, PITCH_MIN , PITCH_MAX );

}

void gimbal_init_handle	( void )
{
    //大幅参数初始化
    first_flag = 0;
		ved = 0;

    int init_rotate_num = 0;
    gimbal_data.gim_ref_and_fdb.pit_angle_ref = 0.0f;
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = gimbal_gyro.pitch_Angle;

    gimbal_data.gim_ref_and_fdb.yaw_angle_ref = 0.0f;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = yaw_Encoder.ecd_angle;

    init_rotate_num = yaw_Encoder.ecd_angle/360;
    gimbal_data.gim_ref_and_fdb.yaw_angle_ref = init_rotate_num*360;

   
    gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_Angle,
                                                                      &gimbal_data.pid_yaw_speed,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
                                                                      gimbal_gyro.yaw_Gyro,
                                                                      0 );
    gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_Angle,
                                                                      &gimbal_data.pid_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
                                                                      gimbal_gyro.pitch_Gyro,
                                                                      0 );
	if (1)//(fabs(gimbal_data.gim_ref_and_fdb.pit_angle_ref - gimbal_data.gim_ref_and_fdb.pit_angle_fdb)<=4&&fabs(gimbal_data.gim_ref_and_fdb.pit_angle_ref - gimbal_data.gim_ref_and_fdb.pit_angle_fdb)<=1.5)
    {
        gimbal_data.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
        
    }
    																																		
}

void gimbal_follow_gyro_handle(void)
{
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = gimbal_gyro.pitch_Angle;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = gimbal_gyro.yaw_Angle;
    if(RC_CtrlData.mouse.press_r)
    {
        if(new_location.flag)
        {
            gimbal_data.gim_ref_and_fdb.pit_angle_ref = new_location.y;
            gimbal_data.gim_ref_and_fdb.yaw_angle_ref = new_location.x;

        }

        gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_follow,
                                                                      &gimbal_data.pid_yaw_speed_follow,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
                                                                      gimbal_gyro.yaw_Gyro,
                                                                      new_location.yaw_speed);
        gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_Angle,
                                                                      &gimbal_data.pid_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
                                                                      gimbal_gyro.pitch_Gyro,
                                                                      0 );
    }else
    {
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref;
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref;
    gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_Angle,
                                                                      &gimbal_data.pid_yaw_speed,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
                                                                      gimbal_gyro.yaw_Gyro,
                                                                      0 );
    gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_Angle,
                                                                      &gimbal_data.pid_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
                                                                      gimbal_gyro.pitch_Gyro,
                                                                      0 );
    }
}


void auto_small_buff_handle(void)
{
    if(first_flag == 0)
	{
		last_pitch_angle=gimbal_gyro.pitch_Angle;
		last_yaw_angle=gimbal_gyro.yaw_Angle;
		first_flag = 1;
	}
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = gimbal_gyro.pitch_Angle;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = gimbal_gyro.yaw_Angle;
    if(new_location.xy_0_flag)
    {
        new_location.xy_o_time++;
    }else
    {
        new_location.xy_o_time=0;
    }
    if(new_location.xy_o_time<1)
    {
        ved = 1;
        if(last_yaw==new_location.x1&&last_pit==new_location.y1)
        {
            Delta_Dect_Angle_Yaw = RAD_TO_ANGLE * atan2 ( ( (double) new_location.x1 ) * TARGET_SURFACE_LENGTH,FOCAL_LENGTH);
            Delta_Dect_Angle_Pit = RAD_TO_ANGLE * atan2 ( ( (double) new_location.y1 ) * TARGET_SURFACE_WIDTH,FOCAL_LENGTH);
				
			yaw_angle_ref_aim=Delta_Dect_Angle_Yaw + new_location.x;
			pit_angle_ref_aim=Delta_Dect_Angle_Pit + new_location.y;
        }
        last_yaw=new_location.x1;
		last_pit=new_location.y1;

        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = 1;
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = 1;
    }
    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref,PITCH_MIN,PITCH_MAX);
    if(ved==0)
    {
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = last_yaw_angle;
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = last_pitch_angle;
    }
    
    gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_small_buff,
                                                                      &gimbal_data.pid_yaw_speed_small_buff,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
                                                                      gimbal_gyro.yaw_Gyro,
                                                                      0 );
    gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_small_buff,
                                                                      &gimbal_data.pid_pit_speed_small_buff,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
                                                                      gimbal_gyro.pitch_Gyro,
                                                                      0 );
}

void auto_big_buff_handle(void)
{
    if(first_flag == 0)
	{
		last_pitch_angle=gimbal_gyro.pitch_Angle;
		last_yaw_angle=gimbal_gyro.yaw_Angle;
		first_flag = 1;
	}
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = gimbal_gyro.pitch_Angle;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = gimbal_gyro.yaw_Angle;
    if(new_location.xy_0_flag)
    {
        new_location.xy_o_time++;
    }else
    {
        new_location.xy_o_time=0;
    }
    if(new_location.xy_o_time<1)
    {
        ved = 1;
        if(last_yaw==new_location.x1&&last_pit==new_location.y1)
        {
            Delta_Dect_Angle_Yaw = RAD_TO_ANGLE * atan2 ( ( (double) new_location.x1 ) * TARGET_SURFACE_LENGTH,FOCAL_LENGTH);
            Delta_Dect_Angle_Pit = RAD_TO_ANGLE * atan2 ( ( (double) new_location.y1 ) * TARGET_SURFACE_WIDTH,FOCAL_LENGTH);
				
			yaw_angle_ref_aim=Delta_Dect_Angle_Yaw + new_location.x;
			pit_angle_ref_aim=Delta_Dect_Angle_Pit + new_location.y;
        }
        last_yaw=new_location.x1;
		last_pit=new_location.y1;

        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = 1;
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = 1;
    }
    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref,PITCH_MIN,PITCH_MAX);
    if(ved==0)
    {
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = last_yaw_angle;
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = last_pitch_angle;
    }
    //gimbal_data.gim_ref_and_fdb.yaw_motor_input = gimbal_yaw_loop_task(&gimbal_data.pid_yaw_big_buff,&gimbal_data.pid_yaw_speed_big_buff,gimbal_data.gim_ref_and_fdb.yaw_angle_ref,gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,0);
    //gimbal_data.gim_ref_and_fdb.pitch_motor_input = gimbal_pit_loop_task(&gimbal_data.pid_pit_big_buff,&gimbal_data.pid_pit_speed_big_buff,gimbal_data.gim_ref_and_fdb.pit_angle_ref,gimbal_data.gim_ref_and_fdb.pit_angle_fdb,0);
    gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_big_buff,
                                                                      &gimbal_data.pid_yaw_speed_big_buff,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
                                                                      gimbal_gyro.yaw_Gyro,
                                                                      0 );
    gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_big_buff,
                                                                      &gimbal_data.pid_pit_speed_big_buff,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
                                                                      gimbal_gyro.pitch_Gyro,
                                                                      0 );
}

