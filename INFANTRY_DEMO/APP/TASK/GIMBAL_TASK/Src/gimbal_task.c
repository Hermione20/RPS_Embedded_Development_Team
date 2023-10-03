#include "gimbal_task.h"


#define STANDARD 3

gimbal_t gimbal_data;


#if STANDARD == 1
float pitch_middle = 0;
float pitch_min = 0;
float pitch_max = 0;
#elif STANDARD == 3
float pitch_min = INFANTRY_PITCH_MIN;
float pitch_max = INFANTRY_PITCH_MAX;
#elif STANDARD == 4
float pitch_min = INFANTRY_PITCH_MIN;
float pitch_max = INFANTRY_PITCH_MAX;
#elif STANDARD == 5
float pitch_min = INFANTRY_PITCH_MIN;
float pitch_max = INFANTRY_PITCH_MAX;
#elif STANDARD == 6
float pitch_min = FIGHTER_PITCH_MIN;
float pitch_max = FIGHTER_PITCH_MAX;
#elif STANDARD == 7
float pitch_min = SECURITY_PITCH_MIN;
float pitch_max = SECURITY_PITCH_MAX;
#endif



void gimbal_parameter_Init(void)
{
    memset(&gimbal_data, 0, sizeof(gimbal_t));
		

    /*******************************pid_Init*********************************/
#if STANDARD == 1
#elif STANDARD == 3
    // 跟随陀螺仪下的参数
    PID_struct_init(&gimbal_data.pid_pit_Angle, POSITION_PID, 500, 4,
                    15, 0.01f, 8); //15, 0.01f, 8
    PID_struct_init(&gimbal_data.pid_pit_speed, POSITION_PID, 27000, 20000,
                    170, 0.001, 60); //170, 0.001f, 60
    //------------------------------------------------
    PID_struct_init(&gimbal_data.pid_yaw_Angle, POSITION_PID, 500, 4,
                    7, 0.15f, 8); 
    PID_struct_init(&gimbal_data.pid_yaw_speed, POSITION_PID, 29000, 10000,
                    150, 0.8f, 40); 

    //自瞄下参数
    PID_struct_init ( &gimbal_data.pid_pit_follow, POSITION_PID, 200, 10, 8
                    , 0.01, 8 );
    PID_struct_init ( &gimbal_data.pid_pit_speed_follow, POSITION_PID, 27000, 25000, 180.0f, 0.2f, 0 ); 

    PID_struct_init ( &gimbal_data.pid_yaw_follow, POSITION_PID,  150,200,
                    20, 0.09f, 40 );
    PID_struct_init ( &gimbal_data.pid_yaw_speed_follow, POSITION_PID, 29800, 29800,
                    350.0f, 0, 100 ); //I太大时，陀螺开启云台抖动严重

    //小幅下的参数
    PID_struct_init(&gimbal_data.pid_pit_small_buff, POSITION_PID, 70, 20,
                    25.0f, 0.3f, 0); 
    PID_struct_init(&gimbal_data.pid_pit_speed_small_buff, POSITION_PID, 25000, 20000,
                    350.0f, 7.0f, 0); 
    PID_struct_init(&gimbal_data.pid_yaw_small_buff, POSITION_PID, 60, 20,
                    15.0f, 0.3f, 20);
    PID_struct_init(&gimbal_data.pid_yaw_speed_small_buff, POSITION_PID, 25000, 25000,
                    450.0f, 4.0f, 200);

    //大幅下的参数
    PID_struct_init(&gimbal_data.pid_pit_big_buff, POSITION_PID, 200, 10,
                    20.0f, 0.2f, 2); 
    PID_struct_init(&gimbal_data.pid_pit_speed_big_buff, POSITION_PID, 27000, 25000,
                    300.0f, 8.0f, 200); 
    PID_struct_init(&gimbal_data.pid_yaw_big_buff, POSITION_PID, 250, 4,
                    20.0f, 0.2f, 2); 
    PID_struct_init(&gimbal_data.pid_yaw_speed_big_buff, POSITION_PID, 25000, 5000,
                    300.0f, 8.0f, 200);
#elif STANDARD == 4
#elif STANDARD == 5
#elif STANDARD == 6
#elif STANDARD == 7
#endif
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
#if STANDARD == 1
    case GIMBAL_AUTO_ANGLE:
    {
        gimbal_auto_angle_handle();
    }
        break;
#elif (STANDARD == 3)||(STANDARD == 4)||(STANDARD == 5)
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
#elif STANDARD == 7
    case GIMBAL_AUTO_AIM:
    {
        security_gimbal_handle();
    }
        break;
#endif
    default:
        break;
    }
    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, pitch_min , pitch_max );

}


#if (STANDARD != 1)


/****************************big_or_small_buff_var************************************/
float last_pitch_angle;
float last_yaw_angle;
float yaw_angle_ref_aim,pit_angle_ref_aim;
float last_yaw,last_pit;
uint8_t first_flag = 0;
uint8_t ved = 0;
float Delta_Dect_Angle_Yaw,Delta_Dect_Angle_Pit;

#if STANDARD == 3
float Yaw_remain = -0.4;//向右为正
float pitch_remain=-0.5;//5.2;
#elif STANDARD == 4
float Yaw_remain = -0.2;//向右为正
float pitch_remain=6.15;//5.2;
#elif STANDARD == 5
float Yaw_remain =1;//向右为正
float pitch_remain=0.8 ;//5.2;
#elif STANDARD == 6
float Yaw_remain = -1.3;//向右为正
float pitch_remain=2.9;//5.2;


#endif
/*************************************************************************************/

void gimbal_init_handle	( void )
{
    //大幅参数初始化
    first_flag = 0;
		ved = 0;

    int init_rotate_num = 0;
    gimbal_data.gim_ref_and_fdb.pit_angle_ref = 0.0f;
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = gimbal_gyro.pitch_Angle;
		//步兵机械将电机反着装导致yaw轴电机向右编码器角度为负，与期望极性相反，需要加负号
    gimbal_data.gim_ref_and_fdb.yaw_angle_ref = 0.0f;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = -yaw_Encoder.ecd_angle;

    init_rotate_num = (-yaw_Encoder.ecd_angle)/360;
    gimbal_data.gim_ref_and_fdb.yaw_angle_ref = init_rotate_num*360;

   //通过劣弧转到正的角度
    if((gimbal_data.gim_ref_and_fdb.yaw_angle_ref-gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)>=181)
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref-=360;
    else if((gimbal_data.gim_ref_and_fdb.yaw_angle_ref-gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)<-179)
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref+=360;

    gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_Angle,
                                                                      &gimbal_data.pid_yaw_speed,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
                                                                      gimbal_gyro.yaw_Gyro,//yaw_Encoder.filter_rate,
                                                                      0 );
    gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_Angle,
                                                                      &gimbal_data.pid_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
                                                                      gimbal_gyro.pitch_Gyro,
                                                                      0 );
	if (fabs(gimbal_data.gim_ref_and_fdb.pit_angle_ref - gimbal_data.gim_ref_and_fdb.pit_angle_fdb)<=4&&fabs(gimbal_data.gim_ref_and_fdb.yaw_angle_ref - gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)<=1.5)
    {
        gimbal_data.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
				gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref = gimbal_gyro.yaw_Angle;
				gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref = gimbal_gyro.pitch_Angle;
        
    }
    																																		
}

void gimbal_follow_gyro_handle(void)
{
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = gimbal_gyro.pitch_Angle;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = gimbal_gyro.yaw_Angle;//yaw_Encoder.ecd_angle;//
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
                                                                      gimbal_gyro.yaw_Gyro,//yaw_Encoder.filter_rate,//
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
				
			yaw_angle_ref_aim=Delta_Dect_Angle_Yaw + new_location.x + Yaw_remain;
			pit_angle_ref_aim=Delta_Dect_Angle_Pit + new_location.y + pitch_remain;
        }
        last_yaw=new_location.x1;
		last_pit=new_location.y1;

        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = buff_kalman_filter.buff_yaw_angle;
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = raw_data_to_pitch_angle(buff_kalman_filter.buff_pitch_angle)+pitch_remain;;
    }
    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref,INFANTRY_PITCH_MIN,INFANTRY_PITCH_MAX);
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
				
			yaw_angle_ref_aim=Delta_Dect_Angle_Yaw + new_location.x + Yaw_remain;
			pit_angle_ref_aim=Delta_Dect_Angle_Pit + new_location.y + pitch_remain;
        }
        last_yaw=new_location.x1;
		last_pit=new_location.y1;

        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = buff_kalman_filter.buff_yaw_angle;
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = raw_data_to_pitch_angle(buff_kalman_filter.buff_pitch_angle)+pitch_remain;;
    }
    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref,INFANTRY_PITCH_MIN,INFANTRY_PITCH_MAX);
    if(ved==0)
    {
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = last_yaw_angle;
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = last_pitch_angle;
    }
    
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


void security_gimbal_handle(void)
{
    static int wait_tick = 0;//识别不到的时候多等两秒，确认看不到了进入巡逻

    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = gimbal_gyro.pitch_Angle;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = gimbal_gyro.yaw_Angle;
    if(new_location.control_flag)
    {
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = new_location.y;
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = new_location.x;

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
        wait_tick++;
        if(wait_tick<=200)
        {
            gimbal_data.gim_ref_and_fdb.pit_angle_ref = new_location.last_y;
            gimbal_data.gim_ref_and_fdb.yaw_angle_ref = new_location.last_x;

            gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_follow,
                                                                      &gimbal_data.pid_yaw_speed_follow,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
                                                                      gimbal_gyro.yaw_Gyro,
                                                                      0);
            gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_Angle,
                                                                      &gimbal_data.pid_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
                                                                      gimbal_gyro.pitch_Gyro,
                                                                      0 );
        }else
        {
            gimbal_data.gim_ref_and_fdb.pit_angle_ref = -2;

            gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_Angle,
                                                                      &gimbal_data.pid_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
                                                                      gimbal_gyro.pitch_Gyro,
                                                                      0 );
            gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_calc(&gimbal_data.pid_yaw_speed_follow,gimbal_gyro.yaw_Gyro,150);

        }
    }
}

float raw_data_to_pitch_angle(float ecd_angle_pit)
{
  int shoot_angle_speed;
  float distance_s;
  float distance_x;
  float distance_y;
  float x1;
  float x2;
  float x3;
  float x4;
  float angle_tan;
  float shoot_radian;
  float shoot_angle;
  float real_angle;
	
  shoot_angle_speed=28;//judge_rece_mesg.shoot_data.bullet_speed;
  distance_s=6.9/cos(gimbal_gyro.pitch_Angle*ANGLE_TO_RAD);//*cos((get_yaw_angle-yaw_Angle)*ANGLE_TO_RAD));//(Gimbal_Auto_Shoot.Distance-7)/100;
	real_angle=ecd_angle_pit+RAD_TO_ANGLE * atan2 ( HEIGHT_BETWEEN_GUN_CAMERA, distance_s );
	
  distance_x=(cos((ecd_angle_pit)*ANGLE_TO_RAD)*distance_s);
  distance_y=(sin((ecd_angle_pit)*ANGLE_TO_RAD)*distance_s);

  x1=shoot_angle_speed*shoot_angle_speed;
  x2=distance_x*distance_x;
  x3=sqrt(x2-(19.6*x2*((9.8*x2)/(2*x1)+distance_y))/x1);
  x4=9.8*x2;
  angle_tan=(x1*(distance_x-x3))/(x4);
  shoot_radian=atan(angle_tan);
  shoot_angle=shoot_radian*RAD_TO_ANGLE;
  return shoot_angle;
}


#else

void gimbal_init_handle	( void )
{

    int init_rotate_num = 0;

    gimbal_data.gim_ref_and_fdb.pit_angle_ref = 0.0f;
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = gimbal_gyro.pitch_Angle;

    gimbal_data.gim_ref_and_fdb.yaw_angle_ref = 0.0f;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = yaw_Encoder.ecd_angle;

    init_rotate_num = yaw_Encoder.ecd_angle/360;
    gimbal_data.gim_ref_and_fdb.yaw_angle_ref = init_rotate_num*360;

    //通过劣弧转到正的角度
    if((gimbal_data.gim_ref_and_fdb.yaw_angle_ref-gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)>=181)
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref-=360;
    else if((gimbal_data.gim_ref_and_fdb.yaw_angle_ref-gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)<-179)
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref+=360;

    //初始化时期的英雄，使用陀螺仪云台pid参数
    gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.hero_pid_yaw_Angle,
                                                                      &gimbal_data.hero_pid_yaw_speed,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
                                                                      gimbal_gyro.yaw_Gyro,//yaw_Encoder.filter_rate,
                                                                      0 );
    gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.hero_pid_pit_Angle,
                                                                      &gimbal_data.hero_pid_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
                                                                      gimbal_gyro.pitch_Gyro,
                                                                      0 );
	if (fabs(gimbal_data.gim_ref_and_fdb.pit_angle_ref - gimbal_data.gim_ref_and_fdb.pit_angle_fdb)<=4&&fabs(gimbal_data.gim_ref_and_fdb.yaw_angle_ref - gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)<=1.5)
    {
        gimbal_data.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
				gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref = gimbal_gyro.yaw_Angle;
				gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref = Pitch_Encoder.ecd_angle;
                pitch_middle = Pitch_Encoder.ecd_angle;
                pitch_max = pitch_middle+HERO_PITCH_MAX;
                pitch_min = pitch_middle+HERO_PITCH_MIN;
        
    }
    																																		
}


void gimbal_follow_gyro_handle(void)
{
    //此时pitch轴使用编码器作为反馈（操作手认为用陀螺仪晃）
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = Pitch_Encoder.ecd_angle;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = gimbal_gyro.yaw_Angle;
    if(RC_CtrlData.mouse.press_r)
    {
        if(new_location.flag)
        {
            //使用自瞄的时候切回陀螺仪给定
            gimbal_data.gim_ref_and_fdb.pit_angle_fdb = gimbal_gyro.pitch_Angle;
            gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = gimbal_gyro.yaw_Angle;
            gimbal_data.gim_ref_and_fdb.pit_angle_ref = new_location.y;
            gimbal_data.gim_ref_and_fdb.yaw_angle_ref = new_location.x;

        }

        gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_follow,
                                                                      &gimbal_data.pid_yaw_speed_follow,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
                                                                      gimbal_gyro.yaw_Gyro,//yaw_Encoder.filter_rate,//
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

void gimbal_auto_angle_handle(void)
{
    //吊射模式下全部使用编码器
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = Pitch_Encoder.ecd_angle;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = yaw_Encoder.ecd_angle;

    gimbal_data.gim_ref_and_fdb.pit_angle_ref = gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref;
    gimbal_data.gim_ref_and_fdb.yaw_angle_ref = gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref;
    gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_auto_yaw_Angle,
                                                                      &gimbal_data.pid_auto_yaw_speed,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
                                                                      gimbal_gyro.yaw_Gyro,
                                                                      0 );
    gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_auto_pit_Angle,
                                                                      &gimbal_data.pid_auto_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
                                                                      gimbal_gyro.pitch_Gyro,
                                                                      0 );
}


#endif
