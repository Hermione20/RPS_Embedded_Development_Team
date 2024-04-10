#include "gimbal_task.h"

/**
  ******************************************************************************
  * @file    gimbal_task.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   该模块为通用云台模块，参数设置位于源文件上部分和头文件上部分：
						 源文件：兵种id（1：英雄，2：工程，3-4-5：步兵，6：飞机，7：哨兵）
						 
										云台限位设置
										视觉云台限位
										云台初始化反馈设置
										普通模式云台反馈设置
										自瞄或吊射云台反馈设置
										电机输出极性设置
										自瞄角度补偿
										pid设置
						 头文件：大幅部分参数设置
						 
	* @notice  该模块通用与所有兵种，请未来的代码维护者与开发人员维护模块的
						 独立性，维护各云台间的通用性，禁止将属于云台部分的逻辑与代码
						 写到别的模块内，该模块仅负责云台的控制，参考输入的赋值请移步
						 模式选择。
						 
	* @notice  云台模块的调用请移步至control_task，推荐云台计算频率为2ms
						 推荐云台电机发送频率为2ms。在controltask里放置gimbal_task
						 在control_task_Init里放置gimbal_parameter_Init
						 
	*	@introduction 本模块采用状态机的方式编写云台的各种模式与功能，控制信号
									的输入与模式的切换与选择来自mode_switch_tasks，全模块的
									变量由gimbal_t结构体包含，模块可自定义状态观测器的更新来
									源，对应接口为宏定义结尾为_FDB，可调用框架的通用传感器结
									构体变量如：
									#define YAW_INIT_ANGLE_FDB          -yaw_Encoder.ecd_angle
									
 ===============================================================================
 **/
 
 
 
 
 
 
 
 /**
  ******************************************************************************
																			参数设置
		兵种id（1：英雄，2：工程，3-4-5：步兵，6：飞机，7：哨兵）
		
		云台限位设置
		视觉云台限位
		云台初始化反馈设置
		普通模式云台反馈设置
		自瞄或吊射云台反馈设置
		电机输出极性设置
		自瞄角度补偿
		
	 =============================================================================
 **/
 
#define STANDARD 3

gimbal_t gimbal_data;
//云台限位
float pitch_min = 0;		
float pitch_max = 0;		

#if STANDARD == 1

		#define HERO_PITCH_MAX 8548
		#define HERO_PITCH_MIN -1963
		
    float pitch_middle = 0;
    float Pitch_min = HERO_PITCH_MIN;
    float Pitch_max = HERO_PITCH_MAX;
    
    #define VISION_PITCH_MIN            -25  //视觉的传感器视角往往都是世界坐标系
    #define VISION_PITCH_MAX            30

    #define YAW_INIT_ANGLE_FDB          -yaw_Encoder.ecd_angle
    #define PITCH_INIT_ANGLE_FDB        -gimbal_gyro.roll_Angle
    #define YAW_INIT_SPEED_FDB          gimbal_gyro.yaw_Gyro
    #define PITCH_INIT_SPEED_FDB        -gimbal_gyro.roll_Gyro

    #define YAW_ANGLE_FDB               gimbal_gyro.yaw_Angle
    #define PITCH_ANGLE_FDB             Pitch_Encoder.ecd_angle
    #define YAW_SPEED_FDB               gimbal_gyro.yaw_Gyro
    #define PITCH_SPEED_FDB             -gimbal_gyro.roll_Gyro

    #define VISION_YAW_ANGLE_FDB        gimbal_gyro.yaw_Angle
    #define VISION_PITCH_ANGLE_FDB      -gimbal_gyro.roll_Angle
    #define VISION_YAW_SPEED_FDB        gimbal_gyro.yaw_Gyro
    #define VISION_PITCH_SPEED_FDB      -gimbal_gyro.roll_Gyro

    #define YAW_AUTO_ANGLE_FDB          -yaw_Encoder.ecd_angle
    #define PITCH_AUTO_ANGLE_FDB        Pitch_Encoder.ecd_angle
    #define YAW_AUTO_SPEED_FDB          gimbal_gyro.yaw_Gyro
    #define PITCH_AUTO_SPEED_FDB        -gimbal_gyro.roll_Gyro

    #define YAW_MOTOR_POLARITY          -1
    #define PITCH_MOTOR_POLARITY        1

    float auto_aim_Yaw_remain = 0;
    float auto_aim_pitch_remain = 0;
#elif STANDARD == 3

		#define INFANTRY_PITCH_MAX 25.0f
		#define INFANTRY_PITCH_MIN -21.5f
    float pitch_middle = 0;
    float Pitch_min = INFANTRY_PITCH_MIN;
    float Pitch_max = INFANTRY_PITCH_MAX;

    #define VISION_PITCH_MIN            -25
    #define VISION_PITCH_MAX            35

    #define YAW_INIT_ANGLE_FDB          -yaw_Encoder.ecd_angle   //步兵机械将电机反着装导致yaw轴电机向右编码器角度为负，与期望极性相反，需要加负号
    #define PITCH_INIT_ANGLE_FDB        gimbal_gyro.pitch_Angle
    #define YAW_INIT_SPEED_FDB          gimbal_gyro.yaw_Gyro
    #define PITCH_INIT_SPEED_FDB        gimbal_gyro.pitch_Gyro

    #define YAW_ANGLE_FDB               gimbal_gyro.yaw_Angle
    #define PITCH_ANGLE_FDB             gimbal_gyro.pitch_Angle
    #define YAW_SPEED_FDB               gimbal_gyro.yaw_Gyro
    #define PITCH_SPEED_FDB             gimbal_gyro.pitch_Gyro

    #define VISION_YAW_ANGLE_FDB        gimbal_gyro.yaw_Angle
    #define VISION_PITCH_ANGLE_FDB      gimbal_gyro.pitch_Angle
    #define VISION_YAW_SPEED_FDB        gimbal_gyro.yaw_Gyro
    #define VISION_PITCH_SPEED_FDB      gimbal_gyro.pitch_Gyro

    #define YAW_MOTOR_POLARITY          -1
    #define PITCH_MOTOR_POLARITY        1

    float Buff_Yaw_remain = 1.3;
    float Buff_pitch_remain=-0.3;

    float auto_aim_Yaw_remain = 0;
    float auto_aim_pitch_remain = 0;
#elif STANDARD == 4
		
		#define INFANTRY_PITCH_MAX 35.0f
		#define INFANTRY_PITCH_MIN -25.0f
		
    float pitch_middle = 0;
    float Pitch_min = INFANTRY_PITCH_MIN;
    float Pitch_max = INFANTRY_PITCH_MAX;

    #define VISION_PITCH_MIN            -25
    #define VISION_PITCH_MAX            30

    #define YAW_INIT_ANGLE_FDB          -yaw_Encoder.ecd_angle
    #define PITCH_INIT_ANGLE_FDB        gimbal_gyro.pitch_Angle
    #define YAW_INIT_SPEED_FDB          gimbal_gyro.yaw_Gyro
    #define PITCH_INIT_SPEED_FDB        gimbal_gyro.pitch_Gyro

    #define YAW_ANGLE_FDB               gimbal_gyro.yaw_Angle
    #define PITCH_ANGLE_FDB             gimbal_gyro.pitch_Angle
    #define YAW_SPEED_FDB               gimbal_gyro.yaw_Gyro
    #define PITCH_SPEED_FDB             gimbal_gyro.pitch_Gyro

    #define VISION_YAW_ANGLE_FDB        gimbal_gyro.yaw_Angle
    #define VISION_PITCH_ANGLE_FDB      gimbal_gyro.pitch_Angle
    #define VISION_YAW_SPEED_FDB        gimbal_gyro.yaw_Gyro
    #define VISION_PITCH_SPEED_FDB      gimbal_gyro.pitch_Gyro

    #define YAW_MOTOR_POLARITY          -1
    #define PITCH_MOTOR_POLARITY        -1

    float Buff_Yaw_remain = -0.2;//向右为正
    float Buff_pitch_remain=6.15;//5.2;

    float auto_aim_Yaw_remain = 0;
    float auto_aim_pitch_remain = 0;
#elif STANDARD == 5
		
		#define INFANTRY_PITCH_MAX 35.0f
		#define INFANTRY_PITCH_MIN -25.0f
		
    float pitch_middle = 0;
    float Pitch_min = INFANTRY_PITCH_MIN;
    float Pitch_max = INFANTRY_PITCH_MAX;

    #define VISION_PITCH_MIN            -25
    #define VISION_PITCH_MAX            30

    #define YAW_INIT_ANGLE_FDB          -yaw_Encoder.ecd_angle
    #define PITCH_INIT_ANGLE_FDB        gimbal_gyro.pitch_Angle
    #define YAW_INIT_SPEED_FDB          gimbal_gyro.yaw_Gyro
    #define PITCH_INIT_SPEED_FDB        gimbal_gyro.pitch_Gyro

    #define YAW_ANGLE_FDB               gimbal_gyro.yaw_Angle
    #define PITCH_ANGLE_FDB             gimbal_gyro.pitch_Angle
    #define YAW_SPEED_FDB               gimbal_gyro.yaw_Gyro
    #define PITCH_SPEED_FDB             gimbal_gyro.pitch_Gyro

    #define VISION_YAW_ANGLE_FDB        gimbal_gyro.yaw_Angle
    #define VISION_PITCH_ANGLE_FDB      gimbal_gyro.pitch_Angle
    #define VISION_YAW_SPEED_FDB        gimbal_gyro.yaw_Gyro
    #define VISION_PITCH_SPEED_FDB      gimbal_gyro.pitch_Gyro

    #define YAW_MOTOR_POLARITY          -1
    #define PITCH_MOTOR_POLARITY        -1

    float Buff_Yaw_remain =1;//向右为正
    float Buff_pitch_remain=0.8 ;//5.2;

    float auto_aim_Yaw_remain = 0;
    float auto_aim_pitch_remain = 0;
#elif STANDARD == 6

		#define FIGHTER_PITCH_MAX 10.0f
		#define FIGHTER_PITCH_MIN -30.0f
		
    float pitch_middle = 0;
    float Pitch_min = FIGHTER_PITCH_MIN;
    float Pitch_max = FIGHTER_PITCH_MAX;

    #define VISION_PITCH_MIN -30
    #define VISION_PITCH_MAX 10

    #define YAW_INIT_ANGLE_FDB          yaw_Encoder.ecd_angle
    #define PITCH_INIT_ANGLE_FDB        gimbal_gyro.roll_Angle
    #define YAW_INIT_SPEED_FDB          gimbal_gyro.yaw_Gyro
    #define PITCH_INIT_SPEED_FDB        gimbal_gyro.roll_Gyro

    #define YAW_ANGLE_FDB               gimbal_gyro.yaw_Angle
    #define PITCH_ANGLE_FDB             gimbal_gyro.roll_Angle
    #define YAW_SPEED_FDB               gimbal_gyro.yaw_Gyro
    #define PITCH_SPEED_FDB             gimbal_gyro.roll_Gyro

    #define VISION_YAW_ANGLE_FDB        gimbal_gyro.roll_Angle
    #define VISION_PITCH_ANGLE_FDB      gimbal_gyro.pitch_Angle
    #define VISION_YAW_SPEED_FDB        gimbal_gyro.yaw_Gyro
    #define VISION_PITCH_SPEED_FDB      gimbal_gyro.roll_Gyro

    #define YAW_MOTOR_POLARITY          1
    #define PITCH_MOTOR_POLARITY        1

    float auto_aim_Yaw_remain = 0;
    float auto_aim_pitch_remain = 0;
#elif STANDARD == 7
		
		#define SECURITY_PITCH_MAX 35.0f
		#define SECURITY_PITCH_MIN -25.0f

    float pitch_middle = 0;
    float Pitch_min = SECURITY_PITCH_MIN;
    float Pitch_max = SECURITY_PITCH_MAX;

    #define VISION_PITCH_MIN            -25
    #define VISION_PITCH_MAX            30

    #define YAW_INIT_ANGLE_FDB          -yaw_Encoder.ecd_angle
    #define PITCH_INIT_ANGLE_FDB        gimbal_gyro.pitch_Angle
    #define YAW_INIT_SPEED_FDB          gimbal_gyro.yaw_Gyro
    #define PITCH_INIT_SPEED_FDB        Pitch_Encoder.filter_rate;

    #define YAW_ANGLE_FDB               gimbal_gyro.yaw_Angle
    #define PITCH_ANGLE_FDB             gimbal_gyro.pitch_Angle
    #define YAW_SPEED_FDB               gimbal_gyro.yaw_Gyro
    #define PITCH_SPEED_FDB             gimbal_gyro.pitch_Gyro

    #define VISION_YAW_ANGLE_FDB        gimbal_gyro.yaw_Angle
    #define VISION_PITCH_ANGLE_FDB      gimbal_gyro.pitch_Angle
    #define VISION_YAW_SPEED_FDB        gimbal_gyro.yaw_Gyro
    #define VISION_PITCH_SPEED_FDB      gimbal_gyro.pitch_Gyro

    #define YAW_MOTOR_POLARITY          -1
    #define PITCH_MOTOR_POLARITY        -1

    float auto_aim_Yaw_remain = 0;
    float auto_aim_pitch_remain = 0;
#endif




 /**
  ******************************************************************************
																云台结构体初始化
		pid参数设置
		
	 =============================================================================
 **/
void gimbal_parameter_Init(void)
{
		//结构体内存置零
    memset(&gimbal_data, 0, sizeof(gimbal_t));
		

    /*******************************pid_Init*********************************/
#if STANDARD == 1
    // 初始化下的参数
    PID_struct_init(&gimbal_data.pid_init_pit_Angle, POSITION_PID, 1000, 10, 
                    -20,-0.1,-10);		//MF5015+滚珠丝杆
  	PID_struct_init(&gimbal_data.pid_init_pit_speed, POSITION_PID, 800, 100,
                    12 , 0, 50);
    //------------------------------------------------
    PID_struct_init(&gimbal_data.pid_init_yaw_Angle, POSITION_PID, 3000, 20, 
                    13 ,0.15f, 10);				//MF9025
	PID_struct_init(&gimbal_data.pid_init_yaw_speed, POSITION_PID, 2000, 50, 
                    12, 0.5f, 0 );

    //普通模式下的参数
    PID_struct_init(&gimbal_data.pid_pit_Angle, POSITION_PID, 400, 20,  
                    5.0f,0.02f,40);	
	PID_struct_init(&gimbal_data.pid_pit_speed, POSITION_PID, 1000, 50,  
                    2,0.01f,2);
    //------------------------------------------------
    PID_struct_init(&gimbal_data.pid_yaw_Angle, POSITION_PID, 3000, 20, 
                    13 ,0.15f, 10);
	PID_struct_init(&gimbal_data.pid_yaw_speed, POSITION_PID, 2000, 50, 
                    12, 0.5f, 0 );

    //自瞄模式下参数
    PID_struct_init(&gimbal_data.pid_pit_follow, POSITION_PID, 1000, 10,  
                    -20,-0.1,-10);		//MF5015+滚珠丝杆
  	PID_struct_init(&gimbal_data.pid_pit_speed_follow, POSITION_PID, 800, 100,
                    12 , 0, 50);
    //------------------------------------------------
    PID_struct_init(&gimbal_data.pid_yaw_Angle, POSITION_PID, 3000, 20,  
                    13 ,0.15f, 10);
	PID_struct_init(&gimbal_data.pid_yaw_speed, POSITION_PID, 2000, 50,  
                    12, 0.5f, 0 );

    //吊射模式下的参数
    PID_struct_init(&gimbal_data.pid_auto_pit_Angle, POSITION_PID, 200, 100, 
                    1.5f, 0.02f, 2);  
    PID_struct_init(&gimbal_data.pid_auto_pit_speed, POSITION_PID, 500, 50, 
                    2, 0.01f, 2); 
    //------------------------------------------------
    PID_struct_init(&gimbal_data.pid_auto_yaw_Angle, POSITION_PID, 3000, 20, 
                    17, 0.1, 2); 
    PID_struct_init(&gimbal_data.pid_auto_yaw_speed, POSITION_PID, 2000, 50,  
                    19, 0.1, 1.9f);
#elif STANDARD == 3
    // 初始化下的参数
    PID_struct_init(&gimbal_data.pid_init_pit_Angle, POSITION_PID, 500, 4,
                    15, 0.01f, 8); //15, 0.01f, 8
    PID_struct_init(&gimbal_data.pid_init_pit_speed, POSITION_PID, 27000, 20000,
                    150, 0.001, 60); //170, 0.001f, 60
    //------------------------------------------------
    PID_struct_init(&gimbal_data.pid_init_yaw_Angle, POSITION_PID, 500, 4,
                    7, 0.15f, 8); 
    PID_struct_init(&gimbal_data.pid_init_yaw_speed, POSITION_PID, 29000, 10000,
                    150, 0.8f, 40); 

    // 跟随陀螺仪下的参数
    PID_struct_init(&gimbal_data.pid_pit_Angle, POSITION_PID, 500, 30,
                    20, 0.01f, 15); //15, 0.01f, 8
    PID_struct_init(&gimbal_data.pid_pit_speed, POSITION_PID, 27000, 20000,
                    150, 0.001, 60); //170, 0.001f, 60
    //------------------------------------------------
		PID_struct_init(&gimbal_data.pid_yaw_Angle, POSITION_PID, 400, 16,
                    13.8, 0.1f, 6.5);
    PID_struct_init(&gimbal_data.pid_yaw_speed, POSITION_PID, 29000, 10000,
                    400, 0.8f, 0); 

    //自瞄下参数
    PID_struct_init ( &gimbal_data.pid_pit_follow, POSITION_PID, 200, 10, 15
                    , 0.01, 40 );
										
    PID_struct_init ( &gimbal_data.pid_pit_speed_follow, POSITION_PID, 27000, 25000, 150.0f, 0.001f, 0 ); 

    PID_struct_init ( &gimbal_data.pid_yaw_follow, POSITION_PID,  150,16,
                    13.8, 0.1, 6.5);//15 0 80
    PID_struct_init ( &gimbal_data.pid_yaw_speed_follow, POSITION_PID, 29800, 29800,
                    400.0f, 0.8, 0 ); //160 0.8 40

    //小幅下的参数
    PID_struct_init(&gimbal_data.pid_pit_small_buff, POSITION_PID, 70, 20,
                    12.0f, 0.2f, 5); 
    PID_struct_init(&gimbal_data.pid_pit_speed_small_buff, POSITION_PID, 25000, 20000,
                    350.0f, 7.0f, 0); 
    PID_struct_init(&gimbal_data.pid_yaw_small_buff, POSITION_PID, 60, 20,
                    20.0f, 0.2f, 10);
    PID_struct_init(&gimbal_data.pid_yaw_speed_small_buff, POSITION_PID, 25000, 25000,
                    450.0f, 4.0f, 200);

    //大幅下的参数
    PID_struct_init(&gimbal_data.pid_pit_big_buff, POSITION_PID, 200, 10,
                    12.0f, 0.2f, 5); 
    PID_struct_init(&gimbal_data.pid_pit_speed_big_buff, POSITION_PID, 27000, 25000,
                    300.0f, 8.0f, 200); 
    PID_struct_init(&gimbal_data.pid_yaw_big_buff, POSITION_PID, 250, 4,
                    20.0f, 0.2f, 10); 
    PID_struct_init(&gimbal_data.pid_yaw_speed_big_buff, POSITION_PID, 25000, 5000,
                    300.0f, 8.0f, 200);
#elif STANDARD == 4
#elif STANDARD == 5
#elif STANDARD == 6
		// 初始化下的参数
    PID_struct_init(&gimbal_data.pid_init_pit_Angle, POSITION_PID, 2000, 50,
                    200,0.3, 10); //15, 0.01f, 8
    PID_struct_init(&gimbal_data.pid_init_pit_speed, POSITION_PID, 27000, 20000,
                    0,0, 0); //170, 0.001f, 60
    //------------------------------------------------
    PID_struct_init(&gimbal_data.pid_init_yaw_Angle, POSITION_PID, 500, 4,
                    10,0.15f, 8); 
    PID_struct_init(&gimbal_data.pid_init_yaw_speed, POSITION_PID, 29000, 10000,
                    150, 0.8f, 40); 

    // 跟随陀螺仪下的参数
    PID_struct_init(&gimbal_data.pid_pit_Angle, POSITION_PID, 2000, 50,
                    200,0.3, 10); //15, 0.01f, 8
    PID_struct_init(&gimbal_data.pid_pit_speed, POSITION_PID, 27000, 20000,
                    0,0.0, 0); //170, 0.001f, 60
    //------------------------------------------------
    PID_struct_init(&gimbal_data.pid_yaw_Angle, POSITION_PID, 500, 4,
                    10,0.15f, 8); 
    PID_struct_init(&gimbal_data.pid_yaw_speed, POSITION_PID, 29000, 10000,
                    150, 0.8f, 40); 

    //自瞄下参数
    PID_struct_init ( &gimbal_data.pid_pit_follow, POSITION_PID, 2000, 50, 
											200, 0.3, 10 );
    PID_struct_init ( &gimbal_data.pid_pit_speed_follow, POSITION_PID, 27000, 25000, 0.0f, 0.0f, 0 ); 

    PID_struct_init ( &gimbal_data.pid_yaw_follow, POSITION_PID,  150,200,
                    20, 0.09f, 40 );
    PID_struct_init ( &gimbal_data.pid_yaw_speed_follow, POSITION_PID, 29800, 29800,
                    350.0f, 0, 100 ); //I太大时，陀螺开启云台抖动严重
#elif STANDARD == 7
    // 初始化下的参数
    PID_struct_init(&gimbal_data.pid_init_pit_Angle, POSITION_PID, 500, 4,
                    15, 0.01f, 8); //15, 0.01f, 8
    PID_struct_init(&gimbal_data.pid_init_pit_speed, POSITION_PID, 27000, 20000,
                    50, 0.001, 60); //170, 0.001f, 60
    //------------------------------------------------
    PID_struct_init(&gimbal_data.pid_init_yaw_Angle, POSITION_PID, 500, 4,
                    7, 0.15f, 8); 
    PID_struct_init(&gimbal_data.pid_init_yaw_speed, POSITION_PID, 29000, 10000,
                    150, 0.8f, 40); 

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
#endif
    /************************************************************************/
}


 /**
  ******************************************************************************
																云台总控制任务		
	 =============================================================================
 **/
void gimbal_task(void)
{
    switch (gimbal_data.ctrl_mode)
    {
    case GIMBAL_RELAX:		//关控
    {
				//关控模式下，所有输入输出置零，初始化标志位清零
        memset(&gimbal_data.gim_ref_and_fdb, 0, sizeof(gim_ref_and_fdb_t));
			 gimbal_data.if_finish_Init = 0;
    }
        break;
    case GIMBAL_INIT:			//初始化
    {
        gimbal_init_handle();
    }
        break;
    case GIMBAL_FOLLOW_ZGYRO:		//跟随陀螺仪
    {
        gimbal_follow_gyro_handle();
    }
        break;
#if STANDARD == 1
    case GIMBAL_AUTO_ANGLE:		//英雄吊射模式
    {
        gimbal_auto_angle_handle();
    }
        break;
#elif (STANDARD == 3)||(STANDARD == 4)||(STANDARD == 5)
    case GIMBAL_AUTO_SMALL_BUFF:	//小福
    {
        auto_small_buff_handle();
    }
        break;
    case GIMBAL_AUTO_BIG_BUFF:		//大幅
    {
        auto_big_buff_handle();
    }
        break;
#elif STANDARD == 7
    case GIMBAL_AUTO_AIM:					//哨兵自动云台
    {
        security_gimbal_handle();
    }
        break;
#endif
    default:
        break;
    }
    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, pitch_min , pitch_max );		//pitch轴云台限幅
		gimbal_data.last_ctrl_mode = gimbal_data.ctrl_mode;//云台模式更新

}



/****************************big_or_small_buff_var************************************/
float last_pitch_angle;
float last_yaw_angle;
float yaw_angle_ref_aim,pit_angle_ref_aim;
float last_yaw,last_pit;
uint8_t first_flag = 0;
uint8_t ved = 0;
float Delta_Dect_Angle_Yaw,Delta_Dect_Angle_Pit;
/*************************************************************************************/


 /**
  ******************************************************************************		
																云台初始化任务		
	 =============================================================================
 **/
void gimbal_init_handle	( void )
{
    //大幅参数初始化
    first_flag = 0;
		ved = 0;
		//指定初始化给定与反馈
    int init_rotate_num = 0;
    gimbal_data.gim_ref_and_fdb.pit_angle_ref = 0.0f;
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = PITCH_INIT_ANGLE_FDB;
		
    gimbal_data.gim_ref_and_fdb.yaw_angle_ref = 0.0f;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = YAW_INIT_ANGLE_FDB;

    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = YAW_INIT_SPEED_FDB;
    gimbal_data.gim_ref_and_fdb.pit_speed_fdb = PITCH_INIT_SPEED_FDB;
		//计算云台多圈，并补偿多余圈数避免编码器的累计值影响初始化
    init_rotate_num = (gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)/360;
    gimbal_data.gim_ref_and_fdb.yaw_angle_ref = init_rotate_num*360;

   //通过劣弧转到正的角度
    if((gimbal_data.gim_ref_and_fdb.yaw_angle_ref-gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)>=181)
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref-=360;
    else if((gimbal_data.gim_ref_and_fdb.yaw_angle_ref-gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)<-179)
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref+=360;
		//pitch轴与yaw轴双环pid计算
    gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_init_yaw_Angle,
                                                                      &gimbal_data.pid_init_yaw_speed,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
                                                                      0 )*YAW_MOTOR_POLARITY;
    gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_init_pit_Angle,
                                                                      &gimbal_data.pid_init_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.pit_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
                                                                      0 )*PITCH_MOTOR_POLARITY;
	 //自主判断是否完成初始化
	if (fabs(gimbal_data.gim_ref_and_fdb.pit_angle_ref - gimbal_data.gim_ref_and_fdb.pit_angle_fdb)<=4&&fabs(gimbal_data.gim_ref_and_fdb.yaw_angle_ref - gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)<=1.5)
    {
			
        gimbal_data.if_finish_Init = 1;		//初始化标志位置1
                pitch_middle = PITCH_ANGLE_FDB;	//初始化完默认转普通模式，获取普通模式下的pitch反馈中值（步兵为陀螺仪，丝杆英雄为5015编码器）
			//计算pitch轴软件限位
                pitch_max = pitch_middle+Pitch_max;
                pitch_min = pitch_middle+Pitch_min;
        
    }
    																																		
}








 /**
  ******************************************************************************
																云台跟随gyro控制任务		
	 =============================================================================
 **/

void gimbal_follow_gyro_handle(void)
{
		//如果刚刚切换至该模式，该模式的增量式输入以当前传感器反馈赋初始值
		if(gimbal_data.last_ctrl_mode != GIMBAL_FOLLOW_ZGYRO)
		{
			gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref   = YAW_ANGLE_FDB;
			gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref = PITCH_ANGLE_FDB;
		}
		//指定云台反馈
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = PITCH_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = YAW_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.pit_speed_fdb = PITCH_SPEED_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = YAW_SPEED_FDB;
    if(RC_CtrlData.mouse.press_r)//鼠标右键按下
    {

                if (My_Auto_Shoot.Auto_Aim.Flag_Get_Target)//视觉完成识别
                {
										//切换云台反馈
									/**/
										float pitch,yaw;
										pitch = convert_ecd_angle_to__pi_pi(VISION_PITCH_ANGLE_FDB,pitch);
										yaw = convert_ecd_angle_to__pi_pi(VISION_YAW_ANGLE_FDB,yaw);
										gimbal_data.gim_ref_and_fdb.pit_angle_fdb = pitch;
                    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = yaw;
									/**/
									if(fabs(gimbal_data.gim_ref_and_fdb.yaw_angle_ref - gimbal_data.gim_ref_and_fdb.yaw_angle_fdb) < 2)
									{
										gimbal_data.if_auto_shoot = 1;
									}else
									{
										gimbal_data.if_auto_shoot = 0;
									}
									
//                    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = VISION_PITCH_ANGLE_FDB;
//                    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = VISION_YAW_ANGLE_FDB;
                    gimbal_data.gim_ref_and_fdb.pit_speed_fdb = VISION_PITCH_SPEED_FDB;
                    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = VISION_YAW_SPEED_FDB;
										//切换云台输入
                    gimbal_data.gim_ref_and_fdb.pit_angle_ref = My_Auto_Shoot.Auto_Aim.Pitch_Angle + auto_aim_pitch_remain;
                    gimbal_data.gim_ref_and_fdb.yaw_angle_ref = My_Auto_Shoot.Auto_Aim.Yaw_Angle + auto_aim_Yaw_remain;
									
									if(gimbal_data.gim_ref_and_fdb.yaw_angle_ref - gimbal_data.gim_ref_and_fdb.yaw_angle_fdb > 180.0)
									{
										gimbal_data.gim_ref_and_fdb.yaw_angle_ref-=360;
									}else if(gimbal_data.gim_ref_and_fdb.yaw_angle_ref - gimbal_data.gim_ref_and_fdb.yaw_angle_fdb < -180.0)
									{
										gimbal_data.gim_ref_and_fdb.yaw_angle_ref+=360;
									}
										//视觉模式下云台限位
                    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, VISION_PITCH_MIN , VISION_PITCH_MAX );
                }
				//pitch轴与yaw轴双环pid计算
        gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_follow,
                                                                      &gimbal_data.pid_yaw_speed_follow,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
                                                                      0)*YAW_MOTOR_POLARITY;
        gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_follow,
                                                                      &gimbal_data.pid_pit_speed_follow,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.pit_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
                                                                      0 )*PITCH_MOTOR_POLARITY;
    }else
    {
			//普通模式下云台输入
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref;
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref;
			//pitch轴与yaw轴双环pid计算
    gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_Angle,
                                                                      &gimbal_data.pid_yaw_speed,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
                                                                      0 )*YAW_MOTOR_POLARITY;
    gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_Angle,
                                                                      &gimbal_data.pid_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.pit_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
                                                                      0 )*PITCH_MOTOR_POLARITY;
    }
}








#if (STANDARD == 3)||(STANDARD == 4)||(STANDARD == 5)
 /**
  ******************************************************************************
																small_buff控制任务		
	 =============================================================================
 **/
void auto_small_buff_handle(void)
{
     if(first_flag == 0)
	{
		last_pitch_angle=VISION_PITCH_ANGLE_FDB;
		last_yaw_angle=VISION_YAW_ANGLE_FDB;
		first_flag = 1;
	}
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = VISION_PITCH_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = VISION_YAW_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.pit_speed_fdb = VISION_PITCH_SPEED_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = VISION_YAW_SPEED_FDB;
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
				
			yaw_angle_ref_aim=Delta_Dect_Angle_Yaw + new_location.x + Buff_Yaw_remain;
			pit_angle_ref_aim=Delta_Dect_Angle_Pit + new_location.y + Buff_pitch_remain;
        }
        last_yaw=new_location.x1;
		last_pit=new_location.y1;

        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = yaw_angle_ref_aim;
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = raw_data_to_pitch_angle(pit_angle_ref_aim)+Buff_pitch_remain;;
    }
    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, VISION_PITCH_MIN , VISION_PITCH_MAX );
    if(ved==0)
    {
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = last_yaw_angle;
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = last_pitch_angle;
    }
    
    gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_small_buff,
                                                                      &gimbal_data.pid_yaw_speed_small_buff,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
                                                                      0 )*YAW_MOTOR_POLARITY;
    gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_small_buff,
                                                                      &gimbal_data.pid_pit_speed_small_buff,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.pit_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
                                                                      0 )*PITCH_MOTOR_POLARITY;
}







 /**
  ******************************************************************************
																big_buff控制任务		
	 =============================================================================
 **/
void auto_big_buff_handle(void)
{
    if(first_flag == 0)
	{
		last_pitch_angle=VISION_PITCH_ANGLE_FDB;
		last_yaw_angle=VISION_YAW_ANGLE_FDB;
		first_flag = 1;
	}
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = VISION_PITCH_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = VISION_YAW_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.pit_speed_fdb = VISION_PITCH_SPEED_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = VISION_YAW_SPEED_FDB;
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
				
			yaw_angle_ref_aim=Delta_Dect_Angle_Yaw + new_location.x + Buff_Yaw_remain;
			pit_angle_ref_aim=Delta_Dect_Angle_Pit + new_location.y + Buff_pitch_remain;
        }
        last_yaw=new_location.x1;
		last_pit=new_location.y1;

        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = yaw_angle_ref_aim;
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = raw_data_to_pitch_angle(pit_angle_ref_aim)+Buff_pitch_remain;;
    }
    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, VISION_PITCH_MIN , VISION_PITCH_MAX );
    if(ved==0)
    {
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = last_yaw_angle;
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = last_pitch_angle;
    }
    
    gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_big_buff,
                                                                      &gimbal_data.pid_yaw_speed_big_buff,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
                                                                      0 )*YAW_MOTOR_POLARITY;
    gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_big_buff,
                                                                      &gimbal_data.pid_pit_speed_big_buff,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.pit_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
                                                                      0 )*PITCH_MOTOR_POLARITY;
}








 /**
  ******************************************************************************
																能量机关位姿解算	
	 =============================================================================
 **/
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

#endif







#if STANDARD == 7
 /**
  ******************************************************************************
																单头哨兵自动云台控制任务	
	 =============================================================================
 **/
void security_gimbal_handle(void)
{
    static int wait_tick = 0;//识别不到的时候多等两秒，确认看不到了进入巡逻
		//视觉云台反馈
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = VISION_PITCH_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = VISION_YAW_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.pit_speed_fdb = VISION_PITCH_SPEED_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = VISION_YAW_SPEED_FDB;
    if(new_location.control_flag)//哨兵视觉确定识别到目标
    {
				//视觉云台给定与限幅
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = new_location.y;
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = new_location.x;
        VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, VISION_PITCH_MIN , VISION_PITCH_MAX );
				//pitch轴与yaw轴双环pid计算
        gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_follow,
                                                                      &gimbal_data.pid_yaw_speed_follow,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
                                                                      new_location.yaw_speed)*YAW_MOTOR_POLARITY;
        gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_Angle,
                                                                      &gimbal_data.pid_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.pit_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
                                                                      0 )*PITCH_MOTOR_POLARITY;
    }else
    {
        wait_tick++;//开始计等待时间
        if(wait_tick<=200)//若wait_tick<=2s，留在上一次识别位置
        {
					//云台输入与限幅
            gimbal_data.gim_ref_and_fdb.pit_angle_ref = new_location.last_y;
            gimbal_data.gim_ref_and_fdb.yaw_angle_ref = new_location.last_x;
            VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, VISION_PITCH_MIN , VISION_PITCH_MAX );
					//pitch轴与yaw轴双环pid计算
            gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_follow,
                                                                      &gimbal_data.pid_yaw_speed_follow,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
                                                                      0)*YAW_MOTOR_POLARITY;
            gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_Angle,
                                                                      &gimbal_data.pid_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.pit_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
                                                                      0 )*PITCH_MOTOR_POLARITY;
        }else//确认没识别到，开始巡逻
        {
					//低头转圈
            gimbal_data.gim_ref_and_fdb.pit_angle_ref = -2;

            gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_Angle,
                                                                      &gimbal_data.pid_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.pit_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
                                                                      0 )PITCH_MOTOR_POLARITY;
            gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_calc(&gimbal_data.pid_yaw_speed_follow,gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,150)*YAW_MOTOR_POLARITY;

        }
    }
}

#endif






#if STANDARD == 1
 /**
  ******************************************************************************
																英雄吊射控制任务		
	 =============================================================================
 **/
void gimbal_auto_angle_handle(void)
{
		//刚进该吊射模式，给定赋初值
		if(gimbal_data.last_ctrl_mode != GIMBAL_AUTO_ANGLE)
		{
			gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref = YAW_AUTO_ANGLE_FDB;
			gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref = PITCH_AUTO_ANGLE_FDB;
		}
		//反馈更新
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = PITCH_AUTO_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = YAW_AUTO_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.pit_speed_fdb = PITCH_AUTO_SPEED_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = YAW_AUTO_SPEED_FDB;
    if(RC_CtrlData.mouse.press_r)//右键按下，开启视觉辅助吊射
    {
        if(new_location.flag)//视觉完成识别
        {
						//反馈更新
            gimbal_data.gim_ref_and_fdb.pit_angle_fdb = VISION_PITCH_ANGLE_FDB;
            gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = VISION_YAW_ANGLE_FDB;
            gimbal_data.gim_ref_and_fdb.pit_speed_fdb = VISION_PITCH_SPEED_FDB;
            gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = VISION_YAW_SPEED_FDB;
						//给定赋值
            gimbal_data.gim_ref_and_fdb.pit_angle_ref = new_location.y;
            gimbal_data.gim_ref_and_fdb.yaw_angle_ref = new_location.x;
						//软件pitch限幅
            VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, VISION_PITCH_MIN , VISION_PITCH_MAX );

        }
				//双环pid输入
        gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_follow,
                                                                      &gimbal_data.pid_yaw_speed_follow,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
                                                                      &gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
                                                                      new_location.yaw_speed)*YAW_MOTOR_POLARITY;
        gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_Angle,
                                                                      &gimbal_data.pid_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
                                                                      &gimbal_data.gim_ref_and_fdb.pit_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
                                                                      0 )*PITCH_MOTOR_POLARITY;
    }else//开始键盘微调模式
    {
				//给定赋值与双环pid输出
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref;
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref;
    gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_auto_yaw_Angle,
                                                                      &gimbal_data.pid_auto_yaw_speed,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
                                                                      &gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
                                                                      0 )*YAW_MOTOR_POLARITY;
    gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_auto_pit_Angle,
                                                                      &gimbal_data.pid_auto_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
                                                                      &gimbal_data.gim_ref_and_fdb.pit_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
                                                                      0 )*PITCH_MOTOR_POLARITY;
    }
	
}

#endif

