#include "gimbal_task.h"

/**
  ******************************************************************************
  * @file    gimbal_task.c
  * @author  Sun
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   该模块为吊射英雄云台模块，参数设置位于源文件上部分和头文件上部分：
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
 
#define STANDARD 1

gimbal_t gimbal_data;
//云台限位
float pitch_min = 0;		
float pitch_max = 0;		

#if STANDARD == 1

		#define HERO_PITCH_MAX 6486
		#define HERO_PITCH_MIN -2981
		
    float pitch_middle = 0;
    float Pitch_min = HERO_PITCH_MIN;
    float Pitch_max = HERO_PITCH_MAX;
    
    #define VISION_PITCH_MIN            -25  //视觉的传感器视角往往都是世界坐标系
    #define VISION_PITCH_MAX            30

    #define YAW_INIT_ANGLE_FDB          -yaw_Encoder.ecd_angle
    #define PITCH_INIT_ANGLE_FDB        gimbal_gyro.pitch_Angle
    #define YAW_INIT_SPEED_FDB          gimbal_gyro.yaw_Gyro
    #define PITCH_INIT_SPEED_FDB        gimbal_gyro.pitch_Gyro

    #define YAW_ANGLE_FDB               gimbal_gyro.yaw_Angle
    #define PITCH_ANGLE_FDB             Pitch_Encoder.ecd_angle
    #define YAW_SPEED_FDB               gimbal_gyro.yaw_Gyro
    #define PITCH_SPEED_FDB             Pitch_Encoder.filter_rate

    #define VISION_YAW_ANGLE_FDB        gimbal_gyro.yaw_Angle
    #define VISION_PITCH_ANGLE_FDB      gimbal_gyro.pitch_Angle
    #define VISION_YAW_SPEED_FDB        gimbal_gyro.yaw_Gyro
    #define VISION_PITCH_SPEED_FDB      gimbal_gyro.pitch_Gyro

    #define YAW_AUTO_ANGLE_FDB          -yaw_Encoder.ecd_angle
    #define PITCH_AUTO_ANGLE_FDB        Pitch_Encoder.ecd_angle
    #define YAW_AUTO_SPEED_FDB          gimbal_gyro.yaw_Gyro
    #define PITCH_AUTO_SPEED_FDB        gimbal_gyro.pitch_Gyro

    #define YAW_MOTOR_POLARITY          -1
    #define PITCH_MOTOR_POLARITY        1

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
    PID_struct_init(&gimbal_data.pid_init_pit_Angle, POSITION_PID, 300, 10, 
                    -7,-0.05f,-60);		//MF5015+滚珠丝杆
  	PID_struct_init(&gimbal_data.pid_init_pit_speed, POSITION_PID, 800, 100,
                    7 , 0, 60);
    //------------------------------------------------
    PID_struct_init(&gimbal_data.pid_init_yaw_Angle, POSITION_PID, 3000, 20, 
                    18 ,0.15f, 10);				//MF9025
	PID_struct_init(&gimbal_data.pid_init_yaw_speed, POSITION_PID, 2000, 50, 
                    12, 0.5f, 20 );

    //普通模式下的参数
    PID_struct_init(&gimbal_data.pid_pit_Angle, POSITION_PID, 10000, 3, 
                    50,0.001f,0.1f);	
	PID_struct_init(&gimbal_data.pid_pit_speed, POSITION_PID, 10000, 10,  
                    0.6f,0.001f,0.4f);
	PID_struct_init(&gimbal_data.pid_yaw_Angle, POSITION_PID, 257, 20,  
                    13 ,0.01f, 3);
	PID_struct_init(&gimbal_data.pid_yaw_speed, POSITION_PID, 3800, 10,  
                    20, 0.01f, 5 );
					
    //------------------------------------------------

    //自瞄模式下参数
    PID_struct_init(&gimbal_data.pid_pit_follow, POSITION_PID, 1000, 10,  
                    -20,-0.1,-10);		//MF5015+滚珠丝杆
  	PID_struct_init(&gimbal_data.pid_pit_speed_follow, POSITION_PID, 800, 100,
                    12 , 0, 50);
    //------------------------------------------------

    //吊射模式下的参数
    PID_struct_init(&gimbal_data.pid_auto_pit_Angle, POSITION_PID, 200, 100, 
                    1.5f, 0.02f, 1);  
    PID_struct_init(&gimbal_data.pid_auto_pit_speed, POSITION_PID, 500, 50, 
                    1.5f, 0.01f, 1); 
    //------------------------------------------------
    PID_struct_init(&gimbal_data.pid_auto_yaw_Angle, POSITION_PID, 3000, 30, 
                    35, 0.2f, 2); 
    PID_struct_init(&gimbal_data.pid_auto_yaw_speed, POSITION_PID, 2000, 50,  
                    25, 0.1f, 1.9f);

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
    case GIMBAL_AUTO_ANGLE:		//英雄吊射模式
    {
        gimbal_auto_angle_handle();
    }
        break;

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
//    if(RC_CtrlData.mouse.press_r)//鼠标右键按下
//    {

//                if (new_location.flag)//视觉完成识别
//                {
//										//切换云台反馈
//                    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = VISION_PITCH_ANGLE_FDB;
//                    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = VISION_YAW_ANGLE_FDB;
//                    gimbal_data.gim_ref_and_fdb.pit_speed_fdb = VISION_PITCH_SPEED_FDB;
//                    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = VISION_YAW_SPEED_FDB;
//										//切换云台输入
//                    gimbal_data.gim_ref_and_fdb.pit_angle_ref = new_location.y;
//                    gimbal_data.gim_ref_and_fdb.yaw_angle_ref = new_location.x;
//										//视觉模式下云台限位
//                    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, VISION_PITCH_MIN , VISION_PITCH_MAX );
//                }
//				//pitch轴与yaw轴双环pid计算
//        gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_follow,
//                                                                      &gimbal_data.pid_yaw_speed_follow,
//                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
//                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
//																																			&gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
//                                                                      gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
//                                                                      new_location.yaw_speed)*YAW_MOTOR_POLARITY;
//        gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_Angle,
//                                                                      &gimbal_data.pid_pit_speed,
//                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
//                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
//																																			&gimbal_data.gim_ref_and_fdb.pit_speed_ref,
//                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
//                                                                      0 )*PITCH_MOTOR_POLARITY;
//    }else
//    {
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
//    }
}















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


