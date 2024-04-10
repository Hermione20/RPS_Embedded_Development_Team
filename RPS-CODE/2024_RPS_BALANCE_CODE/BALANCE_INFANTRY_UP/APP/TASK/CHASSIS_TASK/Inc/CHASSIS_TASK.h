#ifndef __CHASSIS_TASK_H 
#define __CHASSIS_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "public.h"
/*----------------------------------------------------------------------------*/
//底盘类型 1舵轮 2麦轮 3全向轮 4新舵轮
#define CHASSIS_TYPE  4
#define POWER_LIMIT_HANDLE   1//0不开 1为舵轮 2为英雄(麦轮)以及全向轮

/*******************************CONFIG********************************/
#define STANDARD              3  //参数选择  1英雄 2工程(None) 3456步兵 7烧饼
#define YAW_POLARITY 					-1 //逆正      舵轮要顺正，改-1；麦轮1




#if     CHASSIS_TYPE == 1 //舵轮
#define RIGHT_FRONT_REVERSE   -1 
#define LEFT_FRONT_REVERSE    -1
#define LEFT_BEHIND_REVERSE    1
#define RIGHT_BEHIND_REVERSE   1
#define  WARNING_VOLTAGE       12.5
#define STEERING_POLARITY      1 //6020电机的输出极性 解算已考虑 故置1

#elif		CHASSIS_TYPE == 2//麦轮
#define MAX_WHEEL_RPM 				 7400
#define  WARNING_VOLTAGE       13


#elif   CHASSIS_TYPE == 3//全向轮
#define MAX_WHEEL_RPM 				 7400
#define  WARNING_VOLTAGE       12.5

#elif   CHASSIS_TYPE == 4//新舵轮
#define RIGHT_FRONT_REVERSE   -1 
#define LEFT_FRONT_REVERSE    1
#define LEFT_BEHIND_REVERSE   1
#define RIGHT_BEHIND_REVERSE  1
#define  WARNING_VOLTAGE       12.5
#define STEERING_POLARITY      -1 //6020电机的输出极性 解算不考虑 故置-1
#endif
/*******************************CONFIG********************************/




#define  I_TIMES_V_TO_WATT    0.0000225f    //I -16384~+16384 V .filter_rate
//电机发热计算 p=i^2*FACTOR_2+i*FACTOR_1+FACTOR0; i是直接发给电调的数-16384~16384 使用虚拟示波器读值后matlab拟合
#define FACTOR_2	0.000000161f
#define FACTOR_1	-0.0000229f
#define FACTOR_0  0.458f
//#define TOTATE_PARA    PI/180.0f



#define  TARGET_VOLTAGE        12

//定义 NULL
#ifndef NULL
#define NULL 0
#endif

//定义PI 值
#ifndef PI
#define PI 3.14159265358979f
#endif

//定义 角度(度)转换到 弧度的比例
#ifndef ANGLE_TO_RAD
#define ANGLE_TO_RAD 0.01745329251994329576923690768489f
#endif

//定义 弧度 转换到 角度的比例
#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

#define Ref_Fdb(ref,fdb)\
{\
 if((fdb - ref) >= 180)\
{\
	fdb = fdb - 360;\
}\
 else if((fdb - ref) <= -180)\
{\
	fdb = fdb + 360;\
}\
}\
/**
************************************************************************************************************************
* @EnumName : chassis_mode_e
* @brief    : This enumeration describes the various control modes of the chassis
* @param    : CHASSIS_RELAX     
*	@param		:	CHASSIS_STOP		
*	@param		:	MANUAL_SEPARATE_GIMBAL			
* @param		: MANUAL_FOLLOW_GIMBAL 		
* @param		: DODGE_MODE 		
* @param		: AUTO_SEPARATE_GIMBAL 	
* @param		: AUTO_FOLLOW_GIMBAL 	
* @param		: CHASSIS_ROTATE 		
* @param		: CHASSIS_REVERSE 	
* @param		: CHASSIS_CHANGE_REVERSE 		
* @param		: CHASSIS_SEPARATE 	
* @param		: CHASSIS_AUTO_SUP 		
* @Note     : 					
************************************************************************************************************************
**/


typedef enum
{
  CHASSIS_RELAX          = 0,
  CHASSIS_STOP           = 1,
  MANUAL_SEPARATE_GIMBAL = 2,
  MANUAL_FOLLOW_GIMBAL   = 3,
  DODGE_MODE             = 4,
  AUTO_SEPARATE_GIMBAL   = 5,
  AUTO_FOLLOW_GIMBAL     = 6,
  CHASSIS_ROTATE         = 7,
  CHASSIS_REVERSE        = 8,
  CHASSIS_CHANGE_REVERSE = 9,
  CHASSIS_SEPARATE 		 	 = 10,
  CHASSIS_AUTO_SUP       = 11,
} chassis_mode_e;


/**
************************************************************************************************************************
* @StructName : cha_pid_t
* @brief    	: This enumeration describes the various control modes of the chassis
* @param    	: anglex_ref    		
*	@param			:	anglex_fdb			
* @param			: speedx_ref 		
* @param			: speedx_fdb 		 		
* @Note    		: 					
************************************************************************************************************************
**/
typedef struct
{
  /* position loop */

	float angle_ref[4];
	float angle_fdb[4];
  /* speed loop */

	int16_t speed_ref[4];
	int16_t speed_fdb[4];
	
} cha_pid_t;


typedef enum
{
  NORMAL_SPEED_MODE          = 2,
	HIGH_SPEED_MODE            = 3,
	LOW_SPEED_MODE             = 1,
} chassis_speed_mode_e;


typedef __packed struct
{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;
/**
************************************************************************************************************************
* @StructName : cha_pid_t
* @brief    	: This enumeration describes the various control modes of the chassis
* @param    	: anglex_ref    		
*	@param			:	anglex_fdb			
* @param			: speedx_ref 		
* @param			: speedx_fdb 		 		
* @Note    		: 					
************************************************************************************************************************
**/
typedef struct
{
		float           vx; // forward/back
		float           vy; // left/right
		float           vw; // 
		
		chassis_mode_e  			ctrl_mode;
		chassis_mode_e  			last_ctrl_mode;
		chassis_speed_mode_e  chassis_speed_mode;

		ChassisSpeed_Ref_t  ChassisSpeed_Ref;
	
		float           gyro_angle;
		float           gyro_palstance;

		int16_t         current[4];
		int16_t					voltage[4];
		
		float						sin_chassis_angle;
		float						cos_chassis_angle;
		
		int16_t					foward_back_to_foward_back_rotate_speed;
		int16_t					foward_back_to_left_right_rotate_speed;
		int16_t					left_right_to_foward_back_rotate_speed;
		int16_t					left_right_to_left_right_rotate_speed;
		
		int32_t         position_ref;
		uint8_t         follow_gimbal;
		
		cha_pid_t      cha_pid_6020;
		cha_pid_t			 cha_pid_3508;
} chassis_t;

	
/**
************************************************************************************************************************
* @StructName : cha_pid_t
* @brief    	: This enumeration describes the various control modes of the chassis
* @param    	: anglex_ref    		
*	@param			:	anglex_fdb			
* @param			: speedx_ref 		
* @param			: speedx_fdb 		 		
* @Note    		: 					
************************************************************************************************************************
**/
typedef struct
{
		float start_angle[4];
		float include_angle[4];
		float Remote_angle;
		float Remote_speed;
		float deviation_angle[4];
		int16_t handle_speed[4];
		int16_t handle_speed_lim[4];
		float get_speedw;
		float yaw_angle_0_2pi;
		float yaw_angle__pi_pi;
		double yaw_encoder_ecd_angle;
}Chassis_angle_t;



void Motion_resolution(void);
void Chassis_PID_handle(void);
void mecanum_calc(float vx, float vy, float vw, int16_t *speed);
void chassis_param_init(void);
void chassis_task(void);
float limit_angle_to_0_2pi(float angle);
void chassis_stop_handle(void);
void get_remote_set(void);
void start_angle_handle(void);
void start_chassis_6020(void);
float get_6020power(void);
void power_limit_handle(void);
void set_3508current_6020voltage(void);
float get_max_power2(float voltage);
float get_max_power1(float voltage);
static float get_the_limite_rate(float max_power);
void cap_limit_mode_switch(void);
void chassis_mode_select(void);
void chassis_stop_handle(void);
void follow_gimbal_handle(void);
void separate_gimbal_handle(void);
void rotate_follow_gimbal_handle(void);
void reverse_follow_gimbal_handle(void);
void buffer_power(void);
float get_max_power(float voltage);
void steering_wheel_calc(double Length,double Weight); 
void omni_calc1(float vx,float vy,float vw,int16_t *speed);
void omni_calc2(float vx,float vy,float vw,int16_t *speed);



extern Chassis_angle_t 	 Chassis_angle;
extern chassis_t 		 		 chassis;
extern u16 Max_Power;



#endif



