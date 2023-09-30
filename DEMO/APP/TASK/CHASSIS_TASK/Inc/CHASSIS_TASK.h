#ifndef __CHASSIS_TASK_H 
#define __CHASSIS_TASK_H
/* Includes ------------------------------------------------------------------*/
#include "public.h"
/*----------------------------------------------------------------------------*/

#define NORMAL_FORWARD_BACK_SPEED								 550
#define NORMAL_LEFT_RIGHT_SPEED  								 550
#define HIGH_FORWARD_BACK_SPEED 								 1600
#define HIGH_LEFT_RIGHT_SPEED   								 800
#define CHASSIS_ROTATE_MOVING_FORWARD_BACK_SPEED 900
#define CHASSIS_ROTATE_MOVING_LEFT_RIGHT_SPEED   600

#define  I_TIMES_V_TO_WATT    0.0000225f    //I -16384~+16384 V .filter_rate
//电机发热计算 p=i^2*FACTOR_2+i*FACTOR_1+FACTOR0; i是直接发给电调的数-16384~16384 使用虚拟示波器读值后matlab拟合
#define FACTOR_2	0.000000161f
#define FACTOR_1	-0.0000229f
#define FACTOR_0  0.458f
//#define TOTATE_PARA    PI/180.0f


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
	ref = ref + 360;\
}\
 else if((fdb - ref) <= -180)\
{\
	ref = ref - 360;\
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


typedef enum
{
	READY  =0,
	STANDBY=1,
}chassis_gim_e ;
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
  float angle1_ref;
  float angle2_ref;
  float angle1_fdb;
  float angle2_fdb;
	
	float angle3_ref;
  float angle4_ref;
  float angle3_fdb;
  float angle4_fdb;
	
	float angle_ref[4];
	float angle_fdb[4];
  /* speed loop */
  float speed1_ref;
  float speed2_ref;
  float speed1_fdb;
  float speed2_fdb;

  float speed3_ref;
  float speed4_ref;
  float speed3_fdb;
  float speed4_fdb;
	
	float speed_ref[4];
	float speed_fdb[4];
} cha_pid_t;


typedef enum
{
  NORMAL_SPEED_MODE          = 0,
	HIGH_SPEED_MODE            = 1,
	LOW_SPEED_MODE             = 2,
} chassis_speed_mode_e;

//remote data process
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
		double           vx; // forward/back
		double           vy; // left/right
		double           vw; // 
		
		chassis_mode_e  			ctrl_mode;
		chassis_mode_e  			last_ctrl_mode;
		chassis_gim_e					chassis_gim;
		chassis_speed_mode_e  chassis_speed_mode;

		ChassisSpeed_Ref_t ChassisSpeed_Ref;
	
		float           gyro_angle;
		float           gyro_palstance;

		int16_t         wheel_speed_fdb[4];
		int16_t         wheel_speed_ref[4];
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
		float get_speedw;
		float yaw_angle_0_2pi;
		float yaw_angle__pi_pi;
}Chassis_angle_t;






void limit_angle_to_0_2pi(float angle);
//void get_chassis_speed_ref(Remote *rc);
void chassis_stop_handle(void);
void get_remote_set(void);
void start_angle_handle(void);
void start_chassis_6020(void);
float get_6020power(void);
void get_chassis_ctrl_mode(void);
void power_limit_handle(void);
void set_3508current_6020voltage(void);
static float get_the_limite_rate(float max_power);
double convert_ecd_angle_to_0_2pi(double ecd_angle,float _0_2pi_angle);







#endif



