#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__
#include "main.h"
/* chassis control period time (ms) */
#define CHASSIS_PERIOD 10

#define MAX_WHEEL_RPM  7400
#define MAX_CHASSIS_VR_SPEED 200
#define SOFTWARE_LIMIT 0
#define CAP_LIMIT 1
#define chassis_rotate_speed_vw_ref 65
///////////////////////////////////////
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

/////////////////////////////////

///////////////////////////////////////

#define ADJUST_LEVEL   1
///////////////////////////////////////
#define HIGH_FORWARD_BACK_SPEED 			1600
#define HIGH_LEFT_RIGHT_SPEED   			800
#define MIDDLE_FORWARD_BACK_SPEED     900
#define MIDDLE_LEFT_RIGHT_SPEED       600
#define NORMAL_FORWARD_BACK_SPEED 			550
#define NORMAL_LEFT_RIGHT_SPEED   			550
#define  I_TIMES_V_TO_WATT    0.0000231f    //I -16384~+16384 V .filter_rate
//电机发热计算 p=i^2*FACTOR_2+i*FACTOR_1+FACTOR0; i是直接发给电调的数-16384~16384 使用虚拟示波器读值后matlab拟合
#define FACTOR_2	0.000000161f
#define FACTOR_1	-0.0000229f
#define FACTOR_0  0.458f
//#define TOTATE_PARA    PI/180.0f

#define CHASSIS_EXTRA_POWER_BUFFER 10

typedef enum
{
  Power_first       = 0,
  HP_first          = 1,
} chassis_mode_selection_e;

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
  CHASSIS_SEPARATE 		 = 10,
  CHASSIS_AUTO_SUP       = 11,
} chassis_mode_e;


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
  /* speed loop */
  float speed1_ref;
  float speed2_ref;
  float speed1_fdb;
  float speed2_fdb;

  float speed3_ref;
  float speed4_ref;
  float speed3_fdb;
  float speed4_fdb;
} cha_pid_t;






typedef struct
{
  double           vx; // forward/back
  double           vy; // left/right
  double           vw; // 
  
  chassis_mode_e  ctrl_mode;
  chassis_mode_e  last_ctrl_mode;

  float           gyro_angle;
  float           gyro_palstance;

  int16_t         wheel_speed_fdb[4];
  int16_t         wheel_speed_ref[4];
  int16_t         current[4];
  
	float						sin_chassis_angle;
	float						cos_chassis_angle;
	
	int16_t					foward_back_to_foward_back_rotate_speed;
	int16_t					foward_back_to_left_right_rotate_speed;
	int16_t					left_right_to_foward_back_rotate_speed;
	int16_t					left_right_to_left_right_rotate_speed;
	
  int32_t         position_ref;
  uint8_t         follow_gimbal;
	
	cha_pid_t      cha_pid;
} chassis_t;

typedef struct
{
  int16_t power_limit_model ;
} power_limit_t;

typedef struct
{
  float voltage_ref;
  float voltage_fdb;
	
}voltage_pid_t;

typedef enum
{
  NORMAL_SPEED_MODE          = 0,
	HIGH_SPEED_MODE            = 1,
	LOW_SPEED_MODE             = 2,
} chassis_speed_mode_e;

typedef struct
{
  float software_limit_ref;
  float software_limit_fdb;
	
}pid_software_limit_t;

typedef struct
{
  double foward_back_to_foward_back_rotate_speed;
  double foward_back_to_left_right_rotate_speed;
  double left_right_to_left_right_rotate_speed;
  double left_right_to_foward_back_rotate_speed; 
  double sin_chassis_angle;
  double cos_chassis_angle;
	
}chassis_rotate_speed_t;
float get_max_power(float voltage);
void chassis_task(void);
void chassis_param_init(void);
void power_limit_handle(void);
void power_send_handle(void);
void Fault_judge(void);
void start_angle_handle(void);
int  Check_Vcap_recieve(void);
void START_CHASSIS_6020(void);
void power_other_limit(void);
float power_predict();
void buffer_power(void);
static void chassis_twist_handle(void);
static void chassis_stop_handle(void);
static void open_loop_handle(void);
static void follow_gimbal_handle(void);
static void rotate_follow_gimbal_handle(void);
static void reverse_follow_gimbal_handle(void);
static void mecanum_calc(int16_t *speed);
static void chassis_change_reverse_handle(void);
static void separate_gimbal_handle(void);
static void auto_sup_handle(void);
extern chassis_t chassis;
extern u16  Max_Power;
extern u8  Max_Current;
extern u8 Power_Work_Mode;
extern Key_Flag_t Key_Flag;
extern RampGen_t FBSpeedRamp;
extern u32 can_receive_cnt;
extern u32 can_last_receive_cnt;
extern int32_t rotate_num;
extern int rotate_num_ture;

extern volatile capacitance_message1_t capacitance_message1;
extern volatile capacitance_message2_t capacitance_message2;
extern volatile distance_message_t distance_message;
extern chassis_mode_selection_e chassis_mode_selection;
#endif
