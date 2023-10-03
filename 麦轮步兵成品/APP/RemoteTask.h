#ifndef _REOMTE_TASK_H_
#define _REOMTE_TASK_H_
#include "main.h"
#include "ramp_second.h"
/*
****************************************************************************
*
*																	MARCO
****************************************************************************
*/
//remote control parameters
#define INFANTRY 1				//步兵编号
#define REMOTE_CONTROLLER_STICK_OFFSET      1024u   
#define STICK_TO_CHASSIS_SPEED_REF_FACT     0.6f
#define STICK_TO_PITCH_ANGLE_INC_FACT       0.004f
#define STICK_TO_YAW_ANGLE_INC_FACT         0.004f//0.005f
#if INFANTRY == 1
	#define FRICTION_WHEEL_MAX_DUTY             1350//
	#define FRICTION_WHEEL_LOWER_DUTY						1350
#elif INFANTRY == 4
	#define FRICTION_WHEEL_MAX_DUTY             1350//
	#define FRICTION_WHEEL_LOWER_DUTY						1290
#elif INFANTRY == 5
	#define FRICTION_WHEEL_MAX_DUTY             1400//
	#define FRICTION_WHEEL_LOWER_DUTY						1300
#endif
#define MOUSE_TO_PITCH_ANGLE_INC_FACT 		0.06f
#define MOUSE_TO_YAW_ANGLE_INC_FACT 		0.07f//3.16 修改提高 0.03


#define CHASSIS_RAMP_TICK_COUNT    80

#define FRICTION_RAMP_TICK_COUNT			150
#define FRICTION_RAMP_OFF_TICK_COUNT	500
//#define MOUSE_LR_RAMP_TICK_COUNT			30
#define MOUSR_FB_RAMP_TICK_COUNT		    80//斜坡斜率调整
#define REMOTE_SWITCH_VALUE_UP         		0x01u  
#define REMOTE_SWITCH_VALUE_DOWN			0x02u
#define REMOTE_SWITCH_VALUE_CENTRAL			0x03u
#define REMOTE_SWITCH_CHANGE_1TO3      (uint8_t)((REMOTE_SWITCH_VALUE_UP << 2) | REMOTE_SWITCH_VALUE_CENTRAL)   
#define REMOTE_SWITCH_CHANGE_2TO3      (uint8_t)((REMOTE_SWITCH_VALUE_DOWN << 2) | REMOTE_SWITCH_VALUE_CENTRAL)  
#define REMOTE_SWITCH_CHANGE_3TO1      (uint8_t)((REMOTE_SWITCH_VALUE_CENTRAL << 2) | REMOTE_SWITCH_VALUE_UP)
#define REMOTE_SWITCH_CHANGE_3TO2      (uint8_t)((REMOTE_SWITCH_VALUE_CENTRAL << 2) | REMOTE_SWITCH_VALUE_DOWN)
#define REMOTE_SWITCH_CHANGE_1TO3TO2   (uint8_t)((REMOTE_SWITCH_VALUE_UP << 4) |\
                                                 (REMOTE_SWITCH_VALUE_CENTRAL << 2) |\
                                                 (REMOTE_SWITCH_VALUE_DOWN))   

#define REMOTE_SWITCH_CHANGE_2TO3TO1   (uint8_t)((REMOTE_SWITCH_VALUE_DOWN << 4) |\
                                                 (REMOTE_SWITCH_VALUE_CENTRAL << 2) |\
                                                 (REMOTE_SWITCH_VALUE_UP)) 

#define REMOTE_SWITCH_VALUE_BUF_DEEP   16u
#define  KEY_W  0X0001    //前后左右
#define  KEY_S  0X0002
#define  KEY_A  0X0004
#define  KEY_D  0X0008
#define  KEY_SHIFT  0X0010//切换高速模式
#define  KEY_CTRL  0X0020 //小陀螺
#define  KEY_Q  0X0040    //热量
#define  KEY_E  0X0080   //
#define  KEY_R 0X0100    //开弹仓
#define  KEY_F 0X0200      //
#define  KEY_G  0X0400    //关弹仓
#define  KEY_Z  0X0800    //大符
#define  KEY_X  0X1000    //长按软件复位
#define  KEY_C  0X2000   //摩擦轮
#define  KEY_V  0X4000   //打小符   （未使用）
#define  KEY_B  0X8000   //UI刷新

#if NEW_CAP == 1

#define  WARNING_VOLTAGE       12
#define  TARGET_VOLTAGE        12
#elif NEW_CAP == 0

#define  WARNING_VOLTAGE       18
#define  TARGET_VOLTAGE        16

#endif

typedef struct{
	uint8_t rc_bytes[RC_FRAME_LENGTH];
}RC_Raw_t;

typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
}Remote;
typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;	
typedef	__packed struct
{
	uint16_t v;
	uint16_t last_v;
}Key;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctl_t;

typedef enum
{
	NOSHOOTING = 0,
	SHOOTING = 1,
}Shoot_State_e;
typedef enum
{
	NORMAL_SHOOTING = 0,//正常射击模式，鼠标按住一直转
	ONE_SHOOTING = 1,//鼠标单击一次只发一颗弹丸
	THREE_SHOOTING = 2,//单击一次三连发
}Shooting_State_e;//弹丸射击模式
#define MOUSE_SCAN_TIME 10 //
#define MOUSE_DOWN_TIME 10//
#define MOUSE_HOLD_TIME (long unsigned int)4000//
typedef enum 
{      
	MOUSE_DOWN,
    MOUSE_UP

} mouse_states_e;



typedef enum  
{
    MOUSE_S1,
    MOUSE_S2,
    MOUSE_S3,
	MOUSE_S4
}mouse_msg_e;

#define KEYBOARD_SCAN_TIME 10
#define KEYBOARD_DOWN_TIME 10
#define KEYBOARD_HOLD_TIME (long unsigned int)4000

typedef enum 
{      
	KEYBOARD_DOWN,
    KEYBOARD_UP

} keyboard_states_e;



typedef enum  
{
    KEYBOARD_S1,
    KEYBOARD_S2,
    KEYBOARD_S3,
	  KEYBOARD_S4
}keyboard_msg_e;


//输入模式:遥控器/键盘鼠标/停止运行
typedef enum
{
	REMOTE_INPUT = 1,
	KEY_MOUSE_INPUT = 3,
	STOP = 2,
}InputMode_e;
//摩擦轮状态枚举
typedef enum
{
	FRICTION_WHEEL_OFF = 0,//摩擦轮关闭
	FRICTION_WHEEL_START_TURNNING = 1,
	FRICTION_WHEEL_ON = 2,
	FRICTION_WHEEL_STOP_TURNNING = 3,
}FrictionWheelState_e;
//拨杆动作枚举
typedef enum
{
	FROM1TO2,
	FROM1TO3,
	FROM2TO1, 
	FROM3TO1,
	FROM3TO2,
}RC_SWITCH_ACTION_e;

//remote data process
typedef __packed struct
{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;

//remote data process
typedef struct
{
    float pitch_angle_dynamic_ref;
    float yaw_angle_dynamic_ref;
    float pitch_angle_static_ref;
    float yaw_angle_static_ref;
    float pitch_speed_ref;
    float yaw_speed_ref;
}Gimbal_Ref_t;

//to detect the action of the switch
typedef struct RemoteSwitch_t
{
	 uint8_t switch_value_raw;            // the current switch value
	 uint8_t switch_value1;				  //  last value << 2 | value
	 uint8_t switch_value2;				  //
	 uint8_t switch_long_value; 		  //keep still if no switching
	 uint8_t switch_value_buf[REMOTE_SWITCH_VALUE_BUF_DEEP]; 
	 uint8_t buf_index;
	 uint8_t buf_last_index;
	 uint8_t buf_end_index;
}RemoteSwitch_t;



typedef struct
{
  u8 Key_W_Flag ;
	u8 Key_A_Flag ;
	u8 Key_S_Flag ;
	u8 Key_D_Flag ;
	u8 Last_Key_W_Flag ;
	u8 Last_Key_A_Flag ;
	u8 Last_Key_S_Flag ;
	u8 Last_Key_D_Flag ;
	u8 Key_A_D_Flag;
	u8 Key_W_S_Flag;
}Key_Flag_t;

typedef struct
{
  u32 RampCount;
	float Rampdata;
}Ramp_t;


typedef enum
{
	NORMAL_LIMIT_MODE = 0,
	NO_LIMIT_MODE     = 1,
}Shoot_Limit_Mode_e;




extern RC_Ctl_t RC_CtrlData; 
extern RampGen_t frictionRamp ;  //摩擦轮斜坡

extern ramp_second_t FBSpeedRampSecond;

extern ChassisSpeed_Ref_t ChassisSpeedRef;
extern Gimbal_Ref_t GimbalRef;
InputMode_e GetInputMode(void);
void RemoteTaskInit(void);
void SetShootState(Shoot_State_e v);
Shoot_State_e GetShootState(void);
Shooting_State_e GetShootingState(void);
void SetShootingState(Shooting_State_e v);
void SetFrictionState(FrictionWheelState_e v);
FrictionWheelState_e GetFrictionState(void);
void RemoteDataPrcess(uint8_t *pData);
void SoftReset(void);
//static void get_key_last_mode(void);
uint8_t IsRemoteBeingAction(void);
void GimbalAngleLimit(void);
void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val);
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val);
void RemoteControlProcess(Remote *rc);
void MouseShootControl(Mouse *mouse,Key *key);
void MouseKeyControlProcess(Mouse *mouse, Key *key);
void Mouse_Key_Rotate_Reverse_Control(void);
void Remote_Rotate_Reverse_Control(RemoteSwitch_t *sw, uint8_t val);
extern u8 frictionSpeedmode;
extern u32 v;
extern float V_Cap ;
Shooting_State_e GetShootingState(void);
void SetShootingState(Shooting_State_e v);
mouse_msg_e mouse_scan(void);
mouse_states_e mouse_read(void);
extern u8 buff_flag;
extern int16_t FRICTION_SPEED_REF ;

extern uint16_t forward_back_speed;
extern uint16_t left_right_speed;

extern u32 Gimbal_pitch_mid_point;
extern u8 chassis_rotate_flag;
extern u8 bulletspead_level;
extern float auto_shoot_yaw_angle;
extern float yaw_angle_offset;
extern float pit_angle_offset;
extern u32 reverse_time_count;
extern u8  big_buff;
extern u8  small_buff;

extern Shoot_Buff_Dir_e Shoot_Buff_Dir;
extern RampGen_t chassisRamp ; 


extern u8 Cap_Reset_flag;

extern u8 key_F_push_flag;
keyboard_states_e keyboard_read(Key *key,u32 ked_id);
keyboard_msg_e keyboard_scan(Key *key,u32 ked_id);
#endif

