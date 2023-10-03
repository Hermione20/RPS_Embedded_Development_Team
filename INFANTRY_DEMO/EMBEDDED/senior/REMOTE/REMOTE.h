#ifndef __REMOTE_H
#define __REMOTE_H
#include "public.h"

#define  RC_FRAME_LENGTH                            18u

#define REMOTE_CONTROLLER_STICK_OFFSET      1024u   

#define STICK_TO_CHASSIS_SPEED_REF_FACT     1.5f

#define STICK_TO_PITCH_ANGLE_INC_FACT       0.008
#define STICK_TO_YAW_ANGLE_INC_FACT         0.008f

#define MOUSE_TO_PITCH_ANGLE_INC_FACT 		0.04f
#define MOUSE_TO_YAW_ANGLE_INC_FACT 		0.06f

/*************************************键鼠**********************************************/
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

/***********************************遥控器*********************************************/

typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
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

typedef enum
{
  KEY_R_UP=0,
  KEY_R_DOWN=1,
 
} key_state_t;

typedef enum
{
	REMOTE_INPUT = 1,
	KEY_MOUSE_INPUT = 3,
	STOP = 2,
}InputMode_e;

//to detect the action of the switch
typedef __packed struct 
{
	 uint8_t switch_value_raw;            // the current switch value
	uint8_t last_switch_value;
	uint8_t switch_value;
	uint8_t s3to2;
	uint8_t s3to1;
	int s3to2_cnt;
	int s3to1_cnt;
}RemoteSwitch_t;

typedef __packed struct
{
  	u8 Key_W_Flag ;
	u8 Key_A_Flag ;
	u8 Key_S_Flag ;
	u8 Key_D_Flag ;
	u8 Key_SHIFT_Flag ;
	u8 Key_X_Flag ;
	u8 Key_CTRL_Flag ;
	u8 Key_Q_Flag ;
	u8 Key_E_Flag ;
	u8 Key_R_Flag ;
	u8 Key_F_Flag ;
	u8 Key_G_Flag ;
	u8 Key_Z_Flag ;	
	u8 Key_C_Flag ;
	u8 Key_V_Flag ;
	u8 Key_B_Flag ;
	
	u8 Key_W_TFlag ;
	u8 Key_A_TFlag ;
	u8 Key_S_TFlag ;
	u8 Key_D_TFlag ;
	u8 Key_SHIFT_TFlag ;
	u8 Key_X_TFlag ;
	u8 Key_CTRL_TFlag ;
	u8 Key_Q_TFlag ;
	u8 Key_E_TFlag ;
	u8 Key_R_TFlag ;
	u8 Key_F_TFlag ;
	u8 Key_G_TFlag ;
	u8 Key_Z_TFlag ;	
	u8 Key_C_TFlag ;
	u8 Key_V_TFlag ;
	u8 Key_B_TFlag ;
	
}Key_Flag_t;



typedef struct
{
	Remote rc;
	Mouse mouse;
	Key key;

	InputMode_e inputmode;
	RemoteSwitch_t RemoteSwitch;
	Key_Flag_t Key_Flag;

}RC_Ctl_t;


void RemoteDataPrcess(uint8_t *pData,u16 length);
void SetInputMode(RC_Ctl_t *remote);
void updateKeyFlag(uint16_t key,RC_Ctl_t *remote, uint8_t *flag);
void keyborad_process(RC_Ctl_t *remote);
uint8_t T_Key_procces(u8 flag,u8 *a,u8 *i);
void GetRemoteSwitchAction(RC_Ctl_t *remote);

extern RC_Ctl_t RC_CtrlData;





#endif
