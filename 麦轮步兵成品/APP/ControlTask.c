#include "main.h"
#include "student.pb-c.h"
/*--------------------------------------------CTRL Variables----------------------------------------*/
WorkState_e lastWorkState = PREPARE_STATE;
WorkState_e workState = PREPARE_STATE;
RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;
RampGen_t GMYawRamp = RAMP_GEN_DAFAULT;
Power_Control_Struct Power_Control = POWER_CONTROL_DEFAULT;
u8 showflag=0x33;
u16 output_add;

//u8 control_flag = 0 ;



extern char Stand_shoot_state_r;
extern char Stand_shoot_state_l;
char Gim_free_state=1;

extern uint8_t CRC_SEND_COLOR_BUF[5];
Shoot_Aim_Mode_e Shoot_Aim_Mode;
Shoot_Buff_Dir_e Shoot_Buff_Dir;

/*
*********************************************************************************************************
*                                            FUNCTIONS
*********************************************************************************************************
*/


float tp = 0;
float T = 0.001;

void SetWorkState(WorkState_e state)
{
  workState = state;
}

WorkState_e GetWorkState()
{
  return workState;
}

uint32_t time_tick_1ms = 0;
uint32_t bullet_number = 0;//子弹个数


//u32 abcde = 0;

int abbb;
//控制任务，放在timer6 1ms定时中断中执行
void Control_Task(void)
{
//  pid_rotate[2].get = BUS1_CM1Encoder.rate_rpm;
//  pid_rotate[1].get = BUS1_CM2Encoder.rate_rpm;
//	      pid_rotate[2].set=abbb;
//      pid_rotate[1].set=abbb;
//	pid_calc(& pid_rotate[2],pid_rotate[2].get, pid_rotate[2].set);
//  pid_calc(& pid_rotate[1],pid_rotate[1].get, pid_rotate[1].set);
////							OutData[1]= (int)(pid_rotate[1].get*10);
////							OutData[2]= (int)(pid_rotate[2].get*10);
////							OutData[3]= (int)(pid_rotate[2].set*10);
//  Set_CM_Speed(CAN1, pid_rotate[2].out, pid_rotate[1].out,0, 0);
	
	
  time_tick_1ms++;
  IWDG_ReloadCounter();            //喂狗
//  while(time_tick_1ms>1000)
//    {
  if(time_tick_1ms%2== 0)
    {
      mode_switch_task();
      gimbal_task();
      shot_task();
    }

  if(time_tick_1ms%10== 0)
    {
      SuperviseTask();//监控任务
			chassis_task();
    }
		else if(time_tick_1ms%10==9)
		{power_send_handle1();}
		else if(time_tick_1ms%10==5)
		{
		power_send_handle2();
		}
  if(time_tick_1ms%20 == 0) //上传用户信息
    {
      Client_send_handle();
    }

    if(time_tick_1ms%3 == 0)
    {
      send_protocol(yaw_Angle ,pitch_Angle ,judge_rece_mesg.game_robot_state.robot_id);
    }

}

//控制任务初始化程序
void ControtLoopTaskInit(void)
{
  time_tick_1ms = 0;   //中断中的计数清零
  //程序参数初始化
  AppParamInit();
  //设置工作模式
  SetWorkState(PREPARE_STATE);
  //斜坡初始化
  GMPitchRamp.SetScale(&GMPitchRamp, PREPARE_TIME_TICK_MS);
  GMYawRamp.SetScale(&GMYawRamp, PREPARE_TIME_TICK_MS);
  GMPitchRamp.ResetCounter(&GMPitchRamp);
  GMYawRamp.ResetCounter(&GMYawRamp);
  //云台给定角度初始化
  GimbalRef.pitch_angle_dynamic_ref = 0.0f;
  GimbalRef.yaw_angle_dynamic_ref = 0.0f;
  //监控任务初始化
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_RC));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_IMU));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_ZGYRO));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR1));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR2));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR3));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR4));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR5));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR6));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_DEADLOCK));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_NOCALI));

  Shoot_Aim_Mode = auto_aim_mode;

  gimbal_param_init();
  chassis_param_init();
  shot_param_init();

}



