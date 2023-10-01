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
u8 yaw_error_flag_time=0;
u8 pitch_error_flag_time=0;
u8 shoot1_error_flag_time=0;
u8 shoot2_error_flag_time=0;
u8 poke_error_flag_time=0;

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
float click_time_buff;
//控制任务，放在timer6 1ms定时中断中执行
void Control_Task(void)           //1ms
{
if(fabs(gim.pid.yaw_angle_ref-gim.pid.yaw_angle_fdb)>0.5
	||fabs(gim.pid.yaw_angle_ref-gim.pid.yaw_angle_fdb)>0.5)
{click_time_buff++;}
 

  time_tick_1ms++;                 //这就是一个计时变量
  if(xy_0_flag==0)
	 {
				 buff_karman_filter_calc(&buff_kalman_filter,yaw_angle_ref_aim+Yaw_remain,pit_angle_ref_aim);
	 }
	 if(new_location.flag)
	 {
		 autoshoot_karman_filter_calc(&autoshoot_kalman_filter,new_location.x,new_location.y,new_location.yaw_speed);
	 }
  IWDG_ReloadCounter();            //喂狗
  if(time_tick_1ms%2== 0)
    {
      mode_switch_task();
      gimbal_task();
      shot_task();                  //发射任务  2ms一次
    }
  if(time_tick_1ms%2== 1)
    {
     SuperviseTask();//监控任务
     chassis_task();
    }
  if(time_tick_1ms%20 == 0) //上传用户信息
    {
      Client_send_handle();
    }

  if(time_tick_1ms%3 == 0)
    {
      send_protocol(yaw_Angle ,pitch_Angle,judge_rece_mesg.game_robot_state.robot_id);
    }
	if(time_tick_1ms%20==1)      //自检
	 {
		 
		if(yaw_error_flag==1)
		{
			yaw_error_flag_time++;
			if(yaw_error_flag_time>50)
			{
				 checkself.yaw_error_flag=1;
				 yaw_error_flag_time=0;
			}
	  }
	else
	  {
			checkself.yaw_error_flag=0;
			yaw_error_flag_time=0;
		}
		
		if(pitch_error_flag==1)
		{
			pitch_error_flag_time++;
			if(pitch_error_flag_time>50)
			{
				 checkself.pitch_error_flag=1;
				 pitch_error_flag_time=0;
			}
	  }
		else
		{
			checkself.pitch_error_flag=0;
			pitch_error_flag_time=0;
		}
		
		if(shoot1_error_flag==1)
		{
			shoot1_error_flag_time++;
			if(shoot1_error_flag_time>50)
			{
				 checkself.shoot1_error_flag=1;
				 shoot1_error_flag_time=0;
			}
	  }
		else
		{
			checkself.shoot1_error_flag=0;
			shoot1_error_flag_time=0;
		}
		
		if(shoot2_error_flag==1)
		{
			shoot2_error_flag_time++;
			if(shoot2_error_flag_time>50)
			{
				 checkself.shoot2_error_flag=1;
				 shoot2_error_flag_time=0;
			}
	  }
		else
		{
			checkself.shoot2_error_flag=0;
			shoot2_error_flag_time=0;
		}
		
		
		if(poke_error_flag==1)
		{
			poke_error_flag_time++;
			if(poke_error_flag_time>50)
			{
				 checkself.poke_error_flag=1;
				 poke_error_flag_time=0;
			}
	  }
		else
		{
			checkself.poke_error_flag=0;
			poke_error_flag_time=0;
		}
		
		if(checkself.poke_error_flag==1||checkself.shoot2_error_flag==1||checkself.shoot1_error_flag==1)
		{
			checkself.shoot_error_flag=0;
		}
		
		
		
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



