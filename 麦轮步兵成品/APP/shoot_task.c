/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file shoot_task.c
 *  @version 1.1
 *  @date June 2017
 *
 *  @brief shoot bullet task
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */
#include "main.h"
/* shot task global parameter */
shoot_t  shot;
uint32_t shoot_time_last;
uint32_t start_shooting_count = 0;//正转计时
uint32_t start_lock_rotor_count = 0;//堵转计时
uint32_t start_reversal_count = 0;//反转计时
uint8_t lock_rotor = 0;//堵转标志位
//uint8_t overheat = 0;//超功率标志位

int16_t FRICTION_SPEED_REF = 0;
shoot_mode_selection_e shoot_mode_selection;
//发射机构射击电机任务
int16_t pwm_ccr = 0;
int shoot_time_ms;
int PID_SHOOT_MOTOR_SPEED;

int FRICTION_SPEED=0;
extern u8 frictionSpeedmode;

int residual_heat=0;
u32 buff_follwo_times = 0;
u32 ref_shot_bullets = 0;
float shoot_speed = 0;
float last_shoot_speed = 0;
float cooling_rate;
u8 last_shoot_num=0;
float shoot_frequency;
int will_num_shoot;
void shot_task(void)
{

  System_performance();
  shoot_bullet_handle();
  shoot_friction_handle();

}

void heat0_limit(void)
{

  shot.limit_heart0 = judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit-judge_rece_mesg.power_heat_data.shooter_id1_17mm_cooling_heat+0.1*judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_rate;
  if(shot.limit_heart0>0)//residual_heat)
    shot.ctrl_mode=1;
  else
    {
      shot.ctrl_mode=0;
    }

}

int debug_speed = 1500;
int shot_cmd;
int buff_timer=320;//dafu350;
u8 have_heat0_flag = 0;
u8 shoot_number=0;
int lost_buff_timer=0;
int will_time_shoot;
int will_shoot_time_flag;
int click_time;
float shoot_flag;

int BUFF_shoot_flag;
int buff_shoot_flag;
int last_buff_shoot_flag;
int buff_shoot_flag_time;
int buff_shoot_flag_time_flag=0.6 //0.5s一发
*1000/2;
int shoot_buff_time;
int shoot_buff_time_flag=45;
int shoot_buff_click_angle_time;
static void shoot_bullet_handle(void)
{

  if((GetShootState() == SHOOTING) && (lock_rotor == 0)&&shot.ctrl_mode==1)
    {
      start_reversal_count = 0;//清零反转计时
      start_shooting_count++;

      /***********************热量限制************************************/

      if(shot.limit_heart0 > 10)//余量充裕
        {
          have_heat0_flag = 1;
//          pid_trigger_speed.set = PID_SHOOT_MOTOR_SPEED;
        }
      else
        {
//          pid_trigger_speed.set = 0;
          have_heat0_flag = 0;
        }

      /********************************************************************/

      //	pid_trigger_speed.set = -PID_SHOOT_MOTOR_SPEED;             //取消热量限制

      /********************************************************************/
      if((start_shooting_count>250) && (abs(PokeEncoder.filter_rate)<20) &&(have_heat0_flag == 1) )//开始了一段时间并且转速低于一定的值  说明堵转
        {
          lock_rotor = 1;
          start_shooting_count = 0;
        }
    }
  else
    {
      if((GetShootState() == NOSHOOTING)||shot.ctrl_mode==0)
        {
          start_shooting_count = 0;//清零正转计时
          start_reversal_count = 0;//清零反转计时
//          pid_trigger_speed.set= 0;
        }
      else  if(lock_rotor ==1)//堵转
        {
          start_shooting_count = 0;//清零正转计时
          start_reversal_count++;
          if(start_reversal_count>200)//反转一段时间
            {
              lock_rotor =0;
            }
        }
    }


							
buff_shoot_flag_time++;
VAL_LIMIT(buff_shoot_flag_time,0,10000);

//	if((fabs(yaw_err)>2)||
// (fabs(pitch_err)>2))
// {buff_shoot_flag_time=10000;}
		
		
if(GetShootState() == SHOOTING)
{
// if((fabs(gim.pid.yaw_angle_ref-gim.pid.yaw_angle_fdb)<0.35)&&
//  (fabs(gim.pid.pit_angle_ref-gim.pid.pit_angle_fdb)<0.35))


 {shoot_buff_click_angle_time++;}
 if(shoot_buff_click_angle_time>3){
if(buff_shoot_flag_time>=buff_shoot_flag_time_flag)
{
	buff_shoot_flag_time=0;
	BUFF_shoot_flag=1;
}}
}
else
{buff_shoot_flag_time=10000;
BUFF_shoot_flag=0;
}
	

	if(friction_wheel_state == FRICTION_WHEEL_ON&&(GetShootState() == SHOOTING))
	{
		click_time++;
		if(click_time<20&&will_num_shoot>4)
		{shoot_frequency=shoot_frequency*1.2;}

	}
	else
	{  click_time=0;
		if(shoot_frequency!=0)
		{pid_trigger_speed.set=pid_trigger_speed.get;}//松手立即停
	shoot_frequency=0;
	}	
	if(shoot_frequency!=0){
	if(time_tick_1ms%1==0){
		if(shoot_frequency>13){
	will_time_shoot=(will_num_shoot-3)*1000/shoot_frequency + time_tick_1ms;}
		else
		{
		will_time_shoot=(will_num_shoot-1)*1000/shoot_frequency + time_tick_1ms;}
		}
	
	}
	if((GetShootState() == SHOOTING))
	{shoot_flag=1;}
	else
   {shoot_flag=0;}

		if(gim.ctrl_mode!=GIMBAL_AUTO_SMALL_BUFF&&gim.ctrl_mode!=GIMBAL_AUTO_BIG_BUFF){
		if(time_tick_1ms<will_time_shoot&&(GetShootState() == SHOOTING))
	{ 

		pid_trigger_speed.set=pid_trigger_speed.get-shoot_frequency*45*36/500;//一秒shoot_frequency发，一发拨盘转45°，减速比是1：36。每两毫秒执行一次固除以500。
		
  pid_trigger_speed.get = PokeEncoder.ecd_angle;
  pid_calc(&pid_trigger_speed,pid_trigger_speed.get,pid_trigger_speed.set);
	pid_trigger_second_speed.get=PokeEncoder .filter_rate;
  pid_trigger_second_speed.set=pid_trigger_speed.out;
	pid_calc(&pid_trigger_second_speed,pid_trigger_second_speed.get,pid_trigger_second_speed.set);
  Set_Poke_Current(CAN1,pid_trigger_second_speed.out);
	}
	else
{
    pid_trigger_speed.set = pid_trigger_speed.get;//松手必须停
	  Set_Poke_Current(CAN1,0);
}
}
else {
	if(BUFF_shoot_flag==1&&friction_wheel_state == FRICTION_WHEEL_ON)
	{
		pid_trigger_second_speed.set=-400;
		shoot_buff_time++;
		if(shoot_buff_time>shoot_buff_time_flag)
		{
		BUFF_shoot_flag=0;
			shoot_buff_time=0;
			pid_trigger_second_speed.set=0;
		}
	}
	if(BUFF_shoot_flag==0)
	{pid_trigger_second_speed.set=0;}
	pid_trigger_second_speed.get=PokeEncoder .filter_rate;
	pid_calc(&pid_trigger_second_speed,pid_trigger_second_speed.get,pid_trigger_second_speed.set);
  Set_Poke_Current(CAN1,pid_trigger_second_speed.out);
	}
}	

	
	

		


u8  friction_rotor = 0;
float speed_bias_pid_out = 0;
void shoot_friction_handle(void)
{
  FRICTION_SPEED=FRICTION_SPEED_REF;
  if(friction_rotor ==1)
    {
      pid_rotate[2].set=-(FRICTION_SPEED_REF)*frictionRamp.Calc(&frictionRamp);
      pid_rotate[1].set=(FRICTION_SPEED_REF)*frictionRamp.Calc(&frictionRamp);
    }
  else if(friction_rotor ==2)
    {
      pid_rotate[2].set=(FRICTION_SPEED-(FRICTION_SPEED_REF)*frictionRamp.Calc(&frictionRamp));
      pid_rotate[1].set=-(FRICTION_SPEED-(FRICTION_SPEED_REF)*frictionRamp.Calc(&frictionRamp));
    }
  else
    {
      friction_rotor =0;
      pid_rotate[2].set=0;
      pid_rotate[1].set=0;
    }
  pid_rotate[2].get = BUS1_CM1Encoder.filter_rate;
  pid_rotate[1].get = BUS1_CM2Encoder.filter_rate;
  if(judge_rece_mesg.game_robot_state.mains_power_shooter_output==0)
    {
      pid_rotate[2].set=0;
      pid_rotate[1].set=0;
    }



  pid_calc(& pid_rotate[2],pid_rotate[2].get, pid_rotate[2].set);
  pid_calc(& pid_rotate[1],pid_rotate[1].get, pid_rotate[1].set);
//							OutData[1]= (int)(pid_rotate[1].get*10);
//							OutData[2]= (int)(pid_rotate[2].get*10);
//							OutData[3]= (int)(pid_rotate[2].set*10);
  Set_CM_Speed(CAN1, pid_rotate[2].out, pid_rotate[1].out,0, 0);
}
u8 bulletspead_level=0;
//
float shoot_save[100];
//

void Mode_switch(void)
{
  if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==30)
	{shoot_mode_selection=speed;}
  else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==15&judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit==50)
	{ shoot_mode_selection=cooling;}
  else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==15&judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit==150)
	{ shoot_mode_selection=outburst;}
}
float last_bullet_speed;
float difference_FRICTION_speed;
float realtime_diff_fri_speed;
float FRICTION_SPEED_plan_ref;
float last_speed_limit;
float key_shoot_speed;
void Speed_switch(void)
{
	if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==15||judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==18)
		{
		FRICTION_SPEED_plan_ref=judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit-1.5;
		}
		else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==30){
			if(gim.ctrl_mode==GIMBAL_AUTO_SMALL_BUFF||gim.ctrl_mode==GIMBAL_AUTO_BIG_BUFF){
		FRICTION_SPEED_plan_ref=judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit-2;}
			else
			{
			FRICTION_SPEED_plan_ref=judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit-3;
			}
		}
		else {FRICTION_SPEED_plan_ref=13.5;}
		
	if(judge_rece_mesg.shoot_data.bullet_speed>10&&judge_rece_mesg.shoot_data.bullet_speed!=last_bullet_speed)//防止尿弹
	{
    realtime_diff_fri_speed=
		(FRICTION_SPEED_plan_ref-judge_rece_mesg.shoot_data.bullet_speed)*key_shoot_speed;//此系数需要测量。
		VAL_LIMIT(realtime_diff_fri_speed,-100,100);
    if(fabs(FRICTION_SPEED_plan_ref-judge_rece_mesg.shoot_data.bullet_speed)>1.2)
			{realtime_diff_fri_speed*=1.2;}	//次系数为估计值，作用不大，无需精准测量。		
		FRICTION_SPEED_REF-=realtime_diff_fri_speed;
	}
		last_bullet_speed=judge_rece_mesg.shoot_data.bullet_speed;
	
	if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit!=last_speed_limit)
	{ 
	if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit<16)
	{ FRICTION_SPEED_REF = FRICTION_SPEED_15;
	key_shoot_speed=3;}
  else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit>16&judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit<19)
	{FRICTION_SPEED_REF = FRICTION_SPEED_18;
	key_shoot_speed=3;}
  else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit>19&judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit<31)
	{FRICTION_SPEED_REF = FRICTION_SPEED_30;
	key_shoot_speed=3;}
	else
	{ FRICTION_SPEED_REF = FRICTION_SPEED_15;	
	key_shoot_speed=3;}
		difference_FRICTION_speed=0;
	}
	last_speed_limit=judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit;

	
}

void Heat0_switch(void)
{
will_num_shoot=(judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit-judge_rece_mesg.power_heat_data.shooter_id1_17mm_cooling_heat)/10;
  cooling_rate=judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_rate;

			
			   if(judge_rece_mesg.game_robot_state.robot_level==1)//level_冷却
					{
            shoot_frequency=12;
						
					}
          else if(judge_rece_mesg.game_robot_state.robot_level==2)//level_2
            {shoot_frequency=14;

            }
          else if(judge_rece_mesg.game_robot_state.robot_level==3)//level_3
            {shoot_frequency=14;
            }
					else {shoot_frequency=12;}
		  switch(bulletspead_level)
    {			
    case 0://普通模式
		{


  
      if(shoot_mode_selection==cooling)//冷却
        {
          if(judge_rece_mesg.game_robot_state.robot_level==1)//level_1
            {
              PID_SHOOT_MOTOR_SPEED= -400;
              residual_heat=residual_heat_normal_coling_1;
            }
          else if(judge_rece_mesg.game_robot_state.robot_level==2)//level_2
            {
              PID_SHOOT_MOTOR_SPEED= -440;
              residual_heat=residual_heat_normal_coling_2;
            }
          else if(judge_rece_mesg.game_robot_state.robot_level==3)//level_3
            {
              PID_SHOOT_MOTOR_SPEED= -540;
              residual_heat=residual_heat_normal_coling_3;
            }
        }
      else
        {
          if(shot.limit_heart0<=50)
            PID_SHOOT_MOTOR_SPEED=PID_SHOOT_MOTOR_SPEED_1;
          else if(shot.limit_heart0>50&shot.limit_heart0<=100)
            PID_SHOOT_MOTOR_SPEED=PID_SHOOT_MOTOR_SPEED_2;
          else if(shot.limit_heart0>100&shot.limit_heart0<=150)
            PID_SHOOT_MOTOR_SPEED=PID_SHOOT_MOTOR_SPEED_3;
          else if(shot.limit_heart0>150&shot.limit_heart0<=200)
            PID_SHOOT_MOTOR_SPEED=PID_SHOOT_MOTOR_SPEED_4;
          else if(shot.limit_heart0>200)
            PID_SHOOT_MOTOR_SPEED=PID_SHOOT_MOTOR_SPEED_5;
          if(shoot_mode_selection==speed)
            {
              if(judge_rece_mesg.game_robot_state.robot_level==1)//level_1
                {
                  residual_heat=residual_heat_normal_speed_1;
                }
              else if(judge_rece_mesg.game_robot_state.robot_level==2)//level_2
                {
                  residual_heat=residual_heat_normal_speed_2;
                }
              else if(judge_rece_mesg.game_robot_state.robot_level==3)//level_3
                {
                  residual_heat=residual_heat_normal_speed_3;
                }
            }
          if(shoot_mode_selection==outburst)
            {
              if(judge_rece_mesg.game_robot_state.robot_level==1)//level_1
                {
                  residual_heat=residual_heat_normal_outburst_1;
                }
              else if(judge_rece_mesg.game_robot_state.robot_level==2)//level_2
                {
                  residual_heat=residual_heat_normal_outburst_2;
                }
              else if(judge_rece_mesg.game_robot_state.robot_level==3)//level_3
                {
                  residual_heat=residual_heat_normal_outburst_3;
                }
            }
          else

            {
              if(cooling_rate<20)
                residual_heat=residual_heat_normal_else_1;
              else if(cooling_rate>=20&&cooling_rate<50)
                residual_heat=residual_heat_normal_else_2;
              else if(cooling_rate>=50&&cooling_rate<60)
                residual_heat=residual_heat_normal_else_3;
              else if(cooling_rate>=60)
                residual_heat=residual_heat_normal_else_4;
            }
        }
    }
    break;
    case 1://高射频
    {
      switch(shoot_mode_selection)
        {
        case cooling:
        {
          if(judge_rece_mesg.game_robot_state.robot_level==1)//level_冷却
					{
            residual_heat=residual_heat_middle_coling_1;
						
					}
          else if(judge_rece_mesg.game_robot_state.robot_level==2)//level_2
            {
              residual_heat=residual_heat_middle_coling_2;
              PID_SHOOT_MOTOR_SPEED= -600;
            }
          else if(judge_rece_mesg.game_robot_state.robot_level==3)//level_3
            {
              residual_heat=residual_heat_middle_coling_3;
              PID_SHOOT_MOTOR_SPEED= -650;
            }
        }
        break;
        case speed:
        {
          if(judge_rece_mesg.game_robot_state.robot_level==1)//level_1  射速
            residual_heat=residual_heat_middle_speed_1;
          else if(judge_rece_mesg.game_robot_state.robot_level==2)//level_2
            {
              residual_heat=residual_heat_middle_speed_2;
              PID_SHOOT_MOTOR_SPEED= -600;
            }
          else if(judge_rece_mesg.game_robot_state.robot_level==3)//level_3
            {
              residual_heat=residual_heat_middle_speed_3;
              PID_SHOOT_MOTOR_SPEED= -650;
            }
        }
        break;
        case outburst:
        {
          if(judge_rece_mesg.game_robot_state.robot_level==1)//level_1  爆发
            residual_heat=residual_heat_middle_outburst_1;
          else if(judge_rece_mesg.game_robot_state.robot_level==2)//level_2
            {
              residual_heat=residual_heat_middle_outburst_2;
              PID_SHOOT_MOTOR_SPEED= -600;
            }
          else if(judge_rece_mesg.game_robot_state.robot_level==3)//level_3
            {
              residual_heat=residual_heat_middle_outburst_3;
              PID_SHOOT_MOTOR_SPEED= -650;
            }
        }
        break;
        }
    }
    break;


    }

}

void System_performance(void)
{
  Mode_switch();
  Speed_switch();
  Heat0_switch();

}



void shot_param_init(void)
{
  PID_struct_init(&pid_trigger_speed, POSITION_PID, 1000, 1000,2,0.2,15);
	PID_struct_init(&pid_trigger_second_speed,POSITION_PID,10000,10000,100,0.1,4);
  PID_struct_init(&pid_rotate[1], POSITION_PID,15500,11500,50,0.1,10);
  PID_struct_init(&pid_rotate[2], POSITION_PID,15500,11500,50,0.1,10);

  FRICTION_SPEED_REF = 0;
  shot.ctrl_mode=1;
  shot.limit_heart0=80;
  shot.ShootMotorSpeed = PID_SHOOT_MOTOR_SPEED;
  shot.NoLimitHeat = 80;
}






