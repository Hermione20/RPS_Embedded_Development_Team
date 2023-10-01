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
void shot_task(void)
{

  System_performance();
  shoot_bullet_handle();
  shoot_friction_handle();

}

void heat0_limit(void)
{

  shot.limit_heart0 = judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit-judge_rece_mesg.power_heat_data.shooter_id1_17mm_cooling_heat+0.1*judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_rate;
  if(shot.limit_heart0>residual_heat)
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
static void shoot_bullet_handle(void)
{

 

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

void Mode_switch(void)
{
  if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==30)
    shoot_mode_selection=speed;
  else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==15&judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit==50)
    shoot_mode_selection=cooling ;
  else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==15&judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit==150)
    shoot_mode_selection=outburst ;
}
void Speed_switch(void)
{
  if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit<16)
    FRICTION_SPEED_REF = FRICTION_SPEED_15;
  else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit>16&judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit<19)
    FRICTION_SPEED_REF = FRICTION_SPEED_18;
  else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit>19&judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit<31)
    FRICTION_SPEED_REF = FRICTION_SPEED_30;
	else
		    FRICTION_SPEED_REF = FRICTION_SPEED_15;
}
void Heat0_switch(void)
{
  cooling_rate=judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_rate;
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
  PID_struct_init(&pid_trigger_speed, POSITION_PID, 10000, 10000,35, 0.7f, 4);
  PID_struct_init(&pid_rotate[1], POSITION_PID,15500,11500,50,0.0,10);
  PID_struct_init(&pid_rotate[2], POSITION_PID,15500,11500,50,0.0,100);

  FRICTION_SPEED_REF = 0;
  shot.ctrl_mode=1;
  shot.limit_heart0=80;
  shot.ShootMotorSpeed = PID_SHOOT_MOTOR_SPEED;
  shot.NoLimitHeat = 80;
}




