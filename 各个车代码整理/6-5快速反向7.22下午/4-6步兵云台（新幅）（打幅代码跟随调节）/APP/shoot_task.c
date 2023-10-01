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

  System_performance();          //配置系统环境
  shoot_bullet_handle();         //拨盘
  shoot_friction_handle();       //摩擦轮

}

void heat0_limit(void)           //热量限制
{
  //由于发送的数据为50hz，而且每次进入该操作需要5次循环，因此计算时 公式为：冷却上限-热量值+0.1*冷却值
  shot.limit_heart0 = judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit-judge_rece_mesg.power_heat_data.shooter_id1_17mm_cooling_heat+0.1*judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_rate;
  if(shot.limit_heart0>15)//residual_heat)
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
int single_shoot_flag;
int press_l_flag;
extern int press_l_state_switch;
extern int single_shoot_time;
int shoot_lock_time;
int single_angle=60;
static void shoot_bullet_handle(void)    
{
  //判断是否会卡弹
  if((GetShootState() == SHOOTING) && (lock_rotor == 0)&&shot.ctrl_mode==1)          //没有卡弹
    {
      start_reversal_count = 0;//清零反转计时
      start_shooting_count++;

      /***********************热量限制************************************/

      if(shot.limit_heart0 > 15)//余量充裕
        {
          have_heat0_flag = 1;
        }
      else
        {
          have_heat0_flag = 0;
        }
				
      if((start_shooting_count>500) && (abs(PokeEncoder.filter_rate)<10) &&(have_heat0_flag == 1) )//开始了一段时间并且转速低于一定的值  说明堵转
        {
					
					shoot_lock_time++;
        }
			if(shoot_lock_time>200)
			{
				  lock_rotor = 1;
          start_shooting_count = 0;
			}
    }
  else                    //清除卡弹
    {
      if((GetShootState() == NOSHOOTING)||shot.ctrl_mode==0)
        {
          start_shooting_count = 0;//清零正转计时
          start_reversal_count = 0;//清零反转计时
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


							
buff_shoot_flag_time++;                   //打幅发射时间计算
VAL_LIMIT(buff_shoot_flag_time,0,10000);

//	if((fabs(yaw_err)>2)||
// (fabs(pitch_err)>2))
// {buff_shoot_flag_time=10000;}
		
  //打幅部分	
//	if(friction_wheel_state == FRICTION_WHEEL_ON&&(GetShootState() == SHOOTING))
//	{

		//如果yaw轴的误差小于0.35，并且pitch轴的误差小于0.35
	//  if((fabs(yaw_err)<1)&&(fabs(pitch_err)<1))
//		if((fabs(gim.pid.yaw_angle_ref-gim.pid.yaw_angle_fdb)<0.5)&&
//		(fabs(gim.pid.pit_angle_ref-gim.pid.pit_angle_fdb)<0.5))
//		{shoot_buff_click_angle_time++;}   //buff的标志位增加
//		if(shoot_buff_click_angle_time>3)  //如果持续三次
//		{
//			if(buff_shoot_flag_time>=buff_shoot_flag_time_flag)//并且时间误差小于1的持续时间大于0.6秒
//			{
//				buff_shoot_flag_time=0;
//				BUFF_shoot_flag=1;
////				shoot_buff_click_angle_time=0;
//			}
//		}
//	}
//	else
//	{	
//		buff_shoot_flag_time=10000;       //第一次进行发弹可以快一点
//		BUFF_shoot_flag=0;
//	}
	
  
	if(friction_wheel_state == FRICTION_WHEEL_ON&&(GetShootState() == SHOOTING))
	{
		click_time++;
		if(click_time<20&&will_num_shoot>4)           //刚开始发弹且剩余量很多时可以提高射频     
		{shoot_frequency=shoot_frequency*1.2;}

	}
	else
	{ 
		click_time=0;
		if(shoot_frequency!=0)
		{
		  pid_trigger_angle.set=pid_trigger_angle.get;
		}       //松手立即停
	  
		shoot_frequency=0;
	}	
	
	if(shoot_frequency!=0)                          //如果射频不为0（射频用来设定热量限制）
	{
		  //计算2ms一次
		  if(time_tick_1ms%1==0)
			{
			   if(shoot_frequency>20)                   //高射速时，计算射击时间
		     {
		        will_time_shoot=(will_num_shoot-4)*1000/shoot_frequency + time_tick_1ms;             //没有裁判系统就读不到剩余弹量，就没法发射
			   }
				 else if(shoot_frequency>13)
				 {
					 will_time_shoot=(will_num_shoot-3.5)*1000/shoot_frequency + time_tick_1ms;
				 }
			   else
			   {
			      will_time_shoot=(will_num_shoot-1.5)*1000/shoot_frequency + time_tick_1ms;
				 }
			}
	}
	//发射标志位（没用过）
	if((GetShootState() == SHOOTING))
	{shoot_flag=1;}
	else
  {shoot_flag=0;}
	
  //拨盘控制逻辑
	if(gim.ctrl_mode!=GIMBAL_AUTO_SMALL_BUFF&&gim.ctrl_mode!=GIMBAL_AUTO_BIG_BUFF)   //正常模式
	{
			//time_tick_1ms<will_time_shoot为热量限制的核心
			if(time_tick_1ms<will_time_shoot&&(GetShootState() == SHOOTING))  //&&(judge_rece_mesg.power_heat_data.shooter_id1_17mm_cooling_heat<judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit-20)
			{ 
//				if(lock_rotor==1)
//				{
//					pid_trigger_angle.set=pid_trigger_angle.get+2*shoot_frequency*45*36/500;
//				}			
//				else
				{
				//拨盘转速的赋值    pid_trigger_angle.set是角度值   pid_trigger_speed.set是速度值   
				//之前用的是电机的转速，后面用的是射频——因此电机的转速部分代码现在一点用都没有
				//如果要限制的话，最直接的方式就是调低射频，其次就是跟昨晚上一样调整裕度，然后就是热量限制的核心优化
				pid_trigger_angle.set=pid_trigger_angle.get-shoot_frequency*45*36/500;//一秒shoot_frequency发，一发拨盘转45°，减速比是1：36。每两毫秒执行一次固除以500。	
				}					
				pid_trigger_angle.get = PokeEncoder.ecd_angle;
				pid_calc(&pid_trigger_angle,pid_trigger_angle.get,pid_trigger_angle.set);
				pid_trigger_speed.get=PokeEncoder .filter_rate;
				pid_trigger_speed.set=pid_trigger_angle.out;
				pid_calc(&pid_trigger_speed,pid_trigger_speed.get,pid_trigger_speed.set);
				Set_Poke_Current(CAN1,pid_trigger_speed.out);
			}
			else
			{
				pid_trigger_angle.set = pid_trigger_angle.get;//松手必须停
				Set_Poke_Current(CAN1,0);
			}
	}
	else               //打幅模式
	{
		if(BUFF_shoot_flag==1&&friction_wheel_state == FRICTION_WHEEL_ON)
			{
/***********************速度单环********************************/
//			pid_trigger_speed.set=-390;
//			shoot_buff_time++;
//			if(shoot_buff_time>shoot_buff_time_flag)//大于指定值后停转————保证只发一发
//			{
//				BUFF_shoot_flag=0;
//				shoot_buff_time=0;
//				pid_trigger_speed.set=0;
//			}
//		}
//		if(BUFF_shoot_flag==0)
//		{
//			pid_trigger_speed.set=0;   
//		}
//		pid_trigger_speed.get=PokeEncoder.filter_rate;
//		pid_calc(&pid_trigger_speed,pid_trigger_speed.get,pid_trigger_speed.set);
//		Set_Poke_Current(CAN1,pid_trigger_speed.out);
//	}
//}	
/********************速度角度双环*************************/
/********************速度角度双环*************************/
			if(press_l_state_switch==1)
			{
				press_l_flag=1;
			}
			
			if(press_l_flag==1)
			{ 
					pid_trigger_angle_buf.set-=single_angle;
					press_l_flag=0;
			}	
			else if(press_l_flag==0)
			{
				pid_trigger_angle_buf.set=pid_trigger_angle_buf.get;		
			}
			
			
			
			pid_trigger_angle_buf.get = PokeEncoder.ecd_angle;
			pid_calc(&pid_trigger_angle_buf,pid_trigger_angle_buf.get,pid_trigger_angle_buf.set);
			pid_trigger_speed_buf.get=PokeEncoder.filter_rate;
			pid_trigger_speed_buf.set=pid_trigger_angle_buf.out;
			pid_calc(&pid_trigger_speed_buf,pid_trigger_speed_buf.get,pid_trigger_speed_buf.set);

			
		Set_Poke_Current(CAN1,pid_trigger_speed_buf.out);
			

		}
	}

/**************************速度角度电流三环*************************/		
//					if(press_l_state_switch==1)
//			{
//				press_l_flag=1;
//			}
//			
//			if(press_l_flag==1)
//			{ 
//					pid_trigger_angle.set-=single_angle;
//					press_l_flag=0;
//			}	
//			else if(press_l_flag==0)
//			{
//				pid_trigger_angle.set=pid_trigger_angle.get;		
//			}
//			
//			
//			pid_trigger_angle.get = PokeEncoder.ecd_angle;
//			pid_calc(&pid_trigger_angle,pid_trigger_angle.get,pid_trigger_angle.set);
//			pid_trigger_speed.get = PokeEncoder.filter_rate;
//			pid_trigger_speed.set = pid_trigger_angle.out;
//			pid_calc(&pid_trigger_speed,pid_trigger_speed.get,pid_trigger_speed.set);
//			pid_trigger_current.get = PokeEncoder.torque_current;
//			pid_trigger_current.set = pid_trigger_speed.out;
//			pid_calc(&pid_trigger_current,pid_trigger_current.get,pid_trigger_current.set);

//			
//		Set_Poke_Current(CAN1,pid_trigger_current.out);
//		}
//	}
	
}

		


int speed_fiction=1000;
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
//裁判系统发射功率限制
//  if(judge_rece_mesg.game_robot_state.mains_power_shooter_output==0)
//    {
//      pid_rotate[2].set=0;
//      pid_rotate[1].set=0;
//    }
 


  pid_calc(& pid_rotate[2],pid_rotate[2].get, pid_rotate[2].set);
  pid_calc(& pid_rotate[1],pid_rotate[1].get, pid_rotate[1].set);
	Set_CM_Speed(CAN1, pid_rotate[2].out, pid_rotate[1].out,0, 0);
//		Set_CM_Speed(CAN1,0,0,0,0);
}
u8 bulletspead_level=0;
//
float shoot_save[100];
//

void Mode_switch(void)                 
{
	//弹速限制来改变不同的模式 
  if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==30)          //弹速限制在30为速度模式
	{shoot_mode_selection=speed;}
  else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==15&judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit==50)    //弹速限制为15，但是冷却限制为50
	{ shoot_mode_selection=cooling;}
  else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==15&judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit==150)   //弹速限制为15，但是冷却限制为150
	{ shoot_mode_selection=outburst;}
	
	
}
float last_bullet_speed;
float realtime_diff_fri_speed;
float FRICTION_SPEED_plan_ref;
float last_speed_limit;
float key_shoot_speed;


void Speed_switch(void)
{
//	//自适应控制整体
//	if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==15||judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==18)   //射速限制为15，或者18
//	{
//		FRICTION_SPEED_plan_ref=judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit-1.5;                  //调节摩擦轮的速度
//	}
//	else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==30)                                  //射速限制为30
//	{
//			if(gim.ctrl_mode==GIMBAL_AUTO_SMALL_BUFF||gim.ctrl_mode==GIMBAL_AUTO_BIG_BUFF)                            //如果打幅
//			{
//		     FRICTION_SPEED_plan_ref=judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit-2;
//			}
//			else                                                                                                      //如果不打幅
//			{
//			   FRICTION_SPEED_plan_ref=judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit-3;
//			}
//	}
//	else                                                                                                          //其他情况
//  {
//	  FRICTION_SPEED_plan_ref=13.5;
//	}  
//	
//	
//	//更新摩擦轮转速
//	if(judge_rece_mesg.shoot_data.bullet_speed>10&&judge_rece_mesg.shoot_data.bullet_speed!=last_bullet_speed)
//	{
//    realtime_diff_fri_speed=
//		(FRICTION_SPEED_plan_ref-judge_rece_mesg.shoot_data.bullet_speed)*key_shoot_speed;//此系数需要测量
//		VAL_LIMIT(realtime_diff_fri_speed,-100,100);
//    if(fabs(FRICTION_SPEED_plan_ref-judge_rece_mesg.shoot_data.bullet_speed)>1.2)     //如果实际值与参考值差距过大，则将参数修改
//			{realtime_diff_fri_speed*=1.2;}	//次系数为估计值，作用不大，无需精准测量。		
//		FRICTION_SPEED_REF-=realtime_diff_fri_speed;                                      
//	}
//	last_bullet_speed=judge_rece_mesg.shoot_data.bullet_speed;                          //数据更新

	
	
	
		if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit!=last_speed_limit)
		{ 
			if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit<16)
			{FRICTION_SPEED_REF = FRICTION_SPEED_15;}
			else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit>16&judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit<19)
			{FRICTION_SPEED_REF = FRICTION_SPEED_18;}
			else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit>19&judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit<31)
			{FRICTION_SPEED_REF = FRICTION_SPEED_30;}
			else
			{FRICTION_SPEED_REF = FRICTION_SPEED_15;}               
		}
		last_speed_limit=judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit;     //数据更新

}


void Heat0_switch(void)
{
	//剩余发弹量=（热量上限-热量值）/10      ————一个弹丸的热量为10
  will_num_shoot=(judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit-judge_rece_mesg.power_heat_data.shooter_id1_17mm_cooling_heat)/10;    //剩余发射量
  cooling_rate=judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_rate;                  //冷却速率                                                          //冷却速度
      

			  //计算射频，但是实际上没什么用，只不过是一种计算方式
			  if(judge_rece_mesg.game_robot_state.robot_level==1)//level_冷却
				{
          shoot_frequency=10;						
				}
        else if(judge_rece_mesg.game_robot_state.robot_level==2)//level_2
        {
					shoot_frequency=14;
        }
        else if(judge_rece_mesg.game_robot_state.robot_level==3)//level_3
        {
					shoot_frequency=14;
        }
				else
        {
			    shoot_frequency=10;
			  }
				
#if STANDARD == 3				
				if(bulletspead_level==1)
				{
					shoot_frequency=1.5*shoot_frequency;
				}
//				shoot_frequency=24; 
#elif STANDARD == 4
				if(bulletspead_level==1)
				{
					shoot_frequency=15; 
				}
//				shoot_frequency=20; 
#elif STANDARD == 5			
				if(bulletspead_level==1)
				{
					shoot_frequency=1.5*shoot_frequency;
				}
#endif				
				


}

void System_performance(void)
{
  Mode_switch();         //发射模式选择
  Speed_switch();        //摩擦轮转速调整
  Heat0_switch();        //拨盘发射水平

}




void shot_param_init(void)
{
  PID_struct_init(&pid_trigger_angle, POSITION_PID, 5000, 1000,5,0.2,15);
	PID_struct_init(&pid_trigger_speed,POSITION_PID,15000,10000,300,0.1,4);
	
	PID_struct_init(&pid_trigger_angle_buf,POSITION_PID, 6000, 1000,5,0.6,15);
	PID_struct_init(&pid_trigger_speed_buf,POSITION_PID,19000,10000,300,0.3,4);
	
	PID_struct_init(&pid_trigger_current,POSITION_PID,10000,10000,100,0.1,4);
  PID_struct_init(&pid_rotate[1], POSITION_PID,15500,11500,50,0,0);
  PID_struct_init(&pid_rotate[2], POSITION_PID,15500,11500,50,0,0);

  FRICTION_SPEED_REF = 0;
//	FRICTION_SPEED_REF =FRICTION_SPEED_15;            //没装裁判系统，因此先进行摩擦轮速度赋值
	
  shot.ctrl_mode=1;
  shot.limit_heart0=80;
  shot.ShootMotorSpeed = PID_SHOOT_MOTOR_SPEED;       //拨盘赋值初始化
  shot.NoLimitHeat = 80;
}






