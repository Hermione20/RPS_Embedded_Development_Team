#include "shoot_task.h"

/******************** VARIABVLE ******************/
uint16_t frictionSpeed_42=0;		//42mm弹速
uint16_t frictionSpeed_17=0;		//17mm弹速
float residue_heart;						//剩余可用热量

int filter_rate_left[2] ={0};//摩擦轮转速均值滤波
int filter_rate_right[2]={0};
int filter_rate_readtime=0;
int friction_rotor_DIV=11;

int friction_rotor_may_lock=0;//摩擦轮可能堵转计数
int set_time=0;			//下拨盘力度限制计数
float target_angle=0;	//拨盘角度
float target_angle_2=0;	//二级拨盘角度

/********************** Flag **********************/
int over_heat=0;				//超热量标志位
int position_flag=0;		//初始化完成标志位
friction_state_t friction_state;//摩擦轮状态标志位
int friction_normal_flag=0;//摩擦轮正常标志位

int stop_friction_flag=0;//摩擦轮停止标志位
int over_hot_motor=0;		//摩擦轮电机过热

int stop_bullet_flag=0;	//拨盘停止标志位
shoot_state_e shoot_state={0};
int press_l_first_in=1; //1初次进入0非初次进入
int ignore_heat=0;		//忽略量限制标志位
int lock_flag_1times=0;	//反转程序角度控制标志位

/******************** FUNCTION ********************/
//发射任务总成
void shoot_task(void)
{
	Shoot_42mm_speed_Select();//42mm弹速选择
	heat1_limit();						//热量限制
	shoot_friction_handle();	//摩擦轮部分
	shoot_bullet_handle();		//拨盘部分
}

static void Shoot_42mm_speed_Select(void)//42mm弹速
{																					//读裁判系统弹速上限进行选择
  if(judge_rece_mesg.game_robot_state.shooter_id1_42mm_speed_limit==10)
    frictionSpeed_42=FRICTION_SPEED_10;
  else if(judge_rece_mesg.game_robot_state.shooter_id1_42mm_speed_limit==16)
    frictionSpeed_42=FRICTION_SPEED_16;
	else
		frictionSpeed_42=FRICTION_SPEED_10;
}

//热量限制
void heat1_limit(void)
{
//-----------剩余热量计算(最大-累计)----------------
//-----------服务器模式---------------------------
  residue_heart=(judge_rece_mesg.game_robot_state.shooter_id1_42mm_cooling_limit           //获取相关的资料
                       -judge_rece_mesg.power_heat_data.shooter_id1_42mm_cooling_heat);

  if(residue_heart>=100)
      over_heat=0;
  else
      over_heat=1;

	if (RC_CtrlData.Key_Flag.Key_R_Flag)
		ignore_heat=1;		//忽略热量限制

}

//摩擦轮部分
void shoot_friction_handle(void)
{
	if (RC_CtrlData.inputmode != Stop && position_flag==0)
	{
		if(RC_CtrlData.RemoteSwitch.s3to1 || RC_CtrlData.Key_Flag.Key_C_TFlag)
		{
			pid_friction_whell_speed[0].set=-frictionSpeed_42;
			pid_friction_whell_speed[1].set= frictionSpeed_42;
		}
		if (RC_CtrlData.Key_Flag.Key_Z_Flag)
		{
			pid_friction_whell_speed[0].set= frictionSpeed_42;
			pid_friction_whell_speed[1].set=-frictionSpeed_42;
				friction_state = LOCK;
		}
		if ((friction_state != Stop && (RC_CtrlData.RemoteSwitch.s3to1==0 || RC_CtrlData.Key_Flag.Key_C_TFlag==0))
					|| stop_friction_flag)
		{
			pid_friction_whell_speed[0].set= 0;
			pid_friction_whell_speed[1].set= 0;
				friction_state = Stop;
		}
	}
	else
	{
		pid_friction_whell_speed[0].set= 0;
		pid_friction_whell_speed[1].set= 0;
			friction_state = Stop;
		position_flag=1;	//重新初始化
	}
	
	//如果当速度达到要求，摩擦轮正常标志位置1
		if(RC_CtrlData.Key_Flag.Key_C_TFlag && (general_friction.left_motor1.rate_rpm < -50)&&(general_friction.right_motor1.rate_rpm > 50))
		{
			friction_normal_flag = 1;
			friction_rotor_may_lock=0;
				friction_state = NORMAL;
		}
		else if(RC_CtrlData.Key_Flag.Key_C_TFlag &&(general_friction.left_motor1.rate_rpm>-30)&&(general_friction.left_motor1.rate_rpm<30)
																			&&(general_friction.right_motor1.rate_rpm<30)&&(general_friction.right_motor1.rate_rpm>-30))
		{
			friction_rotor_may_lock++;
			if(friction_rotor_may_lock==100)
			{
				friction_state = LOCK;
				friction_rotor_may_lock=0;
				friction_normal_flag = 0;
			}
		}
		else if(friction_state==Stop) //如果模式为停止
		{
			friction_normal_flag = 0;
			friction_rotor_may_lock = 0;
		}
		
	//摩擦轮均值滤波
		filter_rate_readtime++;
		filter_rate_left[0]	+=general_friction.left_motor1.rate_rpm;
		filter_rate_right[0]+=general_friction.right_motor1.rate_rpm;

		if(filter_rate_readtime>=friction_rotor_DIV)//分别获得速度的平均值
		{
			pid_friction_whell_speed[0].get=filter_rate_left[0]/friction_rotor_DIV;
			pid_friction_whell_speed[1].get=filter_rate_right[0]/friction_rotor_DIV;
			filter_rate_readtime=0;
			filter_rate_left[0]=0;
			filter_rate_right[0]=0;
		}
				
	//PID计算
		pid_calc(&pid_friction_whell_speed[0],pid_friction_whell_speed[0].get, pid_friction_whell_speed[0].set);
		pid_calc(&pid_friction_whell_speed[1],pid_friction_whell_speed[1].get, pid_friction_whell_speed[1].set);
	//防止电机过热
		if(general_friction.left_motor1.temperature >= 90||general_friction.right_motor1.temperature >= 90)
		{
			stop_friction_flag = 1;
		}
		if(general_friction.left_motor1.temperature <= 70 && general_friction.right_motor1.temperature <= 70)
			stop_friction_flag = 0;
}

//拨盘
void shoot_bullet_handle(void)
{
	if(!position_flag)            //初始化的时候进入
	{
			target_angle_2 = general_poke.right_poke.ecd_angle;
			position_flag=1;
			pid_clr(&pid_42mm_poke2_angle);
			pid_clr(&pid_42mm_poke2_speed);
	}
	else if((general_friction.left_motor1.rate_rpm<-1000&&general_friction.right_motor1.rate_rpm>1000) && position_flag && (friction_state==NORMAL))
    //如果	摩擦轮开启按键按下	&& 电机达到一定的转速 && 初始化标志位为1	才可以发射
	{
		static int l_key_delay_time=0;
		l_key_delay_time++;
		pid_calc(&pid_42mm_poke_speed,general_poke.left_poke.rate_rpm,POKE_SPEED);
		
	//防止电机过热，下拨盘力度限制
		if(press_l_first_in==0)
			pid_42mm_poke_speed.max_out=POKE_MAX_OUT;
		else
		{
			if(pid_42mm_poke_speed.out<-4200 && abs(general_poke.left_poke.rate_rpm)<20)
				set_time++;
			else
				set_time = 0;
			if(set_time == 2)
				pid_42mm_poke_speed.max_out=2000;
		}

		if(general_poke.left_poke.temperature >= 80)
		{
			stop_bullet_flag = 1;
			over_hot_motor=1;
		}
		if(general_poke.left_poke.temperature <= 60)
			stop_bullet_flag = 0;
				
		switch(friction_state)
		{
			case NORMAL:
			{
				//热量没有问题
				if((over_heat==0)||(ignore_heat==1))
				{
					if((RC_CtrlData.mouse.press_l==1)&&(RC_CtrlData.mouse.last_press_l==0))
					{
						//可以射击并且是吊塔模式时，或者射击模式为正常射击时，进入这个模式
						if((press_l_first_in==1)&&(l_key_delay_time>150))
						{
							//二级拨盘
							target_angle_2 += ONE_POKE_ANGLE_2;                               //目标角度等于原角度减去每一个弹丸发射的角度
							pid_42mm_poke2_angle.set=target_angle_2;        //位置的set值为目标角度
							pid_42mm_poke2_angle.iout=0;                 //积分清零

							press_l_first_in=0;                                                                         
							l_key_delay_time=0;                                          //清空延时
						}
					}
					else
					{
						press_l_first_in=1;                                           
					}
				}
				pid_42mm_poke2_angle.get=general_poke.right_poke.ecd_angle;
				pid_calc(&pid_42mm_poke2_angle, pid_42mm_poke2_angle.get, pid_42mm_poke2_speed.set);
				pid_42mm_poke2_speed.get=general_poke.right_poke.rate_rpm+pid_42mm_poke2_speed.out*k_speed;
				pid_calc(&pid_42mm_poke2_speed, pid_42mm_poke2_speed.get, pid_42mm_poke2_speed.out);
			}
				break;
			case LOCK:            //如果是堵转模式
			{					
			}
				break;
			case Stop:
			{
				pid_clr(&pid_42mm_poke_speed);
				pid_clr(&pid_42mm_poke2_angle);
				pid_clr(&pid_42mm_poke2_speed);
			}
			default:
				break;
		}
	}
	else   //没有开启摩擦轮，且不为初始化时进入——为了不让拨盘动起来
	{
			//二级拨盘
			pid_42mm_poke2_angle.set=target_angle_2;
			pid_42mm_poke2_angle.get=general_poke.right_poke.ecd_angle;
			pid_calc(&pid_42mm_poke2_angle,pid_42mm_poke2_angle.get, pid_42mm_poke2_angle.set);
			pid_42mm_poke2_speed.get=general_poke.right_poke.rate_rpm;
			pid_calc(&pid_42mm_poke2_speed,pid_42mm_poke2_speed.get,pid_42mm_poke2_speed.out);
		
					pid_clr(&pid_42mm_poke_speed);
					pid_clr(&pid_42mm_poke2_speed);
	}
}


//void friction_lock(void)
//{
//	if (!lock_flag)
//	{
//		taget_angle_2-=30;
//		lock_flag=1;
//	}
//	
//	pid_shoot_bullet_position_angle_loop_2.get=BUS1_CM5_Poke_Encoder.rotor_out;
//	pid_calc(&pid_shoot_bullet_position_angle_loop_2, pid_shoot_bullet_position_angle_loop_2.get, pid_shoot_bullet_position_angle_loop_2.set); //2006
//	pid_shoot_bullet_position_speed_loop_2.get=BUS1_CM5_Poke_Encoder.rotor_speed;//+pid_shoot_bullet_position_speed_loop_2.out*k_speed;
//	pid_calc(&pid_shoot_bullet_position_speed_loop_2, pid_shoot_bullet_position_speed_loop_2.get, pid_shoot_bullet_position_angle_loop_2.out);

//	pid_calc(&pid_shoot_bullet_position_speed_loop,BUS2_CM5_Poke_Encoder.rate_rpm,LOCK_SPEED);

//	if(GetInputMode() == STOP)
//	{
//		Set_GM_CM_Current(CAN2,0,0,0,0);
//		Set_GM_CM_Current(CAN1,0,0,0,0);
//	}
//	else
//	{
//		Set_GM_CM_Current(CAN2,pid_shoot_bullet_position_speed_loop.out,0,0,0);
//		Set_GM_CM_Current(CAN1,pid_shoot_bullet_position_speed_loop_2.out,0,0,aim_scope_speed.out);
//	}
//}

//PID参数初始化
void shot_param_init(void)
{
//		Set_GM_CM_Current(CAN1,0,0,0,0);

  PID_struct_init(&pid_friction_whell_speed[0], POSITION_PID,15000,1000, 0, 70 , 0.01f ,100);//两个摩擦轮
  PID_struct_init(&pid_friction_whell_speed[1], POSITION_PID,15000,1000, 0, 72 , 0.01f ,10);

  PID_struct_init(&pid_42mm_poke_speed,   POSITION_PID, 6000 , 13000, 0, 5  , 0.5, 0 );//下拨盘速度环
	PID_struct_init(&pid_42mm_poke2_angle, POSITION_PID, 2000 , 0    , 0, 120, 5  , 10); //二级拨盘
  PID_struct_init(&pid_42mm_poke2_speed, POSITION_PID, 9900 , 5500 , 0, 20 , 0  , 0 );
}
