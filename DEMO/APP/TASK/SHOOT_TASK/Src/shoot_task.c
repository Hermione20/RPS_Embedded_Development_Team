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
float target_angle_17=0;	//拨盘角度
float target_angle_2=0;	//二级拨盘角度

//17mm
uint32_t start_shooting_count = 0;//正转计时
uint32_t start_lock_rotor_count = 0;//堵转计时
uint32_t start_reversal_count = 0;//反转计时
float shot_limit_heart0 = 0;		//可用热量

shoot_mode_selection_e shoot_mode_selection;//步兵发射模式
float shoot_frequency;				//步兵射频
int will_num_shoot;			//步兵可用发射量
int shoot_lock_time;		//堵转计时
int click_time;				//发射计时
int will_time_shoot;		//射击时间
/********************** Flag **********************/
int over_heat=0;				//超热量标志位
int position_flag=0;		//初始化完成标志位
friction_state_t friction_state;//摩擦轮状态标志位

int stop_friction_flag=0;//摩擦轮停止标志位
int over_hot_motor=0;		//摩擦轮电机过热

int stop_bullet_flag=0;	//拨盘停止标志位
shoot_state_e shoot_state={0};
int press_l_first_in=0; //0初次进入1非初次进入
int ignore_heat=0;		//忽略量限制标志位
int lock_flag_1times=0;	//反转程序角度控制标志位
int shoot_flag=0;				//42开火

u8 bulletspead_level=0;		//步兵射频相关
int have_heat0_flag=0;		//余量充裕标志位
int lock_rotor=0;
int shot_ctrl_mode;				//发射控制模式
int BUFF_shoot_flag;			//打符标志位
int single_shoot=1;				//单发标志位
/******************** FUNCTION ********************/
//发射任务总成
void shoot_task(void)
{
	#if TYPE==1		//42mm
		Shoot_42mm_speed_Select();//42mm弹速选择
		heat1_limit_42mm();				//热量限制
		shoot_friction_handle_42();	//摩擦轮部分
		shoot_bullet_handle_42();		//拨盘部分
	#elif	TYPE==2		//17mm
		Shoot_17mm_speed_Select();
		heat1_limit_17mm();				
		Heat0_switch();						//拨盘发射水平
		Mode_switch();						//发射模式选择
		shoot_friction_handle_17();	//摩擦轮部分
		shoot_bullet_handle_17();		//拨盘
	#elif	TYPE==3		//17mm x2
		Shoot_17mm_speed_Select();					//17mm弹速选择
		heat1_limit_17mm();									//热量限制
		shoot_friction_handle_17();					//摩擦轮部分
	#endif
}

//42mm
static void Shoot_42mm_speed_Select(void)//42mm弹速//读裁判系统弹速上限进行选择
{
  if(judge_rece_mesg.game_robot_state.shooter_id1_42mm_speed_limit==10)
    frictionSpeed_42=FRICTION_SPEED_10;
  else if(judge_rece_mesg.game_robot_state.shooter_id1_42mm_speed_limit==16)
    frictionSpeed_42=FRICTION_SPEED_16;
	else
		frictionSpeed_42=FRICTION_SPEED_10;
}

void heat1_limit_42mm(void)//42mm热量限制
{
//-----------剩余热量计算(最大-累计)----------------
//-----------服务器模式---------------------------
	float residue_heart;						//剩余可用热量
  residue_heart=(judge_rece_mesg.game_robot_state.shooter_id1_42mm_cooling_limit           //获取相关的资料
                       -judge_rece_mesg.power_heat_data.shooter_id1_42mm_cooling_heat);
  if(residue_heart>=100)
      over_heat=0;
  else
      over_heat=1;

	if (RC_CtrlData.Key_Flag.Key_R_Flag)
		ignore_heat=1;		//忽略热量限制
}

void shoot_friction_handle_42(void)//42mm摩擦轮
{
	if (RC_CtrlData.inputmode != STOP)
	{
		if(RC_CtrlData.RemoteSwitch.s3to1 || RC_CtrlData.Key_Flag.Key_C_TFlag)
		{
			pid_friction_whell_speed[0].set=-frictionSpeed_42;
			pid_friction_whell_speed[1].set= frictionSpeed_42;
			//如果当速度达到要求，摩擦轮正常标志位置1
			if ((general_friction.right_motor1.rate_rpm < -30)&&(general_friction.left_motor1.rate_rpm > 30))
			{
				friction_rotor_may_lock=0;
				friction_state = NORMAL;
			}
			else
			{
				friction_rotor_may_lock++;
				if(friction_rotor_may_lock==100)
				{
					friction_state = LOCK;
					friction_rotor_may_lock=0;
				}
			}
		}
		if (RC_CtrlData.Key_Flag.Key_Z_Flag)
		{
			pid_friction_whell_speed[0].set= frictionSpeed_42;
			pid_friction_whell_speed[1].set=-frictionSpeed_42;
				friction_state = LOCK;
		}
		if ((RC_CtrlData.RemoteSwitch.s3to1==0 && RC_CtrlData.Key_Flag.Key_C_TFlag==0)
					|| stop_friction_flag)
		{
			pid_friction_whell_speed[0].set= 0;
			pid_friction_whell_speed[1].set= 0;
				friction_state = Stop;
			pid_clr(&pid_friction_whell_speed[0]);
			pid_clr(&pid_friction_whell_speed[1]);

		}
	}
	else
	{
		pid_friction_whell_speed[0].set= 0;
		pid_friction_whell_speed[1].set= 0;
			friction_state = Stop;
		position_flag=0;	//重新初始化
		pid_clr(&pid_friction_whell_speed[0]);
		pid_clr(&pid_friction_whell_speed[1]);
	}
			
	//摩擦轮均值滤波
		filter_rate_readtime++;
		filter_rate_left[0]	+=general_friction.left_motor1.rate_rpm;
		filter_rate_right[0]+=general_friction.right_motor1.rate_rpm;

		if(filter_rate_readtime>=friction_rotor_DIV)//分别获得速度的平均值
		{
			pid_friction_whell_speed[1].get=filter_rate_left[0]/friction_rotor_DIV;
			pid_friction_whell_speed[0].get=filter_rate_right[0]/friction_rotor_DIV;
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
			friction_state = Stop;
		}
}

void shoot_bullet_handle_42(void)//42mm拨盘
{
	switch(friction_state)
		{
			case NORMAL:
			{
				//热量没有问题
				if((over_heat==1)||(ignore_heat==1)||(stop_bullet_flag==1))
				{
					if(RC_CtrlData.mouse.press_l==1||RC_CtrlData.RemoteSwitch.trigger==1)
					{
						//可以射击并且是吊塔模式时，或者射击模式为正常射击时，进入这个模式
							if (press_l_first_in==0)
							{
								press_l_first_in=1;
								shoot_flag=1;
								//二级拨盘
								target_angle_2 += ONE_POKE_ANGLE_42;                               //目标角度等于原角度减去每一个弹丸发射的角度
								pid_42mm_poke2_angle.set=target_angle_2;        //位置的set值为目标角度
								pid_42mm_poke2_angle.iout=0;                 //积分清零
							}
							else
							{shoot_flag=0;}
					}
					else
					{
						press_l_first_in=0;                                           
					}
				}
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
		
		
	if(!position_flag)            //初始化的时候进入
	{
			target_angle_2 = general_poke.right_poke.ecd_angle/36.0f;
			position_flag=1;
			pid_clr(&pid_42mm_poke2_angle);
			pid_clr(&pid_42mm_poke2_speed);
			pid_42mm_poke_speed.max_out = 9000;
	}
	else if(friction_state==NORMAL)
    //如果	摩擦轮开启按键按下	&& 电机达到一定的转速 && 初始化标志位为1	才可以发射
	{
		pid_calc(&pid_42mm_poke_speed,general_poke.left_poke.rate_rpm,POKE_SPEED);
		
	//防止电机过热，下拨盘力度限制
		if(shoot_flag==1)
			pid_42mm_poke_speed.max_out=POKE_MAX_OUT;
		else
		{
			if(pid_42mm_poke_speed.out<-4200 && abs(general_poke.left_poke.rate_rpm)<20)
				set_time++;
			else
				set_time = 0;
			if(set_time == 20)
				pid_42mm_poke_speed.max_out=2000;
		}

		if(general_poke.left_poke.temperature >= 80)
		{
			stop_bullet_flag = 1;
			over_hot_motor=1;
		}
		if(general_poke.left_poke.temperature <= 60)
			stop_bullet_flag = 0;
		
				pid_42mm_poke2_angle.get=(general_poke.right_poke.ecd_angle)/36.0f;
				pid_calc(&pid_42mm_poke2_angle, pid_42mm_poke2_angle.get, pid_42mm_poke2_angle.set);
				pid_42mm_poke2_speed.get=general_poke.right_poke.rate_rpm;
				pid_calc(&pid_42mm_poke2_speed, pid_42mm_poke2_speed.get, pid_42mm_poke2_angle.out);
		
	}
	else   //没有开启摩擦轮，且不为初始化时进入——为了不让拨盘动起来
	{
			//二级拨盘
			pid_42mm_poke2_angle.set=target_angle_2;
			pid_42mm_poke2_angle.get=general_poke.right_poke.ecd_angle;
		
					pid_clr(&pid_42mm_poke_speed);
					pid_clr(&pid_42mm_poke2_speed);
	}
}

//17mm
static void Shoot_17mm_speed_Select(void)//17mm弹速//读裁判系统弹速上限进行选择
{																					
  if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==10)
    frictionSpeed_17=FRICTION_SPEED_15;
  else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==16)
    frictionSpeed_17=FRICTION_SPEED_18;
	else if (judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==30)
		frictionSpeed_17=FRICTION_SPEED_30;
	else
		frictionSpeed_17=FRICTION_SPEED_15;
}

void heat1_limit_17mm(void)//17mm热量限制
{
	float shot_limit_heart0;			//剩余可用热量
	float will_num_shoot;			//剩余发射量
//-----------剩余热量计算(最大-累计)----------------
//-----------服务器模式---------------------------
  shot_limit_heart0=(judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit           //获取相关的资料
                       -judge_rece_mesg.power_heat_data.shooter_id1_17mm_cooling_heat);
  will_num_shoot=shot_limit_heart0/10;
	
	if(shot_limit_heart0 > 15)
    shot_ctrl_mode=1;
  else
	{
		shot_ctrl_mode=0;
	}
}

void Mode_switch(void)//弹速限制来改变不同的模式 
{
  if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==30)          //弹速限制在30为速度模式
	{shoot_mode_selection=speed;}
  else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==15&judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit==50)    //弹速限制为15，但是冷却限制为50
	{ shoot_mode_selection=cooling;}
  else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==15&judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit==150)   //弹速限制为15，但是冷却限制为150
	{ shoot_mode_selection=outburst;}
}

void Heat0_switch(void)
{
	//剩余发弹量=（热量上限-热量值）/10      ————一个弹丸的热量为10
  will_num_shoot=(judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit-judge_rece_mesg.power_heat_data.shooter_id1_17mm_cooling_heat)/10;    //剩余发射量
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
					shoot_frequency=1.5*shoot_frequency ;
				}
		#endif				
}

void shoot_friction_handle_17(void)//17mm摩擦轮
{
	if (RC_CtrlData.inputmode != STOP)
	{
		if(RC_CtrlData.RemoteSwitch.s3to1 || RC_CtrlData.Key_Flag.Key_C_TFlag)
		{
			pid_friction_whell_speed[0].set= frictionSpeed_17;
			pid_friction_whell_speed[1].set=-frictionSpeed_17;
			//如果当速度达到要求，摩擦轮正常标志位置1
			if ((general_friction.left_motor1.rate_rpm < -30)&&(general_friction.right_motor1.rate_rpm > 30))
			{
				friction_rotor_may_lock=0;
				friction_state = NORMAL;
			}
			else
			{
				friction_rotor_may_lock++;
				if(friction_rotor_may_lock==100)
				{
					friction_state = LOCK;
					friction_rotor_may_lock=0;
				}
			}
		}
		if (RC_CtrlData.Key_Flag.Key_Z_Flag)
		{
			pid_friction_whell_speed[0].set=-frictionSpeed_17;
			pid_friction_whell_speed[1].set= frictionSpeed_17;
				friction_state = LOCK;
			pid_clr(&pid_friction_whell_speed[0]);
			pid_clr(&pid_friction_whell_speed[1]);

		}
		if ((RC_CtrlData.RemoteSwitch.s3to1==0 && RC_CtrlData.Key_Flag.Key_C_TFlag==0))
		{
			pid_friction_whell_speed[0].set= 0;
			pid_friction_whell_speed[1].set= 0;
				friction_state = Stop;
			pid_clr(&pid_friction_whell_speed[0]);
			pid_clr(&pid_friction_whell_speed[1]);

		}
	}
	else
	{
		pid_friction_whell_speed[0].set= 0;
		pid_friction_whell_speed[1].set= 0;
			friction_state = Stop;
		position_flag=0;	//重新初始化
		pid_clr(&pid_friction_whell_speed[0]);
		pid_clr(&pid_friction_whell_speed[1]);
	}
			
		//摩擦轮均值滤波
		filter_rate_readtime++;
		filter_rate_left[0]	+=general_friction.left_motor1.rate_rpm;
		filter_rate_right[0]+=general_friction.right_motor1.rate_rpm;

		if(filter_rate_readtime>=friction_rotor_DIV)//分别获得速度的平均值
		{
			pid_friction_whell_speed[1].get=filter_rate_left[0]/friction_rotor_DIV;
			pid_friction_whell_speed[0].get=filter_rate_right[0]/friction_rotor_DIV;
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
			friction_state = Stop;
		}
}

int cnt=0;
void shoot_bullet_handle_17(void)//17mm拨盘
{
	switch(friction_state)
	{
			case NORMAL:
			{
				//热量没有问题
				if((over_heat==0)||(ignore_heat==1)||(stop_bullet_flag==1))
				{
						if(RC_CtrlData.mouse.press_l==1||RC_CtrlData.RemoteSwitch.trigger==1)
						{
							if(single_shoot==1)//单发
							{
							//可以射击并且是吊塔模式时，或者射击模式为正常射击时，进入这个模式
								if (press_l_first_in==0)
								{
									press_l_first_in=1;
									shoot_flag=1;
									//二级拨盘
									target_angle_17 += ONE_POKE_ANGLE_17;                               //目标角度等于原角度减去每一个弹丸发射的角度
									pid_17mm_poke_angle.set=target_angle_17;        //位置的set值为目标角度
									pid_17mm_poke_angle.iout=0;                 //积分清零
								}
								else
								{shoot_flag=0;}
							}
							else
							{
								cnt++;
								if (cnt%50==0)
								{
								target_angle_17 += ONE_POKE_ANGLE_17;                               //目标角度等于原角度减去每一个弹丸发射的角度
								pid_17mm_poke_angle.set=target_angle_17;        //位置的set值为目标角度
								pid_17mm_poke_angle.iout=0;                 //积分清零
								}
							}
						}
						else
						{
							press_l_first_in=0;                                           
						}
					}
				}
				break;
			case LOCK:            //如果是堵转模式
			{
				
			}
				break;
			case Stop:
			{
				pid_clr(&pid_17mm_poke_angle);
				pid_clr(&pid_17mm_poke_speed);
			}
			default:
				break;
		}
	if(!position_flag)            //初始化的时候进入
	{
			target_angle_17 = general_poke.left_poke.ecd_angle/36.0f;
			position_flag=1;
			pid_clr(&pid_17mm_poke_angle);
			pid_clr(&pid_17mm_poke_speed);
	}
	else if(friction_state==NORMAL)
    //如果	摩擦轮开启按键按下	&& 电机达到一定的转速 && 初始化标志位为1	才可以发射
	{
		if(general_poke.left_poke.temperature >= 80)
		{
			stop_bullet_flag = 1;
			over_hot_motor=1;
		}
		if(general_poke.left_poke.temperature <= 60)
			stop_bullet_flag = 0;
		
				pid_17mm_poke_angle.get=(general_poke.left_poke.ecd_angle)/36.0f;
				pid_calc(&pid_17mm_poke_angle, pid_17mm_poke_angle.get, pid_17mm_poke_angle.set);
				pid_17mm_poke_speed.get=general_poke.left_poke.rate_rpm;
				pid_calc(&pid_17mm_poke_speed, pid_17mm_poke_speed.get, pid_17mm_poke_angle.out);
	}
	else   //没有开启摩擦轮，且不为初始化时进入——为了不让拨盘动起来
	{
			pid_17mm_poke_angle.set=target_angle_17;
			pid_17mm_poke_angle.get=general_poke.left_poke.ecd_angle;
		
					pid_clr(&pid_17mm_poke_speed);
					pid_clr(&pid_17mm_poke_angle);
	}
}

void friction_lock(void)
{
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
}

//PID参数初始化
void shot_param_init(void)
{
#if TYPE ==1
//42mm摩擦轮
  PID_struct_init(&pid_friction_whell_speed[0], POSITION_PID,10000,1000, 0, 70 , 0.01f ,100);//两个摩擦轮
  PID_struct_init(&pid_friction_whell_speed[1], POSITION_PID,10000,1000, 0, 72 , 0.01f ,10);
#endif
#if TYPE ==2
//17mm摩擦轮
  PID_struct_init(&pid_friction_whell_speed[0], POSITION_PID,10000,1000, 0, 8 , 0 ,0);//两个摩擦轮
  PID_struct_init(&pid_friction_whell_speed[1], POSITION_PID,10000,1000, 0, 8 , 0 ,0);
#endif
	//17mm摩擦轮2号
  PID_struct_init(&pid_friction_whell_speed[2], POSITION_PID,15000,1000, 0, 70 , 0.01f ,100);//两个摩擦轮
  PID_struct_init(&pid_friction_whell_speed[3], POSITION_PID,15000,1000, 0, 72 , 0.01f ,10);
//42mm拨盘
  PID_struct_init(&pid_42mm_poke_speed,   POSITION_PID, 6000 , 13000, 0, 5  , 0.5, 0 );//下拨盘速度环
	PID_struct_init(&pid_42mm_poke2_angle, POSITION_PID, 2000 , 0    , 0, 120, 5  , 10); //二级拨盘
  PID_struct_init(&pid_42mm_poke2_speed, POSITION_PID, 9900 , 5500 , 0, 20 , 0  , 0 );
//17mm拨盘
	PID_struct_init(&pid_17mm_poke_angle, POSITION_PID, 5000, 1000,0,5,0.2,15);
	PID_struct_init(&pid_17mm_poke_speed, POSITION_PID,15000,10000,0,50,0.1,4);
//打符模式
	PID_struct_init(&pid_17mm_poke_angle_buf, POSITION_PID, 6000, 1000,0,5,0.2,5);
	PID_struct_init(&pid_17mm_poke_speed_buf, POSITION_PID,19000,10000,0,300,0.3,4);

}
