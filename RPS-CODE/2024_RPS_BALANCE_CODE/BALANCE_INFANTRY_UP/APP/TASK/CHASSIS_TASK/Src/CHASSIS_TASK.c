/* Includes ------------------------------------------------------------------*/
#include "CHASSIS_TASK.h"
/*----------------------------------------------------------------------------*/


/* Variables_definination-----------------------------------------------------------------------------------------------*/
 

Chassis_angle_t 	 Chassis_angle;
chassis_t 		 		 chassis;

int   yaw_num_get,run_flag;
double yaw_ecd_angle;
int16_t vx,vy;
u8 chassis_rotate_flag;
int getnumb[4];
float getlastangle1;
float getlastangle2;
float getlastangle3;
float getlastangle4;
float transition1[4];
float transition2[4];
float transition3[4];
float transition4[4];

float retransition1_angle[4];
float retransition2[4];
float retransition3[4];
float retransition4[4];

int yaw_num_get;
float angle_flag[4];
float deviation_angle_flag[4];

float volatilAo;
u16 gyro_speed=400;
u8 get_speedw_flag;

u8      cap_flag;
int16_t cap_flag_time;
u16  Max_Power;
u16  Max_Power_3508;
u16  Max_Power_6020;
float power_limit_start_time=850;
float power_limit_start_flag=1;
float power_limit_rate2;
float power_limit_rate1;
float  max_chassis_power =35;
float I6020[4],sdpower,shpower,dpower[4],hpower[4],all_power1[4],all_power2[4],
	all_power,a6020[4],b6020[4],m6020,n6020,l6020,i6020,kall6020[4];

float k6020=-0.000075,k2_6020=1.278e-7,k1_6020=6.157e-6,k0_6020=1.054;
/*----------------------------------------------------------------------------------------------------------------------*/

void chassis_param_init()//底盘参数初始化
{

  memset(&chassis, 0, sizeof(chassis_t));
  chassis.ctrl_mode      = CHASSIS_STOP;
  chassis.last_ctrl_mode = CHASSIS_RELAX;
	
  chassis.position_ref = 0;
  chassis_rotate_flag = 0;
 
	  
		PID_struct_init(&pid_cha_6020_angle[0], POSITION_PID, 8000, 10, 8,0.1f,5);//24, 0.2f,20);
  	PID_struct_init(&pid_cha_6020_speed[0], POSITION_PID, 15000, 500, 210,0.1f,10);//38,0.5f,20);
	
		PID_struct_init(&pid_cha_6020_angle[1], POSITION_PID, 8000, 10, 8,0.1f,5);//25, 0.2f,15);
  	PID_struct_init(&pid_cha_6020_speed[1], POSITION_PID, 15000, 500, 180,0.1f,4);//39,0.5f,20);
	
	  PID_struct_init(&pid_cha_6020_angle[2], POSITION_PID, 8000, 10, 8,0.1f,4);//20, 0.2f,20);
  	PID_struct_init(&pid_cha_6020_speed[2], POSITION_PID, 15000, 500, 200,0.5,8);//40,0.5f,20);
		
    PID_struct_init(&pid_cha_6020_angle[3], POSITION_PID, 8000, 10, 9,0.4f,4);//23, 0.2f,15);
  	PID_struct_init(&pid_cha_6020_speed[3], POSITION_PID, 15000, 500, 200,0.1f,15);//42,0.5f,20);

#if STANDARD == 1
		for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_cha_3508_speed[k], POSITION_PID,16000,100,50,0.2,20); //24 0.3 10    38.0f,3.0f, 40
    }
			PID_struct_init(&pid_chassis_angle, POSITION_PID, 450, 30,  2.0,0.0f,3.0f);
#endif
#if STANDARD == 3
  for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_cha_3508_speed[k], POSITION_PID,15000, 15000,24,0.3, 10); //24 0.3 10    38.0f,3.0f, 40
    }
  PID_struct_init(&pid_chassis_angle, POSITION_PID, 500, 0, 300,0,500);//xisanhao
#elif STANDARD == 4
  for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_spd[k], POSITION_PID,15000, 15000,25,0.4, 0); //24 0.3 10    38.0f,3.0f, 40
    }
    PID_struct_init(&pid_chassis_angle, POSITION_PID, 500, 0, 300,0,500);//xisanhao
	#elif STANDARD == 5
  for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_spd[k], POSITION_PID,15000, 15000,22,0.3f, 0.5f); //24 0.3 10    38.0f,3.0f, 40
    }
  PID_struct_init(&pid_chassis_angle, POSITION_PID, 500, 0, 300,0,500);//xisanhao
#elif STANDARD == 6
  for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_spd[k], POSITION_PID,15000, 15000,25,0.1, 5); //24 0.3 10    38.0f,3.0f, 40
    }
  PID_struct_init(&pid_chassis_angle, POSITION_PID, 500, 0, 300,0,500);//xisanhao
#endif
}


/**
************************************************************************************************************************
* @Name     : power_limit_handle
* @brief    : 功率控制的主函数
* @param		: None
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/
void power_limit_handle()
{
		volatilAo=capacitance_message.out_v;
		capacitance_message.cap_voltage_filte=volatilAo/100;
		max_chassis_power=get_max_power2(capacitance_message.cap_voltage_filte);

		get_6020power();
		
		if(cap_flag==1||cap_flag==3)//cap_flag==2
		power_limit_rate1=get_the_limite_rate(get_max_power1(capacitance_message.cap_voltage_filte)-Max_Power_6020);
		else
		power_limit_rate2=get_the_limite_rate(get_max_power2(capacitance_message.cap_voltage_filte)-Max_Power_6020);
		
		VAL_LIMIT(power_limit_rate1,0,1);
		VAL_LIMIT(power_limit_rate2,0,1);		 


		if(cap_flag==3||cap_flag==2)
		{
			power_limit_start_flag=1;
		}
		else if(cap_flag==1)
		{			
				if(run_flag ==0)
				{
					power_limit_start_time=850;
				}
				else if(run_flag ==1&&power_limit_start_time>0)
				{
					power_limit_start_time--;
				}
				
				power_limit_start_flag=(1000-power_limit_start_time)/1000;
	  }
    else if(cap_flag==0)
    {		
			if(run_flag ==0)
				{
					power_limit_start_time=judge_rece_mesg.game_robot_state.chassis_power_limit*0.8;
				}
				else if(run_flag ==1&&power_limit_start_time>0)
				{
					power_limit_start_time--;
				}		
				power_limit_start_flag=(judge_rece_mesg.game_robot_state.chassis_power_limit-power_limit_start_time)/judge_rece_mesg.game_robot_state.chassis_power_limit;
		}
			
}

/**
************************************************************************************************************************
* @Name     : cap_limit_mode_switch
* @brief    : canbus处进行电容模式的即时更新
* @param    : none
* @retval   : void
* @Note     : 1->0 , 3->2
************************************************************************************************************************
**/
void cap_limit_mode_switch()
{       
	if(can_chassis_data.speed_mode==0||can_chassis_data.speed_mode==1)
		{
				cap_flag=0;
				if(can_chassis_data.speed_mode==1)
				{
					cap_flag=1;
					cap_flag_time=0;
				}
				if(cap_flag==1)
				{
					cap_flag_time++;
				}
				if(cap_flag_time>=10)
				{
					cap_flag=0;
					cap_flag_time=0;
				}
	 }
	 else
	 {
				cap_flag=2;
				if(can_chassis_data.speed_mode==3)
				{
					cap_flag=3;
					cap_flag_time=0;
				}
				if(cap_flag==3)
				{
					cap_flag_time++;
				}
				if(cap_flag_time>=10)
				{
					cap_flag=2;
					cap_flag_time=0;
				}
	 }
 }

float get_max_power1(float voltage)
{
    int max_power=0;
	  if(cap_flag==3)
		{
			if(voltage>WARNING_VOLTAGE+4)
				max_power=450;
			else
				max_power=(voltage-WARNING_VOLTAGE)/4.0f*450;
		}
	  else
		{
			if(voltage>WARNING_VOLTAGE+4)
				max_power=250;
			else
				max_power=(voltage-WARNING_VOLTAGE)/4.0f*250;			
		}
			VAL_LIMIT(max_power,0,500);
			power_limit_rate2=1;
    return max_power;
}

float get_max_power2(float voltage)
{
	int max_power=0;
		max_power=judge_rece_mesg.game_robot_state.chassis_power_limit+
(judge_rece_mesg.power_heat_data.chassis_power_buffer-5)*2;

		VAL_LIMIT(max_power,0,350);
		power_limit_rate1=1;
    return max_power;
}

static float get_the_limite_rate(float max_power)
{
  float a[4];
  for(int i=0; i<4; i++)
    a[i]=(float)Chassis_angle.handle_speed_lim[i]*(pid_spd[i].p+pid_spd[i].d);
  float b[4];
  for(int i=0; i<4; i++)
    b[i]=-pid_cha_3508_speed[i].p*(float)chassis.cha_pid_3508.speed_fdb[i]+pid_cha_3508_speed[i].iout \
         -pid_cha_3508_speed[i].d*(float)chassis.cha_pid_3508.speed_fdb[i]-pid_cha_3508_speed[i].d*pid_cha_3508_speed[i].err[LAST];
  // Max_power=heat_power+drive_power
  //	i_n=a[n]*k+b[n]	带入
  //Max_Power=m*k^2+n*k+o
  //0=m*k^2+n*k+l(l=o-Max_Power)
  float m=(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]+a[3]*a[3])*FACTOR_2;

  float n=2*FACTOR_2*(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]) + \
          FACTOR_1*(a[0] + a[1] + a[2] + a[3]) + \
          I_TIMES_V_TO_WATT*(a[0]*(float)chassis.cha_pid_3508.speed_fdb[0] + \
                             a[1]*(float)chassis.cha_pid_3508.speed_fdb[1] + \
                             a[2]*(float)chassis.cha_pid_3508.speed_fdb[2] + \
                             a[3]*(float)chassis.cha_pid_3508.speed_fdb[3]);

  float l=(b[0]*b[0] + b[1]*b[1] + b[2]*b[2] + b[3]*b[3])*FACTOR_2 + \
          (b[0] + b[1] + b[2] + b[3])*FACTOR_1 + \
          I_TIMES_V_TO_WATT*(b[0]*(float)chassis.cha_pid_3508.speed_fdb[0] + \
                             b[1]*(float)chassis.cha_pid_3508.speed_fdb[1] + \
                             b[2]*(float)chassis.cha_pid_3508.speed_fdb[2] + \
                             b[3]*(float)chassis.cha_pid_3508.speed_fdb[3]) + \
          4*FACTOR_0 - \
          max_power;
  return (-n+(float)sqrt((double)(n*n-4*m*l)+1.0f))/(2*m);
}

float get_6020power()
{
	for(int i=0;i<4;i++)
	{
	I6020[i]=pid_cha_6020_speed[i].out;
	VAL_LIMIT(I6020[i],-10000,10000);
	dpower[i]=I6020[i]*pid_cha_6020_speed[i].get*k6020;
	hpower[i]=k2_6020*I6020[i]*I6020[i]+k1_6020*I6020[i]+k0_6020;
	all_power1[i]=dpower[i]+hpower[i];
	}
	all_power=all_power1[0]+all_power1[1]+all_power1[2]+all_power1[3];
	VAL_LIMIT(all_power,0,1000);
	Max_Power_6020=all_power;
	VAL_LIMIT(Max_Power_6020,0,max_chassis_power);
	Max_Power_3508=Max_Power-Max_Power_6020;

float a[4],b[4],c[4];
if(all_power>max_chassis_power)
{	
	for(int i = 0; i < 4; i++)
	{
		 all_power2[i]=all_power1[i]*max_chassis_power/all_power;
	if(all_power1[i]>k0_6020&&all_power1[i]>all_power2[i])//或许all_power2>k0_6020更好？
		{
			a[i]=k2_6020;
			b[i]=k1_6020+k6020*pid_cha_6020_speed[i].get;
			c[i]=k0_6020-all_power2[i];
			kall6020[i]=(-b[i]+(double)sqrt(b[i]*b[i]-4*a[i]*c[i]))/(2*a[i])/I6020[i];
		}else
		{kall6020[i]=1;}
	}
}else
	{
		for (int i = 0; i < 4; i++)
		kall6020[i]=1;
	}

for (int i = 0; i < 4; i++)
		VAL_LIMIT(kall6020[i],0,1);
}

/**
************************************************************************************************************************
* @Name     : limit_angle_to_0_2pi
* @brief    : 将角度限制在0~2pi内
* @param    : angle 类型 float
* @retval   : angle 类型 float
* @Note     : 
************************************************************************************************************************
**/
float limit_angle_to_0_2pi(float angle)
{
		if (angle>=2*PI)angle -= 2*PI;
		else if(angle<0)angle += 2*PI;
	
	return angle;
}


/**
************************************************************************************************************************
* @Name     : convert_ecd_angle_to_0_2pi
* @brief    : 将电机编码器的机械角度值（范围正负无穷大）解算为范围在0~2pi的角度值      
* @param		: ecd_angle 电机编码器的机械角度值  类型  double
* @param		: _0_2pi_angle 范围在0~2pi的角度值  类型  float
* @retval   : _0_2pi_angle 范围在0~2pi的角度值  类型  float
* @Note     : 
************************************************************************************************************************
**/
double convert_ecd_angle_to_0_2pi(double ecd_angle,float _0_2pi_angle)
{
	_0_2pi_angle=fmod(YAW_POLARITY*ecd_angle*ANGLE_TO_RAD,2*PI);	
	if(_0_2pi_angle<0)
		 _0_2pi_angle+=2*PI;

	return _0_2pi_angle;
}

/**
************************************************************************************************************************
* @Name     : chassis_mode_select
* @brief    : 将上位机发送的命令经选择后具体实行
* @param		: none
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/
void chassis_mode_select(void)
{

#if 	 CHASSIS_TYPE == 1//舵轮	
	chassis.ctrl_mode=can_chassis_data.chassis_mode;
	chassis.last_ctrl_mode=can_chassis_data.chassis_mode;
#endif

	switch (chassis.ctrl_mode)
    {

    case CHASSIS_STOP:
    {
				chassis_stop_handle();
    }
    break;

    case MANUAL_FOLLOW_GIMBAL:   //跟随云台模式
    {
				follow_gimbal_handle();
    }
    break;
    case CHASSIS_ROTATE:         //小陀螺
    {
				rotate_follow_gimbal_handle();
    }
    break;
    case CHASSIS_REVERSE:
    {
				reverse_follow_gimbal_handle();
    }
    break;
    case CHASSIS_SEPARATE:
    {
				separate_gimbal_handle();
    }
    break;
    default:
    {
				chassis_stop_handle();
    }
    break;
    }
	
}

void chassis_stop_handle(void)
{
  chassis.vy = 0;
  chassis.vx = 0;
	chassis.vw = 0;
  Chassis_angle.get_speedw = 0;
}


//舵轮
#if CHASSIS_TYPE == 1
void follow_gimbal_handle(void)
{
		if(Chassis_angle.yaw_angle_0_2pi>=PI)
		{Chassis_angle.yaw_angle__pi_pi=Chassis_angle.yaw_angle_0_2pi-(2*PI);}
		else
		{Chassis_angle.yaw_angle__pi_pi=Chassis_angle.yaw_angle_0_2pi;}

	 chassis.vy = can_chassis_data.y;
   chassis.vx = can_chassis_data.x;
   Chassis_angle.get_speedw = pid_calc(&pid_chassis_angle,Chassis_angle.yaw_angle__pi_pi,0); 
}
//麦轮
#elif CHASSIS_TYPE == 2
void follow_gimbal_handle(void)
{
	
	if(Chassis_angle.yaw_angle_0_2pi>=PI)
		{Chassis_angle.yaw_angle__pi_pi=Chassis_angle.yaw_angle_0_2pi-(2*PI);}
		else
		{Chassis_angle.yaw_angle__pi_pi=Chassis_angle.yaw_angle_0_2pi;}
			
	chassis.vy = chassis.ChassisSpeed_Ref.left_right_ref;
  chassis.vx = chassis.ChassisSpeed_Ref.forward_back_ref;
	chassis.vw = pid_calc(&pid_chassis_angle,Chassis_angle.yaw_angle__pi_pi*RAD_TO_ANGLE,0);

}

#endif


//舵轮
#if CHASSIS_TYPE == 1
void separate_gimbal_handle(void)
{
  chassis.vy = can_chassis_data.y;
  chassis.vx = can_chassis_data.x;
	Chassis_angle.get_speedw = 0;
}
//麦轮
#elif CHASSIS_TYPE == 2
void separate_gimbal_handle(void)
{
	chassis.vy = chassis.ChassisSpeed_Ref.left_right_ref;
  chassis.vx = chassis.ChassisSpeed_Ref.forward_back_ref;
	chassis.vw = 0;
}
#endif

//舵轮
#if CHASSIS_TYPE == 1
void rotate_follow_gimbal_handle(void)
{

		if(can_chassis_data.speed_mode==1)
		{
			gyro_speed=550;
		}
		else
		{
			gyro_speed=400;
		}

  chassis.vy = can_chassis_data.y;
  chassis.vx = can_chassis_data.x;

if(chassis.last_ctrl_mode!=CHASSIS_ROTATE)
{
	if(get_speedw_flag==0)
	{get_speedw_flag=1;}
	else if(get_speedw_flag==1)
	{get_speedw_flag=0;}
}

if(get_speedw_flag==0)
{Chassis_angle.get_speedw = -gyro_speed;}
else if(get_speedw_flag==1)
{Chassis_angle.get_speedw = gyro_speed;}

}
//麦轮
#elif CHASSIS_TYPE == 2
void rotate_follow_gimbal_handle(void)
{
		if(Chassis_angle.yaw_angle_0_2pi>=PI)
	{Chassis_angle.yaw_angle__pi_pi=Chassis_angle.yaw_angle_0_2pi-(2*PI);}
	else
	{Chassis_angle.yaw_angle__pi_pi=Chassis_angle.yaw_angle_0_2pi;}
	
  chassis.sin_chassis_angle = sinf(Chassis_angle.yaw_angle__pi_pi);
  chassis.cos_chassis_angle = cosf(Chassis_angle.yaw_angle__pi_pi);

  chassis.foward_back_to_foward_back_rotate_speed = chassis.ChassisSpeed_Ref.forward_back_ref*chassis.cos_chassis_angle;
  chassis.foward_back_to_left_right_rotate_speed  = -chassis.ChassisSpeed_Ref.forward_back_ref*chassis.sin_chassis_angle;
  chassis.left_right_to_foward_back_rotate_speed  = chassis.ChassisSpeed_Ref.left_right_ref*chassis.sin_chassis_angle;
  chassis.left_right_to_left_right_rotate_speed   = chassis.ChassisSpeed_Ref.left_right_ref*chassis.cos_chassis_angle;
	
	chassis.vy = chassis.foward_back_to_left_right_rotate_speed  + chassis.left_right_to_left_right_rotate_speed;
	chassis.vx = chassis.foward_back_to_foward_back_rotate_speed + chassis.left_right_to_foward_back_rotate_speed;
	
	chassis.vw = chassis.ChassisSpeed_Ref.rotate_ref;
//出事的话加负号
}
#endif

//舵轮
#if CHASSIS_TYPE == 1
 void reverse_follow_gimbal_handle(void)
{


  chassis.vy = can_chassis_data.y;
  chassis.vx = can_chassis_data.x;

	if(Chassis_angle.yaw_angle_0_2pi>=2*PI)
	{Chassis_angle.yaw_angle__pi_pi=Chassis_angle.yaw_angle_0_2pi-(2*PI);}
	else
	{Chassis_angle.yaw_angle__pi_pi=Chassis_angle.yaw_angle_0_2pi;}
		
   Chassis_angle.get_speedw = pid_calc(&pid_chassis_angle,Chassis_angle.yaw_angle__pi_pi,PI);    
}
//麦轮
#elif CHASSIS_TYPE == 2
 void reverse_follow_gimbal_handle(void)
{}
#endif

	
	
#if CHASSIS_TYPE == 2
void mecanum_calc(float vx, float vy, float vw, int16_t *speed)
{
  int16_t wheel_rpm[4];
  float   max = 0;

	wheel_rpm[0] = (-vx + vy + vw*3.9f)*2;
  wheel_rpm[1] = ( vx + vy + vw*3.9f)*2;
  wheel_rpm[2] = ( vx - vy + vw*3.9f)*2;
  wheel_rpm[3] = (-vx - vy + vw*3.9f)*2;

  for (uint8_t i = 0; i < 4; i++)//找到最大的速度
    {
      if (abs(wheel_rpm[i]) > max)
        max = abs(wheel_rpm[i]);
    }
  if (max > MAX_WHEEL_RPM)
    {
      float rate = MAX_WHEEL_RPM / max;
      for (uint8_t i = 0; i < 4; i++)
        wheel_rpm[i] *= rate;
    }
  memcpy(speed, wheel_rpm, 4*sizeof(int16_t));//内存拷贝函数，rpm一分钟旋转量
}
#endif



/**
************************************************************************************************************************
* @Name     :start_angle_handle
* @brief    :解算出初始角，偏航角，驱动轮速度，以及限制级驱动轮速度
* @param		:None 
* @retval   :void
* @Note     :加入功率限制标志位的驱动轮速度给定 是实际发送的值	
							10.3 考虑transition3为nan的情况
************************************************************************************************************************
**///舵轮
#if CHASSIS_TYPE == 1
void start_angle_handle()
{
	Chassis_angle.start_angle[0] = Chassis_angle.yaw_angle_0_2pi + ((3*PI)/4);
	Chassis_angle.start_angle[1] = Chassis_angle.yaw_angle_0_2pi + ((5*PI)/4);
	Chassis_angle.start_angle[2] = Chassis_angle.yaw_angle_0_2pi + ((7*PI)/4);
	Chassis_angle.start_angle[3] = Chassis_angle.yaw_angle_0_2pi + ((1*PI)/4);
	for(int k=0;k<4;k++)
	{Chassis_angle.start_angle[k]=limit_angle_to_0_2pi(Chassis_angle.start_angle[k]);}

for(int k=0;k<4;k++)
{

	Chassis_angle.include_angle[k] = PI + Chassis_angle.Remote_angle - Chassis_angle.start_angle[k];
	limit_angle_to_0_2pi(Chassis_angle.include_angle[k]);


	Chassis_angle.handle_speed[k] = sqrt(Chassis_angle.get_speedw*Chassis_angle.get_speedw + Chassis_angle.Remote_speed*Chassis_angle.Remote_speed \
																				- 2*Chassis_angle.get_speedw*Chassis_angle.Remote_speed*(cos(Chassis_angle.include_angle[k])));

#if POWER_LIMIT_HANDLE
	Chassis_angle.handle_speed_lim[k] = sqrt(Chassis_angle.get_speedw*Chassis_angle.get_speedw + 
																				(Chassis_angle.Remote_speed*power_limit_start_flag)*(power_limit_start_flag*Chassis_angle.Remote_speed) 
																				- 2*Chassis_angle.get_speedw*(Chassis_angle.Remote_speed*power_limit_start_flag)*(cos(Chassis_angle.include_angle[k])));
#else
	Chassis_angle.handle_speed_lim[k] = sqrt(Chassis_angle.get_speedw*Chassis_angle.get_speedw + Chassis_angle.Remote_speed*Chassis_angle.Remote_speed \
																				- 2*Chassis_angle.get_speedw*Chassis_angle.Remote_speed*(cos(Chassis_angle.include_angle[k])));
#endif

}

for(int k=0;k<4;k++)
{
	transition1[k] = sin(Chassis_angle.include_angle[k]);
	transition2[k] = Chassis_angle.Remote_speed*transition1[k];
	transition3[k] =
	((Chassis_angle.handle_speed[k]==0)?0:((transition2[k])/Chassis_angle.handle_speed[k]));
	VAL_LIMIT(transition3[k],-1,1);
	Chassis_angle.deviation_angle[k] = asin(transition3[k]);
	
	retransition2[k] = Chassis_angle.get_speedw*Chassis_angle.get_speedw+Chassis_angle.handle_speed[k]*Chassis_angle.handle_speed[k];
	retransition3[k] = Chassis_angle.Remote_speed*Chassis_angle.Remote_speed;
	
	if(Chassis_angle.get_speedw==0&&Chassis_angle.handle_speed[k]!=0)
	{
	if(Chassis_angle.include_angle[k]>=0&&Chassis_angle.include_angle[k]<=(PI/2))
	{Chassis_angle.deviation_angle[k] += PI;
	 Chassis_angle.deviation_angle[k] = -Chassis_angle.deviation_angle[k];}

	else if(Chassis_angle.include_angle[k]>=(3*PI/2)&&Chassis_angle.include_angle[k]<=(2*PI))
	{Chassis_angle.deviation_angle[k] += PI;
	 Chassis_angle.deviation_angle[k] = -Chassis_angle.deviation_angle[k];}
	}
	else 
	{
	if(Chassis_angle.get_speedw<0)
	{Chassis_angle.deviation_angle[k] += PI;
	Chassis_angle.deviation_angle[k] = -Chassis_angle.deviation_angle[k];}
	
	if(retransition3[k]>retransition2[k])	//突然需要很大的角速度，那么反向抵达
	{Chassis_angle.deviation_angle[k] += PI;
	Chassis_angle.deviation_angle[k] = -Chassis_angle.deviation_angle[k];}
   }
}
}//麦轮
#elif CHASSIS_TYPE == 2
void start_angle_handle()
{
	mecanum_calc(chassis.vx, chassis.vy, chassis.vw, chassis.cha_pid_3508.speed_ref);
}
#endif


/**
************************************************************************************************************************
* @Name     : start_chassis_6020
* @brief    : 航向轴电机pid速度环/角度环 反馈值的获取，给定值的设置
* @param    : none
* @retval   : void 
* @Note     : 转劣弧的处理
							6020功率的处理可能不完善（l）
************************************************************************************************************************
**/
#if CHASSIS_TYPE == 1
void start_chassis_6020()
{		
	for(int i=0;i<4;i++)
	{
		chassis.cha_pid_6020.angle_fdb[i]=steering_wheel_chassis.Heading_Encoder[i].ecd_angle;
	}
		
	if((Chassis_angle.Remote_speed>50)||
			((Chassis_angle.get_speedw<-5)||
			 (Chassis_angle.get_speedw> 5)))	
	{
		for(int m=0;m<4;m++)
		{
			if((steering_wheel_chassis.Heading_Encoder[m].ecd_angle-getnumb[m]*360)>360)
			{getnumb[m]++;}
			else if
			((steering_wheel_chassis.Heading_Encoder[m].ecd_angle-getnumb[m]*360)<0)
			{getnumb[m]--;}
		}
	}

	if(Chassis_angle.Remote_speed <= 50)//静止时
	{
		for(int n=0;n<4;n++)
		chassis.cha_pid_6020.angle_ref[n] = (getnumb[n])*360;

		if(Chassis_angle.get_speedw < 0) //方向反向
		{
			for(int l=0;l<4;l++)
			chassis.cha_pid_6020.angle_ref[l] = chassis.cha_pid_6020.angle_ref[l] - 180;
		}	
	}
	else
	{
		for(int i=0;i<4;i++)
		chassis.cha_pid_6020.angle_ref[i] = Chassis_angle.deviation_angle[i] * RAD_TO_ANGLE + (getnumb[i])*360;
	}	
	
	
		for(int k=0;k<4;k++)
		Ref_Fdb(chassis.cha_pid_6020.angle_ref[k],chassis.cha_pid_6020.angle_fdb[k]);//最多转劣弧

  
			Chassis_angle.handle_speed_lim[0] = RIGHT_FRONT_REVERSE*Chassis_angle.handle_speed_lim[0];
		  Chassis_angle.handle_speed_lim[1] = LEFT_FRONT_REVERSE*Chassis_angle.handle_speed_lim[1];
			Chassis_angle.handle_speed_lim[2] = LEFT_BEHIND_REVERSE*Chassis_angle.handle_speed_lim[2];
			Chassis_angle.handle_speed_lim[3] = RIGHT_BEHIND_REVERSE*Chassis_angle.handle_speed_lim[3];

//此处	chassis.cha_pid_6020.angle_ref 到时候放功率限制后的。
	for(int i=0;i<4;i++)
	{
	if((chassis.cha_pid_6020.angle_ref[i]-chassis.cha_pid_6020.angle_fdb[i])>90)
	{	  
		chassis.cha_pid_6020.angle_ref[i]-=180;
		Chassis_angle.handle_speed_lim[i] = -Chassis_angle.handle_speed_lim[i];
	}
	else if((chassis.cha_pid_6020.angle_ref[i]-chassis.cha_pid_6020.angle_fdb[i])<-90)
	{
		chassis.cha_pid_6020.angle_ref[i]+=180;
		Chassis_angle.handle_speed_lim[i] = -Chassis_angle.handle_speed_lim[i];
	}
	}
	
	for(int i=0;i<4;i++)
	pid_calc(&pid_cha_6020_angle[i],chassis.cha_pid_6020.angle_fdb[i],chassis .cha_pid_6020.angle_ref[i]);

	if(chassis .ctrl_mode==MANUAL_FOLLOW_GIMBAL||chassis.ctrl_mode==CHASSIS_ROTATE)
	{
		for(int j=0;j<4;j++)
		chassis.cha_pid_6020.speed_fdb[j]=steering_wheel_chassis.Heading_Encoder[j].filter_rate;
		
		for(int i=0;i<4;i++)
		chassis.cha_pid_6020.speed_ref[i]=pid_cha_6020_angle[i].out;
	}	

}
#endif
/**
************************************************************************************************************************
* @Name     : set_3508current_6020voltage
* @brief    : 驱动轮电机速度环反馈获取和给定设置；驱动轮和航向轴电机pid最终给定计算处理
* @param    : None
* @retval   : void
* @Note     : 四轮差速处理（老补丁）
							功率控制切换限制
************************************************************************************************************************
**///舵轮
#if CHASSIS_TYPE == 1
void set_3508current_6020voltage()
{		
		start_chassis_6020();

		for (int i = 0; i < 4; i++)
		{ 
		chassis.cha_pid_3508.speed_fdb[i]=steering_wheel_chassis.Driving_Encoder[i].filter_rate;
		chassis.cha_pid_3508.speed_ref[i]=Chassis_angle.handle_speed_lim[i];

#if POWER_LIMIT_HANDLE
		pid_calc(&pid_cha_6020_speed[i],chassis.cha_pid_6020.speed_fdb[i],kall6020[i]*chassis.cha_pid_6020.speed_ref[i]);
		pid_calc(&pid_cha_3508_speed[i],chassis.cha_pid_3508.speed_fdb[i] , power_limit_rate2*power_limit_rate1*chassis.cha_pid_3508.speed_ref[i]);			
#else
		pid_calc(&pid_cha_6020_speed[i],chassis.cha_pid_6020.speed_fdb[i],chassis.cha_pid_6020.speed_ref[i]);
		pid_calc(&pid_cha_3508_speed[i],chassis.cha_pid_3508.speed_fdb[i],chassis.cha_pid_3508.speed_ref[i]);
#endif		
		}

   for (int i = 0; i < 4; i++)
    {   
    if((chassis.ctrl_mode==MANUAL_FOLLOW_GIMBAL||chassis.ctrl_mode==CHASSIS_ROTATE))				
				chassis.current[i] = 1.0f * pid_cha_3508_speed[i].out;		
		else
			{
//			OpenDoor;
			chassis.current[i]=0;
			}
		}			

//四轮差速处理		
//		if(chassis.ctrl_mode==CHASSIS_ROTATE)
//		mecanum_calc(chassis.current);
		

				
		
		if(fabs(chassis.cha_pid_3508.speed_ref[0])<50&&
			 fabs(chassis.cha_pid_3508.speed_ref[1])<50&&
			 fabs(chassis.cha_pid_3508.speed_ref[2])<50&&
			 fabs(chassis.cha_pid_3508.speed_ref[3])<50)
		{
			run_flag=0;
		}
		else
		{
			run_flag=1;
		}
		
		
for (int i = 0; i < 4; i++)
    {   
		if((chassis.ctrl_mode==MANUAL_FOLLOW_GIMBAL||chassis.ctrl_mode==CHASSIS_ROTATE)&&run_flag)//
		{
			chassis.voltage[i]=1.0f*(int16_t)pid_cha_6020_speed[i].out;
//			CloseDoor;
		}
		else{
//			OpenDoor;
			chassis.voltage[i]=0;
				}
		}
}
//麦轮
#elif CHASSIS_TYPE == 2
 void set_3508current_6020voltage()
 {
 
 for (int i = 0; i < 4; i++)
		{ 
		chassis.cha_pid_3508.speed_fdb[i]=Mecanum_chassis.Driving_Encoder[i].filter_rate; 
		pid_calc(&pid_cha_3508_speed[i],chassis.cha_pid_3508.speed_fdb[i],chassis.cha_pid_3508.speed_ref[i]);
		}
for (int i = 0; i < 4; i++)
    {   
    if(chassis.ctrl_mode!=CHASSIS_RELAX)				
				chassis.current[i] = 1.0f * pid_cha_3508_speed[i].out;		
		else
			{
			chassis.current[i]=0;
			}
		}				
}
 
#endif



void Chassis_PID_handle(void)
{
	#if POWER_LIMIT_HANDLE
			power_limit_handle();	
	#endif
	
			set_3508current_6020voltage();
}

void Motion_resolution(void)
{
		get_remote_set();
		start_angle_handle();
}


/**
************************************************************************************************************************
* @Name     : get_remote_set
* @brief    : 解算遥控器速度方向和大小
* @param    : None
* @retval   : void
* @Note     : 10.3 优化了算法更精确更高刷新率
************************************************************************************************************************
**///舵轮
#if CHASSIS_TYPE == 1
void get_remote_set()
{	
//逆时针增大！	
	
	Chassis_angle.yaw_encoder_ecd_angle=can_chassis_data.yaw_Encoder_ecd_angle/10000.0f;
	Chassis_angle.yaw_angle_0_2pi      =convert_ecd_angle_to_0_2pi(Chassis_angle.yaw_encoder_ecd_angle,Chassis_angle.yaw_angle_0_2pi);
	
	float temp_angle=0;
		vx = can_chassis_data.x;//vx，x是横轴
		vy = can_chassis_data.y;//vy，y是纵轴
		Chassis_angle.Remote_speed = sqrt((vx*vx)+(vy*vy));
		if(Chassis_angle.Remote_speed >= 50)
		{	
			temp_angle=atan2(vy,vx);
			Chassis_angle.Remote_angle = fmod(2*PI+temp_angle,2*PI);
		}
		else
		{Chassis_angle.Remote_speed = 0;}
}
#elif CHASSIS_TYPE == 2
void get_remote_set()
{	
		Chassis_angle.yaw_encoder_ecd_angle=yaw_Encoder.ecd_angle;
		Chassis_angle.yaw_angle_0_2pi=convert_ecd_angle_to_0_2pi(Chassis_angle.yaw_encoder_ecd_angle,Chassis_angle.yaw_angle_0_2pi);
	
//		chassis.vx=chassis.ChassisSpeed_Ref.forward_back_ref;
//		chassis.vy=chassis.ChassisSpeed_Ref.left_right_ref;

}
#elif CHASSIS_TYPE == 4
void get_remote_set()
{	
		Chassis_angle.yaw_encoder_ecd_angle=yaw_Encoder.ecd_angle;
		Chassis_angle.yaw_angle_0_2pi=convert_ecd_angle_to_0_2pi(Chassis_angle.yaw_encoder_ecd_angle,Chassis_angle.yaw_angle_0_2pi);
	
		chassis.vx=ChassisSpeedRef.forward_back_ref;
		chassis.vy=ChassisSpeedRef.left_right_ref;

}
#endif





/**
************************************************************************************************************************
* @Name     : chassis_task
* @brief    : 
* @retval   : None
* @Note     : 
************************************************************************************************************************
**/
void chassis_task()
{
	chassis_mode_select();
	Motion_resolution();
	Chassis_PID_handle();
}
