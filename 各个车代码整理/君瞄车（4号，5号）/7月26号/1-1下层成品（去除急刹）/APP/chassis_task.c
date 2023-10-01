#include  "main.h"
#include "AHRS_MiddleWare.h"
/* chassis twist angle (degree)*/
#define TWIST_ANGLE  40
/* twist period time (ms) */
//#define TWIST_PERIOD   1250
#define TWIST_PERIOD   1000
/* warning surplus energy */
/* chassis task global parameter */


float power_limit_start_time;
float power_limit_start_flag;
float power_limit_rate2;
float power_limit_rate1;
float  max_chassis_power =35;

chassis_t chassis;
uint32_t chassis_time_last;
power_limit_t power_limit;
pid_software_limit_t software_limit;
chassis_rotate_speed_t chassis_rotate_speed;
int chassis_time_ms;
float power_limit_rate=1;
u32 chassis_speed_mode_time = 0;
u32 chassis_follow_flag = 0;
u16 rotate_sin_time=0,rotate_sin_max=5;
float total_chassis_wheel_speed_ref = 0;
float total_chassis_wheel_speed_limit = 0;
cha_pid_t cha;
chassis_speed_mode_e chassis_speed_mode;
u8 CanVcap_flag = 0;
u8 chassis_rotate_flag = 0;
u8 power_now=0;
float car_velocity = 0;
uint16_t forward_back_speed = 0;
uint16_t left_right_speed = 0;

u8 chassis_power_buffer_flag;
u8 chassis_rotate_buffer_flag;
int chassis_power_buffer_time;
int chassis_rotate_buffer_time;

u16  Max_Power_3508;
u16  Max_Power_6020;
float PowerSum;
float heat_power;
float drive_power;
float start_angle = {0};
float include_angle = {0};
float deviation_angle = {0};
float handle_speed = {0};

float I6020[4],sdpower,shpower,dpower[4],hpower[4],all_power1[4],all_power2[4],
	all_power,a6020[4],b6020[4],m6020,n6020,l6020,i6020,kall6020[4];

float k6020=-0.000075,k2_6020=1.278e-7,k1_6020=6.157e-6,k0_6020=1.054;

float  leftangle  =   0.0;  
float   rightangle   =  -0.0;
float num_zero;

u16 test_wheel_speed;

int run_flag=0;
void chassis_task(void)
{	
	num_zero=0;
	
	
	capacitance_message(CAN1,capacitance_message1.cap_voltage_filte*10);
  car_velocity = CM1Encoder.rate_rpm*0.000560998688f;

	start_angle_handle();
//	power_6020_limit_better();
	
  chassis.wheel_speed_fdb[0]=CM1Encoder.filter_rate;
	Chassis_angle.wheel_speed_fdb[0]=CM1Encoder.filter_rate;
  Chassis_angle.wheel_speed_fdb[1]=CM2Encoder.filter_rate;
  Chassis_angle.wheel_speed_fdb[2]=CM3Encoder.filter_rate;
  Chassis_angle.wheel_speed_fdb[3]=CM4Encoder.filter_rate;	


#if POWER_LIMT==1	

  power_limit_handle();// 功率限制
	
#endif

			    pid_calc(&pid_cha1_speed[0],cha.speed1_fdb, kall6020[0]*cha.speed1_ref);
					pid_calc(&pid_cha1_speed[1],cha.speed2_fdb, kall6020[1]*cha.speed2_ref);
					pid_calc(&pid_cha1_speed[2],cha.speed3_fdb, kall6020[2]*cha.speed3_ref);
					pid_calc(&pid_cha1_speed[3],cha.speed4_fdb, kall6020[3]*cha.speed4_ref);

					
		if(Chassis_angle.cap_flag==3||Chassis_angle.cap_flag==2)
		{
			power_limit_start_flag=1;
		}
		else if(Chassis_angle.cap_flag==1)
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
    else if(Chassis_angle.cap_flag==0)
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
					

    
	  for (int i = 0; i < 4; i++)
    { 
//   chassis.current[i] = pid_calc(&pid_spd[i],Chassis_angle.wheel_speed_fdb[i] , test_wheel_speed);
   chassis.current[i] = pid_calc(&pid_spd[i],Chassis_angle.wheel_speed_fdb[i] , power_limit_rate2*power_limit_rate1*Chassis_angle.handle_speed1[i]);
//     chassis.current[i] = pid_calc(&pid_spd[i],Chassis_angle.wheel_speed_fdb[i] , Chassis_angle.handle_speed[i]);
    }		
	
		if(chassis.ctrl_mode==CHASSIS_ROTATE)
			mecanum_calc(chassis.current);

#if POWER_LIMT==0
  power_limit_handle();// 热量限制
#endif
  

		
		
 	if((Chassis_angle.get_control_flag==1||Chassis_angle.get_control_flag==2))
	{
      Set_CM_Speed(CAN2, CHASSIS_SPEED_ATTENUATION * chassis.current[0],\
                   CHASSIS_SPEED_ATTENUATION * chassis.current[1],\
                   CHASSIS_SPEED_ATTENUATION * chassis.current[2],\
                   CHASSIS_SPEED_ATTENUATION * chassis.current[3]);
//		      Set_CM_Speed(CAN2, test_wheel_speed,test_wheel_speed,test_wheel_speed,test_wheel_speed);
		
		

	}
    else{OpenDoor;
   Set_CM_Speed(CAN2, 0,0,0,0);

}
		chassis_task1();
}

void power_6020_limit_better(void)
{
	if(pid_cha1_angle[0].set-pid_cha1_angle[0].get<4&&pid_cha1_angle[0].set-pid_cha1_angle[0].get>-4)
	{
		PID_struct_init(&pid_cha1_angle[0], POSITION_PID, 8000, 0, 8,0.1f,3);//24, 0.2f,20);
  	PID_struct_init(&pid_cha1_speed[0], POSITION_PID, 150, 0, 150,0.1f,2);//38,0.5f,20);
	}
	else
	{
		PID_struct_init(&pid_cha1_angle[0], POSITION_PID, 8000, 10, 8,0.1f,3);//24, 0.2f,20);
  	PID_struct_init(&pid_cha1_speed[0], POSITION_PID, 15000, 500, 150,0.1f,2);//38,0.5f,20);
	}
	
	if(pid_cha1_angle[1].set-pid_cha1_angle[1].get<4&&pid_cha1_angle[1].set-pid_cha1_angle[1].get>-4)
	{
		PID_struct_init(&pid_cha1_angle[1], POSITION_PID, 8000, 0, 8,0.1f,3);//24, 0.2f,20);
  	PID_struct_init(&pid_cha1_speed[1], POSITION_PID, 150, 0, 150,0.1f,2);//38,0.5f,20);
	}
	else
	{
		PID_struct_init(&pid_cha1_angle[1], POSITION_PID, 8000, 10, 8,0.1f,5);//25, 0.2f,15);
  	PID_struct_init(&pid_cha1_speed[1], POSITION_PID, 15000, 500, 150,0.1,4);//39,0.5f,20);
	}
	
	if(pid_cha1_angle[2].set-pid_cha1_angle[2].get<4&&pid_cha1_angle[2].set-pid_cha1_angle[2].get>-4)
	{
		PID_struct_init(&pid_cha1_angle[2], POSITION_PID, 8000, 0, 8,0.1f,3);//24, 0.2f,20);
  	PID_struct_init(&pid_cha1_speed[2], POSITION_PID, 150, 0, 150,0.1f,2);//38,0.5f,20);
	}
	else
	{
	  PID_struct_init(&pid_cha1_angle[2], POSITION_PID, 8000, 10, 8,0.1f,4);//20, 0.2f,20);
  	PID_struct_init(&pid_cha1_speed[2], POSITION_PID, 15000, 2000, 150,0.5,8);//40,0.5f,20);
	}
	
	if(pid_cha1_angle[3].set-pid_cha1_angle[3].get<4&&pid_cha1_angle[3].set-pid_cha1_angle[3].get>-4)
	{
		PID_struct_init(&pid_cha1_angle[3], POSITION_PID, 8000, 0, 8,0.1f,3);//24, 0.2f,20);
  	PID_struct_init(&pid_cha1_speed[3], POSITION_PID, 150, 0, 150,0.1f,2);//38,0.5f,20);
	}
	else
	{
	  PID_struct_init(&pid_cha1_angle[3], POSITION_PID, 8000, 10, 10,0.4f,4);//23, 0.2f,15);
  	PID_struct_init(&pid_cha1_speed[3], POSITION_PID, 15000, 500, 150,0.1,10);//42,0.5f,20);
	}
}


float get_6020power()
{
	
I6020[0]=pid_cha1_speed[0].p*(pid_cha1_speed[0].set-pid_cha1_speed[0].get)+pid_cha1_speed[0].iout+pid_cha1_speed[0].d*(pid_cha1_speed[0].set-pid_cha1_speed[0].get-pid_cha1_speed[0].err[LAST]);	
I6020[1]=pid_cha1_speed[1].p*(pid_cha1_speed[1].set-pid_cha1_speed[1].get)+pid_cha1_speed[1].iout+pid_cha1_speed[1].d*(pid_cha1_speed[1].set-pid_cha1_speed[1].get-pid_cha1_speed[1].err[LAST]);	
I6020[2]=pid_cha1_speed[2].p*(pid_cha1_speed[2].set-pid_cha1_speed[2].get)+pid_cha1_speed[2].iout+pid_cha1_speed[2].d*(pid_cha1_speed[2].set-pid_cha1_speed[2].get-pid_cha1_speed[2].err[LAST]);	
I6020[3]=pid_cha1_speed[3].p*(pid_cha1_speed[3].set-pid_cha1_speed[3].get)+pid_cha1_speed[3].iout+pid_cha1_speed[3].d*(pid_cha1_speed[3].set-pid_cha1_speed[3].get-pid_cha1_speed[3].err[LAST]);	
		VAL_LIMIT(I6020[0],-10000,10000);
		VAL_LIMIT(I6020[1],-10000,10000);
		VAL_LIMIT(I6020[2],-10000,10000);
		VAL_LIMIT(I6020[3],-10000,10000);
       dpower[0]=I6020[0]*pid_cha1_speed[0].get*k6020;
		   dpower[1]=I6020[1]*pid_cha1_speed[1].get*k6020;
		   dpower[2]=I6020[2]*pid_cha1_speed[2].get*k6020;
		   dpower[3]=I6020[3]*pid_cha1_speed[3].get*k6020;	
hpower[0]=k2_6020*I6020[0]*I6020[0]+k1_6020*I6020[0]+k0_6020;
hpower[1]=k2_6020*I6020[1]*I6020[1]+k1_6020*I6020[1]+k0_6020;
hpower[2]=k2_6020*I6020[2]*I6020[2]+k1_6020*I6020[2]+k0_6020;
hpower[3]=k2_6020*I6020[3]*I6020[3]+k1_6020*I6020[3]+k0_6020;
all_power1[0]=dpower[0]+hpower[0];
all_power1[1]=dpower[1]+hpower[1];
all_power1[2]=dpower[2]+hpower[2];
all_power1[3]=dpower[3]+hpower[3];
all_power=all_power1[0]+all_power1[1]+all_power1[2]+all_power1[3];
VAL_LIMIT(all_power,0,1000);

Max_Power_6020=all_power;
VAL_LIMIT(Max_Power_6020,0,max_chassis_power);


Max_Power_3508=Max_Power-Max_Power_6020;


float a[4],b[4],c[4];
if(all_power>max_chassis_power)
{for (int i = 0; i < 4; i++)
{all_power2[i]=all_power1[i]*max_chassis_power/all_power;
if(all_power1[i]>k0_6020&&all_power1[i]>all_power2[i])
{a[i]=k2_6020;
b[i]=k1_6020+k6020*pid_cha1_speed[i].get;
c[i]=k0_6020-all_power2[i];
kall6020[i]=(-b[i]+(double)sqrt(b[i]*b[i]-4*a[i]*c[i]))/(2*a[i])/I6020[i];
}else{kall6020[i]=1;}
}}else{
for (int i = 0; i < 4; i++)
    {kall6020[i]=1;}
}
for (int i = 0; i < 4; i++)
    {
	VAL_LIMIT(kall6020[i],0,1);
		}
}

void chassis_stop_handle(void)
{
  chassis.vy = 0;
  chassis.vx = 0;
  chassis.vw = 0;
}


u32 reverse_init_time = 0;
void power_send_handle2(void)
{
POWER_Control1l(0x610);
POWER_Control1l(0x611);
POWER_Control1l(0x612);
POWER_Control1l(0x613);
}
void power_send_handle1(void)
{

buffer_power();
POWER_Control1(2,0x600);
POWER_Control1(Max_Power*100,0x601);
POWER_Control1(2500,0x602);
POWER_Control1(7*100,0x603);
}
void chassis_change_reverse_handle(void)
{
  chassis.vy = 0;
  chassis.vx = 0;
  chassis.vw = 0;
  if((GimbalRef.yaw_angle_dynamic_ref - GimbalRef.yaw_angle_dynamic_ref <=1.5f)&&(GimbalRef.yaw_angle_dynamic_ref - GimbalRef.yaw_angle_dynamic_ref >=-1.5f))
    {
      reverse_init_time++;
    }

  if(reverse_init_time>80)
    {
      chassis.ctrl_mode = CHASSIS_REVERSE;
      reverse_init_time = 0;
    }
}


static void chassis_twist_handle(void)
{
  chassis.vw = pid_calc(&pid_chassis_angle, GMYawEncoder.ecd_angle, chassis.position_ref);
}

void follow_gimbal_handle(void)
{

  if((Key_Flag.Key_A_D_Flag == 1)||(Key_Flag.Key_W_S_Flag == 1))
    {
      if(chassis_speed_mode == NORMAL_SPEED_MODE)
        {
          forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
          left_right_speed = NORMAL_LEFT_RIGHT_SPEED;
        }
      else if(chassis_speed_mode == HIGH_SPEED_MODE)
        {
          forward_back_speed=HIGH_FORWARD_BACK_SPEED;
          left_right_speed=HIGH_LEFT_RIGHT_SPEED;
        }
    }
  else
    {
      FBSpeedRamp.ResetCounter(&FBSpeedRamp);
//		FBSpeedRampSecond.init(&FBSpeedRampSecond,MOUSR_FB_RAMP_TICK_COUNT);
    }
  chassis.vy = ChassisSpeedRef.left_right_ref;
  chassis.vx = ChassisSpeedRef.forward_back_ref;



  if (chassis.follow_gimbal)
    {
//     Chassis_angle.get_speedw = pid_calc(&pid_chassis_angle,GMYawEncoder.ecd_angle,chassis.position_ref);
//      chassis.vw = pid_calc(&pid_chassis_angle,GMYawEncoder.ecd_angle,chassis.position_ref);
    }
  else
    {
//      chassis.vw = 0;
//			Chassis_angle.get_speedw = 0;
    }
				
}

void separate_gimbal_handle(void)
{

  if((Key_Flag.Key_A_D_Flag == 1)||(Key_Flag.Key_W_S_Flag == 1))
    {
//			voltage_pid.voltage_fdb = capacitance_message1.raw_cap_voltage;
//			if(chassis_speed_mode == NORMAL_SPEED_MODE)
//				voltage_pid.voltage_ref = WARNING_VOLTAGE+4*(1-FBSpeedRamp.Calc(&FBSpeedRamp));
//			else if(chassis_speed_mode == HIGH_SPEED_MODE)
//				voltage_pid.voltage_ref = TARGET_VOLTAGE+4*(1-FBSpeedRamp.Calc(&FBSpeedRamp));斜坡给定速度
      if(chassis_speed_mode == NORMAL_SPEED_MODE)
        {
          forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
          left_right_speed = NORMAL_LEFT_RIGHT_SPEED;
        }
      else if(chassis_speed_mode == HIGH_SPEED_MODE)
        {
          forward_back_speed=HIGH_FORWARD_BACK_SPEED;
          left_right_speed=HIGH_LEFT_RIGHT_SPEED;
        }
    }
  else
    {
      FBSpeedRamp.ResetCounter(&FBSpeedRamp);
    }

  chassis.vy = ChassisSpeedRef.left_right_ref;
  chassis.vx = ChassisSpeedRef.forward_back_ref;

  chassis.vw = 0;

}

u32 rotate_mode_count = 0;
u8  rotate_mode = 0;
u8 sin_constant_flag;
u8 get_speedwf_lag = 0;
void rotate_follow_gimbal_handle(void)
{
}

u32 reverse_time_count = 0;

void reverse_follow_gimbal_handle(void)
{
  reverse_time_count++;

  if(reverse_time_count>850)
    {
      chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
      reverse_time_count = 0;
    }
  if((Key_Flag.Key_A_D_Flag == 1)||(Key_Flag.Key_W_S_Flag == 1))
    {
      if(chassis_speed_mode == NORMAL_SPEED_MODE)
        {
          forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
          left_right_speed = NORMAL_LEFT_RIGHT_SPEED;
        }
      else if(chassis_speed_mode == HIGH_SPEED_MODE)
        {
          forward_back_speed=HIGH_FORWARD_BACK_SPEED;
          left_right_speed=HIGH_LEFT_RIGHT_SPEED;
        }
    }
  else
    {
      FBSpeedRamp.ResetCounter(&FBSpeedRamp);
    }
  chassis.vy = -ChassisSpeedRef.left_right_ref;
  chassis.vx = -ChassisSpeedRef.forward_back_ref;

  if (chassis.follow_gimbal)
    {
      chassis.vw = pid_calc(&pid_chassis_angle,GMYawEncoder.ecd_angle,chassis.position_ref-180);
    }
  else
    {
      chassis.vw = 0;
    }
} 

int read_maxpower;

//上限为50时 max_power=100 
float get_max_power(float voltage)
{
int max_power=0;
  if(voltage>WARNING_VOLTAGE+4)
    max_power=350;
  else
    max_power=(voltage-WARNING_VOLTAGE)/4.0f*350;
  VAL_LIMIT(max_power,0,500);
  return max_power;

}
float get_max_power1(float voltage)
{
    int max_power=0;
	  if(Chassis_angle.cap_flag==3)
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
void auto_sup_handle(void)
{
  distance_message.front_left_right_bias = distance_message.front_left_distance - distance_message.front_right_distance;

  pid_calc(&pid_front_distance,distance_message.front_left_distance,360);

  pid_calc(&pid_right_distance,distance_message.right_distance,360);

  pid_calc(&pid_angle_distance,distance_message.front_left_right_bias,-4);
	VAL_LIMIT(pid_front_distance.out,-90,90);
	VAL_LIMIT(pid_right_distance.out,-90,90);
	VAL_LIMIT(chassis.vw,-25,25);

  chassis.vx = -pid_front_distance.out;
  chassis.vy = pid_right_distance.out;
  chassis.vw = -pid_angle_distance.out;
}
/**
  * @brief mecanum chassis velocity decomposition
  * @param input : ↑=+vx(mm/s)  ←=+vy(mm/s)  ccw=+vw(deg/s)角度/秒，ccw旋转
  *        output: every wheel speed(rpm)
  * @note  1=FR 2=FL 3=BL 4=BR
  */


void mecanum_calc(int16_t *speed)
{
  int16_t wheel_rpm[4];
  float   max = 0;
  wheel_rpm[0] = speed[0];
  wheel_rpm[1] = speed[1];
  wheel_rpm[2] = speed[2];
  wheel_rpm[3] = speed[3];

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
//	chassis.wheel_speed_fdb[0]=CM1Encoder.filter_rate;
//  chassis.wheel_speed_fdb[1]=CM2Encoder.filter_rate;
//  chassis.wheel_speed_fdb[2]=CM3Encoder.filter_rate;
//  chassis.wheel_speed_fdb[3]=CM4Encoder.filter_rate;
}

#if 1
float total_cur_limit;
int32_t total_cur;
u16  Max_Power;
u8  Max_Current   = 9;

u8 Cap_Reset_flag = 0;
float remain_voltage=0;
uint8_t remain_energy=0;
static float heat_power_calc(int i)
{
  return  FACTOR_2*(float)(i*i)+FACTOR_1*(float)(i)+FACTOR_0;
}

//motor_number 0-3  对应四个底盘电机
static float i_predict(int motor_number,float factor)
{
  float power= pid_spd[motor_number].p* \
               (factor*(float)Chassis_angle.handle_speed1[motor_number]-Chassis_angle.wheel_speed_fdb[motor_number])+ \
               pid_spd[motor_number].iout+ \
               pid_spd[motor_number].d*( \
                                         (factor*(float)Chassis_angle.handle_speed1[motor_number]-Chassis_angle.wheel_speed_fdb[motor_number])- \
                                         pid_spd[motor_number].err[LAST]);
  return power;
//	return  pid_spd[motor_number].p*(factor*(float)chassis.wheel_speed_ref[motor_number]-chassis.wheel_speed_fdb[motor_number]);
}

static float get_the_limite_rate(float max_power)
{
  float a[4];
  for(int i=0; i<4; i++)
    a[i]=(float)Chassis_angle.handle_speed1[i]*(pid_spd[i].p+pid_spd[i].d);
  float b[4];
  for(int i=0; i<4; i++)
    b[i]=-pid_spd[i].p*(float)Chassis_angle.wheel_speed_fdb[i]+pid_spd[i].iout \
         -pid_spd[i].d*(float)Chassis_angle.wheel_speed_fdb[i]-pid_spd[i].d*pid_spd[i].err[LAST];
  // Max_power=heat_power+drive_power
  //	i_n=a[n]*k+b[n]	带入
  //Max_Power=m*k^2+n*k+o
  //0=m*k^2+n*k+l(l=o-Max_Power)
  float m=(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]+a[3]*a[3])*FACTOR_2;

  float n=2*FACTOR_2*(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]) + \
          FACTOR_1*(a[0] + a[1] + a[2] + a[3]) + \
          I_TIMES_V_TO_WATT*(a[0]*(float)Chassis_angle.wheel_speed_fdb[0] + \
                             a[1]*(float)Chassis_angle.wheel_speed_fdb[1] + \
                             a[2]*(float)Chassis_angle.wheel_speed_fdb[2] + \
                             a[3]*(float)Chassis_angle.wheel_speed_fdb[3]);

  float l=(b[0]*b[0] + b[1]*b[1] + b[2]*b[2] + b[3]*b[3])*FACTOR_2 + \
          (b[0] + b[1] + b[2] + b[3])*FACTOR_1 + \
          I_TIMES_V_TO_WATT*(b[0]*(float)Chassis_angle.wheel_speed_fdb[0] + \
                             b[1]*(float)Chassis_angle.wheel_speed_fdb[1] + \
                             b[2]*(float)Chassis_angle.wheel_speed_fdb[2] + \
                             b[3]*(float)Chassis_angle.wheel_speed_fdb[3]) + \
          4*FACTOR_0 - \
          max_power;
  return (-n+(float)sqrt((double)(n*n-4*m*l)+1.0f))/(2*m);
}
float power_predict()
{
  float drive_power=( \
                      i_predict(0,1.0f) * (float)chassis.wheel_speed_fdb[0] + \
                      i_predict(1,1.0f) * (float)chassis.wheel_speed_fdb[1] + \
                      i_predict(2,1.0f) * (float)chassis.wheel_speed_fdb[2] + \
                      i_predict(3,1.0f) * (float)chassis.wheel_speed_fdb[3])*I_TIMES_V_TO_WATT;
  float heat_power=heat_power_calc(i_predict(0,1.0f))+ \
                   heat_power_calc(i_predict(1,1.0f))+ \
                   heat_power_calc(i_predict(2,1.0f))+ \
                   heat_power_calc(i_predict(3,1.0f));

  float PowerSum = drive_power+heat_power;
  return PowerSum;
}
float volatilAo;
float abbbbb;
float aaatest;
void power_limit_handle(void)
{
          volatilAo=capacitance_message3.out_v;
	        capacitance_message1.cap_voltage_filte=volatilAo/100;
					max_chassis_power=get_max_power2(capacitance_message1.cap_voltage_filte);
//			    VAL_LIMIT(max_chassis_power,0,45);		
					get_6020power();
					abbbbb=get_max_power1(capacitance_message1.cap_voltage_filte)-Max_Power_6020;
	
	        if(Chassis_angle.cap_flag==1||Chassis_angle.get_mode_flag==2||Chassis_angle.cap_flag==3)
					{
          power_limit_rate1=get_the_limite_rate(get_max_power1(capacitance_message1.cap_voltage_filte)-Max_Power_6020);
					aaatest++;
					}
					else
					{
					  power_limit_rate2=get_the_limite_rate(get_max_power2(capacitance_message1.cap_voltage_filte)-Max_Power_6020);
						aaatest=0;
					}
	        
	        VAL_LIMIT(power_limit_rate1,0,1);
	        VAL_LIMIT(power_limit_rate2,0,1);
	
			    
}

void power_other_limit(void)
{
  power_limit.power_limit_model = SOFTWARE_LIMIT ;
}


u8 Power_Work_Mode;
u32 fault_Time=0;
u32 chassis_power_count = 0;
u8  cap_reset = 0;
u16 cap_reset_times = 0;

void Fault_judge(void)
{
  CanVcap_flag=Check_Vcap_recieve();   //查看是否接收到了新的消息
  if((capacitance_message2.fault_union.fault==0)&&(CanVcap_flag) \
      &&(judge_rece_mesg.game_robot_state.mains_power_chassis_output==1))	 //没有错误

//	  if((capacitance_message2.fault_union.fault==0)&&(CanVcap_flag) \
//      &&(judge_rece_mesg.game_robot_state.mains_power_chassis_output==0))	 //没有错误

    {
      if((capacitance_message2.system_mode == 2))   //正常运行并没有离线（离线时最后传回来的是正常运行）
        {

					Power_Work_Mode=1;
				
        }
    }
  else
    {
      Power_Work_Mode = 0;
    }
		


  if(capacitance_message2.fault_union.fault == 8) //电池断电
    Cap_Reset_flag |= 0x04;
  else
    Cap_Reset_flag &= ~0x04;

}

void power_send_handle(void)
{
//buffer_power();
//static u8 canbuf[8];
//canbuf[0]=0x10;
//canbuf[1]=Max_Power;
//canbuf[2]=Max_Current;
//canbuf[3]=Power_Work_Mode;
//canbuf[4]=Cap_Reset_flag;
//canbuf[5]=0;
//canbuf[6]=0;
//canbuf[7]=0;
//POWER_Control(canbuf);//发送8个字节
}
#endif



int Check_Vcap_recieve(void)
{
  not_receive_time++;
  if(not_receive_time>50)
    return 0;
  else
    return 1;

}
float rate_6020_power_flag;
float rate_6020_power;

void buffer_power(void)
{
 {Max_Power = judge_rece_mesg.game_robot_state.chassis_power_limit+
(judge_rece_mesg.power_heat_data.chassis_power_buffer-5)*2;
} 
 if(capacitance_message1.cap_voltage_filte>=23.0)
 {Max_Power=(23.7-capacitance_message1.cap_voltage_filte)*150;
	VAL_LIMIT(Max_Power,0,judge_rece_mesg.game_robot_state.chassis_power_limit+
(judge_rece_mesg.power_heat_data.chassis_power_buffer-5)*2); 
 }
	
//	Max_Power=50;

	 
  if(capacitance_message1.cap_voltage_filte>=23.7)
 {Max_Power=0;}
	
VAL_LIMIT(Max_Power,0,150);
}
/**
  * @brief  nitialize chassis motor pid parameter
  * @usage  before chassis loop use this function



  */
int getnumb1;
int getnumb2;
int getnumb3;
int getnumb4;
int num1;
int num2;
int num3;
int num4;
float getlastangle1;
float getlastangle2;
float getlastangle3;
float getlastangle4;

float  aa,bb,cc,dd;

void START_CHASSIS_6020(void)
{

cha.angle1_fdb = GM1Encoder1.ecd_angle;
cha.angle2_fdb = GM2Encoder1.ecd_angle;
cha.angle3_fdb = GM3Encoder1.ecd_angle;
cha.angle4_fdb = GM4Encoder1.ecd_angle;
	if((Chassis_angle.Remote_speed > 50)||((Chassis_angle.get_speedw<-5)||(Chassis_angle.get_speedw>5)
		))
	{
if((GM1Encoder1.ecd_angle-getnumb1*360)>360)
{getnumb1++;}
else if((GM1Encoder1.ecd_angle-getnumb1*360)<0)
{getnumb1--;}
if((GM2Encoder1.ecd_angle-getnumb2*360)>360)
{getnumb2++;}
else if((GM2Encoder1.ecd_angle-getnumb2*360)<0)
{getnumb2--;}
if((GM3Encoder1.ecd_angle-getnumb3*360)>360)
{getnumb3++;}
else if((GM3Encoder1.ecd_angle-getnumb3*360)<0)
{getnumb3--;}
if((GM4Encoder1.ecd_angle-getnumb4*360)>360)
{getnumb4++;}
else if((GM4Encoder1.ecd_angle-getnumb4*360)<0)
{getnumb4--;}
}

//getnumb1 = GM1Encoder1.ecd_angle / 360;
//getnumb2 = GM2Encoder1.ecd_angle / 360;
//getnumb3 = GM3Encoder1.ecd_angle / 360;
//getnumb4 = GM4Encoder1.ecd_angle / 360;

//if(GM1Encoder1.ecd_angle < 0)
//{getnumb1 -= 1;}	
//if(GM2Encoder1.ecd_angle < 0)
//{getnumb2 -= 1;}
//if(GM3Encoder1.ecd_angle < 0)
//{getnumb3 -= 1;}
//if(GM4Encoder1.ecd_angle < 0)
//{getnumb4 -= 1;}


//ab=GM1Encoder1.ecd_angle / 360;
if(Chassis_angle.Remote_speed <= 50)
{
	//	{getnumb[0] = GM1Encoder1.ecd_angle / 360;
  cha.angle1_ref = (getnumb1)*360;
  cha.angle2_ref = (getnumb2)*360;
  cha.angle3_ref = (getnumb3)*360;
  cha.angle4_ref = (getnumb4)*360;
	if(Chassis_angle.get_speedw < 0)
	{
	cha.angle1_ref = cha.angle1_ref - 180;
	cha.angle2_ref = cha.angle2_ref - 180;
	cha.angle3_ref = cha.angle3_ref - 180;
	cha.angle4_ref = cha.angle4_ref - 180;
	}	
	}
	else
	{
   cha.angle1_ref = Chassis_angle.deviation_angle[0] * RAD_TO_ANGLE + (getnumb1)*360;
   cha.angle2_ref = Chassis_angle.deviation_angle[1] * RAD_TO_ANGLE + (getnumb2)*360;
   cha.angle3_ref = Chassis_angle.deviation_angle[2] * RAD_TO_ANGLE + (getnumb3)*360;
   cha.angle4_ref = Chassis_angle.deviation_angle[3] * RAD_TO_ANGLE + (getnumb4)*360;
	}
	Ref_Fdb(cha.angle1_ref,cha.angle1_fdb);
	Ref_Fdb(cha.angle2_ref,cha.angle2_fdb);
	Ref_Fdb(cha.angle3_ref,cha.angle3_fdb);
	Ref_Fdb(cha.angle4_ref,cha.angle4_fdb);
	
	//比较重要
	aa=chassisRamp.Calc(&chassisRamp);
		
	switch(Vehicle_Num)
	{
		case 3:
				Chassis_angle.handle_speed1[0] = -Chassis_angle.handle_speed1[0];
//				Chassis_angle.handle_speed1[1] = -Chassis_angle.handle_speed1[1];
				Chassis_angle.handle_speed1[2] = -Chassis_angle.handle_speed1[2];
//				Chassis_angle.handle_speed1[3] = -Chassis_angle.handle_speed1[3];
		break;
		case 4:
				Chassis_angle.handle_speed1[0] = -Chassis_angle.handle_speed1[0];
				Chassis_angle.handle_speed1[1] = -Chassis_angle.handle_speed1[1];
				Chassis_angle.handle_speed1[2] = -Chassis_angle.handle_speed1[2];
				Chassis_angle.handle_speed1[3] = -Chassis_angle.handle_speed1[3];
		break;
		case 5:
				Chassis_angle.handle_speed1[0] = -Chassis_angle.handle_speed1[0];
//		  Chassis_angle.handle_speed1[1] = -Chassis_angle.handle_speed1[1];
//			Chassis_angle.handle_speed1[2] = -Chassis_angle.handle_speed1[2];
//				Chassis_angle.handle_speed1[3] = -Chassis_angle.handle_speed1[3];
		break;
		case 6:
//				Chassis_angle.handle_speed1[0] = -Chassis_angle.handle_speed1[0];
		  Chassis_angle.handle_speed1[1] = -Chassis_angle.handle_speed1[1];
//			Chassis_angle.handle_speed1[2] = -Chassis_angle.handle_speed1[2];
//				Chassis_angle.handle_speed1[3] = -Chassis_angle.handle_speed1[3];
		default:
			
		break;
	}
	
	if((cha.angle1_ref - cha.angle1_fdb) > 90)
{cha.angle1_ref -= 180;
	Chassis_angle.handle_speed1[0] = -Chassis_angle.handle_speed1[0];
}
else if((cha.angle1_ref - cha.angle1_fdb) < (-90))
{cha.angle1_ref += 180;
	Chassis_angle.handle_speed1[0] = -Chassis_angle.handle_speed1[0];
}
	
	if((cha.angle2_ref - cha.angle2_fdb) > 90)
{cha.angle2_ref -= 180;
	Chassis_angle.handle_speed1[1] = -Chassis_angle.handle_speed1[1];
}
else if((cha.angle2_ref - cha.angle2_fdb) < (-90))
{cha.angle2_ref += 180;
	Chassis_angle.handle_speed1[1] = -Chassis_angle.handle_speed1[1];
}
if((cha.angle3_ref - cha.angle3_fdb) > 90)
{cha.angle3_ref -= 180;
	Chassis_angle.handle_speed1[2] = -Chassis_angle.handle_speed1[2];
}
else if((cha.angle3_ref - cha.angle3_fdb) < (-90))
{cha.angle3_ref += 180;
	Chassis_angle.handle_speed1[2] = -Chassis_angle.handle_speed1[2];
}
	
if((cha.angle4_ref - cha.angle4_fdb) > 90)
{cha.angle4_ref -= 180;
	Chassis_angle.handle_speed1[3] = -Chassis_angle.handle_speed1[3];
}
else if((cha.angle4_ref - cha.angle4_fdb) < (-90))
{cha.angle4_ref += 180;
	Chassis_angle.handle_speed1[3] = -Chassis_angle.handle_speed1[3];
}
pid_calc(&pid_cha1_angle[0],cha.angle1_fdb, cha.angle1_ref);
pid_calc(&pid_cha1_angle[1],cha.angle2_fdb, cha.angle2_ref);
pid_calc(&pid_cha1_angle[2],cha.angle3_fdb, cha.angle3_ref);
pid_calc(&pid_cha1_angle[3],cha.angle4_fdb, cha.angle4_ref);


	if(Chassis_angle.get_control_flag==1||Chassis_angle.get_control_flag==2)
	{
					
					
					cha.speed1_fdb = GM1Encoder1.filter_rate ;
					cha.speed2_fdb = GM2Encoder1.filter_rate ;
					cha.speed3_fdb = GM3Encoder1.filter_rate ;
					cha.speed4_fdb = GM4Encoder1.filter_rate ;
										
					getlastangle1=GM1Encoder1.ecd_angle;
					getlastangle2=GM2Encoder1.ecd_angle;
					getlastangle3=GM3Encoder1.ecd_angle;
					getlastangle4=GM4Encoder1.ecd_angle;
					
					
//				Set_Gimbal_Current1(CAN2,((int16_t)pid_cha1_angle[0].out),((int16_t)pid_cha1_angle[1].out),((int16_t)pid_cha1_angle[2].out),((int16_t)pid_cha1_angle[3].out));	
//					
				  cha.speed1_ref = pid_cha1_angle[0].out;//乘以60，除以360
					cha.speed2_ref = pid_cha1_angle[1].out;
					cha.speed3_ref = pid_cha1_angle[2].out;
					cha.speed4_ref = pid_cha1_angle[3].out;
//					pid_calc(&pid_cha1_speed[0],cha.speed1_fdb, cha.speed1_ref);
//					pid_calc(&pid_cha1_speed[1],cha.speed2_fdb, cha.speed2_ref);
//					pid_calc(&pid_cha1_speed[2],cha.speed3_fdb, cha.speed3_ref);
//					pid_calc(&pid_cha1_speed[3],cha.speed4_fdb, cha.speed4_ref);
//					Set_Gimbal_Current1(CAN2,((int16_t)pid_cha1_speed[0].out),((int16_t)pid_cha1_speed[1].out),((int16_t)pid_cha1_speed[2].out),((int16_t)pid_cha1_speed[3].out));
////			{Set_Gimbal_Current1(CAN2,0,0,((int16_t)pid_cha1_speed[2].out),0);}
//Set_Gimbal_Current1(CAN2,0,0,((int16_t)pid_cha1_speed[2].out),0);
	}
				else
				{//Set_Gimbal_Current1(CAN2,0,0,0,0);

}}


float transition1[4];
float transition2[4];
float transition3[4];
float transition4[4];

float retransition1_angle[4];
float retransition2[4];
float retransition3[4];
float retransition4[4];


int yaw_num_get;
float deviation_angle_flag[4];
float angle_flag[4];

int abbc;
int abbc1;

//float leftangle;
//float rightangle;
void start_angle_handle(void)
{
//leftangle=Chassis_angle.get_speedw/4000;
//VAL_LIMIT(leftangle,-0.15,0.15);
if(Chassis_angle.get_mode_flag==2)
{
if(Chassis_angle.get_speedw>0)
{Chassis_angle.get_yaw_angle+=leftangle;}
else if(Chassis_angle.get_speedw<0)
{Chassis_angle.get_yaw_angle+=leftangle;}
}
//	Chassis_angle.get_speedw=Chassis_angle.get_speedw*2;
Chassis_angle.start_angle[0] = Chassis_angle.get_yaw_angle + ((3*PI)/4);
Chassis_angle.start_angle[1] = Chassis_angle.get_yaw_angle + ((5*PI)/4);
Chassis_angle.start_angle[2] = Chassis_angle.get_yaw_angle + ((7*PI)/4);
Chassis_angle.start_angle[3] = Chassis_angle.get_yaw_angle + ((1*PI)/4);

for(int k=0;k<4;k++)
{
	
	if(Chassis_angle.start_angle[k]>= 2*PI)
	{Chassis_angle.start_angle[k] -= 2*PI;}
	else if(Chassis_angle.start_angle[k] < 0)
	{Chassis_angle.start_angle[k] += 2*PI;}
}
for(int k=0;k<4;k++)
{
 Chassis_angle.include_angle[k] = PI + Chassis_angle.Remote_angle - Chassis_angle.start_angle[k];
	if(Chassis_angle.Remote_speed <= 50)
  {Chassis_angle.Remote_speed = 0;}
		if(Chassis_angle.include_angle[k]>(2*PI))
	{Chassis_angle.include_angle[k] -= 2*PI;}
	else if(Chassis_angle.include_angle[k] <0)
	{Chassis_angle.include_angle[k] += 2*PI;}	
Chassis_angle.handle_speed[k] = sqrt(Chassis_angle.get_speedw*Chassis_angle.get_speedw + 
Chassis_angle.Remote_speed*Chassis_angle.Remote_speed - 2*Chassis_angle.get_speedw*Chassis_angle.Remote_speed*(cos(Chassis_angle.include_angle[k])));
	
	Chassis_angle.handle_speed1[k] = sqrt(Chassis_angle.get_speedw*Chassis_angle.get_speedw + 
(Chassis_angle.Remote_speed*power_limit_start_flag)*(power_limit_start_flag*Chassis_angle.Remote_speed) 
	- 2*Chassis_angle.get_speedw*(Chassis_angle.Remote_speed*power_limit_start_flag)*(cos(Chassis_angle.include_angle[k])));

}
for(int k=0;k<4;k++)
{
	
	transition1[k] = sin(Chassis_angle.include_angle[k]);
	transition2[k] = Chassis_angle.Remote_speed*transition1[k];
	transition3[k] = (transition2[k])/Chassis_angle.handle_speed[k];	
	VAL_LIMIT(transition3[k],-1,1);
	Chassis_angle.deviation_angle[k] = asin(transition3[k]);
	
	retransition2[k] = Chassis_angle.get_speedw*Chassis_angle.get_speedw+Chassis_angle.handle_speed[k]*Chassis_angle.handle_speed[k];
	retransition3[k] = Chassis_angle.Remote_speed*Chassis_angle.Remote_speed;
	
	if(Chassis_angle.get_speedw==0)
	{
	if(Chassis_angle.include_angle[k]>=0&&Chassis_angle.include_angle[k]<=(PI/2))
	{
	Chassis_angle.deviation_angle[k] += PI;
	Chassis_angle.deviation_angle[k] = -Chassis_angle.deviation_angle[k];
	}
	else if(Chassis_angle.include_angle[k]>=(3*PI/2)&&Chassis_angle.include_angle[k]<=(2*PI))
	{Chassis_angle.deviation_angle[k] += PI;
	Chassis_angle.deviation_angle[k] = -Chassis_angle.deviation_angle[k];}
	}
	
	else 
	{
	if(Chassis_angle.get_speedw<0)
	{Chassis_angle.deviation_angle[k] += PI;
	Chassis_angle.deviation_angle[k] = -Chassis_angle.deviation_angle[k];}
	
	if(retransition3[k]>retransition2[k])		
	{Chassis_angle.deviation_angle[k] += PI;
	Chassis_angle.deviation_angle[k] = -Chassis_angle.deviation_angle[k];}
}
	
	
	
	
	
//	逻辑不清楚但勉强可以用的代码,可以用做备用解算。
//  retransition2[k] = Chassis_angle.get_speedw*transition1[k];
//	retransition3[k] = retransition2[k]/Chassis_angle.handle_speed[k];
//	VAL_LIMIT(retransition3[k],-1,1);
//	Chassis_angle.redeviation_angle[k] = asin(transition3[k]);
//	
//	
//	retransition1_angle[k]= asin(transition1[k]);
//	angle_flag[k] = fabs(Chassis_angle.deviation_angle[k]+Chassis_angle.redeviation_angle[k]+retransition1_angle[k]);

//if(Chassis_angle.get_mode_flag == 1&&(Chassis_angle.get_yaw_angle<0.57&&Chassis_angle.get_yaw_angle>5.7))
//{
//	if(Chassis_angle.include_angle[k]>=0&&Chassis_angle.include_angle[k]<=(PI/2))
//	{Chassis_angle.deviation_angle[k] += PI;
//	Chassis_angle.deviation_angle[k] = -Chassis_angle.deviation_angle[k];
//	}
//	else if(Chassis_angle.include_angle[k]>=(3*PI/2)&&Chassis_angle.include_angle[k]<=(2*PI))
//	{Chassis_angle.deviation_angle[k] += PI;
//	Chassis_angle.deviation_angle[k] = -Chassis_angle.deviation_angle[k];}

//}	
//else
//{
//if(Chassis_angle.get_speedw<=0)
//		{abbc++;
//		Chassis_angle.deviation_angle[k] += PI;
//   	Chassis_angle.deviation_angle[k] = -Chassis_angle.deviation_angle[k];
//		}
//		else
//		{abbc1++;}
//		
//}

	}



START_CHASSIS_6020();

}
void chassis_task1()
{
	if(fabs(Chassis_angle.handle_speed1[0])<50&&fabs(Chassis_angle.handle_speed1[1]<50)&&fabs(Chassis_angle.handle_speed1[2]<50)&&fabs(Chassis_angle.handle_speed1[3]<50))
	{
		run_flag=0;
	}
	else
	{
		run_flag=1;
	}
	
		if((Chassis_angle.get_control_flag==1||Chassis_angle.get_control_flag==2)&&run_flag)//
	{

					Set_Gimbal_Current1(CAN2,\
		((int16_t)pid_cha1_speed[0].out),\
		((int16_t)pid_cha1_speed[1].out),\
		((int16_t)pid_cha1_speed[2].out),\
		((int16_t)pid_cha1_speed[3].out));
		CloseDoor;
	}
    else{OpenDoor;
		Set_Gimbal_Current1(CAN2,0,0,0,0);
}
}


void chassis_param_init(void)//底盘参数初始化
{
  memset(&chassis, 0, sizeof(chassis_t));
  chassis.ctrl_mode      = CHASSIS_STOP;
  chassis.last_ctrl_mode = CHASSIS_RELAX;
  power_limit.power_limit_model = CAP_LIMIT;
  chassis_speed_mode = NORMAL_SPEED_MODE;
	
  chassis.position_ref = 0;
  chassis_rotate_flag = 0;
  PID_struct_init(&pid_front_distance, POSITION_PID, 120, 10, 0.50f, 0.00005f,8);
  PID_struct_init(&pid_right_distance, POSITION_PID, 120, 10, 0.50f, 0.00005f,10);
  PID_struct_init(&pid_angle_distance, POSITION_PID, 30, 10, 0.15f, 0,0);
  PID_struct_init(&pid_front_distance, POSITION_PID, 120, 10, 0.27f, 0.0001f,4);
  PID_struct_init(&pid_right_distance, POSITION_PID, 120, 10, 0.28f, 0.00015f,4);
  PID_struct_init(&pid_angle_distance, POSITION_PID, 30, 10, 0.15f, 0,0);
	  
		PID_struct_init(&pid_cha1_angle[0], POSITION_PID, 8000, 10, 8,0.1f,3);//24, 0.2f,20);
  	PID_struct_init(&pid_cha1_speed[0], POSITION_PID, 15000, 500, 210,0.1f,2);//38,0.5f,20);
	
		PID_struct_init(&pid_cha1_angle[1], POSITION_PID, 8000, 10, 8,0.1f,5);//25, 0.2f,15);
  	PID_struct_init(&pid_cha1_speed[1], POSITION_PID, 15000, 500, 180,0.1,4);//39,0.5f,20);
	
	  PID_struct_init(&pid_cha1_angle[2], POSITION_PID, 8000, 10, 8,0.1f,4);//20, 0.2f,20);
  	PID_struct_init(&pid_cha1_speed[2], POSITION_PID, 15000, 500, 200,0.5,8);//40,0.5f,20);
		
    PID_struct_init(&pid_cha1_angle[3], POSITION_PID, 8000, 10, 9,0.4f,4);//23, 0.2f,15);
  	PID_struct_init(&pid_cha1_speed[3], POSITION_PID, 15000, 500, 200,0.1,10);//42,0.5f,20);

#if STANDARD == 3
  for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_spd[k], POSITION_PID,15000, 15000,25,0.1, 5.0f); //24 0.3 10    38.0f,3.0f, 40
    }
  PID_struct_init(&pid_chassis_angle, POSITION_PID, MAX_CHASSIS_VR_SPEED, 70, 5.0f, 0.01,30.0f);//xisanhao
#elif STANDARD == 4
  for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_spd[k], POSITION_PID,15000, 15000,25,0.4, 0); //24 0.3 10    38.0f,3.0f, 40
    }
  PID_struct_init(&pid_chassis_angle, POSITION_PID, MAX_CHASSIS_VR_SPEED, 70, 5.0f, 0.02,30.0f);//新四号
	#elif STANDARD == 5
		for(int k=0;k<4;k++)
		{
      PID_struct_init(&pid_spd[k], POSITION_PID,15000, 15000,25,0.1, 5.0f); //24 0.3 10    38.0f,3.0f, 40
		}
  PID_struct_init(&pid_chassis_angle, POSITION_PID, MAX_CHASSIS_VR_SPEED, 70, 5.0f, 0.01,30.0f);//xisanhao
#elif STANDARD == 6
  for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_spd[k], POSITION_PID,15000, 15000,25,0.1, 5); //24 0.3 10    38.0f,3.0f, 40
    }
  PID_struct_init(&pid_chassis_angle, POSITION_PID, MAX_CHASSIS_VR_SPEED, 70, 5.0f, 0.02,30.0f);//新四号
#endif
}