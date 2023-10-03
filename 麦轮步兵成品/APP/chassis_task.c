#include  "main.h"
#include "AHRS_MiddleWare.h"
/* chassis twist angle (degree)*/
#define TWIST_ANGLE  40
/* twist period time (ms) */
//#define TWIST_PERIOD   1250
#define TWIST_PERIOD   1000
/* warning surplus energy */
/* chassis task global parameter */
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


void chassis_task(void)
{
  car_velocity = CM1Encoder.rate_rpm*0.000560998688f;

  switch (chassis.ctrl_mode)
    {

    case DODGE_MODE:       //底盘躲避
    {
      chassis.vx = 0;
      chassis.vy = 0;
      chassis_twist_handle();
    }
    break;

    case CHASSIS_STOP:
    {
      chassis_stop_handle();
    }
    break;

    case MANUAL_FOLLOW_GIMBAL:  //跟随云台模式
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
    case CHASSIS_CHANGE_REVERSE:
    {
      chassis_change_reverse_handle();
    }
    break;
    case CHASSIS_SEPARATE:
    {
      separate_gimbal_handle();
    }
    break;
    case CHASSIS_AUTO_SUP:
    {
      auto_sup_handle();
    }
    break;
    default:
    {
      chassis_stop_handle();
    }
    break;
    }

  mecanum_calc(chassis.vx, chassis.vy, chassis.vw, chassis.wheel_speed_ref);
  chassis.wheel_speed_fdb[0]=CM1Encoder.filter_rate;
  chassis.wheel_speed_fdb[1]=CM2Encoder.filter_rate;
  chassis.wheel_speed_fdb[2]=CM3Encoder.filter_rate;
  chassis.wheel_speed_fdb[3]=CM4Encoder.filter_rate;

  if(judge_rece_mesg.game_robot_state.mains_power_gimbal_output==0)
    {
      for (int i = 0; i < 4; i++)
        {
          chassis.wheel_speed_ref[i]=0;
        }
				
    }
#if POWER_LIMT==1
  power_limit_handle();// 热量限制
#endif

  for (int i = 0; i < 4; i++)
    {
      chassis.current[i] = pid_calc(&pid_spd[i], chassis.wheel_speed_fdb[i], power_limit_rate*chassis.wheel_speed_ref[i]);
    }

#if POWER_LIMT==0
  power_limit_handle();// 热量限制
#endif

  if (!chassis_is_controllable())
    {
      Set_CM_Speed(CAN2, 0,0,0,0);
    }
  else
    {
      Set_CM_Speed(CAN2, CHASSIS_SPEED_ATTENUATION * chassis.current[0],\
                   CHASSIS_SPEED_ATTENUATION * chassis.current[1],\
                   CHASSIS_SPEED_ATTENUATION * chassis.current[2],\
                   CHASSIS_SPEED_ATTENUATION * chassis.current[3]);

    }

}


void chassis_stop_handle(void)
{

  chassis.vy = 0;
  chassis.vx = 0;
  chassis.vw = 0;
}


u32 reverse_init_time = 0;

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




  if (chassis.follow_gimbal&&gim.ctrl_mode!=GIMBAL_AUTO_SMALL_BUFF&&gim.ctrl_mode!=GIMBAL_AUTO_BIG_BUFF)
    {

      chassis.vw = pid_calc(&pid_chassis_angle,GMYawEncoder.ecd_angle,chassis.position_ref);
    }
  else
    {
      chassis.vw = 0;
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

void rotate_follow_gimbal_handle(void)
{
  if(chassis_speed_mode == NORMAL_SPEED_MODE)
    {
      forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;//*chassisRamp.Calc(&chassisRamp);
      left_right_speed = NORMAL_LEFT_RIGHT_SPEED;//*chassisRamp.Calc(&chassisRamp);
    }
  else if(chassis_speed_mode == HIGH_SPEED_MODE)
    {
      forward_back_speed=HIGH_FORWARD_BACK_SPEED;//*chassisRamp.Calc(&chassisRamp);
      left_right_speed=HIGH_LEFT_RIGHT_SPEED;//*chassisRamp.Calc(&chassisRamp);
    }

  if(gim.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)
    {
      int chassis_rotate_speed_vw_ref=65;
      if (chassis.follow_gimbal)
        {
          if(chassis_rotate_flag == 1)
            {
              if(judge_rece_mesg.power_heat_data.chassis_power_buffer>20
                  &&chassis_rotate_buffer_flag
                  &&chassis_rotate_buffer_time<50)
                chassis.vw = -chassis_rotate_speed_vw_ref-20;   //此参数为转速
              else
                chassis.vw = -chassis_rotate_speed_vw_ref;

            }
          else if(chassis_rotate_flag == 0)
            {
              if(judge_rece_mesg.power_heat_data.chassis_power_buffer>20
                  &&chassis_rotate_buffer_flag
                  &&chassis_rotate_buffer_time<50)
                chassis.vw = chassis_rotate_speed_vw_ref +20;    //此参数为转速
              else
                chassis.vw =chassis_rotate_speed_vw_ref;

            }
        }
      else
        {
          chassis.vw = 0;
        }
    }
  chassis_rotate_speed.sin_chassis_angle = arm_sin_f32(GMYawEncoder.ecd_angle*ANGLE_TO_RAD);
  chassis_rotate_speed.cos_chassis_angle = arm_cos_f32(GMYawEncoder.ecd_angle*ANGLE_TO_RAD);

  chassis_rotate_speed.foward_back_to_foward_back_rotate_speed = ChassisSpeedRef.forward_back_ref*chassis_rotate_speed.cos_chassis_angle;
  chassis_rotate_speed.foward_back_to_left_right_rotate_speed  = -ChassisSpeedRef.forward_back_ref*chassis_rotate_speed.sin_chassis_angle;
  chassis_rotate_speed.left_right_to_foward_back_rotate_speed  = ChassisSpeedRef.left_right_ref*chassis_rotate_speed.sin_chassis_angle;
  chassis_rotate_speed.left_right_to_left_right_rotate_speed   = ChassisSpeedRef.left_right_ref*chassis_rotate_speed.cos_chassis_angle;
  chassis.vy = chassis_rotate_speed.foward_back_to_left_right_rotate_speed  + chassis_rotate_speed.left_right_to_left_right_rotate_speed;
  chassis.vx = chassis_rotate_speed.foward_back_to_foward_back_rotate_speed + chassis_rotate_speed.left_right_to_foward_back_rotate_speed;

			          if(rotate_num_ture%2==0)
						{
  chassis.vy = chassis_rotate_speed.foward_back_to_left_right_rotate_speed  + chassis_rotate_speed.left_right_to_left_right_rotate_speed;
  chassis.vx = chassis_rotate_speed.foward_back_to_foward_back_rotate_speed + chassis_rotate_speed.left_right_to_foward_back_rotate_speed;

						}	
						
						
          else if(rotate_num_ture%2==1)
					{
  chassis.vy = -(chassis_rotate_speed.foward_back_to_left_right_rotate_speed  + chassis_rotate_speed.left_right_to_left_right_rotate_speed);
  chassis.vx = -(chassis_rotate_speed.foward_back_to_foward_back_rotate_speed + chassis_rotate_speed.left_right_to_foward_back_rotate_speed);

					}
		

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
float get_max_power(float voltage)
{
  int max_power=0;
  if(voltage>WARNING_VOLTAGE+3)
    max_power=350;
  else
    max_power=(voltage-WARNING_VOLTAGE)/3.0f*200;
  VAL_LIMIT(max_power,0,500);
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

float aaaaa;
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






#if 1
float total_cur_limit;
int32_t total_cur;
u16  Max_Power;
u8  Max_Current   = 6;

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
               (factor*(float)chassis.wheel_speed_ref[motor_number]-chassis.wheel_speed_fdb[motor_number])+ \
               pid_spd[motor_number].iout+ \
               pid_spd[motor_number].d*( \
                                         (factor*(float)chassis.wheel_speed_ref[motor_number]-chassis.wheel_speed_fdb[motor_number])- \
                                         pid_spd[motor_number].err[LAST]);
  return power;
//	return  pid_spd[motor_number].p*(factor*(float)chassis.wheel_speed_ref[motor_number]-chassis.wheel_speed_fdb[motor_number]);
}

static float get_the_limite_rate(float max_power)
{
  float a[4];
  for(int i=0; i<4; i++)
    a[i]=(float)chassis.wheel_speed_ref[i]*(pid_spd[i].p+pid_spd[i].d);
  float b[4];
  for(int i=0; i<4; i++)
    b[i]=-pid_spd[i].p*(float)chassis.wheel_speed_fdb[i]+pid_spd[i].iout \
         -pid_spd[i].d*(float)chassis.wheel_speed_fdb[i]-pid_spd[i].d*pid_spd[i].err[LAST];
  // Max_power=heat_power+drive_power
  //	i_n=a[n]*k+b[n]	带入
  //Max_Power=m*k^2+n*k+o
  //0=m*k^2+n*k+l(l=o-Max_Power)
  float m=(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]+a[3]*a[3])*FACTOR_2;

  float n=2*FACTOR_2*(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]) + \
          FACTOR_1*(a[0] + a[1] + a[2] + a[3]) + \
          I_TIMES_V_TO_WATT*(a[0]*(float)chassis.wheel_speed_fdb[0] + \
                             a[1]*(float)chassis.wheel_speed_fdb[1] + \
                             a[2]*(float)chassis.wheel_speed_fdb[2] + \
                             a[3]*(float)chassis.wheel_speed_fdb[3]);

  float l=(b[0]*b[0] + b[1]*b[1] + b[2]*b[2] + b[3]*b[3])*FACTOR_2 + \
          (b[0] + b[1] + b[2] + b[3])*FACTOR_1 + \
          I_TIMES_V_TO_WATT*(b[0]*(float)chassis.wheel_speed_fdb[0] + \
                             b[1]*(float)chassis.wheel_speed_fdb[1] + \
                             b[2]*(float)chassis.wheel_speed_fdb[2] + \
                             b[3]*(float)chassis.wheel_speed_fdb[3]) + \
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
void power_limit_handle(void)
{
    volatilAo=capacitance_message3.out_v;
	capacitance_message1.cap_voltage_filte=volatilAo/100;

  power_limit_rate=get_the_limite_rate(get_max_power(capacitance_message1.cap_voltage_filte));

      VAL_LIMIT(power_limit_rate,0,1);
	
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
      if((capacitance_message2.system_mode==2))   //正常运行并没有离线（离线时最后传回来的是正常运行）
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
//  buffer_power();
//  static u8 canbuf[8];
//  canbuf[0]=0x10;
//  canbuf[1]=Max_Power;
//  canbuf[2]=Max_Current;
//  canbuf[3]=Power_Work_Mode;
//  canbuf[4]=Cap_Reset_flag;
//  canbuf[5]=0;
//  canbuf[6]=0;
//  canbuf[7]=0;
//  POWER_Control(canbuf);//发送8个字节
}
#endif

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
POWER_Control1(10*100,0x603);
}

int Check_Vcap_recieve(void)
{
  not_receive_time++;
  if(not_receive_time>50)
    return 0;
  else
    return 1;

}

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
  if(capacitance_message1.cap_voltage_filte>=23.7)
 {Max_Power=0;}
VAL_LIMIT(Max_Power,0,150);
}

/**
  * @brief  nitialize chassis motor pid parameter
  * @usage  before chassis loop use this function
  */
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

#if STANDARD == 3
  for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_spd[k], POSITION_PID,15000, 15000,24,0.3, 10); //24 0.3 10    38.0f,3.0f, 40
    }
  PID_struct_init(&pid_chassis_angle, POSITION_PID, MAX_CHASSIS_VR_SPEED, 70, 5.0f, 0.01,30.0f);//xisanhao
#elif STANDARD == 4
  for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_spd[k], POSITION_PID,15000, 15000,25,0.4, 0); //24 0.3 10    38.0f,3.0f, 40
    }
  PID_struct_init(&pid_chassis_angle, POSITION_PID, MAX_CHASSIS_VR_SPEED, 70, 5.0f, 0.02,30.0f);//新四号
	#elif STANDARD == 5
  for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_spd[k], POSITION_PID,15000, 15000,26,0.3, 0); //24 0.3 10    38.0f,3.0f, 40
    }
  PID_struct_init(&pid_chassis_angle, POSITION_PID, MAX_CHASSIS_VR_SPEED, 50, 2.6f, 0,10.0f);//xisanhao
#elif STANDARD == 6
  for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_spd[k], POSITION_PID,15000, 15000,25,0.4, 0); //24 0.3 10    38.0f,3.0f, 40
    }
  PID_struct_init(&pid_chassis_angle, POSITION_PID, MAX_CHASSIS_VR_SPEED, 70, 5.0f, 0.02,30.0f);//新四号
#endif
}