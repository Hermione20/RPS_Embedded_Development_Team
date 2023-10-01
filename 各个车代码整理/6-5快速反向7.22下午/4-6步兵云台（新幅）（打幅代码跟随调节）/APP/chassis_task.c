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
cha_pid_t cha;
chassis_speed_mode_e chassis_speed_mode;
u8 CanVcap_flag = 0;
u8 chassis_rotate_flag = 0;
float car_velocity = 0;
uint16_t forward_back_speed = 0;
uint16_t left_right_speed = 0;


int chassis_speedly_mode;

float start_angle = {0};


float get_yaw_angle;

int yaw_num_get;
int chassis_mode;
void chassis_task(void)
{	
  
		yaw_num_get = -GMYawEncoder.ecd_angle / 360;
	if (-GMYawEncoder.ecd_angle < 0)
	{yaw_num_get -= 1;}
Chassis_angle.get_yaw_angle =	(-GMYawEncoder.ecd_angle - yaw_num_get * 360) * ANGLE_TO_RAD;

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
	
    //计算角度
		remotecontrolangle();

    //底盘模式
		if(chassis.ctrl_mode==MANUAL_FOLLOW_GIMBAL)
		{chassis_mode=1;}
		else if(chassis.ctrl_mode==CHASSIS_ROTATE)
		{chassis_mode=2;}
		if(reset_flag==1)
		{chassis_mode=3;}
    
		//分离与跟随以及是否shift
		if(high_speed_flag==0&&chassis.ctrl_mode==CHASSIS_SEPARATE)
		{chassis_speedly_mode=0;}
		else if(high_speed_flag==1&&chassis.ctrl_mode==CHASSIS_SEPARATE)
		{chassis_speedly_mode=1;}
		else if(high_speed_flag==0&&chassis.ctrl_mode==MANUAL_FOLLOW_GIMBAL)
		{chassis_speedly_mode=2;}
		else if(high_speed_flag==1&&chassis.ctrl_mode==MANUAL_FOLLOW_GIMBAL)
		{chassis_speedly_mode=3;}
		
		if (chassis_is_controllable())
		{							
			if(Chassis_angle.get_speedw<0)
			{sendcan1(CAN2, Chassis_angle.Remote_speed,Chassis_angle.Remote_angle*10000,-Chassis_angle.get_speedw*10,Chassis_angle.get_yaw_angle*10000);
			sendcan2(CAN2,1,chassis_mode,chassis_speedly_mode,GMYawEncoder.filter_rate);
			}
			else
			{
				sendcan1(CAN2, Chassis_angle.Remote_speed,Chassis_angle.Remote_angle*10000,Chassis_angle.get_speedw*10,Chassis_angle.get_yaw_angle*10000);
				sendcan2(CAN2,2,chassis_mode,chassis_speedly_mode,GMYawEncoder.filter_rate);
			}
		}
		else
		{  
			 sendcan2(CAN2,0,chassis_mode,0,GMYawEncoder.filter_rate);					
			 sendcan1(CAN2,0,0,0,0);
		}
				
		sendcan3(CAN2,  judge_rece_mesg.power_heat_data.chassis_power_buffer, 
		judge_rece_mesg.game_robot_state.mains_power_chassis_output, judge_rece_mesg.game_robot_state.chassis_power_limit,
		judge_rece_mesg.power_heat_data.chassis_power);
				
}


void chassis_stop_handle(void)
{

  chassis.vy = 0;
  chassis.vx = 0;
  chassis.vw = 0;
}


static void chassis_twist_handle(void)
{
  chassis.vw = pid_calc(&pid_chassis_angle, GMYawEncoder.ecd_angle, chassis.position_ref);
}


float abbccd;
float vw_homing;
float homing_flag,last_homing_flag;
void follow_gimbal_handle(void)
{

  if((Key_Flag.Key_A_D_Flag == 1)||(Key_Flag.Key_W_S_Flag == 1))
    {
      if(chassis_speed_mode == NORMAL_SPEED_MODE)
        {
          forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
          left_right_speed = NORMAL_LEFT_RIGHT_SPEED;
        }
      else if(chassis_speed_mode == HIGH_SPEED_MODE&&chassis.ctrl_mode==CHASSIS_ROTATE)
        {			
					forward_back_speed=MIDDLE_FORWARD_BACK_SPEED;
          left_right_speed=MIDDLE_LEFT_RIGHT_SPEED;
				}
			else
			  {					
          forward_back_speed=HIGH_FORWARD_BACK_SPEED;
          left_right_speed=HIGH_LEFT_RIGHT_SPEED;
        }
    }
  else
    {
      FBSpeedRamp.ResetCounter(&FBSpeedRamp);
    }


//加入麦轮解算前
  chassis.vy = ChassisSpeedRef.left_right_ref;
  chassis.vx = ChassisSpeedRef.forward_back_ref;
//加入麦轮解算后
//		chassis.sin_chassis_angle = sin(GMYawEncoder.ecd_angle*ANGLE_TO_RAD);
//    chassis.cos_chassis_angle = cos(GMYawEncoder.ecd_angle*ANGLE_TO_RAD);
//    
//    chassis.foward_back_to_foward_back_rotate_speed = ChassisSpeedRef.forward_back_ref*chassis.cos_chassis_angle;
//    chassis.foward_back_to_left_right_rotate_speed = ChassisSpeedRef.forward_back_ref*chassis.sin_chassis_angle;
//    chassis.left_right_to_foward_back_rotate_speed = -ChassisSpeedRef.left_right_ref*chassis.sin_chassis_angle;
//    chassis.left_right_to_left_right_rotate_speed = ChassisSpeedRef.left_right_ref*chassis.cos_chassis_angle;
//    
//    chassis.vy = chassis.foward_back_to_left_right_rotate_speed + chassis.left_right_to_left_right_rotate_speed;
//    chassis.vx = chassis.foward_back_to_foward_back_rotate_speed + chassis.left_right_to_foward_back_rotate_speed;
		
		
		if(Chassis_angle.get_yaw_angle>=PI)
	{get_yaw_angle=Chassis_angle.get_yaw_angle-(2*PI);}
	else
	{get_yaw_angle=Chassis_angle.get_yaw_angle;}
	vw_homing=0;

	
  if (chassis.follow_gimbal&&gim.ctrl_mode!=GIMBAL_AUTO_SMALL_BUFF&&gim.ctrl_mode!=GIMBAL_AUTO_BIG_BUFF)
    {
       Chassis_angle.get_speedw = pid_calc(&pid_chassis_angle,get_yaw_angle,vw_homing); 
    }
  else
    {
       Chassis_angle.get_speedw = 0;
    }

	if(fabs(pid_chassis_angle.get-pid_chassis_angle.set)<0.05)
	{
		Chassis_angle.get_speedw = 0;
	}
}

void separate_gimbal_handle(void)
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
    }

  chassis.vy = ChassisSpeedRef.left_right_ref;
  chassis.vx = ChassisSpeedRef.forward_back_ref;
		
	Chassis_angle.get_speedw = 0;

}

u32 rotate_mode_count = 0;
u8  rotate_mode = 0;
u8 sin_constant_flag;
u8 get_speedwf_lag = 0;
u16 gyro_speed=400;
void rotate_follow_gimbal_handle(void)
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
    }
		if(high_speed_flag==1)
		{
			gyro_speed=550;
		}
		else
		{
			gyro_speed=400;
		}
if(gim.ctrl_mode==GIMBAL_AUTO_SMALL_BUFF||gim.ctrl_mode==GIMBAL_AUTO_BIG_BUFF)
{gyro_speed=100;}
  chassis.vy = ChassisSpeedRef.left_right_ref;
  chassis.vx = ChassisSpeedRef.forward_back_ref;


if(chassis.last_ctrl_mode!=CHASSIS_ROTATE)
{
	if(get_speedwf_lag==0)
	{get_speedwf_lag=1;}
	else if(get_speedwf_lag==1)
	{get_speedwf_lag=0;}
}


if(get_speedwf_lag==0)
{Chassis_angle.get_speedw = -gyro_speed;}
else if(get_speedwf_lag==1)
{Chassis_angle.get_speedw = gyro_speed;}


}



 void reverse_follow_gimbal_handle(void)
{
   if((Key_Flag.Key_A_D_Flag == 1)||(Key_Flag.Key_W_S_Flag == 1))
    {
      if(chassis_speed_mode == NORMAL_SPEED_MODE)
        {
          forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
          left_right_speed = NORMAL_LEFT_RIGHT_SPEED;
        }
      else if(chassis_speed_mode == HIGH_SPEED_MODE&&chassis.ctrl_mode==CHASSIS_ROTATE)
        {			
					forward_back_speed=MIDDLE_FORWARD_BACK_SPEED;
          left_right_speed=MIDDLE_LEFT_RIGHT_SPEED;
				}
			else
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


	if(Chassis_angle.get_yaw_angle>=2*PI)
	{get_yaw_angle=Chassis_angle.get_yaw_angle-(2*PI);}
	else
	{get_yaw_angle=Chassis_angle.get_yaw_angle;}
	vw_homing=0;

  if (chassis.follow_gimbal&&gim.ctrl_mode!=GIMBAL_AUTO_SMALL_BUFF&&gim.ctrl_mode!=GIMBAL_AUTO_BIG_BUFF)
    {

       Chassis_angle.get_speedw = pid_calc(&pid_chassis_angle,get_yaw_angle,vw_homing+PI);    
    }
  else
    {
       Chassis_angle.get_speedw = 0;
    }

	if(fabs(pid_chassis_angle.get-pid_chassis_angle.set)<0.05)
		{
			Chassis_angle.get_speedw = 0;
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









int Check_Vcap_recieve(void)
{
  not_receive_time++;
  if(not_receive_time>50)
    return 0;
  else
    return 1;

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

//	PID_struct_init(&pid_cha1_angle[0], POSITION_PID, 29000, 29000, 240,10,40);//24, 0.2f,20);
//	PID_struct_init(&pid_cha1_angle[1], POSITION_PID, 29000, 29000, 240,10,40);//24, 0.2f,20);
//	PID_struct_init(&pid_cha1_angle[2], POSITION_PID, 29000, 29000, 240,10,40);//24, 0.2f,20);
//	PID_struct_init(&pid_cha1_angle[3], POSITION_PID, 29000, 29000, 240,10,40);//24, 0.2f,20);
	
	  
//		PID_struct_init(&pid_cha1_angle[0], POSITION_PID, 200, 29000, 24,0.2f,20);//24, 0.2f,20);
//  	PID_struct_init(&pid_cha1_speed[0], POSITION_PID, 25000, 25000, 38,0.5f,20);//38,0.5f,20);
//	
//		PID_struct_init(&pid_cha1_angle[1], POSITION_PID, 200, 29000, 25,0.2,15);//25, 0.2f,15);
//  	PID_struct_init(&pid_cha1_speed[1], POSITION_PID, 25000, 25000, 39,0.5,20);//39,0.5f,20);
//	
//	  PID_struct_init(&pid_cha1_angle[2], POSITION_PID, 200, 29000, 20,0.2f,20);//20, 0.2f,20);
//  	PID_struct_init(&pid_cha1_speed[2], POSITION_PID, 25000, 25000, 40,0.5f,20);//40,0.5f,20);

////  PID_struct_init(&pid_cha1_angle[k], POSITION_PID, 100, 29000, 20, 0,0);
////	PID_struct_init(&pid_cha1_speed[k], POSITION_PID, 25000, 25000, 40, 0,0);
//    PID_struct_init(&pid_cha1_angle[3], POSITION_PID, 200, 29000, 23,0.2f,15);//23, 0.2f,15);
//  	PID_struct_init(&pid_cha1_speed[3], POSITION_PID, 25000, 25000, 42,0.5f,20);//42,0.5f,20);
	

#if STANDARD == 3
  for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_spd[k], POSITION_PID,15000, 15000,24,0.3, 10); //24 0.3 10    38.0f,3.0f, 40
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