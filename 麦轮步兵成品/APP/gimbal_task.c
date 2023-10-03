#include "main.h"
#include "AHRS_MiddleWare.h"
/* gimbal back center time (ms) */
#define BACK_CENTER_TIME 1500
gimbal_t gim;
u32 Gimbal_pitch_mid_point = 0;
int32_t x_bias = 0;
int32_t y_bias = 0;
extern Mouse mouse;
float Delta_Dect_Angle_Yaw,Delta_Dect_Angle_Pit;
Gimbal_Auto_Shoot_t Gimbal_Auto_Shoot;
float IMAGE_LENGTH       =        - 3.45e-3f;
int lost_time;
//自瞄参数
Gimbal_Auto_Shoot_t Gimbal_Auto_Shoot;
float Delta_Dect_Angle_Yaw;
float Delta_Dect_Angle_Pit;
float	Machine_Yaw_Compensation = 0;
float	Machine_Pitch_Compensation = 0;
float Speed_Distence_Compensation = 0;   //射速和距离的pitch轴补偿
float now_distance = 0;
float last_distance = 0;
float Rotate_compensation = 0;

//测试用
float det_angle = 0;
float Image_Gimbal_Delay_Compensation;

double shoot_angle_speed = 0;
double distance_s, distance_x, distance_y;
double x1, x2, x3, x4;
float angle_tan, shoot_radian, shoot_angle;
float test_angle;


void gimbal_task(void)
{//Set_Gimbal_Current1(CAN2,0,3000,0,0);

//	  Distance_handle();

#if IMU == ICM20948
  IMU_getYawPitchRoll();
#elif IMU == HI219
  Hi220_getYawPitchRoll();
#endif

  switch ( gim.ctrl_mode)
    {
    case GIMBAL_INIT:
      init_mode_handle();
      break;
    case GIMBAL_FOLLOW_ZGYRO:
      close_loop_handle();
      break;
    case GIMBAL_AUTO_SMALL_BUFF:
      auto_small_buff_handle();
      break;
    case GIMBAL_AUTO_BIG_BUFF:
      auto_big_buff_handle();
      break;
    case GIMBAL_AUTO_ANGLE:
      auto_angle_handle();
      break;
    case GIMBAL_FOLLOW_CHASSIS:
      gimbal_follow_chassis_handle();
      break;
    default:
      break;
    }
		VAL_LIMIT(gim.pid.pit_angle_ref, PITCH_MIN , PITCH_MAX );
  switch (gim.ctrl_mode)
    {
    case GIMBAL_INIT:
    {
      pid_calc(&pid_yaw, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);
      pid_calc(&pid_pit, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);
      cascade_pid_ctrl();
      if (gimbal_is_controllable())
        {
          pid_calc(&pid_yaw_speed, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
          pid_calc(&pid_pit_speed, -gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref);
          Set_Gimbal_Current(CAN2, -(int16_t)pid_yaw_speed.out, (int16_t)pid_pit_speed.out);
        }
      else
        {
          Set_Gimbal_Current(CAN2, 0, 0);     //yaw + pitch
          gim.ctrl_mode = GIMBAL_RELAX;
        }
    }
    break;

    case GIMBAL_FOLLOW_ZGYRO:
    {
			
      if(autoshoot_mode == NORMAL_SHOOT)
        {
          pid_calc(&pid_yaw, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);
          pid_calc(&pid_pit, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);
          cascade_pid_ctrl();
//				OutData[0]=(int)(pid_yaw.set * 100);
//				OutData[1]=(int)(pid_yaw.get * 100);
//				OutData[2]=(int)(pid_pit.set * 100);
//				OutData[3]=(int)(pid_pit.get* 100);
        }
      else
        {
          pid_calc(&pid_yaw_follow, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);	//计算角度环输出
          pid_calc(&pid_pit_follow, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);	//计算角度环输出
          auto_aim_cascade_pid_ctrl();
        }

      /* safe protect */
      if (gimbal_is_controllable())
        {
          if(autoshoot_mode == NORMAL_SHOOT)
            {
              pid_calc(&pid_yaw_speed, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
              pid_calc(&pid_pit_speed, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref);
              Set_Gimbal_Current(CAN2, -(int16_t)pid_yaw_speed.out, -(int16_t)pid_pit_speed.out);
            }
          else
            {
              pid_calc(&pid_yaw_speed_follow,gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
              pid_calc(&pid_pit_speed_follow, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref);
              Set_Gimbal_Current(CAN2, -(int16_t)pid_yaw_speed_follow.out, -(int16_t)pid_pit_speed_follow.out);
            }
        }
      else
        {
          Set_Gimbal_Current(CAN2, 0, 0);     //yaw + pitch
          gim.ctrl_mode = GIMBAL_RELAX;
        }

    }
    break;
    case GIMBAL_AUTO_SMALL_BUFF:
    {
      pid_calc(&pid_yaw_small_buff, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);
      pid_calc(&pid_pit_small_buff, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);
//					OutData[0]=(int)pid_pit_small_buff.iout * 100;
//					OutData[1]=(int)pid_pit_small_buff.dout * 100;
//					OutData[2]=(int)(pid_pit_small_buff.set * 100);
//					OutData[3]=(int)(pid_pit_small_buff.get* 100);
      gim.pid.pit_speed_ref = pid_pit_small_buff.out;
      gim.pid.yaw_speed_ref = pid_yaw_small_buff.out;
      gim.pid.yaw_speed_fdb = yaw_Gyro;
      gim.pid.pit_speed_fdb = -pitch_Gyro;

//      OutData[0]=(int)(pid_yaw_small_buff.set * 100);
//      OutData[1]=(int)(pid_yaw_small_buff.get * 100);
//      OutData[2]=(int)(pid_pit_small_buff.iout * 100);
//      OutData[3]=(int)(pid_pit_small_buff.out * 100);

      if (gimbal_is_controllable())
        {
          pid_calc(&pid_yaw_speed_small_buff, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
          pid_calc(&pid_pit_speed_small_buff, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref);
          //Set_Gimbal_Current(CAN2, -(int16_t)pid_yaw_speed_small_buff.out, 0);

//					OutData[0]=(int)pid_yaw_speed_small_buff.set * 100;
//					OutData[1]=(int)pid_yaw_speed_small_buff.get * 100;
//					OutData[2]=(int)(pid_yaw_speed_small_buff.set * 100);
//					OutData[3]=(int)(pid_yaw_speed_small_buff.get* 100);

          Set_Gimbal_Current(CAN2, -(int16_t)pid_yaw_speed_small_buff.out, -(int16_t)pid_pit_speed_small_buff.out);
        }
      else
        {
          Set_Gimbal_Current(CAN2, 0, 0);     //yaw + pitch
          gim.ctrl_mode = GIMBAL_RELAX;
        }
    }
    break;

    case GIMBAL_AUTO_BIG_BUFF:
    {
      pid_calc(&pid_yaw_big_buff, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);
      pid_calc(&pid_pit_big_buff, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);

      gim.pid.yaw_speed_ref = pid_yaw_big_buff.out;
      gim.pid.pit_speed_ref = pid_pit_big_buff.out;
      gim.pid.yaw_speed_fdb =	yaw_Gyro;
      gim.pid.pit_speed_fdb =	 - pitch_Gyro;

      if (gimbal_is_controllable())
        {
          pid_calc(&pid_yaw_speed_big_buff, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
          pid_calc(&pid_pit_speed_big_buff, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref);
          //Set_Gimbal_Current(CAN2, 0, -(int16_t)pid_pit_speed_big_buff.out);
          Set_Gimbal_Current(CAN2, -(int16_t)pid_yaw_speed_big_buff.out, -(int16_t)pid_pit_speed_big_buff.out);
        }
      else
        {
          Set_Gimbal_Current(CAN2, 0, 0);     //yaw + pitch
          gim.ctrl_mode = GIMBAL_RELAX;
        }

    }
    break;
    case GIMBAL_AUTO_ANGLE:
    {
      pid_calc(&pid_yaw_auto_angle, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);
      pid_calc(&pid_pit_auto_angle, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);

      gim.pid.yaw_speed_ref = pid_yaw_auto_angle.out;
      gim.pid.pit_speed_ref = pid_pit_auto_angle.out;
      gim.pid.yaw_speed_fdb =	yaw_Gyro;
      gim.pid.pit_speed_fdb =	pitch_Gyro;

      if (gimbal_is_controllable())
        {
          pid_calc(&pid_yaw_speed_auto_angle, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
          pid_calc(&pid_pit_speed_auto_angle, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref);
          Set_Gimbal_Current(CAN2, -(int16_t)pid_yaw_speed_auto_angle.out, (int16_t)pid_pit_speed_auto_angle.out);
        }
      else
        {
          Set_Gimbal_Current(CAN2, 0, 0);     //yaw + pitch
          gim.ctrl_mode = GIMBAL_RELAX;
        }
    }
    break;
    case GIMBAL_FOLLOW_CHASSIS:
    {
      pid_calc(&pid_yaw_follow_chassis_angle, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);
      pid_calc(&pid_pit, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);

      gim.pid.yaw_speed_ref = pid_yaw_follow_chassis_angle.out;
      gim.pid.pit_speed_ref = pid_pit.out;
      gim.pid.yaw_speed_fdb =	yaw_Gyro;
      gim.pid.pit_speed_fdb =	pitch_Gyro;

      if (gimbal_is_controllable())
        {
          pid_calc(&pid_yaw_follow_chassis_speed, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
          pid_calc(&pid_pit_speed, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref);
          Set_Gimbal_Current(CAN2, -(int16_t)pid_yaw_follow_chassis_speed.out, (int16_t)pid_pit_speed.out);
        }
      else
        {
          Set_Gimbal_Current(CAN2, 0, 0);     //yaw + pitch
          gim.ctrl_mode = GIMBAL_RELAX;
        }
    }
    break;
    default:
      break;
    }

}
extern u8 debug_high,debug_low;
void cascade_pid_ctrl(void)
{
  gim.pid.yaw_speed_ref = pid_yaw.out;//LQ
  gim.pid.pit_speed_ref = pid_pit.out;
  gim.pid.yaw_speed_fdb =	yaw_Gyro;
  gim.pid.pit_speed_fdb =	-pitch_Gyro;
}

void auto_aim_cascade_pid_ctrl(void)
{
  gim.pid.pit_speed_ref = pid_pit_follow.out;
  gim.pid.yaw_speed_ref = pid_yaw_follow.out;
  gim.pid.yaw_speed_fdb = yaw_Gyro;
  gim.pid.pit_speed_fdb = -pitch_Gyro;
}


int32_t init_rotate_num = 0;
int32_t init_Yaw_angle = 0;

void init_mode_handle(void)
{
  gim.pid.pit_angle_fdb = GMPitchEncoder.ecd_angle;  //向上为负
  gim.pid.pit_angle_ref = 0;//-GMPitchEncoder.ecd_angle* (1 -GMPitchRamp.Calc(&GMPitchRamp));
  gim.pid.yaw_angle_fdb = -GMYawEncoder.ecd_angle;//

  init_rotate_num = -GMYawEncoder.ecd_angle/360;
  init_Yaw_angle = init_rotate_num*360;

  gim.pid.yaw_angle_ref = init_Yaw_angle;//-GMYawEncoder.ecd_angle*(1 -GMYawRamp.Calc(&GMYawRamp));

  if(gim.pid.yaw_angle_fdb-gim.pid.yaw_angle_ref>=180)
    gim.pid.yaw_angle_ref+=360;
  else if(gim.pid.yaw_angle_fdb-gim.pid.yaw_angle_ref<-180)
    gim.pid.yaw_angle_ref-=360;
     
  if (gim.pid.yaw_angle_fdb-gim.pid.yaw_angle_ref >= -1.5f && gim.pid.yaw_angle_fdb-gim.pid.yaw_angle_ref <= 1.5f&& gim.pid.pit_angle_fdb >= -4 && gim.pid.pit_angle_fdb <= 4)
    {
			
      gim.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
      chassis.follow_gimbal = 1;
      gim.pid.yaw_angle_fdb = yaw_Angle;
      GimbalRef.yaw_angle_dynamic_ref = yaw_Angle;		//陀螺仪向右为正
      GimbalRef.pitch_angle_dynamic_ref = pitch_Angle;	//陀螺仪向上为正
      Gimbal_pitch_mid_point = pitch_Angle;

      chassis.position_ref = GMYawEncoder.ecd_angle;
    }

}
void no_action_handle(void)
{
  gim.pid.yaw_angle_fdb = yaw_Angle;
  GimbalRef.yaw_angle_dynamic_ref = yaw_Angle;
  gim.pid.pit_angle_fdb = pitch_Angle;
  GimbalRef.pitch_angle_dynamic_ref = pitch_Angle;
}

auto_shoot_mode_e  autoshoot_mode = NORMAL_SHOOT ;

float Filter_for_Speed_Prediction(float input,float *data_temp)
{
  uint8_t i = 0;
  float sum=0;

  for(i=1; i<4; i++)
    {
      data_temp[i-1] = data_temp[i];
    }
  data_temp[3] = input;

  for(i=0; i<4; i++)
    {
      sum += data_temp[i];
    }
  return(sum*0.25f);
}

void AutoShootAngleLimit()
{
  VAL_LIMIT(GimbalRef.pitch_angle_dynamic_ref,  - PITCH_MAX,  + PITCH_MAX);
//	VAL_LIMIT(GimbalRef.yaw_angle_dynamic_ref, pid_yaw.get - 60,pid_yaw.get + 60);
}
float YawTimeSpeed=0,PitchTimeSpeed=0;
float Army_Speed_Yaw,Army_Speed_Pit,Gimbal_Speed_Yaw,initanleyaw,initanlepitch;
int addli;
float click_x,click_y;
uint8_t Rece_F;
float new_locationx_last,new_locationy_last,x_temp=0,y_temp=0;
float last_click_x,last_click_y;
float click_flag=1;

double c_angle_y[100],c_angle_p[100]={0},c_time[100],last_time[100];
double h_c_angle[100],l_c_angle[100];
float last_dis,last_dis,c_dis,click_spin,click_spin_flag;

float last_click_p,click_p;
int click_hl;
int ti_auto_shoot,last_ti_auto_shoot;
float t_c,t_shoot,tim_shoot;
int ti_shoot;
float first_time_shoot;
float last_y,last_p;


float last_yaw_angle,last_pitch_angle;
float first_time_click_flag;
float record_mean;
int record_mean_time;
float record_mean_flag;
float yaw_angle_ref,last_yaw_angle_ref,last_last_yaw_angle_ref,last_last_last_yaw_angle_ref;
float last_pid_yaw_angle_ref;
float average_yaw_time[4];
float cycle_time[3];
float cycle_time_before,cycle_time_rear;
float cycle_yaw_before,cycle_yaw_rear;
float last_max_ti_auto_shoot;
float speed_yaw;
float last_auto_yaw,last_auto_pitch;

float YAW_ANGLE_BETWEEN_GUN_CAMERA =1;
float PITCH_ANGLE_BETWEEN_GUN_CAMERA=-2.5;



void close_loop_handle(void)
{
	


  static u8 flag_lost = 0;
  switch (autoshoot_mode)
    {
    case NORMAL_SHOOT:
    {
      gim.pid.pit_angle_fdb = pitch_Angle;
      gim.pid.yaw_angle_fdb = yaw_Angle;
      gim.pid.pit_angle_ref = GimbalRef.pitch_angle_dynamic_ref;
      gim.pid.yaw_angle_ref = GimbalRef.yaw_angle_dynamic_ref;

    }
    break;

    case AUTO_SHOOT:
    {           gim.pid.pit_angle_fdb = pitch_Angle;
                gim.pid.yaw_angle_fdb = yaw_Angle;
      if(new_location.flag)
        {      
				   
              
					   gim.pid.yaw_angle_ref = new_location.x+YAW_ANGLE_BETWEEN_GUN_CAMERA;
              gim.pid.pit_angle_ref = new_location.y+PITCH_ANGLE_BETWEEN_GUN_CAMERA;
						
              last_auto_yaw = new_location.x;
					    last_auto_pitch  = new_location.y;
				}
				}
     }
}




#if STANDARD == 3
float Yaw_remain = -1.7;//向右为正
float pitch_remain=6;//5.2;
#elif STANDARD == 4
float Yaw_remain = -0.2;//向右为正
float pitch_remain=6.15;//5.2;
#elif STANDARD == 5
float Yaw_remain = 0.2;//向右为正
float pitch_remain=2.35;//5.2;
#elif STANDARD == 6
float Yaw_remain = -1.3;//向右为正
float pitch_remain=2.9;//5.2;


#endif
u32 lost_aim_time = 0;

float yaw_angle_ref_aim,pit_angle_ref_aim;
float remain_yaw_angle,remain_pitch_angle;

void auto_small_buff_handle(void)
{
  gim.pid.pit_angle_fdb = pitch_Angle;
  gim.pid.yaw_angle_fdb =  yaw_Angle;
if(UART4_DMA_RX_BUF[0]!=0xBE||xy_0_flag==1)
{xy_o_time++;}
else{xy_o_time=0;}

		if(xy_o_time<1)
		{
				{
          Delta_Dect_Angle_Yaw = RAD_TO_ANGLE * atan2 ( ( (double) new_location.x1 ) * TARGET_SURFACE_LENGTH,FOCAL_LENGTH);
          Delta_Dect_Angle_Pit = RAD_TO_ANGLE * atan2 ( ( (double) new_location.y1 ) * TARGET_SURFACE_WIDTH,FOCAL_LENGTH);
				
			    yaw_angle_ref_aim=Delta_Dect_Angle_Yaw + gim.pid.yaw_angle_fdb;
			    pit_angle_ref_aim=Delta_Dect_Angle_Pit + gim.pid.pit_angle_fdb+pitch_remain;
		
          gim.pid.yaw_angle_ref =  yaw_angle_ref_aim+Yaw_remain;
		
          gim.pid.pit_angle_ref =raw_data_to_pitch_angle(pit_angle_ref_aim);								
				}
		}

		
  //-----------------------------云台限幅-----------------------//
  VAL_LIMIT(gim.pid.pit_angle_ref, PITCH_MIN + (GMPitchEncoder.ecd_angle)+gim.pid.pit_angle_fdb, PITCH_MAX + 5 + (GMPitchEncoder.ecd_angle)+gim.pid.pit_angle_fdb);
//		VAL_LIMIT(gim.pid.yaw_angle_ref, YAW_MIN - (-GMYawEncoder.ecd_angle)+gim.pid.yaw_angle_fdb, YAW_MAX - (-GMYawEncoder.ecd_angle)+gim.pid.yaw_angle_fdb);
//-------------------------------------------------------------//
}

void auto_big_buff_handle(void)
{
 gim.pid.pit_angle_fdb = pitch_Angle;
  gim.pid.yaw_angle_fdb =  yaw_Angle;
if(UART4_DMA_RX_BUF[0]!=0xBE||xy_0_flag==1)
{xy_o_time++;}
else{xy_o_time=0;}

		if(xy_o_time<1)
		{
				{
          Delta_Dect_Angle_Yaw = RAD_TO_ANGLE * atan2 ( ( (double) new_location.x1 ) * TARGET_SURFACE_LENGTH,FOCAL_LENGTH);
          Delta_Dect_Angle_Pit = RAD_TO_ANGLE * atan2 ( ( (double) new_location.y1 ) * TARGET_SURFACE_WIDTH,FOCAL_LENGTH);
				
			    yaw_angle_ref_aim=Delta_Dect_Angle_Yaw + gim.pid.yaw_angle_fdb;
			    pit_angle_ref_aim=Delta_Dect_Angle_Pit + gim.pid.pit_angle_fdb+pitch_remain;
		
          gim.pid.yaw_angle_ref =  yaw_angle_ref_aim+Yaw_remain;
          gim.pid.pit_angle_ref =  raw_data_to_pitch_angle(pit_angle_ref_aim);					
				}
		}

		
  //-----------------------------云台限幅-----------------------//
  VAL_LIMIT(gim.pid.pit_angle_ref, PITCH_MIN + (GMPitchEncoder.ecd_angle)+gim.pid.pit_angle_fdb, PITCH_MAX + 5 + (GMPitchEncoder.ecd_angle)+gim.pid.pit_angle_fdb);
//		VAL_LIMIT(gim.pid.yaw_angle_ref, YAW_MIN - (-GMYawEncoder.ecd_angle)+gim.pid.yaw_angle_fdb, YAW_MAX - (-GMYawEncoder.ecd_angle)+gim.pid.yaw_angle_fdb);
//-------------------------------------------------------------//
}


void gimbal_follow_chassis_handle(void)
{
  gim.pid.pit_angle_fdb = GMPitchEncoder.ecd_angle;
  gim.pid.yaw_angle_fdb = -GMYawEncoder.ecd_angle;
  gim.pid.yaw_angle_ref = -chassis.position_ref;
  gim.pid.pit_angle_ref = 0;

}



float yaw_angle_offset = 0;
float pit_angle_offset = 0;

void auto_angle_handle(void)
{

  gim.pid.pit_angle_fdb = GMPitchEncoder.ecd_angle;
  gim.pid.yaw_angle_fdb = -GMYawEncoder.ecd_angle;
  gim.pid.yaw_angle_ref = -chassis.position_ref + yaw_angle_offset;
  gim.pid.pit_angle_ref = shoot_angle + pit_angle_offset;



}

//根据回传的距离，弹速和pitch轴角度来得出俯仰角
float angle_flag;
//float get_yaw_angle;
float ddd;
float raw_data_to_pitch_angle(float ecd_angle_pit)
{
  int shoot_angle_speed;
  float distance_s;
  float distance_x;
  float distance_y;
  float x1;
  float x2;
  float x3;
  float x4;
  float angle_tan;
  float shoot_radian;
  float shoot_angle;
  float real_angle;
	
  shoot_angle_speed=28;//judge_rece_mesg.shoot_data.bullet_speed;
  distance_s=6.65/(cos(pitch_Angle*ANGLE_TO_RAD));//*cos((get_yaw_angle-yaw_Angle)*ANGLE_TO_RAD));//(Gimbal_Auto_Shoot.Distance-7)/100;
	ddd=distance_s;
	real_angle=ecd_angle_pit+RAD_TO_ANGLE * atan2 ( HEIGHT_BETWEEN_GUN_CAMERA, distance_s );
	
  distance_x=(cos((ecd_angle_pit)*ANGLE_TO_RAD)*distance_s);
  distance_y=(sin((ecd_angle_pit)*ANGLE_TO_RAD)*distance_s);

  x1=shoot_angle_speed*shoot_angle_speed;
  x2=distance_x*distance_x;
  x3=sqrt(x2-(19.6*x2*((9.8*x2)/(2*x1)+distance_y))/x1);
  x4=9.8*x2;
  angle_tan=(x1*(distance_x-x3))/(x4);
  shoot_radian=atan(angle_tan);
  shoot_angle=shoot_radian*RAD_TO_ANGLE;
	angle_flag=shoot_angle;
  return shoot_angle;
}



void gimbal_param_init(void)
{
  memset(&gim, 0, sizeof(gimbal_t));
  gim.ctrl_mode      		 = GIMBAL_RELAX;
  gim.last_ctrl_mode 		 = GIMBAL_RELAX;
  gim.input.ac_mode        = NO_ACTION;
  gim.input.action_angle   = 5.0f;


#if STANDARD == 3
//标识3号码步兵

    PID_struct_init(&pid_pit, POSITION_PID, 800, 10,
                     16,0.01f,8); //8.0f, ,40    15,0.003f,5   18    16  5
    PID_struct_init(&pid_pit_speed, POSITION_PID, 27000, 25000,
                    200, 0.2, 0);//   170, 0.001f, 60--160 8 0    ->  180 10 50  ->190 12 70  ->120 8 0-》180.0f, 10.0f, 30 -> 235, 0.002f, 45
										//15 0.01 0...120 0 0       180 0.2 20
    //------------------------------------------------
     PID_struct_init(&pid_yaw, POSITION_PID, 1000, 0,
                    15, 0.1f, 25); //10.0f, 0, 100  13
     PID_struct_init(&pid_yaw_speed, POSITION_PID, 29000, 25000,
                  250,0.1f,10);//250.0f,6.0f, 50-->300.0f,5.0f, 70
	

#elif STANDARD ==4
////标识4号码步兵

	//	//----------------小陀螺P轴陀螺仪------------------------
 PID_struct_init(&pid_pit, POSITION_PID, 800, 10,
                   15,0.01f,8); //16 0.01 5
 PID_struct_init(&pid_pit_speed, POSITION_PID, 27000, 25000,
                   200,0.3,18);//260 0.03 60   ->  180 10 50  ->190 12 70

    //------------------------------------------------

     PID_struct_init(&pid_yaw, POSITION_PID, 1000, 0,
                     12, 0.17, 25); //7 0 10
     PID_struct_init(&pid_yaw_speed, POSITION_PID, 29000, 25000,
                    247,0.2,10);//180  2 
#elif STANDARD == 5
////标识5号码步兵
//----------------小陀螺P轴陀螺仪------------------------
    PID_struct_init(&pid_pit, POSITION_PID, 800, 10,
                     15,0.1f,20); //15,0.01f,8); //8.0f, ,40    15,0.003f,5   18    16  5
    PID_struct_init(&pid_pit_speed, POSITION_PID, 27000, 25000,
                    190,0.3f,25); //200, 0.3f, 18);//   170, 0.001f, 60--160 8 0    ->  180 10 50  ->190 12 70  ->120 8 0-》180.0f, 10.0f, 30 -> 235, 0.002f, 45
										//15 0.01 0...120 0 0       180 0.2 20
    //------------------------------------------------
     PID_struct_init(&pid_yaw, POSITION_PID, 700, 0,
		                12,0.1f,10);//15, 0.1f, 25); //10.0f, 0, 100  13
     PID_struct_init(&pid_yaw_speed, POSITION_PID, 29000, 25000,
                    210,0.1f,10);//250,0,10);//250.0f,6.0f, 50-->300.0f,5.0f, 70

#elif STANDARD == 6
////标识5号码步兵

  //	//----------------小陀螺P轴陀螺仪------------------------
  PID_struct_init(&pid_pit, POSITION_PID, 800, 10,
                  15,0.01f,8); //16 0.01 5
  PID_struct_init(&pid_pit_speed, POSITION_PID, 27000, 25000,
                  200,0.3,18);//260 0.03 60   ->  180 10 50  ->190 12 70

  //------------------------------------------------

  PID_struct_init(&pid_yaw, POSITION_PID, 1000, 0,
                  12, 0.17, 25); //7 0 10
  PID_struct_init(&pid_yaw_speed, POSITION_PID, 29000, 25000,
                  247,0.2,10);//180  2
#endif




#if STANDARD == 3
  /********************小符下的PID参数（3号）****************/

//    8.4调
  PID_struct_init(&pid_pit_small_buff, POSITION_PID, 70, 50,
                  25.0f, 0.5f, 300);  //60  60 16 0.25 150           28 0.5 160
  PID_struct_init(&pid_pit_speed_small_buff, POSITION_PID, 25000, 2000,
                  180.0f, 0.00f, 0);//   
  PID_struct_init(&pid_yaw_small_buff, POSITION_PID, 60, 30,
                  18.0f, 0.5f, 200 ); //               18 0.4 180
  PID_struct_init(&pid_yaw_speed_small_buff, POSITION_PID, 25000, 25000,
                  250.0f,0.0f,0);            




  /****************************************************/

#elif STANDARD == 4
  /********************小符下的PID参数（4号）****************/
  PID_struct_init(&pid_pit_small_buff, POSITION_PID, 70, 50,
                  25.0f, 0.5f, 200);  //60  60 16 0.25 150           28 0.5 160
  PID_struct_init(&pid_pit_speed_small_buff, POSITION_PID, 25000, 2000,
                  180.0f, 0.00f, 0);//
  PID_struct_init(&pid_yaw_small_buff, POSITION_PID, 60, 30,
                  25.0f, 0.5f, 230 ); //               18 0.4 180
  PID_struct_init(&pid_yaw_speed_small_buff, POSITION_PID, 25000, 25000,
                  250.0f,0.0f,0);
  /****************************************************/

#elif STANDARD == 5
  /********************小符下的PID参数（5号）****************/


//  PID_struct_init(&pid_pit_small_buff, POSITION_PID, 
//         	120,10,10,0.1,100);
//  PID_struct_init(&pid_pit_speed_small_buff, POSITION_PID, 
//		23000,5000,160,0.05,10);
  PID_struct_init(&pid_yaw_small_buff, POSITION_PID,
	70,0,9,0.2,70);
  PID_struct_init(&pid_yaw_speed_small_buff, POSITION_PID, 
  	23000, 2000,250.0f,2,20);



//  PID_struct_init(&pid_yaw_big_buff, POSITION_PID, 250, 0,
//                  7.4f, 0, 110 ); //
//  PID_struct_init(&pid_yaw_speed_big_buff, POSITION_PID, 25000, 5000,
//                  210.0f,4.0f,0);

//  PID_struct_init(&pid_pit_big_buff, POSITION_PID, 
//         	120,10,10,0.1,100);
//  PID_struct_init(&pid_pit_speed_big_buff, POSITION_PID, 
//		23000,5000,160,0.05,10);

		
	PID_struct_init(&pid_pit_small_buff, POSITION_PID, 80, 0,
                  7.0f, 0, 110);  //
  PID_struct_init(&pid_pit_speed_small_buff, POSITION_PID, 27000, 25000,
                  160.0f, 2.0f, 20);//160 8 0    ->  180 10 50  ->190 12 70


//  PID_struct_init(&pid_yaw_big_buff, POSITION_PID,
//	50,2,9,0.1,70);
//  PID_struct_init(&pid_yaw_speed_big_buff, POSITION_PID, 
//  	23000, 2000,250.0f,1.5,20);
		

		
  /****************************************************/

#elif STANDARD == 6
  /********************小符下的PID参数（6号）****************/

  PID_struct_init(&pid_pit_small_buff, POSITION_PID, 70, 50,
                  30.0f, 0.5f, 200);  //60  60 16 0.25 150           28 0.5 160
  PID_struct_init(&pid_pit_speed_small_buff, POSITION_PID, 25000, 2000,
                  180.0f, 0.00f, 0);//   
  PID_struct_init(&pid_yaw_small_buff, POSITION_PID, 60, 30,
                  18.0f, 0.5f, 200 ); //               18 0.4 180
  PID_struct_init(&pid_yaw_speed_small_buff, POSITION_PID, 25000, 25000,
                  250.0f,0.0f,0);            

  /****************************************************/

#endif

  /********************大符下的PID参数（3号）****************/


#if STANDARD == 3

  PID_struct_init(&pid_pit_big_buff, POSITION_PID, 200, 10,
                  9.0f, 0, 70);  //
  PID_struct_init(&pid_pit_speed_big_buff, POSITION_PID, 27000, 25000,
                  160.0f, 4.0f, 0);//160 8 0    ->  180 10 50  ->190 12 70



  PID_struct_init(&pid_yaw_big_buff, POSITION_PID, 250, 0,
                  9.0f, 0, 110 ); //
  PID_struct_init(&pid_yaw_speed_big_buff, POSITION_PID, 25000, 5000,
                  200.0f,4.0f,0);

#elif STANDARD == 4
  PID_struct_init(&pid_pit_big_buff, POSITION_PID, 200, 10,
                  11.0f, 0.00f, 0);  //
  PID_struct_init(&pid_pit_speed_big_buff, POSITION_PID, 27000, 25000,
                  170.0f, 0.0f, 0);//160 8 0    ->  180 10 50  ->190 12 70
  PID_struct_init(&pid_yaw_big_buff, POSITION_PID, 250, 0,
                  10.0f, 0, 0 ); //
  PID_struct_init(&pid_yaw_speed_big_buff, POSITION_PID, 25000, 25000,
                  100.0f,0.0f,0);


#elif STANDARD == 5
  PID_struct_init(&pid_yaw_big_buff, POSITION_PID, 250, 0,
                  7.4f, 0, 110 ); //
  PID_struct_init(&pid_yaw_speed_big_buff, POSITION_PID, 25000, 5000,
                  210.0f,4.0f,0);

//  PID_struct_init(&pid_pit_big_buff, POSITION_PID, 
//         	120,10,10,0.1,100);
//  PID_struct_init(&pid_pit_speed_big_buff, POSITION_PID, 
//		23000,5000,160,0.05,10);

		
	PID_struct_init(&pid_pit_big_buff, POSITION_PID, 70, 0,
                  9.0f, 0, 110);  //
  PID_struct_init(&pid_pit_speed_big_buff, POSITION_PID, 27000, 25000,
                  200.0f, 1.3f, 20);//160 8 0    ->  180 10 50  ->190 12 70


//  PID_struct_init(&pid_yaw_big_buff, POSITION_PID,
//	50,2,9,0.1,70);
//  PID_struct_init(&pid_yaw_speed_big_buff, POSITION_PID, 
//  	23000, 2000,250.0f,1.5,20);
		

#elif STANDARD == 6
  PID_struct_init(&pid_pit_big_buff, POSITION_PID, 200, 10,
                  10.0f, 0, 90);  //
  PID_struct_init(&pid_pit_speed_big_buff, POSITION_PID, 27000, 25000,
                  150.0f, 3.0f, 60);//160 8 0    ->  180 10 50  ->190 12 70



  PID_struct_init(&pid_yaw_big_buff, POSITION_PID, 250, 0,
                  10.0f, 0, 100 ); //
  PID_struct_init(&pid_yaw_speed_big_buff, POSITION_PID, 25000, 25000,
                  160.0f,3.0f,50);
#endif

  /****************************************************/


  /********************吊射下的PID参数****************/

  PID_struct_init(&pid_pit_auto_angle, POSITION_PID, 200, 10,
                  12.0f, 0, 90);  //
  PID_struct_init(&pid_pit_speed_auto_angle, POSITION_PID, 27000, 25000,
                  250.0f, 5.0f, 60);//160 8 0    ->  180 10 50  ->190 12 70



  PID_struct_init(&pid_yaw_auto_angle, POSITION_PID, 250, 0,
                  13.0f, 0, 100 ); //
  PID_struct_init(&pid_yaw_speed_auto_angle, POSITION_PID, 25000, 25000,
                  160.0f,5.0f,30);

  /****************************************************/


  /********************自动补弹下YAW的PID参数****************/


  PID_struct_init(&pid_yaw_follow_chassis_angle, POSITION_PID, 250, 0,
                  13.0f, 0, 100 ); //
  PID_struct_init(&pid_yaw_follow_chassis_speed, POSITION_PID, 25000, 25000,
                  160.0f,5.0f,30);

  /****************************************************/

  /********************自瞄下的PID参数****************/
#if STANDARD == 3
  PID_struct_init ( &pid_pit_follow, POSITION_PID, 200, 10, 8
                    , 0.01, 8 ); //200, 10, 10 , 0.005f, 80.0f   200, 10, 10 , 0.005, 80.0f
  PID_struct_init ( &pid_pit_speed_follow, POSITION_PID, 27000, 25000, 180.0f, 0.2f, 0 ); //160 8 0    ->  180 10 50  ->190 12 70     27000, 25000,100.0f, 0.0007, 5


//     PID_struct_init(&pid_yaw_follow, POSITION_PID, 400, 15,12, 0.001f, 0 ); //
//     PID_struct_init(&pid_yaw_speed_follow, POSITION_PID, 29000, 5000,250,5, 10);
//
  PID_struct_init ( &pid_yaw_follow, POSITION_PID,  150,200,
                    20, 0.09f, 40 );
  PID_struct_init ( &pid_yaw_speed_follow, POSITION_PID, 29800, 29800,
                    350.0f, 0, 100 ); //I太大时，陀螺开启云台抖动严重
#elif STANDARD == 4
  PID_struct_init(&pid_pit_follow, POSITION_PID, 100, 10, 10, 0.001f, 60.0f);  //
  PID_struct_init(&pid_pit_speed_follow, POSITION_PID, 27000, 25000,180.0f, 0.8, 150);//160 8 0    ->  180 10 50  ->190 12 70
  PID_struct_init(&pid_yaw_follow, POSITION_PID, 1000, 1000, 13.0f, 0.017f, 8.0);
  PID_struct_init(&pid_yaw_speed_follow, POSITION_PID, 29800, 29800,
                  80.0f,0.01f, 10);//I太大时，陀螺开启云台抖动严重


#elif STANDARD == 5
  PID_struct_init(&pid_pit_follow, POSITION_PID, 2000, 45,
                     10,0.1f,20); //8.0f, ,40    15,0.003f,5   18    16  5
  PID_struct_init(&pid_pit_speed_follow, POSITION_PID, 27000, 5000,
                    190, 0.05, 35);//   170, 0.001f, 60--160 8 0    ->  180 10 50  ->190 12 70  ->120 8 0-》180.0f, 10.0f, 30 -> 235, 0.002f, 45
										//15 0.01 0...120 0 0       180 0.2 20
  PID_struct_init(&pid_yaw_follow, POSITION_PID, 1200, 10,
                    8, 0.1f, 10); //10.0f, 0, 100  13
  PID_struct_init(&pid_yaw_speed_follow, POSITION_PID, 20000, 0,
                  180,0.05,0);//250.0f,6.0f, 50-->300.0f,5.0f, 70
									
									
										

#elif STANDARD == 6
  PID_struct_init(&pid_pit_follow, POSITION_PID, 200, 10, 10, 0.005f, 80.0f);  //
  PID_struct_init(&pid_pit_speed_follow, POSITION_PID, 27000, 25000,120.0f, 6, 150);//160 8 0    ->  180 10 50  ->190 12 70

  PID_struct_init(&pid_yaw_follow, POSITION_PID, 1000, 1000,
                  6.0f, 0.00f, 150);
  PID_struct_init(&pid_yaw_speed_follow, POSITION_PID, 29800, 29800,
                  250.0f,2.0f, 0);//I太大时，陀螺开启云台抖动严重

#endif

}

void gimbal_back_param(void)
{
  //斜坡初始化
  GMPitchRamp.SetScale(&GMPitchRamp, BACK_CENTER_TIME / GIMBAL_PERIOD);
  GMYawRamp.SetScale(&GMYawRamp, BACK_CENTER_TIME / GIMBAL_PERIOD);
  GMPitchRamp.ResetCounter(&GMPitchRamp);
  GMYawRamp.ResetCounter(&GMYawRamp);
  //云台给定角度初始化
  GimbalRef.pitch_angle_dynamic_ref = 0.0f;
  GimbalRef.yaw_angle_dynamic_ref   = 0.0f;
}



float AvgFilter(float new_value)
{
	float avg_value;
	float sum = 0;
	static float value[FILTER_NUM] = {0};
	for(int i = 0; i < FILTER_NUM - 1 ; i ++)
	{
			value[i] = value[i + 1];
			sum += value[i];
	}
	value[FILTER_NUM - 1] = new_value;
	sum += value[FILTER_NUM - 1];
	avg_value = sum / FILTER_NUM;
	return avg_value;
}
