
#include "main.h"
static infantry_mode_e glb_ctrl_mode;
void mode_switch_task(void)
{
    get_chassis_mode();
    get_gimbal_mode();
    get_global_last_mode();
}
static void gimbal_mode_handle(void)
{

//        chassis.follow_gimbal = 1;
	
     if (gim.last_ctrl_mode == GIMBAL_RELAX)
      gim.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
}

u8 flag_gim_mode;
void get_gimbal_mode(void)
{

	  if (gim.ctrl_mode != GIMBAL_INIT && GetInputMode() != STOP)
	  {
		gimbal_mode_handle();//底盘是否跟随
	  } 
	//  /* gimbal back to center */
	  if (gim.last_ctrl_mode == GIMBAL_RELAX && gim.ctrl_mode != GIMBAL_RELAX)
	  {
		flag_gim_mode = gim.ctrl_mode;
		gim.ctrl_mode = GIMBAL_INIT;
			
		gimbal_back_param();
	  }
}

static void get_global_last_mode(void)
{
  gim.last_ctrl_mode = gim.ctrl_mode;
  chassis.last_ctrl_mode = chassis.ctrl_mode;
}
 

static void chassis_mode_handle(void)
{ 
	 chassis.ctrl_mode =MANUAL_FOLLOW_GIMBAL; 
}


void get_chassis_mode(void)//多行
{
	  if (gim.ctrl_mode == GIMBAL_INIT || gim.ctrl_mode == GIMBAL_POSITION_MODE  ||gim.ctrl_mode == GIMBAL_AUTO_ANGLE || GetInputMode() == STOP)
	  {
		chassis.ctrl_mode = CHASSIS_STOP;
	  }
	  else if((chassis.ctrl_mode != CHASSIS_ROTATE)&&(chassis.ctrl_mode != CHASSIS_REVERSE)&&(chassis.ctrl_mode != CHASSIS_CHANGE_REVERSE)&&(chassis.ctrl_mode != CHASSIS_SEPARATE)&&(chassis.ctrl_mode != CHASSIS_AUTO_SUP))
	  {
		chassis_mode_handle();
	  } 
}

uint8_t gimbal_is_controllable(void)
{
	 if (gim.ctrl_mode == GIMBAL_RELAX
		 ||GetInputMode() == STOP
		 ||Is_Lost_Error_Set(LOST_ERROR_RC)
		 ||Is_Lost_Error_Set(LOST_ERROR_IMU) )
	     return 0;
		 else
		   return 1;
}

uint8_t chassis_is_controllable(void)
{
	  if (chassis.ctrl_mode == CHASSIS_RELAX 
	   ||Is_Lost_Error_Set(LOST_ERROR_RC)
		 ||GetInputMode() == STOP
		 ||gim.ctrl_mode==GIMBAL_INIT
//		 ||Is_Lost_Error_Set( LOST_ERROR_MOTOR1)
//		 ||Is_Lost_Error_Set( LOST_ERROR_MOTOR2)
//		 ||Is_Lost_Error_Set( LOST_ERROR_MOTOR3)
//		 ||Is_Lost_Error_Set( LOST_ERROR_MOTOR4)
		 ||Is_Lost_Error_Set(LOST_ERROR_IMU) 
		
		  
		 )
		return 0;
	  else
		return 1;
}

void get_shoot_mode(void)
{

}

