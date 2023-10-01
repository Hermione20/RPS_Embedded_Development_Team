#include "shoot_task.h"

uint16_t frictionSpeed;		//42mm弹速

//PID初始化
void shot_param_init(void)
{
		Set_GM_CM_Current(CAN1,0,0,0,aim_scope_speed.out);

  PID_struct_init(&pid_rotate[1], POSITION_PID,15000,1000, 0, 70 , 0.01f ,100);//两个摩擦轮/7,0.33f,100//75，0，70
  PID_struct_init(&pid_rotate[2], POSITION_PID,15000,1000, 0, 72 , 0.01f ,10);

  PID_struct_init(&pid_shoot_bullet_position_speed_loop, POSITION_PID, poke_max_out, 13000, 0, 5, 0.5, 0);//下拨盘速度环
	PID_struct_init(&pid_shoot_bullet_position_angle_loop_2, POSITION_PID, 2000, 0,  0, 120, 5, 10); //二级拨盘//120 5 10//1200
  PID_struct_init(&pid_shoot_bullet_position_speed_loop_2, POSITION_PID, 9900, 5500, 0, 20, 0, 0 );
}




static void SwitchModeShoot(void)
{
  if(judge_rece_mesg.game_robot_state.shooter_id1_42mm_speed_limit==10)
    frictionSpeed=FRICTION_SPEED_10;
  else if((aim_scope_flag==0) && (judge_rece_mesg.game_robot_state.shooter_id1_42mm_speed_limit==16))
    frictionSpeed=FRICTION_SPEED_16;
	else if (aim_scope_flag==1)		//开镜后吊射前哨站
		frictionSpeed = 3500;
	else
		frictionSpeed=FRICTION_SPEED_10;
}
