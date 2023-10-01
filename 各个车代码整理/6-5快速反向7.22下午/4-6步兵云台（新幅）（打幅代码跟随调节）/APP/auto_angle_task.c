#include "main.h"

float infantry_x=0;
float infantry_y=0;
float infantry_z=0;

float BASE_POSITION_X;
float BASE_POSITION_Y;
float BASE_POSITION_Z;

float shoot_distance=0;
float shoot_height=0;
float shoot_angle_speed=0;
float shoot_angle=0;
float shoot_radian=0;
float x1,x2,x3,x4;
float pro_height=0;
void far_shoot_task(void)//吊射
{  
	if	(judge_rece_mesg.game_robot_state.robot_id==0X03)//红色3
	{
		float BASE_POSITION_X=28-2.16107;
        float BASE_POSITION_Y=15-2.16107;
        float BASE_POSITION_Z=1.515;
	}
	else if	(judge_rece_mesg.game_robot_state.robot_id==0X0D)//蓝色3
	{
		float BASE_POSITION_X=2.16107;
        float BASE_POSITION_Y=2.16107;
        float BASE_POSITION_Z=1.515;
	}
	else if	(judge_rece_mesg.game_robot_state.robot_id==0X04)//红4
	{
		float BASE_POSITION_X=28-2.16107;
        float BASE_POSITION_Y=15-2.16107;
        float BASE_POSITION_Z=1.515;
	}
	else if	(judge_rece_mesg.game_robot_state.robot_id==0X0E)//蓝4
	{
		float BASE_POSITION_X=2.16107;
        float BASE_POSITION_Y=2.16107;
        float BASE_POSITION_Z=1.515;
	}
	else if	(judge_rece_mesg.game_robot_state.robot_id==0X04)//红5
	{
		float BASE_POSITION_X=28-2.16107;
        float BASE_POSITION_Y=15-2.16107;
        float BASE_POSITION_Z=1.515;
	}
	else if	(judge_rece_mesg.game_robot_state.robot_id==0X0E)//蓝5
	{
		float BASE_POSITION_X=2.16107;
        float BASE_POSITION_Y=2.16107;
        float BASE_POSITION_Z=1.515;
		
	}
	
	//自动吊射
	//已知量
	shoot_angle_speed=judge_rece_mesg.shoot_data.bullet_speed;
	infantry_x=judge_rece_mesg.game_robot_pos.x;
	infantry_y=judge_rece_mesg.game_robot_pos.y;
	infantry_z=judge_rece_mesg.game_robot_pos.z;
	shoot_distance=sqrt((infantry_x-BASE_POSITION_X )*(infantry_x-BASE_POSITION_X )+(infantry_y-BASE_POSITION_Y )*(infantry_y-BASE_POSITION_Y ));
	shoot_height=BASE_POSITION_Z-infantry_z;
	//弹道补偿
	if(shoot_distance<=10&&shoot_distance>0)
		pro_height=shoot_height;
	if(shoot_distance<=12&&shoot_distance>10)
		pro_height=shoot_height+(shoot_distance-10)/(2/0.06);
	if(shoot_distance<=13&&shoot_distance>12)
		pro_height=shoot_height+(shoot_distance-12)/(1/0.11)+0.06;
		if(shoot_distance<=14&&shoot_distance>13)
		pro_height=shoot_height+(shoot_distance-13)/(1/0.02)+0.17;
	else if(shoot_distance<=15&&shoot_distance>14)
		pro_height=shoot_height+(shoot_distance-14)/(1/0.08)+0.19;
	else if(shoot_distance<=16&&shoot_distance>15)
		pro_height=shoot_height+(shoot_distance-15)/(1/0.05)+0.27;
	else if(shoot_distance<=17&&shoot_distance>16)
		pro_height=shoot_height+(shoot_distance-16)/(1/0.06)+0.32;
	else if(shoot_distance<=18&&shoot_distance>17)
		pro_height=shoot_height+(shoot_distance-17)/(1/0.07)+0.38;
	else if(shoot_distance<=19&&shoot_distance>18)
		pro_height=shoot_height+(shoot_distance-18)/(1/0.05)+0.45;
	else if(shoot_distance<=20&&shoot_distance>19)
		pro_height=shoot_height+(shoot_distance-19)/(1/0.14)+0.50;
	else if(shoot_distance<=21&&shoot_distance>20)
		pro_height=shoot_height+(shoot_distance-19)/(1/0.07)+0.64;
	//角度公式
	x1=shoot_angle_speed*shoot_angle_speed;
	x2=shoot_distance*shoot_distance;
	x3=sqrt(x2-(19.6*x2*((9.8*x2)/(2*x1)+pro_height))/x1);
	x4=9.8*x2;
	shoot_radian=atan2((float)(x1*(shoot_distance-x3))/(x4),1);	
	shoot_angle=(shoot_radian*180.0f)/3.141593;	
	

	//固定点吊射
	//已知量
	shoot_angle_speed=judge_rece_mesg.shoot_data.bullet_speed;
	infantry_x=judge_rece_mesg.game_robot_pos.x;
	infantry_y=judge_rece_mesg.game_robot_pos.y;
	infantry_z=judge_rece_mesg.game_robot_pos.z;
	shoot_distance=7;
	shoot_height=0.084;
	//弹道补偿
	if(shoot_distance<=10&&shoot_distance>0)
		pro_height=shoot_height;
	if(shoot_distance<=12&&shoot_distance>10)
		pro_height=shoot_height+(shoot_distance-10)/(2/0.06);
	if(shoot_distance<=13&&shoot_distance>12)
		pro_height=shoot_height+(shoot_distance-12)/(1/0.11)+0.06;
		if(shoot_distance<=14&&shoot_distance>13)
		pro_height=shoot_height+(shoot_distance-13)/(1/0.02)+0.17;
	else if(shoot_distance<=15&&shoot_distance>14)
		pro_height=shoot_height+(shoot_distance-14)/(1/0.08)+0.19;
	else if(shoot_distance<=16&&shoot_distance>15)
		pro_height=shoot_height+(shoot_distance-15)/(1/0.05)+0.27;
	else if(shoot_distance<=17&&shoot_distance>16)
		pro_height=shoot_height+(shoot_distance-16)/(1/0.06)+0.32;
	else if(shoot_distance<=18&&shoot_distance>17)
		pro_height=shoot_height+(shoot_distance-17)/(1/0.07)+0.38;
	else if(shoot_distance<=19&&shoot_distance>18)
		pro_height=shoot_height+(shoot_distance-18)/(1/0.05)+0.45;
	else if(shoot_distance<=20&&shoot_distance>19)
		pro_height=shoot_height+(shoot_distance-19)/(1/0.14)+0.50;
	else if(shoot_distance<=21&&shoot_distance>20)
		pro_height=shoot_height+(shoot_distance-19)/(1/0.07)+0.64;
	//角度公式
	x1=shoot_angle_speed*shoot_angle_speed;
	x2=shoot_distance*shoot_distance;
	x3=sqrt(x2-(19.6*x2*((9.8*x2)/(2*x1)+pro_height))/x1);
	x4=9.8*x2;
	shoot_radian=atan2((float)(x1*(shoot_distance-x3))/(x4),1);	
	shoot_angle=(shoot_radian*180.0f)/3.141593;
}

