#include "Auto_shoot.h"


/**
  ******************************************************************************
  * @file    Auto_shoot.c
  * @author  HDW
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   此文件编写了与视觉通信的内容，包含自瞄数据的接收，大小福
						 数据的接收与和串口发送函数
						 
@verbatim
 ===============================================================================
 **/
 
/******************************************auto_shoot_define***************************************/


Auto_Shoot_t My_Auto_Shoot;
New_Auto_Aim_t New_Auto_Aim;
New_Auto_Aim_Send_t New_Auto_Aim_Send;

u16 AUTO_CRC;
void Vision_Process_General_Message_New(unsigned char* address, unsigned int length, Auto_Shoot_t *Auto_Shoot)
{
	
	New_Auto_Aim_t New_Auto_Aim_Medium;
	memcpy(&New_Auto_Aim_Medium.Header,&address[0],1);
	memcpy(&New_Auto_Aim_Medium.Pitch_Angle,&address[0]+1,8);
	memcpy(&New_Auto_Aim_Medium.Priority,&address[0]+9,3);
	memcpy(&New_Auto_Aim_Medium.Check_Sum,&address[0]+12,2);
	
	if(New_Auto_Aim_Medium.Header!=0xbe)
	 return;
	AUTO_CRC=Verify_CRC16_Check_Sum(address,length-1);
	if(!Verify_CRC16_Check_Sum(address,length-1))
		return;
	
	memcpy(&New_Auto_Aim.Header,&address[0],1);
	memcpy(&New_Auto_Aim.Pitch_Angle,&address[0]+1,8);
	memcpy(&New_Auto_Aim.Priority,&address[0]+9,3);
	memcpy(&New_Auto_Aim.Check_Sum,&address[0]+12,2);
	
	/**************************↓自瞄模式下的位置识别↓***************************/
	float Auto_Aim_Yaw_Angle_Medium=New_Auto_Aim.Yaw_Angle;
	float Auto_Aim_Pitch_Angle_Medium=New_Auto_Aim.Pitch_Angle;
	//单片机没法存储bull型变量，当视觉数据为bull型变量时，不进行数据处理
	if(Auto_Aim_Pitch_Angle_Medium==New_Auto_Aim.Pitch_Angle&&Auto_Aim_Yaw_Angle_Medium==New_Auto_Aim.Yaw_Angle)
	{
		if(Auto_Aim_Pitch_Angle_Medium!=0&&Auto_Aim_Pitch_Angle_Medium!=0)
		{
			Auto_Shoot->Auto_Aim.Yaw_Angle_Last = Auto_Shoot->Auto_Aim.Yaw_Angle;
			Auto_Shoot->Auto_Aim.Pitch_Angle_Last = Auto_Shoot->Auto_Aim.Pitch_Angle;
			Auto_Shoot->Auto_Aim.Yaw_Angle = New_Auto_Aim.Yaw_Angle;
			Auto_Shoot->Auto_Aim.Pitch_Angle = New_Auto_Aim.Pitch_Angle;
			Auto_Shoot->Auto_Aim.Priority=New_Auto_Aim.Priority;
			Auto_Shoot->Auto_Aim.Attack_Choice=New_Auto_Aim.Attack_Choice;
			Auto_Shoot->Auto_Aim.Flag_Get_Target = 1;
			Auto_Shoot->Auto_Aim.enable_shoot = New_Auto_Aim.enable_shoot;
			
			Auto_Shoot->Auto_Aim.Lost_Cnt=0;
			//gimbal_gyro.yaw_Angle还没定义先注释
//			if (fabs(new_location.x - gimbal_gyro.yaw_Angle) > 45 || fabs(new_location.y - gimbal_gyro.pitch_Angle) > 70)
//			{

//					new_location.flag = 0;
//			}
		}
		else
		{
			if(Auto_Shoot->Auto_Aim.Lost_Cnt<100)
				Auto_Shoot->Auto_Aim.Lost_Cnt++;
			else
			{
				Auto_Shoot->Auto_Aim.Flag_Get_Target = 0;
				Auto_Shoot->Auto_Aim.Yaw_Angle = 0;
				Auto_Shoot->Auto_Aim.Pitch_Angle = 0;
			}
			Auto_Shoot->Auto_Aim.enable_shoot = 0;
		}
	}
	
	
}


float yaw_angle__pi_pi;
void send_protocol_New(float Yaw, float Pitch, float Roll, int id, float ammo_speed, int Attack_Engineer_Flag, u8* data)
{
	
	yaw_angle__pi_pi = convert_ecd_angle_to__pi_pi(Yaw,yaw_angle__pi_pi);
	New_Auto_Aim_Send.Pitch=Pitch;
	New_Auto_Aim_Send.Roll=Roll;
	New_Auto_Aim_Send.Yaw=yaw_angle__pi_pi;
	if(id>100)
		New_Auto_Aim_Send.Current_Color=1;
	else
		New_Auto_Aim_Send.Current_Color=0;
	New_Auto_Aim_Send.Another_Priority=1;
	New_Auto_Aim_Send.Shoot_Speed=ammo_speed;
	
//	New_Auto_Aim_Send.Current_Color=Sentry_Decision_Data.Current_Color;
//	New_Auto_Aim_Send.Enemy_Survical_State_1=Sentry_Decision_Data.Enemy_Survical_State_1;
//	New_Auto_Aim_Send.Enemy_Survical_State_2=Sentry_Decision_Data.Enemy_Survical_State_2;
//	New_Auto_Aim_Send.Enemy_Survical_State_3=Sentry_Decision_Data.Enemy_Survical_State_3;
//	New_Auto_Aim_Send.Enemy_Survical_State_4=Sentry_Decision_Data.Enemy_Survical_State_4;
//	New_Auto_Aim_Send.Enemy_Survical_State_5=Sentry_Decision_Data.Enemy_Survical_State_5;
//	New_Auto_Aim_Send.Enemy_Survical_State_7=Sentry_Decision_Data.Enemy_Survical_State_7;
	
	
	data[0]=0xbe;
	memcpy(&data[1],&New_Auto_Aim_Send,25);
	Append_CRC16_Check_Sum(&data[0],25+3);
//	data[52]=0xed;
	Uart4SendBytesInfoProc(data, 25+3);
}


