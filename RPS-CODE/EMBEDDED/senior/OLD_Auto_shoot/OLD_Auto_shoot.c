#include "OLD_Auto_shoot.h"


/**
  ******************************************************************************
  * @file    Auto_shoot.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   此文件编写了与视觉通信的内容，包含自瞄数据的接收，大小福
						 数据的接收与和串口发送函数
						 
@verbatim
 ===============================================================================
 **/
 
/******************************************auto_shoot_define***************************************/
location new_location;

HostToDevice__Frame *Uart4_Protobuf_Receive_Gimbal_Angle;


/**********************************************auto_shoot_handle*****************************************/
void vision_process_general_message(unsigned char* address, unsigned int length)
{
	int i = 0;
	static u8 first_len[4];
	static unsigned short content_size;
	static unsigned char getaddress[100];
	if(address[0] != 0xBE)
	return;
	for(u8 k = 0;k < length;k++)
	{
		if(address[k]==0xED)
		{
			first_len[i] = k;
			i++;
		}
	}
	content_size = first_len[0]-3;

	for(int k = 0;k<content_size;k++)
	{
		getaddress[k] = address[k+2];
	}
	if(address[content_size + 3] != 0xED)
	return;

	unsigned char* content_address;
		content_address = getaddress;

	unsigned char crc8 = address[2 + content_size];

	if(crc8 != get_crc8(content_address,content_size))
	return;

	Uart4_Protobuf_Receive_Gimbal_Angle=host_to_device__frame__unpack(NULL,content_size,content_address);

	float flag_x = Uart4_Protobuf_Receive_Gimbal_Angle->target_yaw_;
	float flag_y = Uart4_Protobuf_Receive_Gimbal_Angle->target_pitch_;
	if(flag_y==Uart4_Protobuf_Receive_Gimbal_Angle->target_pitch_&&flag_x==Uart4_Protobuf_Receive_Gimbal_Angle->target_yaw_)
	{
		if(flag_x!=0&&flag_y!=0)
		{
			new_location.last_x = new_location.x;
			new_location.last_y = new_location.y;
			new_location.x = Uart4_Protobuf_Receive_Gimbal_Angle->target_yaw_;
			new_location.y = Uart4_Protobuf_Receive_Gimbal_Angle->target_pitch_;
			new_location.pitch_speed = Uart4_Protobuf_Receive_Gimbal_Angle->pitch_speed;
			new_location.yaw_speed = Uart4_Protobuf_Receive_Gimbal_Angle->yaw_speed;
			new_location.flag = 1;
			if (fabs(new_location.x - gimbal_gyro.yaw_Angle) > 45 || fabs(new_location.y - gimbal_gyro.pitch_Angle) > 70)
			{

					new_location.flag = 0;
			}
		}else
		{
			new_location.flag = 0;
			new_location.yaw_speed = 0;
			new_location.pitch_speed = 0;
		}
	}
	//为防止神经网络误识别导致哨兵不跟随直接进入巡逻，需要做此处理
	if(new_location.flag)
	{
		new_location.lost_cnt++;
	}else
	{
		new_location.lost_cnt--;
	}
  if(new_location.lost_cnt<=0)
	{
		new_location.lost_cnt = 0;
	}
	if(new_location.lost_cnt>=3)
	{
		new_location.lost_cnt = 3;
	}
		
		
	if(new_location.lost_cnt == 3)
	{
		new_location.control_flag = 1;
	}else if(new_location.lost_cnt == 0)
	{
		new_location.control_flag = 0;
	}
    /*****************************************************/
	float flagg_x = Uart4_Protobuf_Receive_Gimbal_Angle->x_;
	float flagg_y = Uart4_Protobuf_Receive_Gimbal_Angle->y_;
	if (flagg_x == Uart4_Protobuf_Receive_Gimbal_Angle->x_ && flagg_y == Uart4_Protobuf_Receive_Gimbal_Angle->y_)
	{
		if (flagg_x != 0 && flagg_y != 0)
		{
			new_location.xy_0_flag = 0;
			new_location.buff_kf_flag = 1;
			new_location.x1 = Uart4_Protobuf_Receive_Gimbal_Angle->x_;
			new_location.y1 = Uart4_Protobuf_Receive_Gimbal_Angle->y_;
		}
		else
		{
			new_location.xy_0_flag = 1;
		}
	}
	host_to_device__frame__free_unpacked(Uart4_Protobuf_Receive_Gimbal_Angle, NULL);
}

DeviceToHost__Frame msg;
u8 DateLength;
#define GIMBAL_AUTO_SMALL_BUFF 11
#define GIMBAL_AUTO_BIG_BUFF 12
void send_protocol(float x, float y, float r, int id, float ammo_speed, int gimbal_mode, u8 *data)
{
	device_to_host__frame__init(&msg);

	if (gimbal_mode == GIMBAL_AUTO_SMALL_BUFF)
	{
		msg.mode_ = 2;
	}
	else if (gimbal_mode == GIMBAL_AUTO_BIG_BUFF)
	{
		msg.mode_ = 1;
	}
	else
	{
		msg.mode_ = 0;
	}
	//	msg.mode_= 1;

	msg.current_pitch_ = y;
	msg.current_yaw_ = x;
	msg.current_color_ = id;
	msg.bullet_speed_ = ammo_speed;
	msg.current_roll_ = r;

	device_to_host__frame__pack(&msg, data + 2);
	DateLength = device_to_host__frame__get_packed_size(&msg);
	data[0] = 0xBE;
	data[1] = DateLength;
	Append_CRC8_Check_Sum(&data[2], DateLength + 1);
	data[DateLength + 3] = 0xED;
	Uart4SendBytesInfoProc(data, DateLength + 4);
}
