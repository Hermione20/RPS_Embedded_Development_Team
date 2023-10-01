#include "main.h"

/* *** *** 自动打击相关定义 开始 *** *** */
uint8_t dataFromMF[7];		        //数据缓存
uint8_t dataFromMFReadyFlag = 0; 		//妙算数据接收完成标志位

location new_location;            //传回来的坐标值

int sucflag = 0;    //成功接收到一组数据
int recflag = 0;    //接受到的数据识别到装甲
int receive = 0;    //接收到数据个数
/* *** *** ********************** *** *** */
int num_command=0;
uint8_t auto_shoot_mode_set;
robot_color_e robot_color = 1 ;   //0-9以下标识自己都是红方，其它都是蓝方
/********* 自动打击相关定义 结束 *********/
int wExpected = 0;

Signal *Uart4_Protobuf_Receive_Message;
int16_t Uart4_Content_Size;
u8 *CRC_Content_buf;
TurretCommand *Uart4_Protobuf_Receive_Gimbal_Angle;
//extern unsigned char get_crc8(unsigned char* data, unsigned int length);
float flagg_pitch;
float flagg_yaw;
void targetOffsetDataDeal ( uint8_t len, u8 *buf )
{
  receive++;
//
//	CRC_Content_buf=(u8*)(buf+5);
//
//	Uart4_Content_Size=(int16_t)buf[4]<<8|(int16_t)buf[3];
//
//	Uart4_Protobuf_Receive_Message=signal__unpack(NULL,Uart4_Content_Size,CRC_Content_buf);
//
//  if (buf[0]==0xBE &&
//			get_crc8( CRC_Content_buf, Uart4_Content_Size) == buf[5+Uart4_Content_Size] &&
//			buf[5+Uart4_Content_Size+2 -1 ]==0xED )
//    {
//      new_location.receNewDataFlag = 1;    //检测丢帧标志位,发过来检测装甲板的数据，可能没检测到，则为丢帧
////      if ( buf[1] != 0 )
////        {
////          new_location.x =  -( ( ( int16_t ) buf[3] << 8 | ( int16_t ) buf[2]  ) - GIMBAL_YAW_MID );  //传回来的图像坐标相对于左上角原点
////          new_location.y =  ( ( ( int16_t ) buf[5] << 8 | ( int16_t ) buf[4] ) - GIMBAL_PIT_MID );
////          new_location.flag =  buf[1];
////          new_location.dis =  ( ( int16_t ) buf[7] << 8 | ( int16_t ) buf[6] );
////        }
////      else
////        {

////          new_location.x = 0;
////          new_location.y = 0;
////          new_location.flag =  0 ;      //0找到装甲板，1未找到装甲板
////          new_location.crc = buf[9] ;
////        }
////			new_location.x=Uart4_Protobuf_Receive_Message->yaw;
////			new_location.x=Uart4_Protobuf_Receive_Message->pitch;
//    }
//		signal__free_unpacked(Uart4_Protobuf_Receive_Message,NULL);
//			parse_signal(buf,len);
  process_general_message(buf,len);
}


void parse_signal(unsigned char* content_address, unsigned int content_length)
{
  //Todo: parse and process signal.
  Uart4_Protobuf_Receive_Message=signal__unpack(NULL,content_length,content_address);
  if(strcmp(Uart4_Protobuf_Receive_Message->name,"t_c_d"))
    {
      chassis.ctrl_mode=MANUAL_SEPARATE_GIMBAL;
    }
  else if(strcmp(Uart4_Protobuf_Receive_Message->name,"t_c_a"))
    {
      gim.ctrl_mode=GIMBAL_INIT;
    }
  signal__free_unpacked(Uart4_Protobuf_Receive_Message,NULL);
}

float yaw_buff=0;

void parse_turret_command(unsigned char* content_address, unsigned int content_length)
{
	// Protobuf 正确用法是新建一个对象去存数据，就算不新建对象好歹初始化一下啊傻X刘恒
	Uart4_Protobuf_Receive_Gimbal_Angle->pitch = 0.0f;
	Uart4_Protobuf_Receive_Gimbal_Angle->yaw = 0.0f;
	Uart4_Protobuf_Receive_Gimbal_Angle->command = 0;
  Uart4_Protobuf_Receive_Gimbal_Angle=turret_command__unpack(NULL,content_length,content_address);
  flagg_pitch=Uart4_Protobuf_Receive_Gimbal_Angle->pitch;
  flagg_yaw=Uart4_Protobuf_Receive_Gimbal_Angle->yaw;
      /*这里有问题，数据是反的*/
	new_location.receNewDataFlag  =  1;
	if(Uart4_Protobuf_Receive_Gimbal_Angle->command == 1)
	{	
			new_location.x=  Uart4_Protobuf_Receive_Gimbal_Angle->yaw-yaw_buff;
			new_location.y=  Uart4_Protobuf_Receive_Gimbal_Angle->pitch;
		  new_location.dis = Uart4_Protobuf_Receive_Gimbal_Angle->distance;
			new_location.flag = 1;	
//		  LASER_ON();
	}
	else
	{
			new_location.x=  0;
			new_location.y=  0;
			new_location.flag = 0;	
			num_command++;		
//		  LASER_OFF();
	}

  turret_command__free_unpacked(Uart4_Protobuf_Receive_Gimbal_Angle,NULL);
}

typedef void(*Parser)(unsigned char* content_address, unsigned int content_length);

Parser parsers[] =
{
  NULL,
  &parse_signal,
  NULL,
  &parse_turret_command
};
unsigned int parsers_count = sizeof(parsers) / sizeof(Parser);

#ifndef DEBUG_MODE
#define DEBUG_MODE
#endif

#ifdef DEBUG_MODE
  // Counter for received packages.
  int counter_receive = 0;
  // Counter for packages that passed the head and tail check.
  int counter_complete = 0;
  // Contuner for pacakages that passed the CRC8 check.
  int counter_crc_passed = 0;
#endif


void process_general_message(unsigned char* address, unsigned int length)
{

#ifdef DEBUG_MODE
  if (length > 0) ++counter_receive;
#endif

  // Package head check.
  if (address[0] != 0xBE) return;

  unsigned short package_id = *(unsigned short*)(&address[1]);
  unsigned short content_size = *(unsigned short*)(&address[3]);

  // Package tail check. 5 is the size of the head part.
  if (address[5 + content_size + 1] != 0xED) return;

#ifdef DEBUG_MODE
  ++counter_complete;
#endif




#ifdef DEBUG_MODE
  ++counter_crc_passed;
#endif

  // Check id in order to prevent array out of range access.
  if (package_id < parsers_count)
    {

    }
}



//void send_signal(char* name)
//{
//	/* example of using protobuf message */

//	Signal msg;
//	signal__init(&msg);
//	msg.name="612";
//	signal__pack(&msg,UART4_DMA_TX_BUF+5);
//
//	UART4_DMA_TX_BUF[0]=0xBE;
//	UART4_DMA_TX_BUF[1]=1;
//	UART4_DMA_TX_BUF[2]=0;
//	UART4_DMA_TX_BUF[3]=2;
//	UART4_DMA_TX_BUF[4]=0;
//
//	Append_CRC8_Check_Sum(&UART4_DMA_TX_BUF[5],4);
//	UART4_DMA_TX_BUF[9]=0xFE;
//
//}
// send_signal("atk_e");