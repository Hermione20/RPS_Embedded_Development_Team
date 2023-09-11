#include "main.h"

volatile ddtEncoder_t ddt_Encoder = {0,0,0,0,0,0,0,0,0,0,0};

uint8_t ddt_data[10]={0};
int16_t speed_ddt=0;
uint8_t crc;

void ddtEncoderProcess(u8* msg)
{
	ddt_Encoder.ID 					= msg[0];
	ddt_Encoder.Mode 				= msg[1];
	ddt_Encoder.current 		= ((msg[2]<<8) | msg[3]);
	ddt_Encoder.rate_rpm 		= ((msg[4]<<8) | msg[5]);
	ddt_Encoder.ecd_value		= ((msg[6]<<8) | msg[7]);
	ddt_Encoder.error_gate	= msg[8];
	ddt_Encoder.crc_check		= msg[9];

	if((ddt_Encoder.last_value-ddt_Encoder.ecd_value)<-16384)
	 {
	 ddt_Encoder.rount_count--;
	 }
	else if((ddt_Encoder.last_value-ddt_Encoder.ecd_value)>16384)
	 {
	 ddt_Encoder.rount_count++;
	 }
	ddt_Encoder.angle = (ddt_Encoder.ecd_value-ddt_Encoder.ecd_bias)*0.0109866634f + ddt_Encoder.rount_count * 360;

	ddt_Encoder.last_value = ddt_Encoder.ecd_value;	
}
/*****************************************************************
	驱动电机转动
******************************************************************/
void ddt_SetMotor(int16_t val)
{
	ddt_data[0]=0x01;
	ddt_data[1]=0x64;
	ddt_data[2]=(int16_t)val>>8;
	ddt_data[3]=(int16_t)val;
	ddt_data[4]=0x00;
	ddt_data[5]=0x00;
	ddt_data[6]=0x00;
	ddt_data[7]=0x00;
	ddt_data[8]=0x00;

	crc=crc8_maxim(ddt_data,9);

	ddt_data[9]=crc;
	
	Usart2SendBytesInfoProc(ddt_data,10);
}

/*****************************************************************************
	模式切换，默认速度环，模式值：0x01（电流环）|| 0x02（速度环）|| 0x03（位置环）
*******************************************************************************/
void ddt_SetMode(u16 mode)
{
	ddt_data[0]=0x01;
	ddt_data[1]=0xA0;
	ddt_data[2]=0x00;
	ddt_data[3]=0x00;
	ddt_data[4]=0x00;
	ddt_data[5]=0x00;
	ddt_data[6]=0x00;
	ddt_data[7]=0x00;
	ddt_data[8]=0x00;
	ddt_data[9]=mode;

	Usart2SendBytesInfoProc(ddt_data,10);
}

/*********************************************************************
	设置ID，电机接收到 5 次 ID设置指令后进行设置
**********************************************************************/
void ddt_SetID(u16 ID)
{
	ddt_data[0]=0xAA;
	ddt_data[1]=0x55;
	ddt_data[2]=0x53;
	ddt_data[3]=ID;
	ddt_data[4]=0x00;
	ddt_data[5]=0x00;
	ddt_data[6]=0x00;
	ddt_data[7]=0x00;
	ddt_data[8]=0x00;
	ddt_data[9]=0x00;
	for(int i=0;i<5;i++)
	{
		Usart2SendBytesInfoProc(ddt_data,10);
	}
}

/*************************************************************************
	电机刹车，1 为刹车，0为不刹车
*************************************************************************/
void ddt_Stop(u16 stop)
{
	ddt_data[0]=0x01;
	ddt_data[1]=0x64;
	ddt_data[2]=0x00;
	ddt_data[3]=0x00;
	ddt_data[4]=0x00;
	ddt_data[5]=0x00;
	ddt_data[6]=0x00;

	if (stop)
		ddt_data[7]=0xFF;
	else if (!stop)
		ddt_data[7]=0x00;

	ddt_data[8]=0x00;

	crc=crc8_maxim(ddt_data,9);

	ddt_data[9]=crc;
	
	Usart2SendBytesInfoProc(ddt_data,10);
}

