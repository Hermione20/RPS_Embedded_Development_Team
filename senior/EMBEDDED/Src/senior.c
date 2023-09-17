#include "senior.h"

/**************general_gyro define**********************/
general_gyro_t gimbal_gyro;
general_gyro_t chassis_gyro;
/*********************general_chassis define***************************/
steering_wheel_t steering_wheel_chassis = {0};
Mecanum_wheel_t Mecanum_chassis  = {0};
/*************************general_gimbal_define****************************************/
volatile Encoder Pitch_Encoder = {0};
volatile Encoder yaw_Encoder = {0};
hero_small_gimbal_t hero_small_gimbal = {0};
/***********************************friction_encoder*****************************************************/
friction_t general_friction = {0};
/************************************poke_encoder*********************************************/
poke_t general_poke = {0};
/********************************HT430_define*****************************************/
HT430_J10_t HT430_J10;
/******************************capacitance_define*************************************/
volatile capacitance_message_t capacitance_message;
/*************************************judge define********************************************/
receive_judge_t judge_rece_mesg;
/******************************************auto_shoot_define***************************************/
location new_location;
HostToDevice__Frame *Uart4_Protobuf_Receive_Gimbal_Angle;
/**********************************************remote_define***************************************/
RC_Ctl_t RC_CtrlData;
/************************ch100******************************/

void CH100_getDATA(uint8_t *DataAddress,general_gyro_t *GYRO)
{
    static __align(4) id0x91_t dat; /* struct must be 4 byte aligned */
    memcpy(&dat, &DataAddress[6], sizeof(id0x91_t));

    volatile static float Last_yaw_temp1, Yaw_temp1;
	volatile static int Yaw_count1;

    GYRO->pitch_Angle = -dat.eul[0];

    Last_yaw_temp1 = Yaw_temp1;
    Yaw_temp1 = dat.eul[2];
    if(Yaw_temp1 - Last_yaw_temp1 >= 324)
    {
        Yaw_count1--;
    }else if (Yaw_temp1 - Last_yaw_temp1 <= -324)
    {
        Yaw_count1++;
    }
    GYRO->yaw_Angle = -(Yaw_temp1 + Yaw_count1*360);

    GYRO->roll_Angle = dat.eul[1];

    GYRO->pitch_Gyro = dat.gyr[1];
    GYRO->yaw_Gyro = -dat.gyr[2];
    GYRO->roll_Gyro = dat.gyr[0];

    GYRO->x_Acc = dat.acc[0];
    GYRO->y_Acc = dat.acc[2];
    GYRO->z_Acc = dat.acc[1];   

}

/***************************hi220**********************************/

static void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
    uint32_t crc = *currectCrc;
    uint32_t j;
    for (j=0; j < lengthInBytes; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    } 
    *currectCrc = crc;
}

void HI220_getDATA(uint8_t *DataAddress,general_gyro_t *GYRO,uint8_t length)
{

    static HI220_Stucture HI220_Data_From_Usart;
    static Hi220_Flags_t Hi220_Flags = {0};
    static char *p2 = 0;

    uint16_t CRCReceived = 0;            /* CRC value received from a frame */
	uint16_t CRCCalculated = 0;          /* CRC value caluated from a frame */
	u8 Length_Deal = 6;
	u8 *p;

    if(Hi220_Flags.Hi220_Flag_Reconfig.Flag_Reconfig != 0)
		{
			DataAddress[100-1] = '\0';
			if(Hi220_Flags.Hi220_Flag_Reconfig.Bits.BAUD_Reconfig)
			{
				p2 = strstr((char*)DataAddress,"OK");
				if(p2 != 0)
					Hi220_Flags.Hi220_Flag_Configured.Bits.BAUD_Configured = 1;
			}
			if(Hi220_Flags.Hi220_Flag_Reconfig.Bits.Eout_Reconfig)
			{
				p2 = strstr((char*)DataAddress,"OK");
				if(p2 != 0)
					Hi220_Flags.Hi220_Flag_Configured.Bits.Eout_Configured = 1;
			}
			if(Hi220_Flags.Hi220_Flag_Reconfig.Bits.ODR_Reconfig)
			{
				p2 = strstr((char*)DataAddress,"OK");
				if(p2 != 0)
					Hi220_Flags.Hi220_Flag_Configured.Bits.ODR_Configured = 1;
			}
			if(Hi220_Flags.Hi220_Flag_Reconfig.Bits.SETPTL_Reconfig)
			{
				p2 = strstr((char*)DataAddress,"new packet items");
				if(p2 != 0)
					Hi220_Flags.Hi220_Flag_Configured.Bits.SETPTL_Configured = 1;
			}
			memset(DataAddress,0,sizeof(&DataAddress));
		}
		else
		{
			if(DataAddress[0] == 0x5a && DataAddress[1] == 0xa5 && (DataAddress[3]<<8) + DataAddress[2] + 6 == length && length >= 8)
			{					
				crc16_update(&CRCCalculated, &DataAddress[0], 4);
				crc16_update(&CRCCalculated, &DataAddress[6], (length-6));				
				CRCReceived = (DataAddress[5]<<8) + DataAddress[4];
				if(CRCCalculated == CRCReceived)
				{
					Length_Deal = 6;
					
					while(Length_Deal < length)
					{
						p = &DataAddress[Length_Deal];
						switch(*p)
						{
							case 0x90:
								HI220_Data_From_Usart.User_ID = p[1];
								Length_Deal += (LENGTH_USER_ID_0x90 + 1);
								break;
							case 0xa0:
								HI220_Data_From_Usart.Acc_X = (p[2]<<8) +p[1];
								HI220_Data_From_Usart.Acc_Y = (p[4]<<8) +p[3];
								HI220_Data_From_Usart.Acc_Z = (p[6]<<8) +p[5];
								Length_Deal += (LENGTH_ACC_0xa0+1);
								break;
							case 0xa5:
								HI220_Data_From_Usart.Linear_Acc_X = (p[2]<<8) +p[1];
								HI220_Data_From_Usart.Linear_Acc_Y = (p[4]<<8) +p[3];
								HI220_Data_From_Usart.Linear_Acc_Z = (p[6]<<8) +p[5];
								Length_Deal += (LENGTH_LINEAR_ACC_0xa5 + 1);
								break;
							case 0xb0:
								HI220_Data_From_Usart.Ang_Velocity_X = (p[2]<<8) +p[1];
								HI220_Data_From_Usart.Ang_Velocity_Y = (p[4]<<8) +p[3];
								HI220_Data_From_Usart.Ang_Velocity_Z = (p[6]<<8) +p[5];
								Length_Deal += (LENGTH_ANG_VEL_0xb0+1); 
								break;
							case 0xc0:
								HI220_Data_From_Usart.Mag_X = (p[2]<<8) +p[1];
								HI220_Data_From_Usart.Mag_Y = (p[4]<<8) +p[3];
								HI220_Data_From_Usart.Mag_Z = (p[6]<<8) +p[5];
								Length_Deal += (LENGTH_MAG_0xc0+1); 
								break;
							case 0xd0:
								HI220_Data_From_Usart.Euler_Angle_Pitch_s16 = (p[2]<<8) +p[1];
								HI220_Data_From_Usart.Euler_Angle_Roll_s16  = (p[4]<<8) +p[3];
								HI220_Data_From_Usart.Euler_Angle_Yaw_s16   = (p[6]<<8) +p[5];
								HI220_Data_From_Usart.Euler_Angle_Pitch_s16_2_f = 0.01f * HI220_Data_From_Usart.Euler_Angle_Pitch_s16; //原始数据放大了100倍
								HI220_Data_From_Usart.Euler_Angle_Roll_s16_2_f = 0.01f * HI220_Data_From_Usart.Euler_Angle_Roll_s16;	 //原始数据放大了100倍
								HI220_Data_From_Usart.Euler_Angle_Yaw_s16_2_f = 0.1f * HI220_Data_From_Usart.Euler_Angle_Yaw_s16;			 //原始数据放大了10倍
								Length_Deal += (LENGTH_EULER_ANG_s16_0xd0+1); 
								break;
							case 0xd9:
								memcpy(&HI220_Data_From_Usart.Euler_Angle_Pitch.Euler_Angle_Pitch_u8, &p[1], 4);
								memcpy(&HI220_Data_From_Usart.Euler_Angle_Roll.Euler_Angle_Roll_u8, &p[5], 4);
								memcpy(&HI220_Data_From_Usart.Euler_Angle_Yaw.Euler_Angle_Yaw_u8, &p[9], 4);
								Length_Deal += (LENGTH_EULER_ANG_f_0xd9+1); 
								break;
							case 0xd1:
								memcpy(&HI220_Data_From_Usart.Quaternion_W.Quaternion_W_u8, &p[1], 4);
								memcpy(&HI220_Data_From_Usart.Quaternion_X.Quaternion_X_u8, &p[5], 4);
								memcpy(&HI220_Data_From_Usart.Quaternion_Y.Quaternion_Y_u8, &p[9], 4);
								memcpy(&HI220_Data_From_Usart.Quaternion_Z.Quaternion_Z_u8, &p[13], 4);
								Length_Deal += (LENGTH_QUATERNION_0xd1+1); 
								break;
							case 0xf0:
								Length_Deal += (LENGTH_AIR_PRESS_0xf0+1); 
								break;
							default:
								Length_Deal = length;
								break;					
							
						}
					}
				}
				
			}
		}

        volatile static float Last_yaw_temp,Yaw_temp,Last_pitch_temp,Pitch_temp,Last_roll_temp,Roll_temp; //
	    volatile static int Yaw_count,Pitch_count,Roll_count;

        GYRO->yaw_Gyro = -HI220_Data_From_Usart.Ang_Velocity_Z * 0.1f;
        GYRO->pitch_Gyro = -HI220_Data_From_Usart.Ang_Velocity_Y * 0.1f;
        GYRO->roll_Gyro = -HI220_Data_From_Usart.Ang_Velocity_X * 0.1f;
        
        Last_yaw_temp = Yaw_temp;
        Yaw_temp = -HI220_Data_From_Usart.Euler_Angle_Yaw_s16_2_f; 
        if(Yaw_temp-Last_yaw_temp>=330)  
        {
            Yaw_count--;
        }
        else if (Yaw_temp-Last_yaw_temp<=-330)
        {
            Yaw_count++;
        }
    //	yaw_Angle = Yaw_temp + Yaw_count*360; 

    //	pitch_Angle = HI220_Data_From_Usart.Euler_Angle_Pitch_s16_2_f;   //*************************去负号*********************************
        Last_pitch_temp = Pitch_temp;
        Pitch_temp = HI220_Data_From_Usart.Euler_Angle_Pitch.Euler_Angle_Pitch_f;;  
        if(Pitch_temp-Last_pitch_temp>=330)  
        {
            Pitch_count--;
        }
        else if (Pitch_temp-Last_pitch_temp<=-330)
        {
            Pitch_count++;
        }
        GYRO->pitch_Angle = Pitch_temp + Pitch_count*360; 
        Last_roll_temp = Roll_temp;
        
        Roll_temp = HI220_Data_From_Usart.Euler_Angle_Roll.Euler_Angle_Roll_f;;  
        if(Roll_temp-Last_roll_temp>=330)  
        {
            Roll_count--;
        }
        else if (Roll_temp-Last_roll_temp<=-330)
        {
            Roll_count++;
        }
    //	roll_Angle = Roll_temp + Roll_count*360; 
        GYRO->yaw_Angle = Yaw_temp + Yaw_count*360; 
        GYRO->roll_Angle = HI220_Data_From_Usart.Euler_Angle_Roll_s16_2_f+4.67f;///////////////////////////////////////////..............................
        GYRO->pitch_Angle = HI220_Data_From_Usart.Euler_Angle_Pitch_s16_2_f;   //*************************去负号*********************************
}


/*****************************dji encoder*************************************/

void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg)
{

            v->ecd_bias = (msg->Data[0]<<8)|msg->Data[1];  //保存初始编码器值作为偏差  
            v->ecd_value = v->ecd_bias;
            v->last_raw_value = v->ecd_bias;
            v->temp_count++;
}

void EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*0.04394531f + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);		
	v->rate_rpm = (msg->Data[2]<<8)|msg->Data[3];
}

void GM6020EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*0.0439453125f  + v->round_cnt * 360;
	v->filter_rate = (msg->Data[2]<<8)|msg->Data[3];
//	if(v->filter_rate>32768)
//	{
//		v->filter_rate = (~((msg->Data[2]<<8)|msg->Data[3])+1);
//	}
	v->temperature = msg->Data[6];
}


void M3508orM2006EncoderTask(uint32_t can_count,volatile Encoder *v, CanRxMsg * msg)
{
	(can_count<=50)?GetEncoderBias(v,msg):EncoderProcess(v,msg);
}


void GM6020EncoderTask(uint32_t can_count,volatile Encoder *v, CanRxMsg * msg,int offset)
{

	GM6020EncoderProcess(v, msg);
	// 码盘中间值设定也需要修改
	if (can_count <= 100)
	{
		if ((v->ecd_bias - v->ecd_value) < -4000)
		{
				v->ecd_bias = offset + 8192;
		}
		else if ((v->ecd_bias - v->ecd_value) > 4000)
		{
				v->ecd_bias = offset - 8192;
		}
	}
}

/*******************************LK_Tech电机***********************************/

void MF_EncoderProcess(volatile Encoder *v, CanRxMsg * msg)//云台yaw，pitch共用
{
	int i=0;
	int32_t temp_sum = 0;
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[7]<<8)|msg->Data[6];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -32768)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 65536;
	}
	else if(v->diff>32768)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 65536;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	v->ecd_value = v->raw_value + v->round_cnt * 65536;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*0.0054931641f  + v->round_cnt * 360;
	//从电机编码器读取的速度
	v->filter_rate = (msg->Data[5]<<8)|msg->Data[4];
	v->temperature = msg->Data[1];
}

void MF_EncoderTask(uint32_t can_count,volatile Encoder *v, CanRxMsg * msg,int offset)
{

	MF_EncoderProcess(v, msg);
	// 码盘中间值设定也需要修改
	if (can_count <= 100)
	{
		if ((v->ecd_bias - v->ecd_value) < -32700)
		{
				v->ecd_bias = offset + 65536;
		}
		else if ((v->ecd_bias - v->ecd_value) > 32700)
		{
				v->ecd_bias = offset - 65536;
		}
	}
}

/********************************HT430_J10************************************/

void HT_430_Information_Receive(CanRxMsg * msg,HT430_J10_t *HT430_J10_t,volatile Encoder *v)
{
  switch ((msg->StdId&0xfffe)>>4)
	{
		case 0x2f:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=msg->Data[7]<<8|msg->Data[6];
		}break;
		
		case 0x40:
		{
			HT430_J10_t->Voltage=msg->Data[0]*0.2;
			HT430_J10_t->Currents=msg->Data[1]*0.03;
			HT430_J10_t->Temperature=msg->Data[2]*0.4;
			HT430_J10_t->DTC=msg->Data[3];
			HT430_J10_t->Operating_State=msg->Data[4];
		}break;
		
		case 0x53:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=msg->Data[7]<<8|msg->Data[6];
		}break;
		
		case 0x54:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=(msg->Data[7]<<8|msg->Data[6]);
		}break;
		
		case 0x55:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=msg->Data[7]<<8|msg->Data[6];
		}break;
		
		case 0x56:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=msg->Data[7]<<8|msg->Data[6];
		}break;
		
		case 0x57:
		{
//			HT430_J10_t->V=(msg->Data[1]<<8|msg->Data[0])*0.1;
		}break;
		
		default:
		{
		}break;
	}
	HT430_J10_t->V=HT430_J10_t->V*360/16384/6;
	v->ecd_angle = HT430_J10_t->Total_Angle;
	v->filter_rate = HT430_J10_t->V;
}
/*******************************capacitance*****************************************/

void PM01_message_Process(volatile capacitance_message_t *v,CanRxMsg * msg)
{
	switch (msg->StdId)
	{
	case 0x610:
	{
		v->mode = (msg->Data[0] << 8) | msg->Data[1];
		v->mode_sure = (msg->Data[2] << 8) | msg->Data[3];
	}
	break;
	case 0x611:
	{
		v->in_power = (msg->Data[0] << 8) | msg->Data[1];
		v->in_v = (msg->Data[2] << 8) | msg->Data[3];
		v->in_i = (msg->Data[4] << 8) | msg->Data[5];
	}
	break;
	case 0x612:
	{
		v->out_power = (msg->Data[0] << 8) | msg->Data[1];
		v->out_v = (msg->Data[2] << 8) | msg->Data[3];
		v->out_i = (msg->Data[4] << 8) | msg->Data[5];
	}
	break;
	case 0x613:
	{
		v->tempureture=(msg->Data[0]<<8)|msg->Data[1];
		v->time=(msg->Data[2]<<8)|msg->Data[3];
        v->this_time=(msg->Data[4]<<8)|msg->Data[5];
	}break;

	default:
		break;
	}
}

/***********************************    ↓    DJI提供的CRC校检函数   ↓  ***********************************/
//crc8 generator polynomial:G(x)=x8+x5+x4+1
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
{
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};


unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8)
{
    unsigned char ucIndex;
    while (dwLength--)
    {
        ucIndex = ucCRC8^(*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return(ucCRC8);
}


/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2)) 
        return 0;
    ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
    return ( ucExpected == pchMessage[dwLength-1] );
}


/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2)) 
        return;
    ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
    pchMessage[dwLength-1] = ucCRC;
}

uint16_t CRC_INIT = 0xffff;

const uint16_t wCRC_Table[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage ==NULL)
    {
        return 0xFFFF;
    }
    while(dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^
        (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}


/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return 0;
//		return __FALSE; 
    }
    wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}


/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return;
    }
    wCRC = Get_CRC16_Check_Sum ( (u8 *)pchMessage, dwLength-2, CRC_INIT );
    pchMessage[dwLength-2] = (u8)(wCRC & 0x00ff);
    pchMessage[dwLength-1] = (u8)((wCRC >> 8)& 0x00ff);
}

unsigned char get_crc8(unsigned char* data, unsigned int length)
{
	unsigned char ucExpected = 0;
  if ((data == 0) || (length <= 2))
    return 0xFF;
  return Get_CRC8_Check_Sum (data, length, CRC8_INIT);
}
/***********************************    ↑    DJI提供的CRC校检函数   ↑  ***********************************/   

/**********************************************Judge_handle****************************************/
void judgement_data_handle(uint8_t *p_frame, u16 rec_len)
{
  u8 header[HEADER_LEN];
  u8 data[32];
  u16 deal_cnt = 0;
  u8 sof;
  uint16_t data_length;
  uint16_t cmd_id;
  uint8_t *data_addr;
  u16 Frame_length = 0;

  //  frame_header_t *p_header = (frame_header_t*)p_frame;
  //  memcpy(p_header, p_frame, HEADER_LEN);

  while (rec_len > deal_cnt)
  {
	sof = p_frame[deal_cnt];
	data_length = ((u16)p_frame[deal_cnt + 2] << 8) | p_frame[deal_cnt + 1]; // p_header->data_length;
	cmd_id = ((u16)p_frame[deal_cnt + 6] << 8) | p_frame[deal_cnt + 5];		 //*(uint16_t *)(p_frame + HEADER_LEN);
	data_addr = &p_frame[deal_cnt] + HEADER_LEN + CMD_LEN;

	memcpy(header, &p_frame[deal_cnt], HEADER_LEN);

	Frame_length = HEADER_LEN + CMD_LEN + data_length + CRC_LEN;

	if (sof == DN_REG_ID && Verify_CRC8_Check_Sum(header, HEADER_LEN) && Verify_CRC16_Check_Sum(&p_frame[deal_cnt], Frame_length))
	{
				switch (cmd_id)
				{

				case GAME_STATE_ID: // 比赛状态数据：0x0001。发送频率：1Hz
				{
					memcpy(&judge_rece_mesg.game_state, data_addr, data_length);
				}
				break;

				case GAME_ROBOT_SURVIVORS_ID: // 机器人存活数据：0x0003。发送频率：1Hz
				{
					memcpy(&judge_rece_mesg.game_robot_HP, data_addr, data_length);
				}
				break;

				case SUPPLY_PROJECTILE_BOOKING_ID: // 请求补给站补弹子弹：cmd_id (0x0103)。发送频率：上限 10Hz
				{
					memcpy(&judge_rece_mesg.supply_projectile_booking, data_addr, data_length);
					
				}
				break;

				case GAME_ROBOT_STATE_ID: // 比赛机器人状态：0x0201。发送频率：10Hz

					if (data_length == 27)
					{
						memcpy(&judge_rece_mesg.game_robot_state, data_addr, data_length);

						if (judge_rece_mesg.game_robot_state.robot_id >= 1 && judge_rece_mesg.game_robot_state.robot_id <= 7)
						{
								judge_rece_mesg.robot_color = red;
						}
						else if (judge_rece_mesg.game_robot_state.robot_id >= 11 && judge_rece_mesg.game_robot_state.robot_id <= 17)
						{
								judge_rece_mesg.robot_color = blue;
						}
					}

					break;
				case POWER_HEAT_DATA_ID: // 实时功率热量数据：0x0202。发送频率：50Hz
				{

					if (data_length == 16)
					{
						memcpy(&judge_rece_mesg.power_heat_data, data_addr, data_length);


					}
				}
				break;

				case BUFF_MUSK_ID: // 机器人增益：0x0204。发送频率：状态改变后发送
								   // memcpy(&data, data_addr, data_length)
					if (data_length == 1)
					{
						judge_rece_mesg.buff_musk.power_rune_buff = data_addr[0];
					}
					break;
				case ROBOT_HURT_ID: // 伤害状态：0x0206。发送频率：伤害发生后发送
				{
					memcpy(&judge_rece_mesg.robot_hurt, data_addr, data_length);
				}
				break;
				case SHOOT_DATA_ID: // 实时射击信息：0x0207。发送频率：射击后发送
				{

					if (data_length == 7)
					{
						memcpy(&judge_rece_mesg.shoot_data, data_addr, data_length);
					}
				}
				break;
				case BULLET_REMAINING_ID:
				{
					memcpy(&judge_rece_mesg.ext_bullet_remaining, data_addr, data_length);
				}
				break;
				case STUDENT_INTERACTIVE_HEADER_DATA_ID: // 交互数据接收信息：0x0301。发送频率：上限 10Hz
					memcpy(&judge_rece_mesg.student_interactive_header_data, data_addr, data_length);

					break;
				case ROBOT_COMMAND_ID:
				{
					memcpy(&judge_rece_mesg.ext_robot_command, data_addr, data_length);
				}
				break;
				default:
					break;
				}
	}
	deal_cnt += Frame_length;
  }
}

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
	float flagg_x = Uart4_Protobuf_Receive_Gimbal_Angle->x_;
	float flagg_y = Uart4_Protobuf_Receive_Gimbal_Angle->y_;
	if (flagg_x == Uart4_Protobuf_Receive_Gimbal_Angle->x_ && flagg_y == Uart4_Protobuf_Receive_Gimbal_Angle->y_)
	{
		if (flagg_x != 0 && flagg_y != 0)
		{
			new_location.xy_0_flag = 0;
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
//	Uart4SendBytesInfoProc(data, DateLength + 4);
}
/***********************************遥控器接收*************************************************************/
void RemoteDataPrcess(uint8_t *pData)
{
	if (pData == NULL)
	{
		return;
	}
	// 遥控器部分
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
						  ((int16_t)pData[4] << 10)) &
						 0x07FF;
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) & 0x07FF;
	RC_CtrlData.rc.ch4 = ((int16_t)pData[16] | ((int16_t)pData[17] << 8)) & 0x07FF;
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003); // 模式切换
	// 鼠标部分
	RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
	RC_CtrlData.mouse.press_l = pData[12];
	RC_CtrlData.mouse.press_r = pData[13];
	RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);

	/***********************remote_task*****************************/

	/*****************************************************************/
}
