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

/*******************************领空电机***********************************/

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

