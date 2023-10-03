#include "HI220.h"

/**
  ******************************************************************************
  * @file    HI220.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   此文件编写了与HI220陀螺仪的数据接收与解算，
							函数入口参数为串口dma接收地址与通用陀螺仪
							结构体,和数据长度
						 
@verbatim
 ===============================================================================
 **/

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
