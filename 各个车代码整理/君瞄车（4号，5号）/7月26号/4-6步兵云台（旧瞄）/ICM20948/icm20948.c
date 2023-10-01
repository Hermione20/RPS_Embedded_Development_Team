#include "icm20948.h"
#include "arm_math.h"
#include "ICM20948_driver.h"
#include "delay.h"
#include "AHRS.h"
#include "main.h"

IMU_RAW_DATA    	 IMU_Raw_Data;
IMU_REAL_DATA      IMU_Real_Data, IMU_Calied_Real_Data;


float yaw_Angle,pitch_Angle,roll_Angle; 
float pitch_Gyro,yaw_Gyro;

float INS_gyro[3] = {0.0f, 0.0f, 0.0f};
float INS_accel[3] = {0.0f, 0.0f, 0.0f};
float INS_mag[3] = {0.0f, 0.0f, 0.0f};

float INS_Angle[3] = {0.0f, 0.0f, 0.0f};      //欧拉角 单位 rad
float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f}; //四元数


//--------------------------------------------
//数据处理
//--------------------------------------------


/*全部原始数据获取*/
void IMU_GetRawData( IMU_RAW_DATA *raw_data ) 
{
	uint8_t buff[23] = {0};

  ICM94_ReadRegs(ICM20948_ACCEL_XOUT_H, buff, 23);  /* Acc, Gyr, Mag, Temp */

  raw_data->Gyro_X =  (int16_t)((buff[6]   << 8) | buff[7]);    /* Gyr.X */
  raw_data->Gyro_Y =  (int16_t)((buff[8]   << 8) | buff[9]);    /* Gyr.Y */
  raw_data->Gyro_Z =  (int16_t)((buff[10]  << 8) | buff[11]);   /* Gyr.Z */
	
  raw_data->Accel_X = (int16_t)((buff[0]   << 8) | buff[1]);    /* Acc.X */
  raw_data->Accel_Y = (int16_t)((buff[2]   << 8) | buff[3]);    /* Acc.Y */
  raw_data->Accel_Z = (int16_t)((buff[4]   << 8) | buff[5]);    /* Acc.Z */
	
	raw_data->Mag_X =  (int16_t)(((buff[16]  << 8) | buff[15])); /* Mag.X */
  raw_data->Mag_Y =  (int16_t)(((buff[18]  << 8) | buff[17])); /* Mag.Y */
  raw_data->Mag_Z =  (int16_t)(((buff[20]  << 8) | buff[19])); /* Mag.Z */

  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_IMU));   //feed(clear) the IMU ERROR Count
}

void IMU_Trans_RawData_2_RealData(IMU_RAW_DATA *rawdata, IMU_REAL_DATA *realdata)
{
	realdata->Accel_X = rawdata->Accel_X * ACCEL_SEN;
	realdata->Accel_Y = rawdata->Accel_Y * ACCEL_SEN;
	realdata->Accel_Z = rawdata->Accel_Z * ACCEL_SEN;

	realdata->Gyro_X = rawdata->Gyro_X * GYRO_SEN;
	realdata->Gyro_Y = rawdata->Gyro_Y * GYRO_SEN;
	realdata->Gyro_Z = rawdata->Gyro_Z * GYRO_SEN;
	
	realdata->Mag_X = rawdata->Mag_X * MAG_SEN;
	realdata->Mag_Y = rawdata->Mag_Y * MAG_SEN;
	realdata->Mag_Z = rawdata->Mag_Z * MAG_SEN;
}

float Filter_for_GYRO(float input,float *data_temp)
{
	uint8_t i = 0;
	float sum=0;
	
	for(i=1;i<10;i++)
	{
		data_temp[i-1] = data_temp[i];
	}
	data_temp[9] = input;
	
	for(i=0;i<10;i++)
	{	
		 sum += data_temp[i];
	}
	return(sum*0.1f);
}

static void IMU_Cali_Slove(IMU_REAL_DATA *realdata, IMU_REAL_DATA *calied_realdata)
{
	static float temp_Gx[10]={0}; 
	static float temp_Gy[10]={0}; 
	static float temp_Gz[10]={0}; 

	calied_realdata->Gyro_X = Filter_for_GYRO((realdata->Gyro_X ), temp_Gx)- realdata->GyroXOffset;
	calied_realdata->Gyro_Y = Filter_for_GYRO((realdata->Gyro_Y ), temp_Gy)- realdata->GyroYOffset;
	calied_realdata->Gyro_Z = Filter_for_GYRO((realdata->Gyro_Z ), temp_Gz)- realdata->GyroZOffset;
	
	calied_realdata->Accel_X 	= 	(realdata->Accel_X );
	calied_realdata->Accel_Y 	= 	(realdata->Accel_Y );
	calied_realdata->Accel_Z 	= 	(realdata->Accel_Z );
	calied_realdata->Mag_X 		= 	-(realdata->Mag_X );
	calied_realdata->Mag_Y 		= 	(realdata->Mag_Y );
	calied_realdata->Mag_Z 		= 	(realdata->Mag_Z );	
}

void IMU_getYawPitchRoll() 
{  
	volatile static float Last_yaw_temp,Yaw_temp,Last_pitch_temp,Pitch_temp; //
	volatile static int Yaw_count,Pitch_count;

	IMU_GetRawData		 (&IMU_Raw_Data);//原始数据读取
	IMU_Trans_RawData_2_RealData(&IMU_Raw_Data,&IMU_Real_Data);	//转换成实际值
	IMU_Cali_Slove(&IMU_Real_Data, &IMU_Calied_Real_Data);			//减去零漂以及统一坐标系

	pitch_Gyro 	= -IMU_Calied_Real_Data.Gyro_Y * RAD_TO_ANGLE;//-values[2];
	yaw_Gyro 		= -IMU_Calied_Real_Data.Gyro_Z * RAD_TO_ANGLE;//-values[3];
	
	//重新分配输入量，以保证计算的pitch角度是-180到180度，从而转换为连续角度
	INS_gyro[0] = -IMU_Calied_Real_Data.Gyro_Y;
	INS_gyro[1] = IMU_Calied_Real_Data.Gyro_X;
	INS_gyro[2] = IMU_Calied_Real_Data.Gyro_Z;
	INS_accel[0] = -IMU_Calied_Real_Data.Accel_Y;
	INS_accel[1] = IMU_Calied_Real_Data.Accel_X;
	INS_accel[2] = IMU_Calied_Real_Data.Accel_Z;
	INS_mag[0] = -IMU_Calied_Real_Data.Mag_Y;
	INS_mag[1] = IMU_Calied_Real_Data.Mag_X;
	INS_mag[2] = IMU_Calied_Real_Data.Mag_Z;	
	
	//加速度计低通滤波//反推得 截至频率为7.67Hz
	static float accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
	static float accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
	static float accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
	static const float fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
	
	//判断是否第一次进入，如果第一次则初始化四元数，之后更新四元数计算角度单位rad
	static uint8_t updata_count = 0;
	if (updata_count == 0)
	{
			//初始化四元数
			AHRS_init(INS_quat, INS_accel, INS_mag);
			get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);

			accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
			accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
			accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
			updata_count++;
	}
	else
	{	
	 //加速度计低通滤波
		accel_fliter_1[0] = accel_fliter_2[0];
		accel_fliter_2[0] = accel_fliter_3[0];

		accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

		accel_fliter_1[1] = accel_fliter_2[1];
		accel_fliter_2[1] = accel_fliter_3[1];

		accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

		accel_fliter_1[2] = accel_fliter_2[2];
		accel_fliter_2[2] = accel_fliter_3[2];

		accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

		//更新四元数
		AHRS_update(INS_quat, Ts, INS_gyro, accel_fliter_3, INS_mag);
		get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);			
	}
	
	Last_yaw_temp = Yaw_temp;
	Yaw_temp = -INS_Angle[0] * RAD_TO_ANGLE; 
	if(Yaw_temp-Last_yaw_temp>=330)  
	{
		Yaw_count--;
	}
	else if (Yaw_temp-Last_yaw_temp<=-330)
	{
		Yaw_count++;
	}
	yaw_Angle = (Yaw_temp + Yaw_count*360); 

	Last_pitch_temp = Pitch_temp;
	Pitch_temp = INS_Angle[2] * RAD_TO_ANGLE;  
	if(Pitch_temp-Last_pitch_temp>=330)  
	{
		Pitch_count--;
	}
	else if (Pitch_temp-Last_pitch_temp<=-330)
	{
		Pitch_count++;
	}
	pitch_Angle =-(Pitch_temp +Pitch_count*360); 
	
}

void IMU_Gyro_calibration(IMU_REAL_DATA *real_data)
{
	uint16_t loopCount = 0;	
	uint16_t loopTime = 1024;
	
	u8 state_cali = 0;		//是否校正完成标志位
	u16 Cnt_Check = 1000;	//检查校正是否完成的次数
	
	float gXSum = 0;
	float gYSum = 0;
	float gZSum = 0;
	IMU_RAW_DATA raw_data;

	for(int i = 0;i<50;i++)//读取十次，去掉刚开始不稳定的值
	{
		//IMU_GetData(&temp[1],&temp[2],&temp[3],&temp[4],&temp[5],&temp[6]);//这个函数是已经减去零偏的函数
		IMU_GetRawData(&raw_data);//原始数据读取
		delay_ms(10);
	}
	while(state_cali == 0)
	{
		while(loopCount < loopTime)
		{
				IMU_GetRawData(&raw_data);//原始数据读取
				IMU_Trans_RawData_2_RealData(&raw_data, real_data);	//转换成实际值

				gXSum += real_data->Gyro_X;//[1];
				gYSum += real_data->Gyro_Y;//temp[2];
				gZSum += real_data->Gyro_Z;//temp[3];
				if(fabs(real_data->Gyro_X) > 0.1f || fabs(real_data->Gyro_Y) > 0.1f || fabs(real_data->Gyro_Z) > 0.1f )
				{
					loopCount = 0;
					gXSum = 0;
					gXSum = 0;
					gXSum = 0;
				}
				else
					loopCount++;
				delay_ms(1);
		}
		real_data->GyroXOffset = gXSum/loopTime;   //读取pitch轴陀螺仪作为偏差
		real_data->GyroYOffset = gYSum/loopTime;		//
		real_data->GyroZOffset = gZSum/loopTime;		//读取yaw轴陀螺仪作为偏差
		
		delay_ms(1);
		IMU_GetRawData(&raw_data);//原始数据读取
		IMU_Trans_RawData_2_RealData(&raw_data, real_data);	//转换成实际值
		
		while(Cnt_Check)
		{
			if(	 fabs(real_data->Gyro_X - real_data->GyroXOffset) < 0.02f 
				&& fabs(real_data->Gyro_Y - real_data->GyroYOffset) < 0.02f
				&& fabs(real_data->Gyro_Z - real_data->GyroZOffset) < 0.02f)
			{
				Cnt_Check--;			
				if(Cnt_Check == 0)
				{
					state_cali = 1;
					break;
				}				
				else
				{
					IMU_GetRawData(&raw_data);//原始数据读取
					IMU_Trans_RawData_2_RealData(&raw_data, real_data);	//转换成实际值
					delay_ms(1);
				}
			}	
			else
			{
				state_cali = 0;
				Cnt_Check = 1000;
				gXSum = 0;
				gYSum = 0;
				gZSum = 0;
				loopCount = 0;
				break;
			}
		}
	}
}


