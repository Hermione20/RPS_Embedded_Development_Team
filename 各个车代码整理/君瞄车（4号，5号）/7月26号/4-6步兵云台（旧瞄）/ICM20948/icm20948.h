
#ifndef __ICM20948_H
#define __ICM20948_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes --------------------------------------------------------------------------------*/
#include "stm32f4xx.h"

#define Ts 2e-3f //Ω‚À„÷‹∆⁄



typedef struct 
{
	s16 Accel_X;  
	s16 Accel_Y;  
	s16 Accel_Z;  
	s16 Temp;     
	s16 Gyro_X;   
	s16 Gyro_Y;   
	s16 Gyro_Z;  
	s16 Mag_X;    
	s16 Mag_Y;   
	s16 Mag_Z;    
	
}IMU_RAW_DATA;



typedef struct 
{
    float Accel_X;  
    float Accel_Y;  
    float Accel_Z;  
    float Temp;     
    float Gyro_X;   
    float Gyro_Y;   
    float Gyro_Z;   
		float GyroXOffset;
		float GyroYOffset;
		float GyroZOffset;
	  float Mag_X;    
    float Mag_Y;    
    float Mag_Z;    
}IMU_REAL_DATA;



extern float yaw_Angle,pitch_Angle,roll_Angle; 
extern float pitch_Gyro,yaw_Gyro;


extern IMU_RAW_DATA    		IMU_Raw_Data;
extern IMU_REAL_DATA      IMU_Real_Data;

void IMU_getYawPitchRoll(void);
void IMU_Gyro_calibration(IMU_REAL_DATA *real_data);


#ifdef __cplusplus
}
#endif

#endif

/*************************************** END OF FILE ****************************************/
