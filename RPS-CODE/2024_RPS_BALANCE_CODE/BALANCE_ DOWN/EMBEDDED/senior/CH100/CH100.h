#ifndef __CH100_H
#define __CH100_H
#include "public.h"


/***************************CH100********************************/

__packed typedef struct
{
uint8_t tag; /* ????:0x91 */
uint8_t id; /* ??ID */
uint8_t rev[2];
float prs; /* ?? */
uint32_t ts; /* ??? */
float acc[3]; /* ??? */
float gyr[3]; /* ??? */
float mag[3]; /* ?? */
float eul[3]; /* ???:
Roll,Pitch,Yaw */
float quat[4]; /* ??? */
}id0x91_t;


#ifndef GENERAL_GYRO_T
/**************************general gyro*********************************/
#define GENERAL_GYRO_T
typedef struct 
{
	float pitch_Angle;
	float yaw_Angle;
	float roll_Angle;
	float pitch_Gyro;
	float yaw_Gyro;
	float roll_Gyro;
	float x_Acc;
	float y_Acc;
	float z_Acc;
}general_gyro_t;



#endif


void CH100_getDATA(uint8_t *DataAddress,general_gyro_t *GYRO,float pitch_offset,
                                                            float roll_offset,
                                                            float pitch_gyro_offset,
                                                            float yaw_gyro_offset,
                                                            float roll_gyro_offset,
                                                            float acc_x_offset,
                                                            float acc_y_offset,
                                                            float acc_z_offset);


void Gyroscope_calibration(float value);


#endif
