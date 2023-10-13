#include "CH100.h"


/**
  ******************************************************************************
  * @file    CH100.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   此文件编写了与CH100陀螺仪的数据接收与解算，
							函数入口参数为串口dma接收地址与通用陀螺仪
							结构体
						 
@verbatim
 ===============================================================================
 **/
 
 
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

    GYRO->pitch_Gyro = -dat.gyr[1];
    GYRO->yaw_Gyro = -dat.gyr[2];
    GYRO->roll_Gyro = dat.gyr[0];

    GYRO->x_Acc = dat.acc[0];
    GYRO->y_Acc = dat.acc[2];
    GYRO->z_Acc = dat.acc[1];   

}

