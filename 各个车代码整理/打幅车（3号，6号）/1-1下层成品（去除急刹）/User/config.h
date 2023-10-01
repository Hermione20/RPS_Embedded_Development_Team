  #ifndef __SYS_H_、_
#define __SYS_H__

//-----------允许修改部分-----------//
//==========================================================
// <o> STANDARD  - 几号步兵
// <3=> NUM_3
// <4=> NUM_4
// <5=> NUM_5
#define STANDARD 			3 //选择步兵为3号 或者4号

//==========================================================
//<h> 陀螺仪设置
#define HI219 		0
#define ICM20948 	1
// <o> IMU  陀螺仪型号
// <0=> HI219
// <1=> ICM20948
//#define IMU 		ICM20948			//IMU选择为HI219 或者ICM20948
#define IMU  HI219
// <q> GYRO_CALI  - 是否校准陀螺仪
#define GYRO_CALI 			1                   //1为校准0为不校准
// <q> HI219_FIRST_USED  - 是否是第一次使用HI219
#define HI219_FIRST_USED 	0	 				//是否是第一次使用HI219
//</h>
//==========================================================
// <o> REMOTE_SHOOT  - 遥控器左拨杆的功能选择
// <0=> 小陀螺
// <1=> 发射
#define REMOTE_SHOOT        1                 //1:遥控器左拨杆为发射  0:小陀螺

//==========================================================
// <q> NEW_CAP  - 是否为新电容控制板
#define NEW_CAP             1                   //1:新电容控制板   0:旧电容控制板

//==========================================================
//<h> 自瞄设置
// <q> NEW_CAP  - 是否使能速度预测
#define ARMY_SPEED_PREDICTION 1
// <q> ENABLE_KALMAN_FILTER  - 是否使能卡尔曼滤波
#define ENABLE_KALMAN_FILTER  1
//</h>
#define  POWER_LIMT                   1  //1时使用上交算法，0时使用直接限电流法

//==========================================================
#if STANDARD == 3                                          //
//标识3号码步兵
//pitch轴电机初始位置
#define  GMPitchEncoder_Offset 6930
//yaw轴电机初始位置
#define GMYawEncoder_Offset   2709//4053

#define  Vehicle_Num         3
#define  GM1Encoder_Offset   1411+1024+2048       //5335
#define  GM2Encoder_Offset   5433+1024-50   //3595   //7910
#define  GM3Encoder_Offset   1269+1024+2048
#define  GM4Encoder_Offset   8037-1024-2048  //3676



//陀螺仪X轴默认校准值
#define GYRO_REAL_X_OFFSET 0.00559780467f
//陀螺仪Y轴默认校准值
#define GYRO_REAL_Y_OFFSET -0.0156064359
//陀螺仪Z轴默认校准值
#define GYRO_REAL_Z_OFFSET -0.0091483118

//图像X轴中心位置
#define IMAGE_X_OFFET 0 //激光相对于实际落点偏右，补偿为负
//图像Y轴中心位置
#define IMAGE_Y_OFFET 0
//摄像头和枪管中心的安装偏差角-YAW方向  右上是正
#define YAW_ANGLE_BETWEEN_GUN_CAMERA 		-1.2
//摄像头和枪管中心的安装偏差角
#define PITCH_ANGLE_BETWEEN_GUN_CAMERA 		1.8F
//摄像头和枪管中心的距离
#define HEIGHT_BETWEEN_GUN_CAMERA 			78.5F
//相机焦距mm
#define FOCAL_LENGTH                6.0F
//靶面长mm
#define TARGET_SURFACE_LENGTH      3.45e-3F
//靶面宽mm
#define TARGET_SURFACE_WIDTH        3.45e-3F
//像素尺寸mm
#define IMAGE_LENGTH                3.45e-3f

//远距离时枪管角度补偿  //no used
#define ANGLE_COMPENSATION_LONG_DISTANCE 	2
//图像和云台控制延迟时间 - 秒 /
#define YAW_IMAGE_GIMBAL_DELAY_TIME   0e-3f//   0//s 
#define PIT_IMAGE_GIMBAL_DELAY_TIME       	0
#define DISTANCE_OFFSET           50


#elif STANDARD == 4
////标识4号码步兵
#define  GMPitchEncoder_Offset 6758
//yaw轴电机初始位置
#define  GMYawEncoder_Offset   2750

#define  GM1Encoder_Offset   1350+1024+2048//5335
#define  GM2Encoder_Offset   2731+1024  //3595   //7910
#define  GM3Encoder_Offset   1304+1024+2048
#define  GM4Encoder_Offset   5400+1024  //3676

#define  Vehicle_Num         4

#define GYRO_REAL_X_OFFSET 0.00559780467
#define GYRO_REAL_Y_OFFSET -0.0156064359
#define GYRO_REAL_Z_OFFSET -0.0091483118

//图像X轴中心位置
#define IMAGE_X_OFFET 0 //激光相对于实际落点偏右，补偿为负
//图像Y轴中心位置
#define IMAGE_Y_OFFET 0
//摄像头和枪管中心的安装偏差角-YAW方向
#define YAW_ANGLE_BETWEEN_GUN_CAMERA 		-0.8f
//摄像头和枪管中心的安装偏差角
#define PITCH_ANGLE_BETWEEN_GUN_CAMERA   3.5f
//摄像头和枪管中心的距离
#define HEIGHT_BETWEEN_GUN_CAMERA 	78.5f
//相机焦距mm
#define FOCAL_LENGTH                6.0F
//靶面长mm
#define TARGET_SURFACE_LENGTH      3.45e-3F
//靶面宽mm
#define TARGET_SURFACE_WIDTH        3.45e-3F
#define IMAGE_LENGTH                3.45e-3f
//图像和云台控制延迟时间 - 秒 /
#define YAW_IMAGE_GIMBAL_DELAY_TIME   50e-3f         // 单位s
#define PIT_IMAGE_GIMBAL_DELAY_TIME    0
#define DISTANCE_ENABLE  0        //自瞄是否能够使用距离信息
#define MAX_ATTACK_DISTANCE 800  //单位cm



#elif STANDARD == 5                                    //
////标识5号码步兵
#define  GMPitchEncoder_Offset 0
//yaw轴电机初始位置
#define  GMYawEncoder_Offset   4758


/*顺时针高速转动贼稳

*/
#define  GM1Encoder_Offset   5471        //5335
#define  GM2Encoder_Offset   2703  //3595   //7910
#define  GM3Encoder_Offset   5510
#define  GM4Encoder_Offset   1326   //3676


#define  Vehicle_Num         5
#define GYRO_REAL_X_OFFSET 0.00559780467
#define GYRO_REAL_Y_OFFSET -0.0156064359
#define GYRO_REAL_Z_OFFSET -0.0091483118

//图像X轴中心位置
#define IMAGE_X_OFFET 0 //激光相对于实际落点偏右，补偿为负
//图像Y轴中心位置
#define IMAGE_Y_OFFET 0
//摄像头和枪管中心的安装偏差角-YAW方向
#define YAW_ANGLE_BETWEEN_GUN_CAMERA 		-0.5f
//摄像头和枪管中心的安装偏差角
#define PITCH_ANGLE_BETWEEN_GUN_CAMERA   1.2f
//摄像头和枪管中心的距离
#define HEIGHT_BETWEEN_GUN_CAMERA 	4.89f
//相机焦距mm
#define FOCAL_LENGTH                4.0F
//靶面长mm
#define TARGET_SURFACE_LENGTH      3.45e-3F
//靶面宽mm
#define TARGET_SURFACE_WIDTH        3.45e-3F
#define IMAGE_LENGTH                4.8e-3f
//图像和云台控制延迟时间 - 秒 /
#define YAW_IMAGE_GIMBAL_DELAY_TIME   50e-3f         // 单位s
#define PIT_IMAGE_GIMBAL_DELAY_TIME    0
#define DISTANCE_ENABLE  0        //自瞄是否能够使用距离信息
#define MAX_ATTACK_DISTANCE 800  //单位cm

#elif STANDARD == 6                                 //备用车
////标识4号码步兵
#define  GMPitchEncoder_Offset 6800
//yaw轴电机初始位置
#define  GMYawEncoder_Offset   4000
#define GYRO_REAL_X_OFFSET 0.00559780467
#define GYRO_REAL_Y_OFFSET -0.0156064359
#define GYRO_REAL_Z_OFFSET -0.0091483118

#define Vehicle_Num 6

#define  GM1Encoder_Offset   5205-1024       //5335
#define  GM2Encoder_Offset   4333+1024  //3595   //7910
#define  GM3Encoder_Offset   1110+1024+2048
#define  GM4Encoder_Offset   4339+1024  //3676


//图像X轴中心位置
#define IMAGE_X_OFFET 0 //激光相对于实际落点偏右，补偿为负
//图像Y轴中心位置
#define IMAGE_Y_OFFET 0
//摄像头和枪管中心的安装偏差角-YAW方向
#define YAW_ANGLE_BETWEEN_GUN_CAMERA 		-0.5f
//摄像头和枪管中心的安装偏差角
#define PITCH_ANGLE_BETWEEN_GUN_CAMERA   1.2f
//摄像头和枪管中心的距离
#define HEIGHT_BETWEEN_GUN_CAMERA 	4.89f
//相机焦距mm
#define FOCAL_LENGTH                6.0F
//靶面长mm
#define TARGET_SURFACE_LENGTH      3.45e-3F
//靶面宽mm
#define TARGET_SURFACE_WIDTH        3.45e-3F
#define IMAGE_LENGTH                4.8e-3f

//图像和云台控制延迟时间 - 秒 /
#define YAW_IMAGE_GIMBAL_DELAY_TIME   50e-3f         // 单位s
#define PIT_IMAGE_GIMBAL_DELAY_TIME    0
#define DISTANCE_ENABLE  0        //自瞄是否能够使用距离信息
#define MAX_ATTACK_DISTANCE 800  //单位cm

#else
"ERROR,please define STANDARD as 3 or 4 or 5 or 6 "

#endif

//==========================================================
//<<< end of configuration section >>>\n
#endif
