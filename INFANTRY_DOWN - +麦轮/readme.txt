RPS 2024 赛季工程模板

本嵌入式框架分算法层，嵌入层，硬件层，固件库四部分

本框架头文件由main.h下辖：
	1.公共头文件，工程中每个.c文件必须包含一个.h文件，这些.h文件需要包含公共头文件
		公共头文件包含 
			  固件库
		      工具库
			  数学库
			  外设库
			  传感器
			  算法库
		（若之后添加的模块中有以上部分，请添加到公共头文件库）
	2.控制任务头文件
	     下辖各个控制任务的头文件
		 
外设初始化在BSP.c下调用

RTOS下的文件需包含MAIN.H

所有现役传感器的解算，均在传感器.c文件和头文件

10.3 chassis_task已改
//void get_remote_set()
//{
//		vx = can_chassis_data.x;//vx，x是横轴
//		vy = can_chassis_data.y;//vy，y是纵轴
// Chassis_angle.Remote_speed = sqrt((vx*vx)+(vy*vy));
//if(Chassis_angle.Remote_speed >= 50)
//{
// if(vx > 0)
// {
//	 Chassis_angle.Remote_angle = atan(vy / vx);
//   if(Chassis_angle.Remote_angle < 0)
//   {
//		 Chassis_angle.Remote_angle += 2*PI;
//	 } 
// }
// else if(vx<0) 
// {
//	 Chassis_angle.Remote_angle = atan(vy / vx);
//	 Chassis_angle.Remote_angle += PI;
// }
// else
//  {
//		 if(vy < 0)
//		 {
//		 Chassis_angle.Remote_angle = 3 * PI /2;}
//		 else if(vy > 0)
//		 {
//		 Chassis_angle.Remote_angle = PI / 2;
//		 }
//  }
// } 
//}


10.3 chassis_task已改
//	yaw_num_get = -yaw_Encoder.ecd_angle/360;
//	if(-yaw_Encoder .ecd_angle<0)
//	{yaw_num_get -=1;}
//	Chassis_angle.yaw_angle_0_2pi=(-yaw_Encoder.ecd_angle-yaw_num_get*360)*ANGLE_TO_RAD;
//逆时针增大！