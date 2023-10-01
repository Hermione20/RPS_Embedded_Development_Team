云台PITCH   6020
云台YAW     6020
底盘电机    3508*4
摩擦轮电机  3508*2
拨弹电机    2006

20190701
1、修改can初始化参数：
    can.CAN_ABOM = DISABLE ;    
    can.CAN_TXFP = ENABLE; 
	改为
    can.CAN_ABOM = ENABLE;    
    can.CAN_TXFP = DISABLE; 
2、注释掉can发送空闲中断；
3、can2接收部分清标志位提前至数据处理之前；
4、修改地盘电机控制参数：积分限幅提高至10000。

20190722
1、加入图像延迟和云台动作延迟；图像延时100ms+无卡尔曼+独立云台参数 
2、yaw速度环I太大时，陀螺开启后云台抖动严重；
3、预测中速度由卡尔曼滤波的速度给定，跟踪效果更稳定；

20190723
1、get_gimbal_mode和get_chassis_mode加入对遥控器输入的判断，解决了开机电机抖动问题；
2、将chassis.follow_gimbal = 1; 移至 if (gim.pid.yaw_angle_fdb-gim.pid.yaw_angle_ref >= -1.5f && gim.pid.yaw_angle_fdb-gim.pid.yaw_angle_ref <= 1.5f && gim.pid.pit_angle_fdb >= -1.5f && gim.pid.pit_angle_fdb <= 1.5f)
判断里。解决遥控开启时地盘随动的问题。
3、yaw轴极限速度1300左右
4、1000速，内环400，10，0 外环15 0 200
5、修改步兵5的参数：
    PID_struct_init(&pid_pit, POSITION_PID, 800, 10,
                    13.0f, 0,40); //
    PID_struct_init(&pid_pit_speed, POSITION_PID, 27000, 25000,
                    250.0f, 6.0f, 0);//160 8 0    ->  180 10 50  ->190 12 70   ->250 8 60
    //------------------------------------------------

     PID_struct_init(&pid_yaw, POSITION_PID, 1000, 0,
                    13.0f, 0, 100); //
     PID_struct_init(&pid_yaw_speed, POSITION_PID, 30000, 30000,
                    350.0f,6.0f, 0);//LQ //150.0f,4.0f, 20);
					
1、大小符模式是否正常
2、4号车，云台参数P减小；

	 PID_struct_init(&pid_yaw_follow, POSITION_PID, 1000, 1000,
                    6.0f, 0.0f, 100); 
     PID_struct_init(&pid_yaw_speed_follow, POSITION_PID, 29800, 29800,
                   280.0f,2.0f, 0);//I太大时，陀螺开启云台抖动严重
				   
3、云台超调减小，增大图像和控制延时，保证超前一个装甲，方可击中装甲片。


20190727
自动补弹功能完成
杀死后关断电容
底盘断电后关断

LQ
1、优化自瞄的静态和追踪效果；
2、添加相机和枪管的yaw角度安装偏差, 4号5号已经调整完成，3号有待调整。


20190802
因烧电调改电调限符值16000改为12000

20190804
启动加斜坡

20190806
传输数据
按键复位电容控制板，主控复位改为长按防止误触。 

20190807
软件限功率缓冲能量小于10时直接限制速度为0。
3号和5号长按Z键改为不限热量模式，按E关闭。

2021119   wyw
删除不必要的注释代码
包括：调试所用代码 
增加函数注释


20210305
增加激光测距 distance_measure.c

20210310
加入左右直线补偿

20210319
新版功率限制

20210320
升级固件 修改通讯协议
生机性能体系 由裁判系统读数给限制

20210325
优化功率限制  解决电容电压给定在限定低值处的反复横跳引起的抖动
float get_max_power(float voltage)
{
	int max_power=0;
    if(voltage>WARNING_VOLTAGE+3)
			max_power=350;
    else
			max_power=(voltage-WARNING_VOLTAGE)/3.0f*200;
	VAL_LIMIT(max_power,0,500);
   return max_power;

}

20210329
删除bsp.c中重复代码
优化初始化时间至3.85s

20210403
删除关弹仓反身操作（漏弹）

20210403
新设性能体系 按冷却速度给定拨盘转速 

20210404
裁判系统云台断电底盘电机给定置0 防疯车
将陀螺仪SPI.c文件时钟分频改为32（原16） 改善车抖
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//84/4 21Mhz
	
2021.04.07(cc)	
键盘：
 Bit0-----W
 Bit1-----S
 Bit2-----A
 Bit3-----D
 Bit4-----Shift
 Bit5-----Ctrl
 Bit6-----Q
 Bit7-----E
 Bit8-----R
 Bit9-----F
 Bit10-----G
 Bit11-----Z
 Bit12-----X
 Bit13-----C
 Bit14-----V
 Bit15-----B

加距离信息处理
加自瞄补偿
未测补偿，未调预测
如果判断yaw轴进行pitch补偿可能会点头，因此如果参数不好可以加一个延时的判断

2021.04.08(cc)
近距离下自瞄补偿（打哨兵）参数调整
30射速下跟随ok
低射速下自瞄方案需要重新设定
20210415（wyw）

20210501\（wyw）
增加新版性能体系System_performance；
增加中速、高速射频档（KEY_Q）
添加protobuf进行与视觉通训的协议自动生成

20210521（lzt）
存在的问题：
	1、Auto_Shoot_Control_Task.c   yaw轴和pitch赋值是反的，需要和视觉确认一下，最好是视觉修改，坐标的方向也需要根据陀螺仪
	方向重新确认一下
		有关new_location.flag需要和视觉确认一下
	2、RemoteTask.c    103行修改了测试大符时的拨杆状态，记得修改回来
	3、Gimbal_task.c   修改了能量机关
	4、整理PID参数


20220724（lsk）
存在问题：
    普通步兵面临解散，改上平衡步兵。




