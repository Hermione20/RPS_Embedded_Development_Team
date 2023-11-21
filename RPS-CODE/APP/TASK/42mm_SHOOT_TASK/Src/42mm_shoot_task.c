#include "42mm_shoot_task.h"

/**
  ******************************************************************************
  * @file   42mm_shoot_task.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   42mm弹丸发射模块，该模块定制于英雄二级拨盘供弹双摩擦轮的
							的设计。参数设置位置位于源文件上部，注意！！！英雄发射电机
							的速度反馈选用rate_rpm非filter_rate。
	
	* @notice  该模块仅仅适用于23赛季英雄机械方案，请未来的代码维护者与开发
							人员维护模块的独立性，维护通用性，禁止将属于该部分的逻辑与代码
						 写到别的模块内，该模块仅负责发射的控制，参考输入的赋值请移步
						 模式选择。
						 
	* @notice  发射模块的调用请移步至control_task，推荐云台计算频率为2ms
						 推荐云台电机发送频率为2ms。在controltask里shoot_task
						 在control_task_Init里放置shoot_param_init
						 
	*	@introduction 本模块采用状态机的方式管理各种发射的状态，控制信号
									的输入与模式的切换与选择来自mode_switch_tasks，全模块的
									变量_42mm_shoot结构体包含，模块可自定义摩擦轮的极性，设置
									各个发射参数如
									#define RIGHT_FRICTION_POLARITY -1
									
 ===============================================================================
 **/


_42mm_shoot_t _42mm_shoot;


 /**
  ******************************************************************************
																			参数设置
	摩擦轮转速设置（弹速对应10，12，14，16）
	英雄下拨盘转速
	英雄下拨盘力度限制
	42mm单个弹丸上拨盘角度
	摩擦轮旋转方向设置
	 =============================================================================
 **/
#define FRICTION_SPEED_10 2000
#define FRICTION_SPEED_12 2100
#define FRICTION_SPEED_14 2575
#define FRICTION_SPEED_16 3200

#define POKE_SPEED -200			//英雄下拨盘转速
#define POKE_MAX_OUT 6000		//英雄下拨盘力度限制
#define ONE_POKE_ANGLE_42 90.0f	//42mm单个弹丸上拨盘角度
//摩擦轮旋转方向设置
#define RIGHT_FRICTION_POLARITY -1
#define LEFT_FRICTION_POLARITY  1



/******************** VARIABVLE ******************/
uint16_t frictionSpeed_42=0;		//42mm弹速
u8 over_heat = 0;                   //超热量标志位
int lock_cnt = 0;
int reverse_cnt = 0;
int set_cnt = 0;
u8 poke_init_flag = 0;
u8 press_l_first_in = 0;

 /**
  ******************************************************************************
																云台结构体初始化
		pid参数设置
		
	 =============================================================================
 **/
void shoot_param_init(void)
{
		//结构体内存置零
    memset(&_42mm_shoot,0,sizeof(_42mm_shoot_t));

    //42mm摩擦轮
  PID_struct_init(&_42mm_shoot.pid_right_friction_speed, POSITION_PID,10000,1000,  70 , 0.01f ,100);//两个摩擦轮
  PID_struct_init(&_42mm_shoot.pid_left_friction_speed, POSITION_PID,10000,1000,  72 , 0.01f ,10);

  //42mm拨盘
    PID_struct_init(&_42mm_shoot.pid_downpoke_speed,   POSITION_PID, 6000 , 13000,  5  , 0.5, 0 );//下拨盘速度环
	PID_struct_init(&_42mm_shoot.pid_uppoke_angle, POSITION_PID, 2000 , 0    ,  120, 5  , 10); //二级拨盘
    PID_struct_init(&_42mm_shoot.pid_uppoke_speed, POSITION_PID, 9900 , 5500 ,  20 , 0  , 0 );
}


 /**
  ******************************************************************************
																云台总控制任务		
	 =============================================================================
 **/
void shoot_task(void)
{
    Shoot_42mm_speed_Select(0);//弹速自动选择
    heat_limit_42mm(RC_CtrlData.Key_Flag.Key_Z_Flag);//热量限制
    shoot_friction_handle_42();//摩擦轮控制任务
    shoot_bullet_handle_42();//拨盘控制任务

}


 /**
  ******************************************************************************
																弹速自动选择
		括号入口参数为测试摩擦轮速度，若为0，则默认裁判系统选择，反之执行参数内的速度
	 =============================================================================
 **/
static void Shoot_42mm_speed_Select(uint16_t test_frictionSpeed_42) // 42mm弹速
{
    if (test_frictionSpeed_42 == 0)
    {
			//读裁判系统弹速上限进行选择
        if (judge_rece_mesg.game_robot_state.shooter_id1_42mm_speed_limit == 10)
            frictionSpeed_42 = FRICTION_SPEED_10;
        else if (judge_rece_mesg.game_robot_state.shooter_id1_42mm_speed_limit == 16)
            frictionSpeed_42 = FRICTION_SPEED_16;
        else
            frictionSpeed_42 = FRICTION_SPEED_10;
    }
    else
    {
        frictionSpeed_42 = test_frictionSpeed_42;
    }
}




 /**
  ******************************************************************************
																热量限制		
		括号入口参数为是否忽略热量标志，为1忽略，为0限制
	 =============================================================================
 **/
void heat_limit_42mm(u8 ifignore)
{
    float residue_heart;//剩余热量
    residue_heart=(judge_rece_mesg.game_robot_state.shooter_id1_42mm_cooling_limit           //通过裁判系统计算剩余热量
                       -judge_rece_mesg.power_heat_data.shooter_id1_42mm_cooling_heat);
    if (ifignore)
    {
        over_heat = 0;
    }
    else
    {
        if (residue_heart >= 100)
            over_heat = 0;
        else
            over_heat = 1;
    }
}


 /**
  ******************************************************************************
																摩擦轮控制任务	
	摩擦轮任务内涵堵转检测与反转处理
	 =============================================================================
 **/
void shoot_friction_handle_42(void)
{
    //摩擦轮状态判断与选择
    if (RC_CtrlData.inputmode != STOP)//非关控
	{
        if((RC_CtrlData.RemoteSwitch.s3to1 || RC_CtrlData.Key_Flag.Key_C_TFlag))//开启摩擦轮输入
        {
					 if(_42mm_shoot.friction_state!=BACK)
					 {
						 _42mm_shoot.friction_state = START;
					 }
            
            if (general_friction.right_motor.rate_rpm>(30*RIGHT_FRICTION_POLARITY)||general_friction.left_motor.rate_rpm<(30*LEFT_FRICTION_POLARITY))//判断摩擦轮转没转起来
            {
							//没转起来
                lock_cnt++;
                if(lock_cnt == 500)//如果计数至500，判断摩擦轮堵转
                {
                     _42mm_shoot.friction_state = LOCK;
                     lock_cnt = 0;//清零计数器
                }
            }else
            {		//转起来了，设为普通
                _42mm_shoot.friction_state = NORMAL;
                lock_cnt = 0;//清零计数器
            }

            if (_42mm_shoot.friction_state == LOCK)//若堵转，说明弹丸卡在摩擦轮内，组织反转
            {
                _42mm_shoot.friction_state = BACK;
            }
            if (_42mm_shoot.friction_state == BACK)
            {
                reverse_cnt++;//反转时间计数
                if (reverse_cnt == 100)//反转一定时间后恢复摩擦轮启动
                {
                    _42mm_shoot.friction_state = START;
                    reverse_cnt = 0;
                }
                
            }           
            
        }else
        {
             _42mm_shoot.friction_state = Stop;
        }
    }else
    {
        _42mm_shoot.friction_state = Stop;
    }

    //根据摩擦轮状态开始执行摩擦轮
    switch (_42mm_shoot.friction_state)
    {
    case START:
     {
        _42mm_shoot.shoot_ref_and_fdb.right_friction_speed_ref = RIGHT_FRICTION_POLARITY*frictionSpeed_42;
        _42mm_shoot.shoot_ref_and_fdb.left_friction_speed_ref = LEFT_FRICTION_POLARITY*frictionSpeed_42;
     }
        break;
    case NORMAL:
    {
        _42mm_shoot.shoot_ref_and_fdb.right_friction_speed_ref = RIGHT_FRICTION_POLARITY*frictionSpeed_42;
        _42mm_shoot.shoot_ref_and_fdb.left_friction_speed_ref = LEFT_FRICTION_POLARITY*frictionSpeed_42;
    }
         break;
    case BACK:
    {
        _42mm_shoot.shoot_ref_and_fdb.right_friction_speed_ref = -RIGHT_FRICTION_POLARITY*frictionSpeed_42;
        _42mm_shoot.shoot_ref_and_fdb.left_friction_speed_ref = -LEFT_FRICTION_POLARITY*frictionSpeed_42;
    }
				break;
    default:
    {
        _42mm_shoot.shoot_ref_and_fdb.right_friction_speed_ref = 0;
        _42mm_shoot.shoot_ref_and_fdb.left_friction_speed_ref = 0;
    }
        break;
    }
		//更新摩擦轮反馈
    _42mm_shoot.shoot_ref_and_fdb.left_friction_speed_fdb = general_friction.left_motor.rate_rpm;
    _42mm_shoot.shoot_ref_and_fdb.right_friction_speed_fdb = general_friction.right_motor.rate_rpm;
		//计算摩擦轮pid输出
    _42mm_shoot.shoot_ref_and_fdb.left_friction_motor_input = pid_calc(&_42mm_shoot.pid_left_friction_speed,_42mm_shoot.shoot_ref_and_fdb.left_friction_speed_fdb,_42mm_shoot.shoot_ref_and_fdb.left_friction_speed_ref);
    _42mm_shoot.shoot_ref_and_fdb.right_friction_motor_input = pid_calc(&_42mm_shoot.pid_right_friction_speed,_42mm_shoot.shoot_ref_and_fdb.right_friction_speed_fdb,_42mm_shoot.shoot_ref_and_fdb.right_friction_speed_ref);

}


 /**
  ******************************************************************************
																拨盘控制任务		
		控制逻辑：上波盘采用角度环拨弹，下拨盘为保持弹路饱满，采用速度环控制
							但是限制堵转电流，避免烧坏电机
	 =============================================================================
 **/
void shoot_bullet_handle_42(void)
{
		//反馈赋值
    _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_fdb = general_poke.up_poke.ecd_angle/36.0f;  //2006电机减速比1：36
    _42mm_shoot.shoot_ref_and_fdb.up_poke_speed_fdb = general_poke.up_poke.rate_rpm;

    _42mm_shoot.shoot_ref_and_fdb.down_poke_speed_fdb = general_poke.down_poke.rate_rpm;
		
	//本部分扳机逻辑为不连续按键，长按仅有一次输出信号
    if (RC_CtrlData.mouse.press_l == 1 || RC_CtrlData.RemoteSwitch.trigger == 1)//扳机扣下
    {
        if (press_l_first_in == 0)
        {
             press_l_first_in = 1;
             _42mm_shoot.shoot_flag = 1;
        }
        else
        {
             _42mm_shoot.shoot_flag = 0;
        }
    }
    else
    {
        press_l_first_in = 0;
    }
		
    if(_42mm_shoot.friction_state == NORMAL&&over_heat==0)
    {
			
			 _42mm_shoot.shoot_ref_and_fdb.down_poke_speed_ref = POKE_SPEED;
        if(poke_init_flag == 0)
        {
					//拨盘参数初始化
            _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_ref = _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_fdb;
            _42mm_shoot.pid_downpoke_speed.max_out = 6000;
            poke_init_flag = 1;
        }
        if(_42mm_shoot.shoot_flag)
        {
					//如果发射，上拨盘给定累加，积分清零
            _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_ref+=ONE_POKE_ANGLE_42;
            _42mm_shoot.pid_uppoke_angle.iout = 0;
        }
        if(_42mm_shoot.shoot_flag)
			_42mm_shoot.pid_downpoke_speed.max_out=POKE_MAX_OUT;
		else
		{
			//检测弹路是否饱满，若饱满则降低电流输出至2000
			if(_42mm_shoot.pid_downpoke_speed.out<-4200 && abs(general_poke.down_poke.rate_rpm)<20)
				set_cnt++;
			else
				set_cnt = 0;
			if(set_cnt == 20)
				_42mm_shoot.pid_downpoke_speed.max_out=2000;
		}

        _42mm_shoot.shoot_ref_and_fdb.up_poke_motor_input = pid_double_loop_cal(&_42mm_shoot.pid_uppoke_angle,
                                                                                &_42mm_shoot.pid_uppoke_speed,
                                                                                _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_ref,
                                                                                _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_fdb,
                                                                                &_42mm_shoot.shoot_ref_and_fdb.up_poke_speed_ref,
                                                                                _42mm_shoot.shoot_ref_and_fdb.up_poke_speed_fdb,
                                                                                0);
        _42mm_shoot.shoot_ref_and_fdb.down_poke_motor_input = pid_calc(&_42mm_shoot.pid_downpoke_speed,_42mm_shoot.shoot_ref_and_fdb.down_poke_speed_fdb,_42mm_shoot.shoot_ref_and_fdb.down_poke_speed_ref);
    }else if(_42mm_shoot.friction_state == BACK)
		{
			_42mm_shoot.shoot_ref_and_fdb.up_poke_angle_ref-=0.5f;
			_42mm_shoot.shoot_ref_and_fdb.up_poke_motor_input = pid_double_loop_cal(&_42mm_shoot.pid_uppoke_angle,
                                                                                &_42mm_shoot.pid_uppoke_speed,
                                                                                _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_ref,
                                                                                _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_fdb,
                                                                                &_42mm_shoot.shoot_ref_and_fdb.up_poke_speed_ref,
                                                                                _42mm_shoot.shoot_ref_and_fdb.up_poke_speed_fdb,
                                                                                0);
		}else
    {
        poke_init_flag = 0;
        _42mm_shoot.shoot_ref_and_fdb.up_poke_motor_input = 0;
         _42mm_shoot.shoot_ref_and_fdb.down_poke_motor_input = 0;
    }
}

