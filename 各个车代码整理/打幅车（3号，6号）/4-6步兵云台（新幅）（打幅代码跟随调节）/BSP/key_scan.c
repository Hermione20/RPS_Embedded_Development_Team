/*
 * @file       key_scan.c
 * @brief      按键扫描函数实现（状态机）
 * @author     李宜君
 * @version    v1.0
 * @date       2017-10-10
 */

#include "main.h"


void key_init(void)
{
    GPIO_InitTypeDef gpio;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_1 ;
	gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio);
}





    key_states_e key_read()
{
	key_states_e key_states;
	/*****再此处添加读取按键的程序****/
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) ==1 )
	{
		key_states = KEY_UP;
	}
	else
	{
		key_states = KEY_DOWN;
	}



	/*****添加结束****/
	return key_states;
}




key_msg_e key_scan(void)
{
		static key_msg_e  key_now_msg = KEY_S1;     //这次按键的状态
		static key_msg_e  key_last_msg = KEY_S1;     //上次按键的状态
		key_msg_e key_return_msg = KEY_S1;//函数返回
		static unsigned int  keytime = 0;
    switch (key_last_msg)
    {
      case KEY_S1:                 // 按键初始态
			if (KEY_DOWN == key_read()) //按键被按下，等待确认
			{
				key_now_msg = KEY_S2;
			}

	    break;

        case KEY_S2:                   // 按键确认态
				if (KEY_DOWN == key_read())
				{
						keytime = keytime + SCAN_TIME;
						if(keytime >= DOWN_TIME)
						{
						    key_now_msg = KEY_S3;// 进入短按状态
						}
						else
						{
							key_now_msg = KEY_S2;// 维持当前状态
						}

        }
				else if(KEY_UP == key_read())//如果按键抬起，说明误触发
				{
						key_now_msg = KEY_S1; // 按键已抬起，转换到按键初始态
						keytime = 0;
				}
	  	break;

        case KEY_S3://短按状态
            if (KEY_DOWN == key_read())
            {
				keytime = keytime + SCAN_TIME;
				if(keytime >= HOLD_TIME)
				{
					  key_now_msg = KEY_S4;// 进入长按状态
				}
				else
				{
					key_now_msg = KEY_S3;// 维持当前状态
				}

            }
            else if(KEY_UP == key_read())//如果按键抬起，说明短按一次完成
            {
				key_now_msg = KEY_S1; // 按键已抬起，转换到按键初始态
				keytime = 0;
			}
			break;
		case KEY_S4://长按状态
			if (KEY_DOWN == key_read())
			{
                key_now_msg = KEY_S4;
				key_return_msg = KEY_S4;
			}
			else if(KEY_UP == key_read())//如果按键抬起，说明短按一次完成
			{
				key_now_msg = KEY_S1; // 按键已抬起，转换到按键初始态
				keytime = 0;
			}
	break;
   }//end switch
		
	 
		if(key_now_msg == KEY_S1)
		{
				if(key_last_msg == KEY_S3)
				key_return_msg = KEY_S3;//短按松开后才返回短按信息
		}
		key_last_msg = key_now_msg;
		return key_return_msg;                            //返回按键值
}

