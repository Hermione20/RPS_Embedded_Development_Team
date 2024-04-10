#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "public.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


//LED端口定义
#define LED0_ON  GPIO_ResetBits(GPIOC,GPIO_Pin_1)
#define LED1_ON  GPIO_ResetBits(GPIOC,GPIO_Pin_2)
#define LED0_OFF GPIO_SetBits(GPIOC,GPIO_Pin_1)
#define LED1_OFF GPIO_SetBits(GPIOC,GPIO_Pin_2)	 
#define HARD_WDG_ON  GPIO_SetBits(GPIOC,GPIO_Pin_0)
#define HARD_WDG_OFF  GPIO_ResetBits(GPIOC,GPIO_Pin_0)	 

#define LASER_ON()   GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define LASER_OFF()  GPIO_ResetBits(GPIOA, GPIO_Pin_8)
void LED_Init(void);//初始化		 				    
#endif
