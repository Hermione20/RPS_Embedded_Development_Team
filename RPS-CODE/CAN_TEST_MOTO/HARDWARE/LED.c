#include "stm32f4xx.h"                  // Device header
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "can.h"

void LED_Init(void)
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
 GPIO_InitStructure.GPIO_OType =GPIO_OType_PP;
 GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_UP ;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

 GPIO_Init(GPIOC, &GPIO_InitStructure);

}

