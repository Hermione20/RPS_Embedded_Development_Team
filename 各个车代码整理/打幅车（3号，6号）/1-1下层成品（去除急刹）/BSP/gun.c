/*-LEFT---(PB3---TIM2_CH2)--*/
/*-RIGHT--(PA15--TIM2_CH1)--*/

#include "stm32f4xx.h"
#include "gun.h"
int door_flag=0;
//???????
/*-LEFT---(PC6---TIM3_CH1)--TIM3->CCR1 ???? PWM4*/
/*-RIGHT--(PC7---TIM3_CH2)--TIM3->CCR2 ???? PWM5*/
void PWM_Configuration(void)
{ GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC ,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  

	  GPIO_PinAFConfig(GPIOC,GPIO_PinSource8, GPIO_AF_TIM3);//????
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource9, GPIO_AF_TIM3);
	                        
	  gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;       
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
	  gpio.GPIO_OType = GPIO_OType_PP;
	  gpio.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOC,&gpio);
	
	  GPIO_PinAFConfig(GPIOC,GPIO_PinSource8, GPIO_AF_TIM3);  //TIM3->CCR1 ????PWM4
	  GPIO_PinAFConfig(GPIOC,GPIO_PinSource9, GPIO_AF_TIM3);  //TIM3->CCR2 ????PWM5
	
	  /* ??????TIM3 */
	  tim.TIM_Prescaler = 999;    //???
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 1679;   
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3,&tim);

	  oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 1000;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
		
    TIM_OC3Init(TIM3,&oc);
   	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);  
	  TIM_OC4Init(TIM3,&oc);
	  TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);  
		
    TIM_ARRPreloadConfig(TIM3,ENABLE);
    TIM_Cmd(TIM3,ENABLE);
		
		OpenDoor;
		door_flag=0;
}

void PWM_Configuration1(void)
{
   GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  

	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource0, GPIO_AF_TIM5);//????
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource1, GPIO_AF_TIM5);
	                        
	  gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;       
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
	  gpio.GPIO_OType = GPIO_OType_PP;
	  gpio.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOA,&gpio);
	
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource0, GPIO_AF_TIM5);  //TIM3->CCR1 ????PWM4
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource1, GPIO_AF_TIM5);  //TIM3->CCR2 ????PWM5
	
	  /* ??????TIM3 */
	  tim.TIM_Prescaler = 999;    //???
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 1679;   
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5,&tim);

	  oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 1000;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
		
    TIM_OC1Init(TIM5,&oc);
   	TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable);  
	  TIM_OC2Init(TIM5,&oc);
	  TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Enable);  
		
    TIM_ARRPreloadConfig(TIM5,ENABLE);
    TIM_Cmd(TIM5,ENABLE);
		
		OpenDoor;
		door_flag=0;
}

//    ??????
//    int dir = 1;
//    int led0pwmval = 0;
//		if(dir)led0pwmval++; 
//		else led0pwmval--;
// 		if(led0pwmval>210)dir=0;
//		if(led0pwmval==0)dir=1;