#include "pwm.h"

void PWM_Configuration(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC ,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  
	                        
	  gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;       
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
	  gpio.GPIO_OType = GPIO_OType_PP;
	  gpio.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOC,&gpio);
	
	  GPIO_PinAFConfig(GPIOC,GPIO_PinSource8, GPIO_AF_TIM3);  //TIM3->CCR1 ????PWM4
	  GPIO_PinAFConfig(GPIOC,GPIO_PinSource9, GPIO_AF_TIM3);  //TIM3->CCR2 ????PWM5
	
	  tim.TIM_Prescaler = 1000-1;    //psc
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 1680-1;   //arr
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
		
		PWM2=85;
}

int pwm_flag=0;
u32 pwm_mode=PWM_STOP;
void PWM(void)
{
	switch(pwm_mode)
	{
		case PWM_STOP:
			pwm_flag=0;
			break;
		case PWM_UP:
			pwm_flag=1;
			break;
		case PWM_DOWN:
			pwm_flag=2;
			break;
		default:
			pwm_mode=PWM_STOP;
			break;
	}
	if(pwm_flag==1)
	{		
			PWM2=PWM_MAX;//PWM2+=3;
	}
	else if(pwm_flag==2)
			PWM2=PWM_MIN;//PWM2-=3;
	if((PWM2>=PWM_MAX)||(PWM2<=PWM_MIN))
	{
			pwm_flag=0;
			pwm_mode=PWM_STOP;
	}
	VAL_LIMIT(PWM2,  PWM_MIN,  + PWM_MAX);
}



