#include "timer.h"

#if EN_TIM8
void TIM8_Configuration(void)
{
		TIM_TimeBaseInitTypeDef tim;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);    
		tim.TIM_Period = 0xFFFFFFFF;     
		tim.TIM_Prescaler = 168-1;	 //1M 的时钟  
		tim.TIM_ClockDivision = TIM_CKD_DIV1;	
		tim.TIM_CounterMode = TIM_CounterMode_Up;  
		TIM_ARRPreloadConfig(TIM8, ENABLE);	
		TIM_TimeBaseInit(TIM8, &tim);
		TIM_ARRPreloadConfig(TIM8, ENABLE);	
		TIM_PrescalerConfig(TIM8, 0, TIM_PSCReloadMode_Update);
		TIM_UpdateDisableConfig(TIM8, ENABLE);
		TIM_Cmd(TIM8,ENABLE);	   
}
#endif

#if EN_TIM6
void TIM6_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = 84-1;        //84M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = 0;//TIM_CKD_DIV1;
    tim.TIM_Period = 1000-1;  //1ms,1000Hz
    TIM_TimeBaseInit(TIM6,&tim);
	
    TIM_Cmd(TIM6, ENABLE);	 
    TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);	
}
#endif

#if EN_TIM6_IRQ
void TIM6_DAC_IRQHandler(void)  
{	
    if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET) 
	  {
				TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
				TIM_ClearFlag(TIM6, TIM_FLAG_Update);
				TIM6_IRQProcess;
		}
}
#endif


void TIM4_Configuration(void)
{
    TIM_TimeBaseInitTypeDef tim;
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE); 

	  NVIC_InitTypeDef         nvic;	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
    nvic.NVIC_IRQChannel = TIM4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
	
    tim.TIM_Period = 0xFFFFFFFF; //ARR的值 寄存器周期   
    tim.TIM_Prescaler = 168-1;	 //1M 的时钟  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;	
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_ARRPreloadConfig(TIM4, ENABLE);	//允·许或禁止定时器工作时向ARR缓冲器中写入新值，在更新发生时载入覆盖以前的值
    TIM_TimeBaseInit(TIM4, &tim);
//    TIM_ARRPreloadConfig(TIM4, ENABLE);	
//    TIM_PrescalerConfig(TIM4, 0, TIM_PSCReloadMode_Update);  //设置预分频
//    TIM_UpdateDisableConfig(TIM4, ENABLE);  //？？？？？？？？？？？用了中断不了
    TIM_Cmd(TIM4,ENABLE);	
		
		TIM_ITConfig(TIM4, TIM_IT_Update,ENABLE);
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
}

//TIME4
void TIM4_IRQHandler(void)
{
	  if (TIM_GetITStatus(TIM4,TIM_IT_Update)!= RESET) 
		{
				TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
				TIM_ClearFlag(TIM4, TIM_FLAG_Update);
				TIM4_IRQProcess;
		}
}

void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef tim;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    tim.TIM_Period = 0xFFFFFFFF;
    tim.TIM_Prescaler = 84 - 1;	 //1M 的时钟  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;	
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_ARRPreloadConfig(TIM2, ENABLE);	
    TIM_TimeBaseInit(TIM2, &tim);
    TIM_Cmd(TIM2,ENABLE);	
}

//TIME2
void TIM2_IRQHandler(void)
{
	  if (TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET) 
		{
				TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
				TIM_ClearFlag(TIM2, TIM_FLAG_Update);
		}
}

