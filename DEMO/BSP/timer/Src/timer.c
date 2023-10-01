#include "timer.h"

/***********************************************
		Advanced-control timers (TIM1 and TIM8)
************************************************/
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

#if EN_TIM1
void TIM1_Configuration(void)
{
		TIM_TimeBaseInitTypeDef tim;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);    
		tim.TIM_Period = 0xFFFFFFFF;     
		tim.TIM_Prescaler = 168-1;	 //1M 的时钟  
		tim.TIM_ClockDivision = TIM_CKD_DIV1;	
		tim.TIM_CounterMode = TIM_CounterMode_Up;  
		TIM_ARRPreloadConfig(TIM8, ENABLE);	
		TIM_TimeBaseInit(TIM8, &tim);
		TIM_ARRPreloadConfig(TIM1, ENABLE);	
		TIM_PrescalerConfig(TIM1, 0, TIM_PSCReloadMode_Update);
		TIM_UpdateDisableConfig(TIM1, ENABLE);
		TIM_Cmd(TIM1,ENABLE);	   
}
#endif

/**********Please write the IRQHandler under the here.**********/
#if EN_TIM1_IRQ

#endif

#if EN_TIM8_IRQ

#endif

/**************************************************
		General-purpose timers (TIMx)
***************************************************/
#if EN_TIM2
void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef tim;
	  NVIC_InitTypeDef nvic;
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
			
    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
    tim.TIM_Period = 0xFFFFFFFF;
    tim.TIM_Prescaler = 84 - 1;	 //1M 的时钟  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;	
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_ARRPreloadConfig(TIM2, ENABLE);	
    TIM_TimeBaseInit(TIM2, &tim);
    TIM_Cmd(TIM2,ENABLE);	
}
#endif

#if EN_TIM3
void TIM3_Configuration(void)
{
    TIM_TimeBaseInitTypeDef tim;
	  NVIC_InitTypeDef nvic;
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
    nvic.NVIC_IRQChannel = TIM3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
	
    tim.TIM_Period = 0xFFFFFFFF; //ARR的值 寄存器周期   
    tim.TIM_Prescaler = 168-1;	 //1M 的时钟  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_ARRPreloadConfig(TIM3, ENABLE);	//允许或禁止定时器工作时向ARR缓冲器中写入新值，在更新发生时载入覆盖以前的值
    TIM_TimeBaseInit(TIM4, &tim);
    TIM_Cmd(TIM3,ENABLE);	
		
		TIM_ITConfig(TIM3, TIM_IT_Update,ENABLE);
		TIM_ClearFlag(TIM3, TIM_FLAG_Update);
}
#endif

#if EN_TIM4
void TIM4_Configuration(void)
{
    TIM_TimeBaseInitTypeDef tim;
	  NVIC_InitTypeDef nvic;
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
    TIM_ARRPreloadConfig(TIM4, ENABLE);	//允许或禁止定时器工作时向ARR缓冲器中写入新值，在更新发生时载入覆盖以前的值
    TIM_TimeBaseInit(TIM4, &tim);
    TIM_Cmd(TIM4,ENABLE);	
		
		TIM_ITConfig(TIM4, TIM_IT_Update,ENABLE);
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
}
#endif

#if EN_TIM5
void TIM5_Configuration(void)
{
    TIM_TimeBaseInitTypeDef tim;
	  NVIC_InitTypeDef nvic;
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
    nvic.NVIC_IRQChannel = TIM5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
	
    tim.TIM_Period = 0xFFFFFFFF; //ARR的值 寄存器周期   
    tim.TIM_Prescaler = 168-1;	 //1M 的时钟  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_ARRPreloadConfig(TIM5, ENABLE);	//允许或禁止定时器工作时向ARR缓冲器中写入新值，在更新发生时载入覆盖以前的值
    TIM_TimeBaseInit(TIM5, &tim);
    TIM_Cmd(TIM5,ENABLE);	
		
		TIM_ITConfig(TIM5, TIM_IT_Update,ENABLE);
		TIM_ClearFlag(TIM5, TIM_FLAG_Update);
}
#endif

/**********Please write the IRQHandler under the here.**********/
#if EN_TIM2_IRQ
void TIM4_IRQHandler(void)
{
	  if (TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET) 
		{
				TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
				TIM_ClearFlag(TIM2, TIM_FLAG_Update);
				TIM2_IRQProcess;
		}
}
#endif

#if EN_TIM3_IRQ
void TIM3_IRQHandler(void)
{
	  if (TIM_GetITStatus(TIM3,TIM_IT_Update)!= RESET) 
		{
				TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
				TIM_ClearFlag(TIM3, TIM_FLAG_Update);
				TIM3IRQProcess;
		}
}
#endif

#if EN_TIM4_IRQ
void TIM4_IRQHandler(void)
{
	  if (TIM_GetITStatus(TIM4,TIM_IT_Update)!= RESET) 
		{
				TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
				TIM_ClearFlag(TIM4, TIM_FLAG_Update);
				TIM4_IRQProcess;
		}
}
#endif

#if EN_TIM5_IRQ
void TIM5_IRQHandler(void)
{
	  if (TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET) 
		{
				TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
				TIM_ClearFlag(TIM5, TIM_FLAG_Update);
				TIM5_IRQProcess;
		}
}
#endif

/***************************************************
		Basic timers (TIM6 and TIM7)
****************************************************/
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

#if EN_TIM7
void TIM7_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    nvic.NVIC_IRQChannel = TIM7_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = 84-1;        //84M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = 0;//TIM_CKD_DIV1;
    tim.TIM_Period = 1000-1;  //1ms,1000Hz
    TIM_TimeBaseInit(TIM7,&tim);
	
    TIM_Cmd(TIM7, ENABLE);	 
    TIM_ITConfig(TIM7, TIM_IT_Update,ENABLE);
    TIM_ClearFlag(TIM7, TIM_FLAG_Update);	
}
#endif

/**********Please write the IRQHandler under the here.**********/
#if EN_TIM6_IRQ
int a=0;
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

#if EN_TIM7_IRQ
void TIM7_IRQHandler(void)  
{	
    if (TIM_GetITStatus(TIM7,TIM_IT_Update)!= RESET) 
	  {
				TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
				TIM_ClearFlag(TIM7, TIM_FLAG_Update);
				TIM7_IRQProcess;
		}
}
#endif
