#ifndef __TIMER_H__
#define __TIMER_H__

#include "public.h"

/***********************************************
		Advanced-control timers (TIM1 and TIM8)
************************************************/
#define EN_TIM1									0
#define	EN_TIM1_IRQ							0

#define EN_TIM8									0
#define	EN_TIM8_IRQ							0

/**************************************************
		General-purpose timers (TIMx)
***************************************************/
#define EN_TIM2									0
#define	EN_TIM2_IRQ							0

#define EN_TIM3									0
#define	EN_TIM3_IRQ							0

#define EN_TIM4									0
#define	EN_TIM4_IRQ							0
	
#define EN_TIM5									0
#define	EN_TIM5_IRQ							0

/***************************************************
		Basic timers (TIM6 and TIM7)
****************************************************/

#define EN_TIM6									1
#define	EN_TIM6_IRQ							1

#define EN_TIM7									0
#define	EN_TIM7_IRQ							0


/*****************************************************
		IRQProcess Set
*****************************************************/
#define TIM1_IRQProcess 				do{}while(0);
#define TIM2_IRQProcess 				do{}while(0);
#define TIM3_IRQProcess 				do{}while(0);
#define TIM4_IRQProcess 				do{}while(0);
#define TIM5_IRQProcess 				do{}while(0);
#define TIM6_IRQProcess 				do{}while(0);//Control_Task
#define TIM7_IRQProcess 				do{}while(0);
#define TIM8_IRQProcess 				do{}while(0);

void TIM1_Configuration(void);
void TIM2_Configuration(void);
void TIM3_Configuration(void);
void TIM4_Configuration(void);
void TIM5_Configuration(void);
void TIM6_Configuration(void);
void TIM7_Configuration(void);
void TIM8_Configuration(void);

void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM5_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void TIM7_IRQHandler(void);

#endif
