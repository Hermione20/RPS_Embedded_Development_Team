/* Includes ------------------------------------------------------------------*/
#include "can.h"
/**
  ******************************************************************************
  * @file    can.c
  * @author  TC
  * @version V1.0.0
  * @date    07-September-2023
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Controller area network (CAN) peripheral:
  *           + Initialization and Configuration 
  *           + CAN Frames Transmission
  *           + CAN Frames Reception
  *           + Operation modes switch
  *           + Error management
  *           + Interrupts and flags
  *
@verbatim
 ===============================================================================
                        ##### How to use #####
 ===============================================================================
    [..]
      (#) Enable the CAN controller interface clock using 
          RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); for CAN1 
          and RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE); for CAN2
      -@- 如果您只使用CAN2，则必须启用CAN1时钟
       
      (#) CAN pins configuration
        (++) 启用CAN gpio时钟，使用如下功能:
							RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOx,ENABLE); 
        (++) 使用以下功能将涉及的CAN引脚连接到AF9
							GPIO_PinAFConfig(GPIOx, GPIO_PinSourcex, GPIO_AF_CANx);
        (++) 通过调用将这些CAN引脚配置为备用功能模式
							函数GPIO_Init();
							使用CAN_Init()和初始化和配置CAN
							CAN_FilterInit()函数。
			(#)使用CAN_Transmit()函数发送所需的CAN帧。

			(#)使用CAN_TransmitStatus()检查CAN帧的传输函数。

			(#)使用CAN_CancelTransmit()取消CAN帧的传输函数。
				 使用can_receive()函数接收一个CAN帧。

		  (#)使用CAN_FIFORelease()函数释放接收fifo。

			(#)返回等待接收帧的个数
					CAN_MessagePending()函数。
											 
      (#) 要控制CAN事件，可以使用以下两种方法之一:
        (++) 使用CAN_GetFlagStatus()函数检查CAN标志。 
        (++) 通过初始化阶段的CAN_ITConfig()函数和中断例程中的CAN_GetITStatus()函数使用CAN中断来检查事件是否发生。
             检查一个标志后，你应该使用CAN_ClearFlag()函数清除它。在检查中断事件后，您应该使用CAN_ClearITPendingBit()函数清除它。  

@endverbatim
           
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************  
  */
	
 
/* Variables_definination-----------------------------------------------------------------------------------------------*/
u8 CAN1_receive_buf[20];
u8 CAN2_receive_buf[20];

int16_t  pitch_ecd_bias =6000;
int16_t  yaw_ecd_bias  = 5000;

/*----------------------------------------------------------------------------------------------------------------------*/

#if EN_CAN1
/*----CAN1_TX-----PA12----*/
/*----CAN1_RX-----PA11----*/
void CAN1_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{   
		GPIO_InitTypeDef       gpio;
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

    gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &gpio);
    
    nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
//    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 1;
//    nvic.NVIC_IRQChannelSubPriority = 0;
//    nvic.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic);    
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
    
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = ENABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = DISABLE;
    can.CAN_Mode = mode;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = tbs1;
    can.CAN_BS2 = tbs2;
    can.CAN_Prescaler = brp;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN1, &can);

		can_filter.CAN_FilterNumber=0;
		can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
		can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
		can_filter.CAN_FilterIdHigh=0x0000;
		can_filter.CAN_FilterIdLow=0x0000;
		can_filter.CAN_FilterMaskIdHigh=0x0000;
		can_filter.CAN_FilterMaskIdLow=0x0000;
		can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
		can_filter.CAN_FilterActivation=ENABLE;
		CAN_FilterInit(&can_filter);
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
//    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
}

void CAN1_TX_IRQHandler(void) //CAN TX
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
		{
				CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
		}
}

/*************************************************************************
                          CAN1_RX0_IRQHandler
描述：CAN1的接收中断函数
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{   
		CanRxMsg rx_message;	
		if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
		{
				CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
				CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 
				CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
				CAN1_Data_Receive_Process
		}
}
#endif

#if EN_CAN2
/*----CAN2_TX-----PB13----*/
/*----CAN2_RX-----PB12----*/

void CAN2_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &gpio);

    nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    CAN_DeInit(CAN2);
    CAN_StructInit(&can);

    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = ENABLE;    
    can.CAN_AWUM = DISABLE;    
    can.CAN_NART = DISABLE;    
    can.CAN_RFLM = DISABLE;    
    can.CAN_TXFP = DISABLE;     
    can.CAN_Mode = CAN_Mode_Normal; 
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = tbs1;
    can.CAN_BS2 = tbs2;
    can.CAN_Prescaler = brp;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN2, &can);
    
    can_filter.CAN_FilterNumber=14;
    can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh=0x0000;
    can_filter.CAN_FilterIdLow=0x0000;
    can_filter.CAN_FilterMaskIdHigh=0x0000;
    can_filter.CAN_FilterMaskIdLow=0x0000;
    can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
    can_filter.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
//    CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
}

void CAN2_TX_IRQHandler(void) //CAN TX
{
  if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET)    //if transmit mailbox is empty 
  {
	   CAN_ClearITPendingBit(CAN2,CAN_IT_TME);   
  }
//	CAN_ClearITPendingBit(CAN2, CAN_IT_EWG|CAN_IT_EPV|CAN_IT_BOF|CAN_IT_LEC|CAN_IT_ERR);

}

void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);//|CAN_IT_FF0|CAN_IT_FOV0
        CAN_Receive(CAN2, CAN_FIFO0, &rx_message);  
       //电机编码器数据处理
       
				CAN2_Data_Receive_Process
//			CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
    }
		
//		CAN_ClearITPendingBit(CAN2, CAN_IT_EWG|CAN_IT_EPV|CAN_IT_BOF|CAN_IT_LEC|CAN_IT_ERR);
}
#endif

