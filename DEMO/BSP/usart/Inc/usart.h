#ifndef __USART_H__
#define __USART_H__
#include "public.h"


#define EN_USART1 								1
//#define EN_USART1_DMA_SECOND_FIFO 1


#define EN_USART2       					1
#define EN_UART2_DMA_SECOND_FIFO  0


#define EN_USART3									1
#define EN_USART3_DMA_SECOND_FIFO 0


#define EN_UART4									1
#define EN_UART4_DMA_SECOND_FIFO  0


#define EN_UART5									1
#define EN_UART5_DMA_SECOND_FIFO  0


#define EN_USART6									1

/*
*********************************************************************************************************
*                                    中断调用函数接口
*********************************************************************************************************
*/
#define USART1_Data_Receive_Process_0				do{RemoteDataPrcess(_USART1_DMA_RX_BUF[0]);}while(0);
																										
#define USART1_Data_Receive_Process_1				do{RemoteDataPrcess(_USART1_DMA_RX_BUF[1]);}while(0);

#define USART2_Data_Receive_Process					do{}while(0);
#define USART3_Data_Receive_Process					do{}while(0);
#define UART4_Data_Receive_Process					do{}while(0);
#define UART5_Data_Receive_Process					do{}while(0);
#define USART6_Data_Receive_Process					do{}while(0);


/*
*********************************************************************************************************
*                                               MACROS
*********************************************************************************************************
*/

#define  BSP_USART1_DMA_RX_BUF_LEN               64u                   
#define BSP_USART1_RX_BUF_SIZE_IN_FRAMES         (BSP_USART1_RX_BUF_SIZE / RC_FRAME_LENGTH)
#define BSP_USART2_DMA_RX_BUF_LEN 100
#define CH100_RX_BUFF_SIZE 100
#define UART4_RX_BUF_LENGTH   100
#define UART4_TX_BUF_LENGTH	100
#define BSP_USART6_DMA_RX_BUF_LEN 100
#define  RC_FRAME_LENGTH                            18u
#define PITCH_MAX 35.0f
#define PITCH_MIN -25.0f
#define YAW_MAX 40				//cyq:云台角度的范围
#define YAW_MIN -40
/*
*********************************************************************************************************
*                                             FUNCTION PROTOTYPES
*********************************************************************************************************
*/





 /* Definition for USART_CH100 resources ******************************************/
  #define USART_CH100                           USART3
  #define USART_CH100_CLK                       RCC_APB1Periph_USART3
  #define USART_CH100_CLK_INIT                  RCC_APB1PeriphClockCmd
  #define USART_CH100_IRQn                      USART3_IRQn
  #define USART_CH100_IRQHandler                USART3_IRQHandler

  #define USART_CH100_TX_PIN                    GPIO_Pin_10                
  #define USART_CH100_TX_GPIO_PORT              GPIOB                      
  #define USART_CH100_TX_GPIO_CLK               RCC_AHB1Periph_GPIOB
  #define USART_CH100_TX_SOURCE                 GPIO_PinSource10
  #define USART_CH100_TX_AF                     GPIO_AF_USART3

  #define USART_CH100_RX_PIN                    GPIO_Pin_11              
  #define USART_CH100_RX_GPIO_PORT              GPIOB                   
  #define USART_CH100_RX_GPIO_CLK               RCC_AHB1Periph_GPIOB
  #define USART_CH100_RX_SOURCE                 GPIO_PinSource11
  #define USART_CH100_RX_AF                     GPIO_AF_USART3

  /* Definition for DMAx resources ********************************************/
  #define USART_CH100_DR_ADDRESS                ((uint32_t)USART3 + 0x04) 

  #define USART_CH100_DMA                       DMA1
  #define USART_CH100_DMAx_CLK                  RCC_AHB1Periph_DMA1
     
  #define USART_CH100_TX_DMA_CHANNEL            DMA_Channel_4
  #define USART_CH100_TX_DMA_STREAM             DMA1_Stream3
  #define USART_CH100_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF3
  #define USART_CH100_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF3
  #define USART_CH100_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF3
  #define USART_CH100_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF3
  #define USART_CH100_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF3
              
  #define USART_CH100_RX_DMA_CHANNEL            DMA_Channel_4
  #define USART_CH100_RX_DMA_STREAM             DMA1_Stream1
  #define USART_CH100_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF1
  #define USART_CH100_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF1
  #define USART_CH100_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF1
  #define USART_CH100_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF1
  #define USART_CH100_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF1

//  #define USART_CH100_DMA_TX_IRQn               DMA2_Stream6_IRQn
//  #define USART_CH100_DMA_RX_IRQn               DMA2_Stream1_IRQn
//  #define USART_CH100_DMA_TX_IRQHandler         DMA2_Stream6_IRQHandler
//  #define USART_CH100_DMA_RX_IRQHandler         DMA2_Stream1_IRQHandler

void usart1_init(uint32_t baud_rate);
void usart2_init(u32 bound);
void usart3_init(u32 bound);
void uart4_init (u32 bound);
void uart5_init (u32 bound);
void usart6_init(u32 bound);
void RemoteDataPrcess(uint8_t *pData);
void Uart4DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
void Uart4SendByteInfoProc(u8 nSendInfo);
void Uart4SendBytesInfoProc(u8* pSendInfo, u16 nSendCount);

#endif
