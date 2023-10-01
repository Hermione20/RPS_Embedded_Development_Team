#include "main.h"
#include "FIFO.h"
#include "protocal.h"

/*-----UART4_TX-----PC10-----*/
/*-----UART4_RX-----PC11-----*/


uint8_t UART4_DMA_RX_BUF[UART4_RX_BUF_LENGTH];
uint8_t UART4_DMA_TX_BUF[UART4_TX_BUF_LENGTH];

FIFO_S_t* UART_TranFifo1;
void UART4_Configuration(void)
{
  USART_InitTypeDef uart4;
  GPIO_InitTypeDef  gpio;
  NVIC_InitTypeDef  nvic;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_Speed = GPIO_Speed_100MHz;
  gpio.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC,&gpio);

  uart4.USART_BaudRate = 115200;          // speed 10byte/ms
  uart4.USART_WordLength = USART_WordLength_8b;
  uart4.USART_StopBits = USART_StopBits_1;
  uart4.USART_Parity = USART_Parity_No;
  uart4.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
  uart4.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(UART4,&uart4);

  USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);


  DMA_InitTypeDef dma;
  DMA_DeInit(DMA1_Stream2);
  DMA_StructInit(&dma);
  dma.DMA_Channel = DMA_Channel_4;
  dma.DMA_PeripheralBaseAddr		= (uint32_t)(&UART4->DR);
  dma.DMA_Memory0BaseAddr   		= (uint32_t)&UART4_DMA_RX_BUF;
  dma.DMA_DIR 					= DMA_DIR_PeripheralToMemory;
  dma.DMA_BufferSize				= UART4_RX_BUF_LENGTH;//sizeof(USART1_DMA_RX_BUF);
  dma.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
  dma.DMA_MemoryInc 				= DMA_MemoryInc_Enable;
  dma.DMA_PeripheralDataSize 		= DMA_PeripheralDataSize_Byte;
  dma.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;
  dma.DMA_Mode 					= DMA_Mode_Normal;
  dma.DMA_Priority 				= DMA_Priority_Medium;
  dma.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
  dma.DMA_FIFOThreshold 			= DMA_FIFOThreshold_1QuarterFull;
  dma.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;
  dma.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream2, &dma);
  DMA_Cmd(DMA1_Stream2, ENABLE);

  nvic.NVIC_IRQChannel = UART4_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 3;
  nvic.NVIC_IRQChannelSubPriority =3;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

  USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);

  DMA_Cmd(DMA1_Stream4, DISABLE);                           // 关DMA通道
  DMA_DeInit(DMA1_Stream4);
  while(DMA_GetCmdStatus(DMA1_Stream4) != DISABLE) {}
  dma.DMA_Channel = DMA_Channel_4;
  dma.DMA_PeripheralBaseAddr	= (uint32_t)(&UART4->DR);
  dma.DMA_Memory0BaseAddr   	= (uint32_t)&UART4_DMA_TX_BUF[0];
  dma.DMA_DIR 			    = DMA_DIR_MemoryToPeripheral;
  dma.DMA_BufferSize			= 0;//sizeof(USART1_DMA_TX_BUF);
  dma.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
  dma.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
  dma.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
  dma.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
  dma.DMA_Mode 				= DMA_Mode_Normal;
  dma.DMA_Priority 			= DMA_Priority_Medium;
  dma.DMA_FIFOMode 			= DMA_FIFOMode_Disable;
  dma.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;
  dma.DMA_MemoryBurst 		= DMA_MemoryBurst_Single;
  dma.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream4,&dma);

//	DMA_Cmd(DMA1_Stream3, DISABLE);                           // 关DMA通道
  nvic.NVIC_IRQChannel = DMA1_Stream4_IRQn;   // 发送DMA通道的中断配置
  nvic.NVIC_IRQChannelPreemptionPriority = 3;     // 优先级设置
  nvic.NVIC_IRQChannelSubPriority = 3;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
  DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);


  USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);
  USART_Cmd(UART4,ENABLE);



}

void UART4_IRQHandler(void)
{
  uint8_t length=0;
  if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)    //接收中断
    {
      (void)UART4->SR;
      (void)UART4->DR;
      DMA_Cmd(DMA1_Stream2, DISABLE);
      DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
      length = UART4_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA1_Stream2);

      targetOffsetDataDeal(length, UART4_DMA_RX_BUF );
      DMA_SetCurrDataCounter(DMA1_Stream2,UART4_RX_BUF_LENGTH);
      DMA_Cmd(DMA1_Stream2, ENABLE);

    }
}


void Uart4DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)

{
//    DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输
//    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}  //确保DMA可以被设置
  DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量
  DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输
}



void DMA1_Stream4_IRQHandler(void)
{
  //清除标志
  if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)//等待DMA1_Steam3传输完成
    {
      DMA_Cmd(DMA1_Stream4, DISABLE);                      //关闭DMA传输
      DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);//清除DMA1_Steam3传输完成标志
    }

}


//发送单字节
void Uart4SendByteInfoProc(u8 nSendInfo)
{
  u8 *pBuf = NULL;
  //指向发送缓冲区
  pBuf = UART4_DMA_TX_BUF;
  *pBuf++ = nSendInfo;

  Uart4DmaSendDataProc(DMA1_Stream4,1); //开始一次DMA传输！

}

//发送多字节

void Uart4SendBytesInfoProc(u8* pSendInfo, u16 nSendCount)
{
  u16 i = 0;
  u8 *pBuf = NULL;
  //指向发送缓冲区
  pBuf = UART4_DMA_TX_BUF;
  for (i=0; i<nSendCount; i++)
    {
      *(pBuf+i) = pSendInfo[i];
    }

  //DMA发送方式

  Uart4DmaSendDataProc(DMA1_Stream4,nSendCount); //开始一次DMA传输！


}
