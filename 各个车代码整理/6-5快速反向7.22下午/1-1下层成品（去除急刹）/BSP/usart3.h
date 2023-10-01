#ifndef __USART3_H
#define __USART3_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"

#define USART3_RX_BUF_LENGTH 100
#define USART3_TX_BUF_LENGTH 100

extern u8 USART3_DMA_RX_BUF[USART3_RX_BUF_LENGTH];

void USART3_Configuration_Send();
void USART3_Configuration_For_Hi220();
void Usart3DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
void Usart3SendByteInfoProc(u8 nSendInfo);
void Usart3SendBytesInfoProc(u8* pSendInfo, u16 nSendCount);
#endif
