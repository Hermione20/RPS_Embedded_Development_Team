#ifndef __UART4_H__
#define __UART4_H__
#include "main.h"
void UART4_Configuration(void);


#define UART4_RX_BUF_LENGTH   100
#define UART4_TX_BUF_LENGTH   100
extern uint8_t UART4_DMA_RX_BUF[UART4_RX_BUF_LENGTH];
extern uint8_t UART4_DMA_TX_BUF[UART4_TX_BUF_LENGTH];
void Uart4DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
void Uart4SendByteInfoProc(u8 nSendInfo);
void Uart4SendBytesInfoProc(u8* pSendInfo, u16 nSendCount);

#endif