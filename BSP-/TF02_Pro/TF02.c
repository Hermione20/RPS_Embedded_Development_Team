#include "usart3.h"
#include "TF02.h"

TF02_T TF02;

void TF02_IdleCallback()
{
	  TF02.Dist = ((USART3_DMA_RX_BUF[3] << 8) | USART3_DMA_RX_BUF[2]);
	  TF02.Strength = ((USART3_DMA_RX_BUF[5] << 8) | USART3_DMA_RX_BUF[4]);
		TF02.Temp = ((USART3_DMA_RX_BUF[7] << 8) | USART3_DMA_RX_BUF[6]);

//		HAL_UART_DMAStop(&huart3); 
//	  HAL_UART_Receive_DMA(&huart3,rx_buffer_gyro,BUFFER_SIZE_gyro);//不加这行不能收到第一次数据

  
}
