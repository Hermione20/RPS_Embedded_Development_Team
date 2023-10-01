#include "main.h"

float pitch_Angle1, yaw_Angle1, roll_Angle1; 
float pitch_Gyro1, yaw_Gyro1, roll_Gyro1;
float x_Acc1, y_Acc1, z_Acc1;

uint8_t ch100_Rx_Buffer[CH100_RX_BUFF_SIZE];
__align(4) id0x91_t dat; /* struct must be 4 byte aligned */

void ch100_USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  /* Peripheral Clock Enable -------------------------------------------------*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(USART_CH100_TX_GPIO_CLK | USART_CH100_RX_GPIO_CLK, ENABLE);
  
  /* Enable USART clock */
  USART_CH100_CLK_INIT(USART_CH100_CLK, ENABLE);
  
  /* Enable the DMA clock */
  RCC_AHB1PeriphClockCmd(USART_CH100_DMAx_CLK, ENABLE);
  
  /* USART_CH100 GPIO configuration -----------------------------------------------*/ 
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(USART_CH100_TX_GPIO_PORT, USART_CH100_TX_SOURCE, USART_CH100_TX_AF);
  GPIO_PinAFConfig(USART_CH100_RX_GPIO_PORT, USART_CH100_RX_SOURCE, USART_CH100_RX_AF);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_InitStructure.GPIO_Pin = USART_CH100_TX_PIN;
  GPIO_Init(USART_CH100_TX_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = USART_CH100_RX_PIN;
  GPIO_Init(USART_CH100_RX_GPIO_PORT, &GPIO_InitStructure);
 
 
		NVIC_InitStructure.NVIC_IRQChannel = USART_CH100_IRQn;

    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//


    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;//

    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ


NVIC_Init(&NVIC_InitStructure);

  /* Enable the USART OverSampling by 8 */
  USART_InitStructure.USART_BaudRate = 921600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* When using Parity the word length must be configured to 9 bits */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART_CH100, &USART_InitStructure);

	/*使能空闲帧中断*/
   USART_ITConfig(USART_CH100,USART_IT_IDLE,ENABLE);
	 USART_ClearFlag(USART_CH100,USART_FLAG_TC|USART_FLAG_IDLE);
  /* Configure DMA controller to manage USART TX and RX DMA request ----------*/ 
   
  /* Configure DMA Initialization Structure */
  DMA_InitStructure.DMA_BufferSize = CH100_RX_BUFF_SIZE ;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART_CH100->DR)) ;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  /* Configure RX DMA */
  DMA_InitStructure.DMA_Channel = USART_CH100_RX_DMA_CHANNEL ;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)ch100_Rx_Buffer ; 
  DMA_Init(USART_CH100_RX_DMA_STREAM,&DMA_InitStructure);
   /* Enable DMA USART RX Stream */
  DMA_Cmd(USART_CH100_RX_DMA_STREAM,ENABLE);
  /* Enable USART DMA RX Requsts */
  USART_DMACmd(USART_CH100, USART_DMAReq_Rx, ENABLE);
  /* Enable USART */
  USART_Cmd(USART_CH100, ENABLE);
}
void CH100_getDATA(void)
{
	volatile static float Last_yaw_temp1, Yaw_temp1;
	volatile static int Yaw_count1;
	
	pitch_Angle=-dat.eul[0];
	
	Last_yaw_temp1 = Yaw_temp1;
	Yaw_temp1 = dat.eul[2];
	if(Yaw_temp1-Last_yaw_temp1>=324)  
	{
		Yaw_count1--;
	}
	else if (Yaw_temp1-Last_yaw_temp1<=-324)
	{
		Yaw_count1++;
	}
	yaw_Angle = -(Yaw_temp1 + Yaw_count1*360); 
	
	roll_Angle=dat.eul[1];

//	pitch_Gyro=dat.gyr[0] ;
	yaw_Gyro=-dat.gyr[2] ;
	pitch_Gyro=dat.gyr[1] ;

	x_Acc1=dat.acc[0];
	y_Acc1=dat.acc[2];
	z_Acc1=dat.acc[1];
}	

void USART_CH100_IRQHandler(void)
{
	if(USART_GetITStatus(USART_CH100, USART_IT_IDLE)!= RESET)//
	{
		USART_ReceiveData(USART_CH100); //一定要读一次，不然可能会丢第一个字节，原因未知
		USART_ClearITPendingBit(USART_CH100,USART_IT_IDLE);//清除中断标志位
		DMA_Cmd(USART_CH100_RX_DMA_STREAM,DISABLE);  
		USART_DMACmd(USART_CH100, USART_DMAReq_Rx, DISABLE);
		memcpy(&dat, &ch100_Rx_Buffer[6], sizeof(id0x91_t));
		CH100_getDATA();
		USART_DMACmd(USART_CH100, USART_DMAReq_Rx, ENABLE);
		DMA_Cmd(USART_CH100_RX_DMA_STREAM,ENABLE);//重新置位后，地址指针变成0
	}
}