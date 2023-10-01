 #include "main.h"
 
 
#define SECOND_FIFO 0
receive_judge_t judge_rece_mesg; 



uint8_t UART5_DMA_RX_BUF[UART5_RX_BUF_LENGTH];
uint8_t UART5_DMA_TX_BUF[UART5_TX_BUF_LENGTH];


void UART5_Configuration(void)
{
    USART_InitTypeDef uart5;
    GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12, GPIO_AF_UART5);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource2, GPIO_AF_UART5);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
  //IO初始化
  gpio.GPIO_Mode   =   GPIO_Mode_AF;
  gpio.GPIO_OType  =   GPIO_OType_PP;
  gpio.GPIO_Pin    =   GPIO_Pin_12;
  gpio.GPIO_PuPd   =   GPIO_PuPd_UP;
  gpio.GPIO_Speed  =   GPIO_Speed_100MHz;
  GPIO_Init(GPIOC, &gpio);
  gpio.GPIO_Mode   =   GPIO_Mode_AF;
  gpio.GPIO_OType  =   GPIO_OType_PP;
  gpio.GPIO_Pin    =   GPIO_Pin_2;
  gpio.GPIO_PuPd   =   GPIO_PuPd_UP;
  gpio.GPIO_Speed  =   GPIO_Speed_100MHz;
  GPIO_Init(GPIOD, &gpio);

  //串口5初始化
  uart5.USART_BaudRate              =   115200;
  uart5.USART_Mode                  =   USART_Mode_Tx|USART_Mode_Rx;
  uart5.USART_Parity                =   USART_Parity_No;
  uart5.USART_StopBits              =   USART_StopBits_1;
  uart5.USART_WordLength            =   USART_WordLength_8b;
  uart5.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None;
  USART_Init(UART5, &uart5);
  USART_Cmd(UART5, ENABLE);

  USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);    //启用USART的DMA接口，DMA1、数据流0、通道4

	DMA_InitTypeDef dma;
  DMA_DeInit(DMA1_Stream0);
  DMA_StructInit(&dma);
  dma.DMA_Channel = DMA_Channel_4;
  dma.DMA_PeripheralBaseAddr		= (uint32_t)(&UART5->DR);
  dma.DMA_Memory0BaseAddr   		= (uint32_t)&UART5_DMA_RX_BUF;
  dma.DMA_DIR 					= DMA_DIR_PeripheralToMemory;
  dma.DMA_BufferSize				= UART5_RX_BUF_LENGTH;//sizeof(USART1_DMA_RX_BUF);
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
  DMA_Init(DMA1_Stream0, &dma);
  DMA_Cmd(DMA1_Stream0, ENABLE);
		
		
    nvic.NVIC_IRQChannel = UART5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority =1;
    nvic.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&nvic);		
		
    USART_ITConfig(UART5,USART_IT_RXNE,ENABLE);

  DMA_Cmd(DMA1_Stream7, DISABLE);                           // 关DMA通道
  DMA_DeInit(DMA1_Stream7);
  while(DMA_GetCmdStatus(DMA1_Stream7) != DISABLE) {}
  dma.DMA_Channel = DMA_Channel_4;
  dma.DMA_PeripheralBaseAddr	= (uint32_t)(&UART5->DR);
  dma.DMA_Memory0BaseAddr   	= (uint32_t)&(UART5_DMA_TX_BUF);
  dma.DMA_DIR 			    = DMA_DIR_MemoryToPeripheral;
  dma.DMA_BufferSize			= UART5_TX_BUF_LENGTH;//sizeof(USART1_DMA_TX_BUF);
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
  DMA_Init(DMA1_Stream7,&dma);

	nvic.NVIC_IRQChannel = DMA1_Stream7_IRQn;   // 发送DMA通道的中断配置
  nvic.NVIC_IRQChannelPreemptionPriority = 2;     // 优先级设置
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
  DMA_ITConfig(DMA1_Stream7,DMA_IT_TC,ENABLE);

  USART_ITConfig(UART5,USART_IT_IDLE,ENABLE);
  USART_Cmd(UART5,ENABLE);

}


uint8_t length=0;

void UART5_IRQHandler(void)
{
	 
		if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)    //接收中断
		{
			(void)UART5->SR;
		    (void)UART5->DR;
			DMA_Cmd(DMA1_Stream0, DISABLE);  
			DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);  
			length = UART5_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA1_Stream0);		
//			targetOffsetDataDeal(length , UART4_DMA_RX_BUF );
			
			get_distance3();
//			DMA_SetCurrDataCounter(DMA1_Stream0,UART5_RX_BUF_LENGTH);
			DMA_Cmd(DMA1_Stream0, ENABLE);
		}       


	
}

float af,bf,cf,df,abcdf;
float af1,bf1,cf1,df1,ef1,abcdf1;
float fghj;
float fghj1;
void get_distance3(void)
{	
	for(int k=0;k<length;k++)
	{
	if(UART5_DMA_RX_BUF[k]==111&&UART5_DMA_RX_BUF[k-1]==86){
		if(UART5_DMA_RX_BUF[k+4]==46&&UART5_DMA_RX_BUF[k+7]==32){
	fghj++;	
		af=(UART5_DMA_RX_BUF[k+2]-48);
		bf=(UART5_DMA_RX_BUF[k+3]-48);
		cf=(UART5_DMA_RX_BUF[k+5]-48);
		df=(UART5_DMA_RX_BUF[k+6]-48);
		abcdf=af*10+bf+cf*0.1+df*0.01;
	}
	else if(UART5_DMA_RX_BUF[k+3]==46&&UART5_DMA_RX_BUF[k+6]==32){
	fghj++;	
		bf=(UART5_DMA_RX_BUF[k+2]-48);
		cf=(UART5_DMA_RX_BUF[k+4]-48);
		df=(UART5_DMA_RX_BUF[k+5]-48);
		abcdf=af*10+bf+cf*0.1+df*0.01;
	}}
	if(UART5_DMA_RX_BUF[k]==115&&UART5_DMA_RX_BUF[k-1]==80){
		if(UART5_DMA_RX_BUF[k+4]==46&&UART5_DMA_RX_BUF[k+7]==13&&UART5_DMA_RX_BUF[k+8]==10)
  {
	fghj1++;	
		af1 =(UART5_DMA_RX_BUF[k+2]-48);
		bf1 =(UART5_DMA_RX_BUF[k+3]-48);
		cf1 =(UART5_DMA_RX_BUF[k+5]-48);
		df1 =(UART5_DMA_RX_BUF[k+6]-48);
		abcdf1=af1*10+bf1+cf1*0.1+df1*0.01;
//		abcd=(UART5_DMA_RX_BUF[k+2]-48)*10+(UART5_DMA_RX_BUF[k+3]-48);
	}
	else if(UART5_DMA_RX_BUF[k+5]==46&&UART5_DMA_RX_BUF[k+8]==13&&UART5_DMA_RX_BUF[k+9]==10)
		{ 
		ef1 =(UART5_DMA_RX_BUF[k+2]-48);
		af1 =(UART5_DMA_RX_BUF[k+3]-48);
		bf1 =(UART5_DMA_RX_BUF[k+4]-48);
		cf1 =(UART5_DMA_RX_BUF[k+6]-48);
		df1 =(UART5_DMA_RX_BUF[k+7]-48);
		abcdf1=ef1*100+af1*10+bf1+cf1*0.1+df1*0.01;
		}
	else if(UART5_DMA_RX_BUF[k+3]==46&&UART5_DMA_RX_BUF[k+6]==13&&UART5_DMA_RX_BUF[k+7]==10)
		{ 
		bf1 =(UART5_DMA_RX_BUF[k+2]-48);
		cf1 =(UART5_DMA_RX_BUF[k+4]-48);
		df1 =(UART5_DMA_RX_BUF[k+5]-48);
		abcdf1=bf1+cf1*0.1+df1*0.01;
	}}
}}
float abcad;
void Usart5DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)

{
  DMA_ClearFlag(DMA1_Stream7,DMA_FLAG_TCIF7); 
	USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(DMA1_Stream7, DISABLE);                      //关闭DMA传输 
	while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE){}	//确保DMA可以被设置  
	DMA_SetCurrDataCounter(DMA1_Stream7,ndtr);          //数据传输量  
	DMA_Cmd(DMA1_Stream7, ENABLE);                      //开启DMA传输                   //开启DMA传输
}
void DMA1_Stream7_IRQHandler(void)
{
  //清除标志
  if(DMA_GetFlagStatus(DMA1_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA1_Steam3传输完成
    {
      DMA_Cmd(DMA1_Stream7, DISABLE);                      //关闭DMA传输
      DMA_ClearFlag(DMA1_Stream7,DMA_FLAG_TCIF7);//清除DMA1_Steam3传输完成标志
    }

} 

void Usart5SendBytesInfoProc(u8* pSendInfo, u16 nSendCount)
{
  u16 i = 0;
  u8 *pBuf = NULL;

  pBuf = UART5_DMA_TX_BUF;
  for (i=0; i<nSendCount; i++)
    {
      *(pBuf+i) = pSendInfo[i];
    }

  Usart5DmaSendDataProc(DMA1_Stream7,nSendCount); //开始一次DMA传输！
}



/**
  * @brief    get judgement system message
  */

