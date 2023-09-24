//#include "judge.h"
//#define SECOND_FIFO 0

//static uint8_t USART5_DMA_TX_BUF[100];
//static uint8_t judge_rxdata_buf[100];
//static uint8_t _UART5_RX_BUF[100];


//uint8_t judge_dma_rxbuff[2][100];
//uint8_t  tx_buf[66];
//uint8_t  pdata[32];
//uint8_t  ddata[66];
//uint8_t client_graphic_busy = 0;
//u32 bullet_num=0;
//#if SECOND_FIFO == 1	
//uint8_t _UART5_DMA_RX_BUF[2][BSP_UART5_DMA_RX_BUF_LEN];
//#else
//uint8_t _UART5_DMA_RX_BUF[100];
//#endif


///**
//  * @brief  UART初始化
//  * @param  void
//  * @retval void
//  */
////FIFO_S_t* UART5_TranFifo;
//void BSP_UART5_InitConfig(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//  USART_InitTypeDef USART_InitStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;
//  DMA_InitTypeDef DMA_InitStructure;

//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
//  //IO初始化
//  GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_12;
//  GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
//  GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);
//  GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_2;
//  GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
//  GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
//  GPIO_Init(GPIOD, &GPIO_InitStructure);
//  GPIO_PinAFConfig(GPIOC,GPIO_PinSource12, GPIO_AF_UART5);
//  GPIO_PinAFConfig(GPIOD,GPIO_PinSource2, GPIO_AF_UART5);
//  //串口5初始化
//  USART_InitStructure.USART_BaudRate              =   115200;
//  USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None;
//  USART_InitStructure.USART_Mode                  =   USART_Mode_Tx|USART_Mode_Rx;
//  USART_InitStructure.USART_Parity                =   USART_Parity_No;
//  USART_InitStructure.USART_StopBits              =   USART_StopBits_1;
//  USART_InitStructure.USART_WordLength            =   USART_WordLength_8b;
//  USART_Init(UART5, &USART_InitStructure);
//  USART_Cmd(UART5, ENABLE);
//  //串口5配置接收DMA
//  USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);    //启用USART的DMA接口，DMA1、数据流0、通道4
//  DMA_StructInit(&DMA_InitStructure);              //DMA各个参数赋初值

//#if SECOND_FIFO == 1
//  DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)&_UART5_DMA_RX_BUF[0][0];
//  DMA_InitStructure.DMA_BufferSize        =   sizeof(_UART5_DMA_RX_BUF)/2;
//#else
//  DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)&_UART5_DMA_RX_BUF;    //DMA接收基地址
//  DMA_InitStructure.DMA_BufferSize        =   sizeof(_UART5_DMA_RX_BUF);       //传输数据数量
//#endif
//  DMA_InitStructure.DMA_Channel           =   DMA_Channel_4;
//  DMA_InitStructure.DMA_PeripheralBaseAddr=   (uint32_t)(&UART5->DR);       //外设基地址
//  DMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory;   //外设到存储器
//  DMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable;    //外设地址不递增
//  DMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable;         //存储器地址递增
//  DMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte;      //存储器数据宽度
//  DMA_InitStructure.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte;  //外设数据宽度
//  DMA_InitStructure.DMA_Mode              =   DMA_Mode_Normal;              //是否开循环模式
//  DMA_InitStructure.DMA_Priority          =   DMA_Priority_Medium  ;        //优先级中等
//  DMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Disable;
//  DMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_1QuarterFull;
//  DMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single;       //存储器突发，单次传输
//  DMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single;   //外设突发，单次传输
//  DMA_Init(DMA1_Stream0, &DMA_InitStructure);
//  DMA_Cmd(DMA1_Stream0, ENABLE);                        //使能DMA通道开启


//  //配置Memory1,Memory0是第一个使用的Memory
//#if SECOND_FIFO == 1
//  DMA_DoubleBufferModeConfig(DMA1_Stream0,  (uint32_t)&_UART5_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
//  DMA_DoubleBufferModeCmd(DMA1_Stream0, ENABLE);
//#endif
//  NVIC_InitStructure.NVIC_IRQChannel						=	UART5_IRQn;          //串口5接收中断
//  NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	2;
//  NVIC_Init(&NVIC_InitStructure);
//  USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);
//  USART_Cmd(UART5, ENABLE);


//  //串口5配置发送DMA
//  USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE);   //启用USART的DMA接口，DMA1、数据流7、通道4
//  DMA_Cmd(DMA1_Stream7, DISABLE);                 // 关DMA通道
//  DMA_DeInit(DMA1_Stream7);

//  while(DMA_GetCmdStatus(DMA1_Stream7) != DISABLE) {}
//  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
//  DMA_InitStructure.DMA_PeripheralBaseAddr	= (uint32_t)(&UART5->DR);
//  DMA_InitStructure.DMA_Memory0BaseAddr   	= (uint32_t)&tx_buf[0];
//  DMA_InitStructure.DMA_DIR 			    = DMA_DIR_MemoryToPeripheral;
//	DMA_InitStructure.DMA_BufferSize			= sizeof(tx_buf);   //发送数据放在该数组中
//  DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
//  DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
//  DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
//  DMA_InitStructure.DMA_Mode 				= DMA_Mode_Normal;
//  DMA_InitStructure.DMA_Priority 			= DMA_Priority_Low ;
//  DMA_InitStructure.DMA_FIFOMode 			= DMA_FIFOMode_Disable;
//  DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;
//  DMA_InitStructure.DMA_MemoryBurst 		= DMA_MemoryBurst_Single;
//  DMA_InitStructure.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single;
//  DMA_Init(DMA1_Stream7,&DMA_InitStructure);
//  DMA_Cmd(DMA1_Stream7, ENABLE);                        //使能DMA通道开启

//  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn;     // DMA发送中断
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   // 优先级设置
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//}
//void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
//{
// 
//	DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输 
//	
//	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//确保DMA可以被设置  
//		
//	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量  
// 
//	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输 
//}	  


///**
//  * @brief  裁判系统空闲中断(UART4)
//  * @param  void
//  * @retval void
//  * @note   DJI很毒,以50HZ发送Info包，当有实时数据包产生时，在最近的一帧Info前无间隔加上实时数据包
//  */

//void UART5_IRQHandler(void)
//{
//  static uint32_t this_time_rx_len = 0;
////	 if(USART_GetITStatus(UART5, USART_IT_TXE) != RESET)
////	 {
////        if(!FIFO_S_IsEmpty(UART5_TranFifo))
////        {
////        uint16_t data = (uint16_t)FIFO_S_Get(UART5_TranFifo);
////        USART_SendData(UART5, data);
////        }
////        else
////        {
////        USART_ITConfig(UART5, USART_IT_TXE, DISABLE);
////        }
////
////		   USART_ClearITPendingBit(UART5, USART_IT_TXE);
////	 }
//  if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)
//    {
//      (void)UART5->SR;
//      (void)UART5->DR;

//#if SECOND_FIFO == 1
//      if(DMA_GetCurrentMemoryTarget(DMA1_Stream0) == 0)
//        {
//          DMA_Cmd(DMA1_Stream0, DISABLE);
//          DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);
//          this_time_rx_len = BSP_UART5_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);

//          DMA_SetCurrDataCounter(DMA1_Stream0, BSP_UART5_DMA_RX_BUF_LEN);
//          DMA_MemoryTargetConfig (DMA1_Stream0,(uint32_t)&_UART5_DMA_RX_BUF[1][0],DMA_Memory_1);
////			DMA1_Stream0->NDTR =(uint16_t)BSP_UART5_DMA_RX_BUF_LEN;     //relocate the dma memory pointer to the beginning position
////			DMA1_Stream0->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
//          DMA_Cmd(DMA1_Stream0, ENABLE);
////			FIFO_S_Puts(&_UART5_RX_FIFO,(void *)&_UART5_DMA_RX_BUF[0][0],this_time_rx_len);
////			unpack_fifo_data(&judge_unpack_obj,DN_REG_ID);
//          if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
//            judgement_data_handle(_UART5_DMA_RX_BUF[0],this_time_rx_len);
//        }
//      //Target is Memory1
//      else
//        {
//          DMA_Cmd(DMA1_Stream0, DISABLE);
//          DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);

//          this_time_rx_len =BSP_UART5_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);

//          DMA_SetCurrDataCounter(DMA1_Stream0, BSP_UART5_DMA_RX_BUF_LEN);
//          DMA_MemoryTargetConfig (DMA1_Stream0,(uint32_t)&_UART5_DMA_RX_BUF[0][0],DMA_Memory_0);
////			DMA1_Stream0->NDTR = (uint16_t)BSP_UART5_DMA_RX_BUF_LEN;      //relocate the dma memory pointer to the beginning position
////			DMA1_Stream0->CR &= ~(uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 0
//          DMA_Cmd(DMA1_Stream0, ENABLE);
////			FIFO_S_Puts(&_UART5_RX_FIFO,(void *)&_UART5_DMA_RX_BUF[1][0],this_time_rx_len);
////			unpack_fifo_data(&judge_unpack_obj,DN_REG_ID);
////          if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
////            judgement_data_handle(_UART5_DMA_RX_BUF[1],this_time_rx_len);
//        }
//#else
//      DMA_Cmd(DMA1_Stream0, DISABLE);                          //关闭串口5的DMA接收通道
//      DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);
//      this_time_rx_len = 100 - DMA_GetCurrDataCounter(DMA1_Stream0); //获取DMA_GetCurrDataCounter剩余数据量

//      DMA_SetCurrDataCounter(DMA1_Stream0, 100);      //设置当前DMA剩余数据量
//      DMA_Cmd(DMA1_Stream0, ENABLE);                                       //开启串口5的DMA接收通道

////      if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
////        judgement_data_handle(_UART5_DMA_RX_BUF,this_time_rx_len);  //_UART5_DMA_RX_BUF作为DMA接收缓存区
//#endif

//    }
//}

