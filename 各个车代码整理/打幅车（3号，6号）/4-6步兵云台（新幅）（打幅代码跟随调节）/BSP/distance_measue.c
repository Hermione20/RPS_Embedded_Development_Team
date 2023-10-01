
//#include "main.h"

// /**
//  * @brief  配置嵌套向量中断控制器NVIC
//  * @param  无
//  * @retval 无
//  */
//	
//uint8_t measure_recive_buffer[RECEIVEBUFF_SIZE]={0};
//uint8_t measure_recive_count=0;
//measure_frame_t measure_frame_sent=FREAM_DEFAULT;
//measure_frame_t measure_frame_receive=FREAM_DEFAULT;
//measure_single_frame_t measure_single_frame_receive;//单次测量数据接收帧

//uint8_t frame_value[4]={0,0,0,0};   //一帧数据的value
//measure_mode_e measure_mode=continue_measure;  //测量模式状态

//uint8_t Usart3_rec_len;
///**
//  * @brief  DEBUG_USART GPIO 配置,工作模式配置。115200 8-N-1 ，中断接收模式
//  * @param  无
//  * @retval 无
//  */
//void MEASURE_DISTANCE_USART_Config(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//  USART_InitTypeDef USART_InitStructure;
//  NVIC_InitTypeDef NVIC_InitStruct;	
//	
//  RCC_AHB1PeriphClockCmd( DEBUG_USART_RX_GPIO_CLK|DEBUG_USART_TX_GPIO_CLK, ENABLE);

//  /* Enable UART clock */
//  RCC_APB1PeriphClockCmd(DEBUG_USART_CLK, ENABLE);
//  
//  /* Connect PXx to USARTx_Tx*/
//  GPIO_PinAFConfig(DEBUG_USART_RX_GPIO_PORT,DEBUG_USART_RX_SOURCE, DEBUG_USART_RX_AF);

//  /* Connect PXx to USARTx_Rx*/
//  GPIO_PinAFConfig(DEBUG_USART_TX_GPIO_PORT,DEBUG_USART_TX_SOURCE,DEBUG_USART_TX_AF);
//	
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//  NVIC_InitStruct.NVIC_IRQChannel = DEBUG_USART_IRQ;
//  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
//  NVIC_Init(&NVIC_InitStruct);

//  /* Configure USART Tx as alternate function  */
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

//  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN  ;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

//  /* Configure USART Rx as alternate function  */
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;
//  GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
//			
//  /* USART mode config */
//  USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
//  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//  USART_InitStructure.USART_StopBits = USART_StopBits_1;
//  USART_InitStructure.USART_Parity = USART_Parity_No ;
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//  USART_Init(DEBUG_USART, &USART_InitStructure); 
//  
//  USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);
//  
//  USART_Cmd(USART3, ENABLE);
//}


//void MEASURE_DISTANCE_USART_DMA_Config(void)
//{
//  DMA_InitTypeDef DMA_InitStructure;
////  NVIC_InitTypeDef  nvic;
//  /*开启DMA时钟*/
//  RCC_AHB1PeriphClockCmd(DEBUG_USART_DMA_CLK, ENABLE);
//  
//  /* 复位初始化DMA数据流 */
//  DMA_DeInit(DEBUG_USART_DMA_STREAM);

//  /* 确保DMA数据流复位完成 */
//  while (DMA_GetCmdStatus(DEBUG_USART_DMA_STREAM) != DISABLE)  {
//  }

//	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
//  /*usart1 rx对应dma1，通道4，数据流1*/	
//  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
//  /*设置DMA源：串口数据寄存器地址*/
//  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&USART3->DR);	 
//  /*内存地址(要传输的变量的指针)*/
//  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)measure_recive_buffer;
//  /*方向：从外设到内存*/		
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	
//  /*传输大小DMA_BufferSize=RECEIVEBUFF_SIZE*/	
//  DMA_InitStructure.DMA_BufferSize = RECEIVEBUFF_SIZE;
//  /*外设地址不增*/	    
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
//  /*内存地址自增*/
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
//  /*外设数据单位*/	
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//  /*内存数据单位 8bit*/
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	
//  /*DMA模式：不断循环*/
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;	 
//  /*优先级：中*/	
//  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;      
//  /*禁用FIFO*/
//  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;        
//  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;    
//  /*存储器突发传输 16个节拍*/
//  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;    
//  /*外设突发传输 1个节拍*/
//  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;    
//  /*配置DMA2的数据流2*/		   
//  DMA_Init(DMA1_Stream1, &DMA_InitStructure);
//  
////  nvic.NVIC_IRQChannel = DMA1_Stream1_IRQn;   // 发送DMA通道的中断配置
////  nvic.NVIC_IRQChannelPreemptionPriority = 2;     // 优先级设置
////  nvic.NVIC_IRQChannelSubPriority = 1;
////  nvic.NVIC_IRQChannelCmd = ENABLE;
////  NVIC_Init(&nvic);
////	DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);
//	
//  /*使能DMA*/
//  DMA_Cmd(DMA1_Stream1, ENABLE);
//  
//  /* 等待DMA数据流有效*/
//  while(DMA_GetCmdStatus(DEBUG_USART_DMA_STREAM) != ENABLE)
//  {
//  }   
//}


///*****************  发送一个字符 **********************/
//void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
//{
//	/* 发送一个字节数据到USART */
//	USART_SendData(pUSARTx,ch);
//	/* 等待发送数据寄存器为空 */
//	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
//}



////制作发送帧
//void Frame_Make(KEY key,uint8_t* value,uint8_t value_len)
//{ 
//	int i=0;
//	measure_frame_sent.key=key;
//	for(i=0;i<value_len;i++)
//	{
//	   measure_frame_sent.value[i]=value[i];
//	}
//	measure_frame_sent.CRC8=crc_high_first(&measure_frame_sent.key,total_crc_byte);
//}


////发送帧
//void Frame_Send()
//{
//	uint8_t *prt_to_frame=(uint8_t*)(&measure_frame_sent);
//	int index;
//	for(index=0;index<sizeof(measure_frame_sent);index++)
//	{
//	Usart_SendByte(DEBUG_USART,*(prt_to_frame));
//		prt_to_frame++;
//	}
//}


//////接收函数
////void DEBUG_USART_IRQHandler(void)
////{
////	if(USART_GetITStatus(DEBUG_USART,USART_IT_RXNE)!=RESET)
////	{		
////		measure_recive_buffer[measure_recive_count] = USART_ReceiveData(DEBUG_USART);
////		measure_recive_count++;
////	}	
////	if(measure_recive_buffer[0]!=frame_head)
////	{
////			measure_recive_count=0;
////	}
////	if(measure_recive_count>(sizeof(measure_recive_buffer)-1))
////	{
////		measure_frame_receive=*(measure_frame_t*)(measure_recive_buffer);
////		measure_recive_count=0;
////	}

////}

//void DMA1_Stream1_IRQHandler(void)  //不进该中断
//{
//	DMA_Cmd(DMA1_Stream4, DISABLE); 
//	DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);                //先清除标志位，否则只测量一次   
//	
////  if(DMA_GetFlagStatus(DMA1_Stream1,DMA_FLAG_TCIF1)!=RESET)          //检查中断是否发生
////	{	
////		DMA_Cmd(DEBUG_USART_DMA_STREAM,DISABLE);                   //关闭DMA传输
////		DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);                //先清除标志位，否则只测量一次   
////    DMA_SetCurrDataCounter(DMA1_Stream1,RECEIVEBUFF_SIZE);    //重新设置传输的数据数量
////	  DMA_Cmd(DMA1_Stream1,ENABLE);    		
////		
////	  Usart3_rec_len = RECEIVEBUFF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1);
////		measure_frame_receive=*(measure_frame_t*)(measure_recive_buffer);                        

////	}
//}


///************************与Hi220冲突,启用时取消注释*****************************************************/
////void DEBUG_USART_IRQHandler(void)
////{
////	if(USART_GetITStatus(DEBUG_USART,USART_IT_IDLE) != RESET)          //检查中断是否发生
////	{	
////		(void)USART3->SR;
////		(void)USART3->DR;
////			
////		DMA_Cmd(DEBUG_USART_DMA_STREAM,DISABLE);                //关闭DMA传输
////	  DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);             //清除DMA1_Steam1传输完成标志,放在dma开启之前，防止数据错位
////		                                                        //先清除标志位，否则只测量一次 
////	  Usart3_rec_len = RECEIVEBUFF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1);
////		measure_frame_receive=*(measure_frame_t*)(measure_recive_buffer);
////	  DMA_SetCurrDataCounter(DMA1_Stream1,RECEIVEBUFF_SIZE);  //重新设置传输的数据数量
////		DMA_Cmd(DMA1_Stream1,ENABLE);                           //开启DMA传输
////	}
////}
///***************************************************************************************/




//uint8_t check_the_recive(void)
//{
//if((measure_frame_receive.head==frame_head)&&(measure_frame_receive.tail==frame_tail))
//	return 1;
//else
//	{
//	measure_recive_count=0;
//		return 0;
//	}
//}

//uint32_t get_the_distance()
//{

//		return from_frame_to_the_distance_mm(&measure_frame_receive);
//}

////输入帧得到距离
//uint32_t from_frame_to_the_distance_mm(void *frame)
//{
//	uint32_t distance=0;
//  distance+=((measure_frame_t*)(frame))->value[3];
//	distance+=((measure_frame_t*)(frame))->value[2]<<8;
//  distance+=((measure_frame_t*)(frame))->value[1]<<16;
//	return distance;
//}

////单次测量数据帧接收
//uint8_t check_the_single_recive(void)
//{
//if((measure_frame_receive.head==frame_head)&&(measure_frame_receive.tail==single_frame_tail))
//	return 1;
//else
//	{
//	measure_recive_count=0;
//		return 0;
//	}
//}

//void stop_measure()
//{
//	frame_value[0]=0; 
//	frame_value[1]=0; 
//	frame_value[2]=0; 
//	frame_value[3]=0; 
//	Frame_Make(STOP_MEASURE,frame_value,sizeof(frame_value));
//	Frame_Send();
//}

//void set_continue_measure_mode()
//{
//	frame_value[0]=0; 
//	frame_value[1]=0; 
//	frame_value[2]=0; 
//	frame_value[3]=0; //单次测量1，连续测量0
//	Frame_Make(SET_MEASURE_MODE,frame_value,sizeof(frame_value));
//	Frame_Send();
//}

//void set_single_measure_mode()
//{
//	frame_value[0]=0; 
//	frame_value[1]=0; 
//	frame_value[2]=0; 
//	frame_value[3]=1; //单次测量1，连续测量0
//	Frame_Make(SET_MEASURE_MODE,frame_value,sizeof(frame_value));
//	Frame_Send();
//}

//void start_measure()
//{
//	frame_value[0]=0; 
//	frame_value[1]=0; 
//	frame_value[2]=0; 
//	frame_value[3]=0; 
//  Frame_Make(START_MEASURE,frame_value,sizeof(frame_value));
//  Frame_Send();
//}


////生成crc
//uint8_t crc_high_first(uint8_t *ptr,uint8_t len)
//{
//uint8_t i;
//uint8_t crc=0x00;

//while(len--)
//{
//	crc^=*ptr++;
//	for(i=8;i>0;--i)
//	{
//		if(crc&0x80)
//			crc=(crc<<1)^0x31;
//		else
//			crc=(crc<<1);
//	}
//}
//return crc;
//}

//uint32_t average_filter( float new_value)
//{
//    static float value_buf[AVERAGE_FILTER_N];
//    uint32_t sum  = 0;
//    uint8_t count = 0, i = 0;
//    for ( count = 0; count < AVERAGE_FILTER_N - 1; count++)
//    {
//        value_buf[count] = value_buf[count + 1] ;
//				sum += value_buf[count];
//    }
//    value_buf[AVERAGE_FILTER_N - 1] = new_value;
//		sum += value_buf[AVERAGE_FILTER_N - 1];
//		return sum / AVERAGE_FILTER_N;
//}


//void Distance_handle(void)    //传感器读取距离信息单位为mm，处理时单位为cm
//{
//	//1、获取距离信息
////  uint32_t distance ;
//	distance = (get_the_distance() - 80)/10.0 ;
//	if( distance  <  MAX_MEASURE)            //处理偶然数据
//	{
//		 distance_new = distance;               //测距大于40m错误数据，选择使用上次的距离信息
//		 distance_last = distance;
//	}
//	else                                     
//	{
//		 distance_new = distance_last ;	
//	}
//	if(distance_new > MAX_ATTACK_DISTANCE)
//	{
//		 distance_new = MAX_ATTACK_DISTANCE;
//	}
//	
//	Measure_filter_distance = average_filter(distance_new);     //距离数据均值滤波
//	
//	#if DISTANCE_ENABLE                       
//	 if(Measure_filter_distance == 0 && new_location.dis !=0)
//			Gimbal_Auto_Shoot.Distance = new_location.dis ;
//	 else if(Measure_filter_distance != 0)
//			Gimbal_Auto_Shoot.Distance = Measure_filter_distance ;
//	 else
//			Gimbal_Auto_Shoot.Distance = 150 ;
//  #else                                    
//	    Gimbal_Auto_Shoot.Distance = 150 ;
//  #endif
//	 
//	//2、处理距离信息，根据距离信息进行补偿(参数待调整)
//	if(Gimbal_Auto_Shoot.Distance < MAX_ATTACK_DISTANCE) 
//	{
//		 now_distance = Gimbal_Auto_Shoot.Distance;
//		 last_distance = Gimbal_Auto_Shoot.Distance;
//	}
//	else                                     
//	{
//		 now_distance = last_distance ;	
//	}
//	if(now_distance > MAX_ATTACK_DISTANCE)
//	{
//		 now_distance = MAX_ATTACK_DISTANCE;
//	}
//}



/*********************************************END OF FILE**********************/
