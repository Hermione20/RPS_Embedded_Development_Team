/* Includes ------------------------------------------------------------------*/

#include "usart.h"	

////////////////////////////////////////////////////////////////////////////////// 	 
 
/**
  ******************************************************************************
  * @file    usart.c
  * @author  TC
  * @version V1.0.0
  * @date    07-September-2023
  * @brief   该文件提供固件功能来管理通用同步异步接收发送器(USART)的以下功能:
  *           + Initialization and Configuration
  *           + Data transfers
  *           + Multi-Processor Communication
  *           + LIN mode
  *           + Half-duplex mode
  *           + Smartcard mode
  *           + IrDA mode
  *           + DMA transfers management
  *           + Interrupts and flags management 
  *           
  @verbatim       
 ===============================================================================
                        ##### How to use this driver #####
 ===============================================================================
    [..]
      (#)使用以下功能开启外设时钟
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_USARTx, ENABLE)用于USART1和USART6
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_USARTx, ENABLE)用于USART2, USART3，UART4或UART5。
  
      (#) 根据USART模式，使用RCC_AHB1PeriphClockCmd()函数。I/O可以是TX, RX, CTS，或/和SCLK)。
  
      (#) Peripheral's alternate function: 
        (++) 使用GPIO_PinAFConfig()函数将引脚连接到所需外设的备用功能(AF)
        (++) 通过以下方式配置备用功能中的所需引脚:
            GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
        (++) 选择类型，上拉/下拉和输出速度通过
            GPIO_PuPd, GPIO_OType and GPIO_Speed members
        (++) Call GPIO_Init() function
          
      (#) 使用USART_Init()函数编程波特率，字长，停止位，奇偶校验，硬件流控制和模式(接收器/发射器)。
  
      (#) 对于同步模式，启用时钟并使用USART_ClockInit()函数对极性、相位和最后一位进行编程。
  
      (#) 如果需要使用中断模式，请使用USART_ITConfig()函数启用NVIC和相应的中断。
  
      (#) When using the DMA mode 
        (++) 使用DMA Init()函数配置DMA
        (++) 使用USART_DMACmd()函数激活所需的通道请求
   
      (#) 使用USART_Cmd()函数启用USART。
   
      (#) 当使用DMA模式时，使用DMA_Cmd()函数启用DMA。
    
      -@- 请参阅多处理器，LIN，半双工，智能卡，IrDA子部分
					欲知详情
    
    [..]        
				为了达到更高的通信波特率，可以使用USART_OverSampling8Cmd()函数启用8模式的过采样。
				这个函数应该在启用USART时钟(RCC_APBxPeriphClockCmd())之后，在调用USART_Init()函数之前调用。


            
    @endverbatim        
  ******************************************************************************
  * @attention
	*
  *  1.请注意printf函数重定向的开启，若无明确使用意向，请置零勿启用，否则容易进入死循环判断；
  *  2.每个串口发送接收的buf地址在其定义上方，提供数组长度的更改；
  *  3.USART3移植于成熟的CH100接收串口的配置，提供了接口移植功能；
  *  4.接上，如效果好，可以考虑其他串口同样写法；
  *  5.使用说明：在头文件处选择串口功能的开启，已同步主函数的初始化，无需去补；
  * 						在头文件宏定义处提供回调函数统一接口，加入回调函数名自行调用中断处理；
  *
  *  @更多详情，请右键移步usart.h更改配置开启。
  ******************************************************************************  
  */ 
/*
串口1/3是5V
*/

#if 1
#pragma import(__use_no_semihosting)  
/**
************************************************************************************************************************
* @Name     : fputc/_sys_exit
* @brief    : 加入以下代码,支持printf函数,而不需要选择use MicroLIB
* @param    : ch
* @param    : FILE *f
* @retval   : void
* @Note     : 加入以下代码,支持printf函数,而不需要选择use MicroLIB
************************************************************************************************************************
**/ 
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART3->SR&0X40)==0);//循环发送,直到发送完毕   
	USART3 ->DR = (u8) ch;      
	return ch;
}
#endif
#if EN_USART1 
/* Variables_definination-----------------------------------------------------------------------------------------------*/
	#define  UART1_RX_BUF_LENGTH 100
	#define  UART1_TX_BUF_LENGTH 100
	uint8_t _UART1_DMA_TX_BUF[UART1_TX_BUF_LENGTH];

		#if EN_USART1_DMA_SECOND_FIFO
				uint8_t _UART1_DMA_RX_BUF[2][UART1_RX_BUF_LENGTH];
		#else
				uint8_t _UART1_DMA_RX_BUF[UART1_RX_BUF_LENGTH];
		#endif
/*----------------------------------------------------------------------------------------------------------------------*/

/**
************************************************************************************************************************
* @Name     : uart_init
* @brief    : This function process the can message representing the encoder data received from the CAN2 bus.
* @param    : bound     the baud rate setting,bound:波特率
* @retval   : void
* @Note     : 初始化IO 串口1 
************************************************************************************************************************
**/
void uart1_init(u32 bound)
{
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1); //GPIOA9复用为USART1
	
	GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

   //USART1 初始化设置
	USART_DeInit(USART1);
  USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
		USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

		DMA_InitTypeDef dma;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

    DMA_DeInit(DMA2_Stream2);
    DMA_StructInit(&dma);
		
		#if EN_USART1_DMA_SECOND_FIFO
						dma.DMA_Memory0BaseAddr   =   (uint32_t)&_UART1_DMA_RX_BUF[0][0];
						dma.DMA_BufferSize        =   sizeof(_UART1_DMA_RX_BUF)/2;
		#else
						dma.DMA_Memory0BaseAddr   =   (uint32_t)&_UART1_DMA_RX_BUF;    //DMA接收基地址
						dma.DMA_BufferSize        =   sizeof(_UART1_DMA_RX_BUF);       //传输数据数量
		#endif
				
    dma.DMA_Channel = DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);

    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;

    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_Medium;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream2, &dma);
	
	//配置Memory1,Memory0是第一个使用的Memory
				#if EN_USART1_DMA_SECOND_FIFO
					DMA_DoubleBufferModeConfig(DMA2_Stream2,  (uint32_t)&_UART1_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
					DMA_DoubleBufferModeCmd(DMA2_Stream2, ENABLE);
				#endif
		DMA_Cmd(DMA2_Stream2, ENABLE);

		
			USART_Cmd(USART1, ENABLE);
}




/**
************************************************************************************************************************
* @Name     : USART1_IRQHandler
* @brief    : 串口1中断服务程序
* @param    : none
* @retval   : void
* @Note     : 注意,读取USARTx->SR能避免莫名其妙的错误   	
************************************************************************************************************************
**/
void USART1_IRQHandler(void)                	//串口1中断服务程序
{

	static uint32_t this_time_rx_len = 0;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		(void)USART1->SR;
		(void)USART1->DR;
		if(DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0)
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);	
			this_time_rx_len = UART1_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)UART1_RX_BUF_LENGTH;     //relocate the dma memory pointer to the beginning position
			DMA2_Stream2->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
			DMA_Cmd(DMA2_Stream2, ENABLE);
     if(this_time_rx_len == 18u)
			{
			 USART1_Data_Receive_Process;
			}
		}
		
		else 
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
			this_time_rx_len = UART1_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)UART1_RX_BUF_LENGTH;      //relocate the dma memory pointer to the beginning position
			DMA2_Stream2->CR &= ~(uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 0
			DMA_Cmd(DMA2_Stream2, ENABLE);
       if(this_time_rx_len == 18u)
			{
			USART1_Data_Receive_Process;
			}
		}
	}       

} 

#endif


#if EN_USART2
/* Variables_definination-----------------------------------------------------------------------------------------------*/
	#define  UART2_RX_BUF_LENGTH 100
	#define  UART2_TX_BUF_LENGTH 100
	uint8_t _UART2_DMA_TX_BUF[UART2_TX_BUF_LENGTH];

		#if EN_UART2_DMA_SECOND_FIFO
				uint8_t _UART2_DMA_RX_BUF[2][UART2_RX_BUF_LENGTH];
		#else
				uint8_t _UART2_DMA_RX_BUF[UART2_RX_BUF_LENGTH];
		#endif
/*----------------------------------------------------------------------------------------------------------------------*/

void uart2_init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef dma;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART1时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//使能DMA1时钟
	
	//串口4对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA10复用为USART1

	//UART4端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

	//UART4 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART2, &USART_InitStructure); //初始化串口1

		
				USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
				DMA_DeInit(DMA1_Stream5);
				DMA_StructInit(&dma);
				
#if EN_UART2_DMA_SECOND_FIFO
						dma.DMA_Memory0BaseAddr   =   (uint32_t)&_UART2_DMA_RX_BUF[0][0];
						dma.DMA_BufferSize        =   sizeof(_UART2_DMA_RX_BUF)/2;
					#else
						dma.DMA_Memory0BaseAddr   =   (uint32_t)&_UART2_DMA_RX_BUF;    //DMA接收基地址
						dma.DMA_BufferSize        =   sizeof(_UART2_DMA_RX_BUF);       //传输数据数量
					#endif

				dma.DMA_Channel = DMA_Channel_4;
				dma.DMA_PeripheralBaseAddr		= (uint32_t)(&USART2->DR);

				dma.DMA_DIR 									= DMA_DIR_PeripheralToMemory;

				dma.DMA_PeripheralInc 				= DMA_PeripheralInc_Disable;
				dma.DMA_MemoryInc 						= DMA_MemoryInc_Enable;
				dma.DMA_PeripheralDataSize 		= DMA_PeripheralDataSize_Byte;
				dma.DMA_MemoryDataSize 				= DMA_MemoryDataSize_Byte;
				dma.DMA_Mode 									= DMA_Mode_Normal;
				dma.DMA_Priority 							= DMA_Priority_Medium;
				dma.DMA_FIFOMode 							= DMA_FIFOMode_Disable;
				dma.DMA_FIFOThreshold 				= DMA_FIFOThreshold_1QuarterFull;
				dma.DMA_MemoryBurst 					= DMA_MemoryBurst_Single;
				dma.DMA_PeripheralBurst 			= DMA_PeripheralBurst_Single;
				DMA_Init(DMA1_Stream5, &dma);
				DMA_Cmd(DMA1_Stream5, ENABLE);
				
//配置Memory1,Memory0是第一个使用的Memory
#if EN_UART2_DMA_SECOND_FIFO
					DMA_DoubleBufferModeConfig(DMA1_Stream5,  (uint32_t)&_UART2_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
					DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);
				#endif
					
				USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
				DMA_Cmd(DMA1_Stream6, DISABLE);                           // 关DMA通道
				DMA_DeInit(DMA1_Stream6);
				while(DMA_GetCmdStatus(DMA1_Stream6) != DISABLE) {}
				dma.DMA_Channel = DMA_Channel_4;
				dma.DMA_PeripheralBaseAddr	= (uint32_t)(&USART2->DR);
				dma.DMA_Memory0BaseAddr   	= (uint32_t)&_UART2_DMA_TX_BUF[0];
				dma.DMA_DIR 			    = DMA_DIR_MemoryToPeripheral;
				dma.DMA_BufferSize			= 0;//sizeof(_USART1_DMA_TX_BUF);
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
				DMA_Init(DMA1_Stream6,&dma);				
				DMA_Cmd(DMA1_Stream6, ENABLE); 
					
				DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);
					
				NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;   // 发送DMA通道的中断配置
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;     // 优先级设置
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&NVIC_InitStructure);
						
			  //USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启发送非空中断
				USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);  //开启空闲帧中断

				NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口4中断通道
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
				NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
				NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

		
				USART_Cmd(USART2, ENABLE);  //使能串口1 
			
			
}
			uint8_t leng=0;
			void USART2_IRQHandler(void)
			{
			
			if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)    //接收中断
			{	
				(void)USART2->SR;
				(void)USART2->DR;
				DMA_Cmd(DMA1_Stream5, DISABLE);
				DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);
				leng = UART2_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA1_Stream5);

				USART2_Data_Receive_Process;
				DMA_SetCurrDataCounter(DMA1_Stream5,UART2_RX_BUF_LENGTH);
				DMA_Cmd(DMA1_Stream5, ENABLE);
			}
		}

		/**
		************************************************************************************************************************
		* @Name     : UART2_MYDMA_Enable
		* @brief    : 专属于串口2的发起一次DMA数据传输函数，已内置发送完成标志位的清除。
		* @param    : ndtr  数据传输量,即帧长度
		* @retval   : void
		* @Note     : 发送内容是_UART2_DMA_TX_BUF[100]数组,利用DMA1_Stream4_IRQHandler去清除标志位
		************************************************************************************************************************
		**/	
		void UART2_MYDMA_Enable(u16 ndtr)
		{	
			while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE){}	//确保DMA可以被设置  
				
			DMA_SetCurrDataCounter(DMA1_Stream6,ndtr);          //数据传输量  
		 
			DMA_Cmd(DMA1_Stream6, ENABLE);                      //开启DMA传输 	
		}	 

		void DMA1_Stream6_IRQHandler(void)
		{
			//清除标志
			if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET)//等待DMA1_Steam3传输完成
			{
				DMA_Cmd(DMA1_Stream6, DISABLE);                      //关闭DMA传输
				DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//清除DMA1_Steam3传输完成标志
			}
		}

#endif

#if EN_USART3
/* Variables_definination-----------------------------------------------------------------------------------------------*/
   #define USART3_DMA_RX_BUF_LEN 100
	 #define USART3_DMA_TX_BUF_LEN 100
	 uint8_t  _USART3_DMA_TX_BUF[USART3_DMA_TX_BUF_LEN];
		 #if EN_UART5_DMA_SECOND_FIFO
				uint8_t _USART3_DMA_RX_BUF[2][USART3_DMA_RX_BUF_LEN];
		 #else
				uint8_t _USART3_DMA_RX_BUF[USART3_DMA_RX_BUF_LEN];
		 #endif
/*----------------------------------------------------------------------------------------------------------------------*/

void uart3_init(u32 bound)//921600
	{
	USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	/* Peripheral Clock Enable -------------------------------------------------*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(USART_CH100_TX_GPIO_CLK | USART_CH100_RX_GPIO_CLK, ENABLE);
  
  /* Enable USART clock */
  USART_CH100_CLK_INIT(USART_CH100_CLK, ENABLE);
  
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
 
   /* Enable the USART OverSampling by 8 */
  USART_InitStructure.USART_BaudRate = bound;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* When using Parity the word length must be configured to 9 bits */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART_CH100, &USART_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = USART_CH100_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;//
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ
		NVIC_Init(&NVIC_InitStructure);
		
			/*使能空闲帧中断*/
   USART_ITConfig(USART_CH100,USART_IT_IDLE,ENABLE);
// USART_ITConfig(USART_CH100,USART_IT_RXNE,ENABLE);
	 USART_ClearFlag(USART_CH100,USART_FLAG_TC|USART_FLAG_IDLE);
		
  /* Configure DMA controller to manage USART TX and RX DMA request ----------*/ 
		
				/* Enable the DMA clock */
			RCC_AHB1PeriphClockCmd(USART_CH100_DMAx_CLK, ENABLE);
			DMA_InitTypeDef  DMA_InitStructure;
			

						#if EN_USART3_DMA_SECOND_FIFO
							DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)&_USART3_DMA_RX_BUF[0][0];
							DMA_InitStructure.DMA_BufferSize        =   sizeof(_USART3_DMA_RX_BUF)/2;
						#else
							DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)&_USART3_DMA_RX_BUF;    //DMA接收基地址
							DMA_InitStructure.DMA_BufferSize        =   sizeof(_USART3_DMA_RX_BUF);       //传输数据数量
						#endif	
				/* Configure DMA Initialization Structure */;
				DMA_InitStructure.DMA_FIFOMode 						 = DMA_FIFOMode_Disable ;
				DMA_InitStructure.DMA_FIFOThreshold 		 	 = DMA_FIFOThreshold_1QuarterFull ;
				DMA_InitStructure.DMA_MemoryBurst 				 = DMA_MemoryBurst_Single ;
				DMA_InitStructure.DMA_MemoryDataSize 			 = DMA_MemoryDataSize_Byte;
				DMA_InitStructure.DMA_MemoryInc 					 = DMA_MemoryInc_Enable;
				DMA_InitStructure.DMA_Mode								 = DMA_Mode_Circular;
				DMA_InitStructure.DMA_PeripheralBaseAddr 	 =(uint32_t) (&(USART_CH100->DR)) ;
				DMA_InitStructure.DMA_PeripheralBurst 		 = DMA_PeripheralBurst_Single;
				DMA_InitStructure.DMA_PeripheralDataSize   = DMA_PeripheralDataSize_Byte;
				DMA_InitStructure.DMA_PeripheralInc 			 = DMA_PeripheralInc_Disable;
				DMA_InitStructure.DMA_Priority 						 = DMA_Priority_High;
				/* Configure RX DMA */
				DMA_InitStructure.DMA_Channel 						 = USART_CH100_RX_DMA_CHANNEL ;
				DMA_InitStructure.DMA_DIR 							 	 = DMA_DIR_PeripheralToMemory ;
				DMA_Init(USART_CH100_RX_DMA_STREAM,&DMA_InitStructure);
				 /* Enable DMA USART RX Stream */
				DMA_Cmd(USART_CH100_RX_DMA_STREAM,ENABLE);
				/* Enable USART DMA RX Requsts */
				USART_DMACmd(USART_CH100, USART_DMAReq_Rx, ENABLE);
				
		//配置Memory1,Memory0是第一个使用的Memory
						#if EN_USART3_DMA_SECOND_FIFO
							DMA_DoubleBufferModeConfig(USART_CH100_RX_DMA_STREAM,  (uint32_t)&_USART3_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
							DMA_DoubleBufferModeCmd(USART_CH100_RX_DMA_STREAM, ENABLE);
						#endif

				//串口3配置发送DMA
				USART_DMACmd(USART_CH100, USART_DMAReq_Tx, ENABLE);   //启用USART的DMA接口，DMA1、数据流7、通道4
				DMA_Cmd(USART_CH100_TX_DMA_STREAM, DISABLE);                 // 关DMA通道
				DMA_DeInit(USART_CH100_TX_DMA_STREAM);

				while(DMA_GetCmdStatus(USART_CH100_TX_DMA_STREAM) != DISABLE) {}
				DMA_InitStructure.DMA_Channel 						= USART_CH100_TX_DMA_CHANNEL;
				DMA_InitStructure.DMA_PeripheralBaseAddr	= (uint32_t)(&USART_CH100->DR);
				DMA_InitStructure.DMA_Memory0BaseAddr   	= (uint32_t)&_USART3_DMA_TX_BUF[0];
				DMA_InitStructure.DMA_DIR 			    			= DMA_DIR_MemoryToPeripheral;
				DMA_InitStructure.DMA_BufferSize					= sizeof(_USART3_DMA_TX_BUF);   //发送数据放在该数组中
				DMA_InitStructure.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
				DMA_InitStructure.DMA_MemoryInc 					= DMA_MemoryInc_Enable;
				DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
				DMA_InitStructure.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;
				DMA_InitStructure.DMA_Mode 								= DMA_Mode_Normal;
				DMA_InitStructure.DMA_Priority 						= DMA_Priority_Low ;
				DMA_InitStructure.DMA_FIFOMode 						= DMA_FIFOMode_Disable;
				DMA_InitStructure.DMA_FIFOThreshold 			= DMA_FIFOThreshold_Full;
				DMA_InitStructure.DMA_MemoryBurst 				= DMA_MemoryBurst_Single;
				DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
				DMA_Init(USART_CH100_TX_DMA_STREAM,&DMA_InitStructure);
				DMA_Cmd(USART_CH100_TX_DMA_STREAM, ENABLE);                        //使能DMA通道开启

				NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;     // DMA发送中断
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   // 优先级设置
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&NVIC_InitStructure);


  /* Enable USART */
	
  USART_Cmd(USART_CH100, ENABLE);
}
	
uint8_t lengt=0;
void USART3_IRQHandler(void)
			{
			
			if(USART_GetITStatus(USART_CH100, USART_IT_IDLE) != RESET)    //接收中断
			{	
				(void)USART_CH100->SR;
				(void)USART_CH100->DR;
				DMA_Cmd(USART_CH100_TX_DMA_STREAM, DISABLE);
				DMA_ClearFlag(USART_CH100_TX_DMA_STREAM, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);
				lengt = USART3_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(USART_CH100_TX_DMA_STREAM);

				USART3_Data_Receive_Process;
				DMA_SetCurrDataCounter(USART_CH100_TX_DMA_STREAM,USART3_DMA_RX_BUF_LEN);
				DMA_Cmd(USART_CH100_TX_DMA_STREAM, ENABLE);
			}
		}

			/**
			************************************************************************************************************************
			* @Name     : UART3_MYDMA_Enable
			* @brief    : 专属于串口3的发起一次DMA数据传输函数，已内置发送完成标志位的清除。
			* @param    : ndtr  数据传输量,即帧长度
			* @retval   : void
			* @Note     : 发送内容是_UART3_DMA_TX_BUF[100]数组
			************************************************************************************************************************
			**/	
		void UART3_MYDMA_Enable(u16 ndtr)
		{	
			DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//清除DMA2_Steam7传输完成标志
			
			DMA_Cmd(DMA1_Stream3, DISABLE);                      //关闭DMA传输 
			
			while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}	//确保DMA可以被设置  
				
			DMA_SetCurrDataCounter(DMA1_Stream3,ndtr);          //数据传输量  
		 
			DMA_Cmd(DMA1_Stream3, ENABLE);                      //开启DMA传输 
				
		}	  
#endif
		
		
#if EN_UART4
/* Variables_definination-----------------------------------------------------------------------------------------------*/
	#define  UART4_RX_BUF_LENGTH 100
	#define  UART4_TX_BUF_LENGTH 100
	uint8_t _UART4_DMA_TX_BUF[UART4_TX_BUF_LENGTH];

		#if EN_UART4_DMA_SECOND_FIFO
				uint8_t _UART4_DMA_RX_BUF[2][UART4_RX_BUF_LENGTH];
		#else
				uint8_t _UART4_DMA_RX_BUF[UART4_RX_BUF_LENGTH];
		#endif
/*----------------------------------------------------------------------------------------------------------------------*/

void uart4_init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef dma;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能USART1时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//使能DMA1时钟


	
	//串口4对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOA10复用为USART1

	//UART4端口配置
	GPIO_InitStructure.GPIO_Pin 									= GPIO_Pin_10 | GPIO_Pin_11; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode 									= GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed 								= GPIO_Speed_100MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType 								= GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd									= GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PA9，PA10

	//UART4 初始化设置
	USART_InitStructure.USART_BaudRate 						= bound;//波特率设置
	USART_InitStructure.USART_WordLength 					= USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits	 					= USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity 							= USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode 								= USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(UART4, &USART_InitStructure); //初始化串口1
	
	USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);
	DMA_DeInit(DMA1_Stream2);
	DMA_StructInit(&dma);
				
#if EN_UART4_DMA_SECOND_FIFO
						dma.DMA_Memory0BaseAddr   =   (uint32_t)&_UART4_DMA_RX_BUF[0][0];
						dma.DMA_BufferSize        =   sizeof(_UART4_DMA_RX_BUF)/2;
#else
						dma.DMA_Memory0BaseAddr   =   (uint32_t)&_UART4_DMA_RX_BUF;    //DMA接收基地址
						dma.DMA_BufferSize        =   sizeof(_UART4_DMA_RX_BUF);       //传输数据数量
#endif

	dma.DMA_Channel = DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr		= (uint32_t)(&UART4->DR);
	dma.DMA_DIR 									= DMA_DIR_PeripheralToMemory;
	dma.DMA_PeripheralInc 				= DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc 						= DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize 		= DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize 				= DMA_MemoryDataSize_Byte;
	dma.DMA_Mode 									= DMA_Mode_Normal;
	dma.DMA_Priority 							= DMA_Priority_Medium;
	dma.DMA_FIFOMode 							= DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold 				= DMA_FIFOThreshold_1QuarterFull;
	dma.DMA_MemoryBurst 					= DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst 			= DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream2, &dma);
	DMA_Cmd(DMA1_Stream2, ENABLE);
				
//配置Memory1,Memory0是第一个使用的Memory
#if EN_UART4_DMA_SECOND_FIFO
				DMA_DoubleBufferModeConfig(DMA1_Stream2,  (uint32_t)&_UART4_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
				DMA_DoubleBufferModeCmd(DMA1_Stream2, ENABLE);
#endif
						
				USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
				DMA_Cmd(DMA1_Stream4, DISABLE);                           // 关DMA通道
				DMA_DeInit(DMA1_Stream4);
				while(DMA_GetCmdStatus(DMA1_Stream4) != DISABLE) {}
				dma.DMA_Channel = DMA_Channel_4;
				dma.DMA_PeripheralBaseAddr	= (uint32_t)(&UART4->DR);
				dma.DMA_Memory0BaseAddr   	= (uint32_t)&_UART4_DMA_TX_BUF[0];
				dma.DMA_DIR 			    			= DMA_DIR_MemoryToPeripheral;
				dma.DMA_BufferSize					= 0;//sizeof(_USART1_DMA_TX_BUF);
				dma.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
				dma.DMA_MemoryInc 					= DMA_MemoryInc_Enable;
				dma.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
				dma.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;
				dma.DMA_Mode 								= DMA_Mode_Normal;
				dma.DMA_Priority 						= DMA_Priority_Medium;
				dma.DMA_FIFOMode 						= DMA_FIFOMode_Disable;
				dma.DMA_FIFOThreshold 			= DMA_FIFOThreshold_Full;
				dma.DMA_MemoryBurst 				= DMA_MemoryBurst_Single;
				dma.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
				DMA_Init(DMA1_Stream4,&dma);				
					
				DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);
					
				NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;   // 发送DMA通道的中断配置
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;     // 优先级设置
				NVIC_InitStructure.NVIC_IRQChannelSubPriority 			 = 3;
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&NVIC_InitStructure);

			//USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启发送非空中断
				USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);  //开启空闲帧中断

				NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//串口4中断通道
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
				NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
				NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
				USART_Cmd(UART4, ENABLE);  //使能串口1 		
}
	

uint8_t length=0;
void UART4_IRQHandler(void)
		{
			if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)    //接收中断
			{	
				(void)UART4->SR;
				(void)UART4->DR;
				DMA_Cmd(DMA1_Stream2, DISABLE);
				DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
				length = UART4_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA1_Stream2);
				
				USART4_Data_Receive_Process
				
				DMA_SetCurrDataCounter(DMA1_Stream2,UART4_RX_BUF_LENGTH);
				DMA_Cmd(DMA1_Stream2, ENABLE);
			}
		}
	
		/**
		************************************************************************************************************************
		* @Name     : UART5_MYDMA_Enable
		* @brief    : 专属于串口5的发起一次DMA数据传输函数，已内置发送完成标志位的清除。
		* @param    : ndtr  数据传输量,即帧长度
		* @retval   : void
		* @Note     : 发送内容是_UART5_DMA_TX_BUF[100]数组,利用DMA1_Stream4_IRQHandler去清除标志位
		************************************************************************************************************************
		**/	
		void UART4_MYDMA_Enable(u16 ndtr)
		{	
			while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}	//确保DMA可以被设置  
				
			DMA_SetCurrDataCounter(DMA1_Stream4,ndtr);          //数据传输量  
		 
			DMA_Cmd(DMA1_Stream4, ENABLE);                      //开启DMA传输 	
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
#endif
	
#if EN_UART5

/* Variables_definination-----------------------------------------------------------------------------------------------*/
   #define UART5_DMA_RX_BUF_LEN 100
	 #define UART5_DMA_TX_BUF_LEN 100
	 uint8_t  _UART5_DMA_TX_BUF[UART5_DMA_TX_BUF_LEN];
	 #if EN_UART5_DMA_SECOND_FIFO
			uint8_t _UART5_DMA_RX_BUF[2][UART5_DMA_RX_BUF_LEN];
	 #else
			uint8_t _UART5_DMA_RX_BUF[UART5_DMA_RX_BUF_LEN];
   #endif
/*----------------------------------------------------------------------------------------------------------------------*/	

void uart5_init(u32 bound) //C12  D2
	{
		  //结构体初始化
			GPIO_InitTypeDef  GPIO_InitStructure;
			USART_InitTypeDef USART_InitStructure;
			NVIC_InitTypeDef  NVIC_InitStructure;
			DMA_InitTypeDef   DMA_InitStructure;
			//各时钟使能
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
			//IO初始化
			GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_12;
			GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
			GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
			GPIO_Init(GPIOC, &GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_2;
			GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
			GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
			GPIO_Init(GPIOD, &GPIO_InitStructure);

			GPIO_PinAFConfig(GPIOC,GPIO_PinSource12, GPIO_AF_UART5);
			GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,  GPIO_AF_UART5);
			//串口5初始化
			USART_InitStructure.USART_BaudRate              =   bound;
			USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None;
			USART_InitStructure.USART_Mode                  =   USART_Mode_Tx|USART_Mode_Rx;
			USART_InitStructure.USART_Parity                =   USART_Parity_No;
			USART_InitStructure.USART_StopBits              =   USART_StopBits_1;
			USART_InitStructure.USART_WordLength            =   USART_WordLength_8b;
			USART_Init(UART5, &USART_InitStructure);	
			//串口5配置接收DMA
			USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);     //启用USART的DMA接口，DMA1、数据流0、通道4
		
//			DMA_DeInit(DMA1_Stream0);
			DMA_StructInit(&DMA_InitStructure);              	//DMA各个参数赋初值
#if EN_UART5_DMA_SECOND_FIFO 

				DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)&_UART5_DMA_RX_BUF[0][0];
				DMA_InitStructure.DMA_BufferSize        =   sizeof(_UART5_DMA_RX_BUF)/2;
#else
				DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)&_UART5_DMA_RX_BUF;    //DMA接收基地址
				DMA_InitStructure.DMA_BufferSize        =   sizeof(_UART5_DMA_RX_BUF);       //传输数据数量
#endif
				DMA_InitStructure.DMA_Channel           =   DMA_Channel_4;
				DMA_InitStructure.DMA_PeripheralBaseAddr=   (uint32_t)(&UART5->DR);       //外设基地址
				DMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory;   //外设到存储器
				DMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable;    //外设地址不递增
				DMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable;         //存储器地址递增
				DMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte;      //存储器数据宽度
				DMA_InitStructure.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte;  //外设数据宽度
				DMA_InitStructure.DMA_Mode              =   DMA_Mode_Normal;              //是否开循环模式
				DMA_InitStructure.DMA_Priority          =   DMA_Priority_Medium  ;        //优先级中等
				DMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Disable;
				DMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_1QuarterFull;
				DMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single;       //存储器突发，单次传输
				DMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single;   //外设突发，单次传输
				DMA_Init(DMA1_Stream0, &DMA_InitStructure);
				DMA_Cmd(DMA1_Stream0, ENABLE);                        //使能DMA通道开启

  //配置Memory1,Memory0是第一个使用的Memory
#if EN_UART5_DMA_SECOND_FIFO
					DMA_DoubleBufferModeConfig(DMA1_Stream0,  (uint32_t)&_UART5_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
					DMA_DoubleBufferModeCmd(DMA1_Stream0, ENABLE);
#endif			
			NVIC_InitStructure.NVIC_IRQChannel										=	UART5_IRQn;          //串口5接收中断
			NVIC_InitStructure.NVIC_IRQChannelCmd									=	ENABLE;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority					=	2;
			NVIC_Init(&NVIC_InitStructure);
      USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);
			USART_Cmd(UART5, ENABLE);
				
			//串口5配置发送DMA
			USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE);   //启用USART的DMA接口，DMA1、数据流7、通道4
			DMA_Cmd(DMA1_Stream7, DISABLE);                 // 关DMA通道
			DMA_DeInit(DMA1_Stream7);

			while(DMA_GetCmdStatus(DMA1_Stream7) != DISABLE) {}
			DMA_InitStructure.DMA_Channel 						= DMA_Channel_4;
			DMA_InitStructure.DMA_PeripheralBaseAddr	= (uint32_t)(&UART5->DR);
			DMA_InitStructure.DMA_Memory0BaseAddr   	= (uint32_t)&_UART5_DMA_TX_BUF[0];
			DMA_InitStructure.DMA_DIR 			    			= DMA_DIR_MemoryToPeripheral;
			DMA_InitStructure.DMA_BufferSize					= sizeof(_UART5_DMA_TX_BUF);   //发送数据放在该数组中
			DMA_InitStructure.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
			DMA_InitStructure.DMA_MemoryInc 					= DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
			DMA_InitStructure.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;
			DMA_InitStructure.DMA_Mode 								= DMA_Mode_Normal;
			DMA_InitStructure.DMA_Priority 						= DMA_Priority_Low ;
			DMA_InitStructure.DMA_FIFOMode 						= DMA_FIFOMode_Disable;
			DMA_InitStructure.DMA_FIFOThreshold 			= DMA_FIFOThreshold_Full;
			DMA_InitStructure.DMA_MemoryBurst 				= DMA_MemoryBurst_Single;
			DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
			DMA_Init(DMA1_Stream7,&DMA_InitStructure);
			DMA_Cmd(DMA1_Stream7, ENABLE);                        //使能DMA通道开启
				

			NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn;     // DMA发送中断
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   // 优先级设置
			NVIC_InitStructure.NVIC_IRQChannelSubPriority				 = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
//			DMA_ITConfig(DMA1_Stream7,DMA_IT_TC,ENABLE);	
 }

			/**
			************************************************************************************************************************
			* @Name     : UART5_MYDMA_Enable
			* @brief    : 专属于串口5的发起一次DMA数据传输函数，已内置发送完成标志位的清除。
			* @param    : ndtr  数据传输量,即帧长度
			* @retval   : void
			* @Note     : 发送内容是_UART5_DMA_TX_BUF[100]数组
			************************************************************************************************************************
			**/	
void UART5_MYDMA_Enable(u16 ndtr)
		{	
			DMA_ClearFlag(DMA1_Stream7,DMA_FLAG_TCIF7);					//清除DMA2_Steam7传输完成标志
			
			DMA_Cmd(DMA1_Stream7, DISABLE);                     //关闭DMA传输 
			
			while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE){}	//确保DMA可以被设置  
				
			DMA_SetCurrDataCounter(DMA1_Stream7,ndtr);          //数据传输量  
		 
			DMA_Cmd(DMA1_Stream7, ENABLE);                      //开启DMA传输 
				
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
		
void UART5_IRQHandler(void)
			{
				static uint32_t this_time_rx_len = 0;
				if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)
					{
						(void)UART5->SR;
						(void)UART5->DR;
#if EN_UART5_DMA_SECOND_FIFO == 1
				if(DMA_GetCurrentMemoryTarget(DMA1_Stream0) == 0)
					{
						DMA_Cmd(DMA1_Stream0, DISABLE);
						DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);
						this_time_rx_len = UART5_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);

						DMA_SetCurrDataCounter(DMA1_Stream0, UART5_DMA_RX_BUF_LEN);
						DMA_MemoryTargetConfig (DMA1_Stream0,(uint32_t)&_UART5_DMA_RX_BUF[1][0],DMA_Memory_1);
						DMA_Cmd(DMA1_Stream0, ENABLE);

//						if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
							USART5_Data_Receive_Process;
					}
						//Target is Memory1
						else
							{
								DMA_Cmd(DMA1_Stream0, DISABLE);
								DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);

								this_time_rx_len =UART5_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);

								DMA_SetCurrDataCounter(DMA1_Stream0, UART5_DMA_RX_BUF_LEN);
								DMA_MemoryTargetConfig (DMA1_Stream0,(uint32_t)&_UART5_DMA_RX_BUF[0][0],DMA_Memory_0);
								DMA_Cmd(DMA1_Stream0, ENABLE);
								
//							if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
								USART5_Data_Receive_Process;
							}
#else
						DMA_Cmd(DMA1_Stream0, DISABLE);                          												//关闭串口5的DMA接收通道
						DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);
						this_time_rx_len = UART5_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0); //获取DMA_GetCurrDataCounter剩余数据量

								USART5_Data_Receive_Process		
						DMA_SetCurrDataCounter(DMA1_Stream0, UART5_DMA_RX_BUF_LEN);      								//设置当前DMA剩余数据量
						DMA_Cmd(DMA1_Stream0, ENABLE);                                       						//开启串口5的DMA接收通道

//						if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
								USART5_Data_Receive_Process																						//_UART5_DMA_RX_BUF作为DMA接收缓存区

#endif
					}
			}

#endif
			
#if EN_UART6
/* Variables_definination-----------------------------------------------------------------------------------------------*/
   #define UART6_DMA_RX_BUF_LEN 100
	 #define UART6_DMA_TX_BUF_LEN 100
	 uint8_t  _UART6_DMA_TX_BUF[UART6_DMA_TX_BUF_LEN];
	 #if EN_UART6_DMA_SECOND_FIFO
			uint8_t _UART6_DMA_RX_BUF[2][UART6_DMA_RX_BUF_LEN];
	 #else
			uint8_t _UART6_DMA_RX_BUF[UART6_DMA_RX_BUF_LEN];
   #endif
/*----------------------------------------------------------------------------------------------------------------------*/	

   void uart6_init(u32 bound)
	 {
			GPIO_InitTypeDef GPIO_InitStructure;
			USART_InitTypeDef USART_InitStructure;
			NVIC_InitTypeDef NVIC_InitStructure;
		 DMA_InitTypeDef DMA_uart6;

			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
		 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

			//IO初始化
			GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_6|GPIO_Pin_7;
			GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
			GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
			GPIO_Init(GPIOC, &GPIO_InitStructure);
		
			GPIO_PinAFConfig(GPIOC,GPIO_PinSource6, GPIO_AF_USART6);
			GPIO_PinAFConfig(GPIOC,GPIO_PinSource7, GPIO_AF_USART6);
		
		//串口5初始化
			USART_DeInit(USART6);
			USART_InitStructure.USART_BaudRate              =   bound;
			USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None;
			USART_InitStructure.USART_Mode                  =   USART_Mode_Tx|USART_Mode_Rx;
			USART_InitStructure.USART_Parity                =   USART_Parity_No;
			USART_InitStructure.USART_StopBits              =   USART_StopBits_1;
			USART_InitStructure.USART_WordLength            =   USART_WordLength_8b;
			USART_Init(USART6, &USART_InitStructure);

					
					
			
			//串口5配置接收DMA
				
				DMA_StructInit(&DMA_uart6);              //DMA各个参数赋初值

					#if EN_UART6_DMA_SECOND_FIFO 
						DMA_uart6.DMA_Memory0BaseAddr   =   (uint32_t)&_UART6_DMA_RX_BUF[0][0];
						DMA_uart6.DMA_BufferSize        =   sizeof(_UART6_DMA_RX_BUF)/2;
					#else
						DMA_uart6.DMA_Memory0BaseAddr   =   (uint32_t)&_UART6_DMA_RX_BUF;    //DMA接收基地址
						DMA_uart6.DMA_BufferSize        =   UART6_DMA_RX_BUF_LEN;       //传输数据数量
					#endif
				DMA_uart6.DMA_Channel           =   DMA_Channel_5;
				DMA_uart6.DMA_PeripheralBaseAddr=   (uint32_t)(&USART6->DR);       //外设基地址
				DMA_uart6.DMA_DIR               =   DMA_DIR_PeripheralToMemory;   //外设到存储器
				DMA_uart6.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable;    //外设地址不递增
				DMA_uart6.DMA_MemoryInc         =   DMA_MemoryInc_Enable;         //存储器地址递增
				DMA_uart6.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte;      //存储器数据宽度
				DMA_uart6.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte;  //外设数据宽度
				DMA_uart6.DMA_Mode              =   DMA_Mode_Circular;              //是否开循环模式
				DMA_uart6.DMA_Priority          =   DMA_Priority_High  ;        //优先级中等
				DMA_uart6.DMA_FIFOMode          =   DMA_FIFOMode_Disable;
				DMA_uart6.DMA_FIFOThreshold     =   DMA_FIFOThreshold_1QuarterFull;
				DMA_uart6.DMA_MemoryBurst       =   DMA_MemoryBurst_Single;       //存储器突发，单次传输
				DMA_uart6.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single;   //外设突发，单次传输
				DMA_Init(DMA2_Stream1, &DMA_uart6);
				DMA_Cmd(DMA2_Stream1, ENABLE);                        //使能DMA通道开启


  //配置Memory1,Memory0是第一个使用的Memory
				#if EN_UART6_DMA_SECOND_FIFO
					DMA_DoubleBufferModeConfig(DMA2_Stream1,  (uint32_t)&_UART6_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
					DMA_DoubleBufferModeCmd(DMA2_Stream1, ENABLE);
				#endif
			USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);    //启用USART的DMA接口，DMA1、数据流0、通道4
			
			//串口5配置发送DMA
			USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);   //启用USART的DMA接口，DMA1、数据流7、通道4
			DMA_Cmd(DMA2_Stream7, DISABLE);                 // 关DMA通道
			DMA_DeInit(DMA2_Stream7);

			while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE) {}
			DMA_uart6.DMA_Channel = DMA_Channel_5;
			DMA_uart6.DMA_PeripheralBaseAddr	= (uint32_t)(&USART6->DR);
			DMA_uart6.DMA_Memory0BaseAddr   	= (uint32_t)&_UART6_DMA_TX_BUF[0];
			DMA_uart6.DMA_DIR 			    = DMA_DIR_MemoryToPeripheral;
			DMA_uart6.DMA_BufferSize			= sizeof(_UART6_DMA_TX_BUF);   //发送数据放在该数组中
			DMA_uart6.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
			DMA_uart6.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
			DMA_uart6.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
			DMA_uart6.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
			DMA_uart6.DMA_Mode 				= DMA_Mode_Normal;
			DMA_uart6.DMA_Priority 			= DMA_Priority_Low ;
			DMA_uart6.DMA_FIFOMode 			= DMA_FIFOMode_Disable;
			DMA_uart6.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;
			DMA_uart6.DMA_MemoryBurst 		= DMA_MemoryBurst_Single;
			DMA_uart6.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single;
			DMA_Init(DMA2_Stream7,&DMA_uart6);
			DMA_Cmd(DMA2_Stream7, ENABLE);                        //使能DMA通道开启
				
			NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;     // DMA发送中断
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   // 优先级设置
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);


				NVIC_InitStructure.NVIC_IRQChannel						=	USART6_IRQn;          //串口5接收中断
				NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	2;
				NVIC_Init(&NVIC_InitStructure);
				
				USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);//空闲中断
			//USART_ITConfig(USART6, USART_IT_TXNE, ENABLE);//接收非空中断
				USART_Cmd(USART6, ENABLE);

				USART_Cmd(USART6, ENABLE);
 }

			/**
			************************************************************************************************************************
			* @Name     : UART5_MYDMA_Enable
			* @brief    : 专属于串口5的发起一次DMA数据传输函数，已内置发送完成标志位的清除。
			* @param    : ndtr  数据传输量,即帧长度
			* @retval   : void
			* @Note     : 发送内容是_UART5_DMA_TX_BUF[100]数组
			************************************************************************************************************************
			**/	
		void UART6_MYDMA_Enable(u16 ndtr)
		{	
			DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//清除DMA2_Steam7传输完成标志
			
			DMA_Cmd(DMA2_Stream7, DISABLE);                      //关闭DMA传输 
			
			while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}	//确保DMA可以被设置  
				
			DMA_SetCurrDataCounter(DMA2_Stream7,ndtr);          //数据传输量  
		 
			DMA_Cmd(DMA2_Stream7, ENABLE);                      //开启DMA传输 
				
		}	  

			void UART6_IRQHandler(void)
			{
				static uint32_t this_time_rx_len = 0;
				if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
					{
						(void)USART6->SR;
						(void)USART6->DR;

			#if EN_UART6_DMA_SECOND_FIFO 
						if(DMA_GetCurrentMemoryTarget(DMA1_Stream0) == 0)
							{
								DMA_Cmd(DMA1_Stream0, DISABLE);
								DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);
								this_time_rx_len = UART6_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);

								DMA_SetCurrDataCounter(DMA1_Stream0, UART5_DMA_RX_BUF_LEN);
								DMA_MemoryTargetConfig (DMA1_Stream0,(uint32_t)&_UART6_DMA_RX_BUF[1][0],DMA_Memory_1);
								DMA_Cmd(DMA1_Stream0, ENABLE);

								if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
									USART6_Data_Receive_Process;
							}
						//Target is Memory1
						else
							{
								DMA_Cmd(DMA1_Stream0, DISABLE);
								DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);

								this_time_rx_len =UART5_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);

								DMA_SetCurrDataCounter(DMA1_Stream0, UART6_DMA_RX_BUF_LEN);
								DMA_MemoryTargetConfig (DMA1_Stream0,(uint32_t)&_UART6_DMA_RX_BUF[0][0],DMA_Memory_0);
								DMA_Cmd(DMA1_Stream0, ENABLE);
								
								if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
									USART6_Data_Receive_Process;
							}
				#else
						DMA_Cmd(DMA1_Stream0, DISABLE);                          //关闭串口5的DMA接收通道
						DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);
						this_time_rx_len = UART6_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0); //获取DMA_GetCurrDataCounter剩余数据量

						DMA_SetCurrDataCounter(DMA1_Stream0, UART6_DMA_RX_BUF_LEN);      //设置当前DMA剩余数据量
						DMA_Cmd(DMA1_Stream0, ENABLE);                                       //开启串口5的DMA接收通道

						if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
							USART6_Data_Receive_Process;  //_UART5_DMA_RX_BUF作为DMA接收缓存区
			//		
			#endif
				}
			}
#endif
