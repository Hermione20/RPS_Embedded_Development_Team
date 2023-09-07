#include "stm32f4xx.h"                  // Device header
#include "string.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "can.h"

uint8_t uart_RxData;
uint8_t uart_RxFlag=0;
uint8_t* MyArr;

void uart_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
//引脚复用_USART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //Tx
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //Rx
//初始化GPIO_USART2
	GPIO_InitTypeDef GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF ;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
//初始化USART2
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None ;
	USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_StopBits = USART_StopBits_1 ;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b ;
	USART_Init(USART2,&USART_InitStructure);
//初始化USART2(Receive)NVIC中断
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2 );

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn ;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART2,ENABLE);
}

//USART2->Send
void uart_SendByte(uint8_t Byte)
{
	USART_SendData(USART2,Byte);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
}

void uart_SendArr(uint8_t* MyArr,uint8_t length)
{
	int i;
//	length=strlen(MyArr);
	for (i=0;i<length;i++)
	{
		uart_SendByte(MyArr[i]);
	}
}

void uart_SendString(char* string)
{
	int i;
	for (i=0;string[i]!=0;i++)
	{
		uart_SendByte(string[i]);
	}
}


uint32_t uart_Pow(uint32_t X,uint32_t Y)
{
	uint32_t Result=1;
	while(Y--)
	{
		Result*=X;
	}
	return Result;
}

void uart_SendNumber(uint32_t Number)
{
	int i;
	uint32_t num=Number;
	uint8_t Length;
	for(Length=0;num;Length++)
	{
		num/=10;
	}
	for (i=0;i<Length;i++)
	{
		uart_SendByte(Number/uart_Pow(10,Length-i-1)%10+'0');
	}
}

//USART2(Receive)中断服务
void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2,USART_IT_RXNE)==SET)
	{
		uart_RxData=USART_ReceiveData(USART2);
		uart_RxFlag=1;
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	}
}

uint8_t uart_GetRxFlag (void)
{
	if (uart_RxFlag == 1)
	{
		uart_RxFlag = 0;
		return 1;
	}
	return 0;
}

uint8_t uart_GetRxData (void)
{
	return uart_RxData;
}


