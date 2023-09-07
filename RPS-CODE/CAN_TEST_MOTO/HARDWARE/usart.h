#ifndef __USART_H
#define __USART_H

#include "stm32f4xx.h"

void uart_Init(void);
void uart_SendByte(uint8_t Byte);
//void uart_SendArr(uint8_t* arr);
void uart_SendArr(uint8_t* MyArr,uint8_t length);
void uart_SendString(char* string);

uint32_t uart_Pow(uint32_t X,uint32_t Y);
void uart_SendNumber(uint32_t Number);

void USART2_IRQHandler(void);
uint8_t uart_GetRxFlag (void);
uint8_t uart_GetRxData (void);

#endif

