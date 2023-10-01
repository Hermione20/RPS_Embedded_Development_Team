#ifndef _SPI_H_
#define _SPI_H_
#include "main.h"


#define CS_L()	GPIO_ResetBits(GPIOC, GPIO_Pin_4)
#define CS_H()	GPIO_SetBits(GPIOC, GPIO_Pin_4)





void spi1_init(void);

uint8_t spi1_read_write_byte(uint8_t txc);
uint8_t spi1_write_reg(uint8_t reg_addr,uint8_t reg_val);
uint8_t spi1_read_reg(uint8_t reg_addr);
uint8_t spi1_read_reg_buffer(uint8_t reg_addr,void *buffer,uint16_t len);
void GPIO_mode_out(GPIO_TypeDef* GPIOx,uint16_t pin);
void GPIO_mode_af(GPIO_TypeDef* GPIOx,uint16_t pin);
void spi1_init(void);
void GPIO_Set(GPIO_TypeDef* GPIOx,u32 BITx,u32 MODE,u32 OTYPE,u32 OSPEED,u32 PUPD);
void GPIO_AF_Set(GPIO_TypeDef* GPIOx,u8 BITx,u8 AFx);
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler);
#endif
