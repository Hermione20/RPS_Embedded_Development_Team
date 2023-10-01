#ifndef __CH100_DMA_H
#define __CH100_DMA_H
extern float pitch_Angle1, yaw_Angle1, roll_Angle1; 
extern float pitch_Gyro1, yaw_Gyro1, roll_Gyro1;
extern float x_Acc1, y_Acc1, z_Acc1;

#define CH100_RX_BUFF_SIZE 100

__packed typedef struct
{
uint8_t tag; /* ????:0x91 */
uint8_t id; /* ??ID */
uint8_t rev[2];
float prs; /* ?? */
uint32_t ts; /* ??? */
float acc[3]; /* ??? */
float gyr[3]; /* ??? */
float mag[3]; /* ?? */
float eul[3]; /* ???:
Roll,Pitch,Yaw */
float quat[4]; /* ??? */
}id0x91_t;

 /* Definition for USART_CH100 resources ******************************************/
  #define USART_CH100                           USART3
  #define USART_CH100_CLK                       RCC_APB1Periph_USART3
  #define USART_CH100_CLK_INIT                  RCC_APB1PeriphClockCmd
  #define USART_CH100_IRQn                      USART3_IRQn
  #define USART_CH100_IRQHandler                USART3_IRQHandler

  #define USART_CH100_TX_PIN                    GPIO_Pin_10                
  #define USART_CH100_TX_GPIO_PORT              GPIOB                      
  #define USART_CH100_TX_GPIO_CLK               RCC_AHB1Periph_GPIOB
  #define USART_CH100_TX_SOURCE                 GPIO_PinSource10
  #define USART_CH100_TX_AF                     GPIO_AF_USART3

  #define USART_CH100_RX_PIN                    GPIO_Pin_11              
  #define USART_CH100_RX_GPIO_PORT              GPIOB                   
  #define USART_CH100_RX_GPIO_CLK               RCC_AHB1Periph_GPIOB
  #define USART_CH100_RX_SOURCE                 GPIO_PinSource11
  #define USART_CH100_RX_AF                     GPIO_AF_USART3

  /* Definition for DMAx resources ********************************************/
  #define USART_CH100_DR_ADDRESS                ((uint32_t)USART3 + 0x04) 

  #define USART_CH100_DMA                       DMA1
  #define USART_CH100_DMAx_CLK                  RCC_AHB1Periph_DMA1
     
  #define USART_CH100_TX_DMA_CHANNEL            DMA_Channel_4
  #define USART_CH100_TX_DMA_STREAM             DMA1_Stream3
  #define USART_CH100_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF3
  #define USART_CH100_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF3
  #define USART_CH100_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF3
  #define USART_CH100_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF3
  #define USART_CH100_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF3
              
  #define USART_CH100_RX_DMA_CHANNEL            DMA_Channel_4
  #define USART_CH100_RX_DMA_STREAM             DMA1_Stream1
  #define USART_CH100_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF1
  #define USART_CH100_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF1
  #define USART_CH100_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF1
  #define USART_CH100_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF1
  #define USART_CH100_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF1

//  #define USART_CH100_DMA_TX_IRQn               DMA2_Stream6_IRQn
//  #define USART_CH100_DMA_RX_IRQn               DMA2_Stream1_IRQn
//  #define USART_CH100_DMA_TX_IRQHandler         DMA2_Stream6_IRQHandler
//  #define USART_CH100_DMA_RX_IRQHandler         DMA2_Stream1_IRQHandler

void ch100_USART_Config(void);
void CH100_getDATA(void);
#endif