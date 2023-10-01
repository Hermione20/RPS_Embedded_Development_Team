#ifndef __DEISTANCE_MEASURE
#define __DEISTANCE_MEASURE
#include "stm32f4xx.h"

//主控制逻辑的中断优先级最好低于此串口
//引脚定义
/*******************************************************/
//#define USE_MEASURE_DISTANCE//使用测距模块当使用此模块时，调试串口不能使用
#define DEBUG_USART                             USART3
#define DEBUG_USART_CLK                         RCC_APB1Periph_USART3
#define DEBUG_USART_BAUDRATE                    115200 //串口波特率波特率太高会时信号失真

#define DEBUG_USART_RX_GPIO_PORT                GPIOB
#define DEBUG_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOB
#define DEBUG_USART_RX_PIN                      GPIO_Pin_11
#define DEBUG_USART_RX_AF                       GPIO_AF_USART3
#define DEBUG_USART_RX_SOURCE                   GPIO_PinSource11

#define DEBUG_USART_TX_GPIO_PORT                GPIOB
#define DEBUG_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOB
#define DEBUG_USART_TX_PIN                      GPIO_Pin_10
#define DEBUG_USART_TX_AF                       GPIO_AF_USART3
#define DEBUG_USART_TX_SOURCE                   GPIO_PinSource10

#define DEBUG_USART_IRQHandler                  USART3_IRQHandler
#define DEBUG_USART_IRQ                 				USART3_IRQn

//DMA
#define DEBUG_USART_DR_BASE               (&USART3->DR)		
#define RECEIVEBUFF_SIZE                  8				//接收缓冲大小
#define DEBUG_USART_DMA_CLK               RCC_AHB1Periph_DMA1	
#define DEBUG_USART_DMA_CHANNEL           DMA_Channel_4
#define DEBUG_USART_DMA_STREAM            DMA1_Stream1

#define AVERAGE_FILTER_N  10
#define FREAM_DEFAULT {0x55,0,{0,0,0,0},0,0xAA}
#define MAX_MEASURE  4000              //最大测量距离40m

#define total_crc_byte   5
#define frame_head       0x55
#define frame_tail       0xaa //多次测量的帧尾
#define single_frame_tail  0xff  //单次测量的帧尾

#define total_crc_byte   5
#define frame_head       0x55
#define frame_tail       0xaa //多次测量的帧尾
#define single_frame_tail  0xff  //单次测量的帧尾

#define FREAM_DEFAULT {0x55,0,{0,0,0,0},0,0xAA}

typedef enum{
GET_INFORMATION=0x01,  //获取设备信息
SET_MEASURE_FREQUENCY=0x03,//设置测量频率
SET_MEASURE_FORMAT=0x04,   //设置数据格式
SET_MEASURE_MODE=0x0D,      //设置测量模式
START_MEASURE=0x05,     //启动测量
STOP_MEASURE=0x06,       //停止测量
CONSERVE_SET=0x08,       //保存设置
SET_BOUNT_RATE=0x12,     //设置波特率
}KEY;


typedef struct{
uint8_t head;
uint8_t key;
uint8_t value[4];
uint8_t CRC8;
uint8_t tail;
}measure_frame_t;


//单次测量无crc校验
typedef struct
{
uint8_t head;
uint8_t key;
uint8_t value[4];
	uint8_t CRC8;
uint8_t tail;
}measure_single_frame_t;

typedef enum{
single_measure=1,
continue_measure=0,	
}measure_mode_e;


void Frame_Make(KEY key,uint8_t* value,uint8_t value_len);
void Frame_Send(void);
void MEASURE_DISTANCE_USART_Config(void);
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void MEASURE_DISTANCE_USART_Config(void);
void Frame_Make(KEY key,uint8_t* value,uint8_t value_len);


void stop_measure(void);//每次切换测量模式时要先进入停止模式
void set_continue_measure_mode(void);
void set_single_measure_mode(void);
void start_measure(void);

uint32_t get_the_distance(void);
uint8_t check_the_recive(void);
uint8_t crc_high_first(uint8_t *ptr,uint8_t len);
uint8_t check_the_single_recive(void);
uint32_t from_frame_to_the_distance_mm(void *frame);
extern measure_frame_t measure_frame_sent;
extern measure_frame_t measure_frame_receive;
extern uint8_t frame_value[4];
extern uint32_t distance_last;
extern uint32_t Measure_filter_distance;

extern void MEASURE_DISTANCE_USART_DMA_Config(void);
void Distance_handle(void) ;

#endif
