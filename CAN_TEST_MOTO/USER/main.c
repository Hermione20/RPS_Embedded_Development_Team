#include "stm32f4xx.h"                  // Device header
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "can.h"

uint8_t usart_RxData;
uint8_t usart_RxFlag;
uint8_t CAN_RxData[8];
uint8_t CAN_RxMsgDLC;
int i=0;


int main(void)
{ 
	delay_init(168);	//初始化延时函数
	uart_Init();			//初始化串口,波特率为115200
	LED_Init();				//初始化LED 
	CAN1_Init();			//CAN初始化环回模式,波特率1Mbps

	CAN1_MF9025(-50);		//设置速度

	while(1)
	{

    i++;
    if(i%84000 == 0)
   {


			CAN1_M3508_2006(500);	//转动M3508，参数电流设置范围-16384~0~16384，对应范围-20A~20A,819.2=1A
													//转动M2006，参数电流设置范围-10000~0~10000，对应范围-10A~10A,1000=1A
			CAN1_GM6020(15000);	//转动GM6020，参数电压设置设置范围-30000~0~30000
//			CAN1_GM6020_p(15000);	//转动GM6020，参数电压设置设置范围-30000~0~30000

			i=0;
    }
	}
}
