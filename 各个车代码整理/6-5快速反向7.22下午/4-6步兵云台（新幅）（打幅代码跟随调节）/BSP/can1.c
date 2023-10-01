#include "main.h"

/*----CAN1_TX-----PA12----*/
/*----CAN1_RX-----PA11----*/

void CAN1_Configuration(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

    gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &gpio);
    
    nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);    
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
    
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = ENABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = DISABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN1, &can);

	can_filter.CAN_FilterNumber=0;
	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh=0x0000;
	can_filter.CAN_FilterIdLow=0x0000;
	can_filter.CAN_FilterMaskIdHigh=0x0000;
	can_filter.CAN_FilterMaskIdLow=0x0000;
	can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
//    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
}






void Set_Poke_Current(CAN_TypeDef *CANx, int16_t Poke_iq)
{
	CanTxMsg tx_message;
    
    tx_message.StdId = 0x200;//send to gyro controll board
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] =0;
    tx_message.Data[1] = 0;
    tx_message.Data[2] = 0;
    tx_message.Data[3] = 0;
    tx_message.Data[4] = (unsigned char)(Poke_iq >> 8);
    tx_message.Data[5] = (unsigned char)Poke_iq;
    tx_message.Data[6] = 0;
    tx_message.Data[7] = 0;
    
    CAN_Transmit(CAN1,&tx_message);

}
void POWER_Control(u8* msg)
{
    CanTxMsg tx_message;
    
    tx_message.StdId = 0x405;//send to  controll board
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = msg[0];
    tx_message.Data[1] = msg[1];
    tx_message.Data[2] = msg[2];
    tx_message.Data[3] = msg[3];
    tx_message.Data[4] = msg[4];
    tx_message.Data[5] = msg[5];
    tx_message.Data[6] = msg[6];
    tx_message.Data[7] = msg[7];
    
    CAN_Transmit(CAN2,&tx_message);
}
void sendcan1(CAN_TypeDef *CANx, int16_t romate_speed,int16_t romate_angle,int16_t get_speedw,int16_t start_angle)
{
	CanTxMsg tx_message;
    
    tx_message.StdId = 0x409;//send to gyro controll board
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (unsigned char)(romate_speed >> 8);
    tx_message.Data[1] = (unsigned char)romate_speed;;
    tx_message.Data[2] = (unsigned char)(romate_angle >> 8);
    tx_message.Data[3] = (unsigned char)romate_angle;;
    tx_message.Data[4] = (unsigned char)(get_speedw >> 8);
    tx_message.Data[5] = (unsigned char)get_speedw;
    tx_message.Data[6] = (unsigned char)(start_angle >> 8);
    tx_message.Data[7] = (unsigned char)start_angle;
	
    
    CAN_Transmit(CANx,&tx_message);

}



void sendcan2(CAN_TypeDef *CANx, int16_t get_control_flag,int16_t get_mode_flag,int16_t die_flag,int16_t speed_yaw)
{
	CanTxMsg tx_message;
    
    tx_message.StdId = 0x408;//send to gyro controll board
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (unsigned char)(get_control_flag >> 8);
    tx_message.Data[1] = (unsigned char)get_control_flag;;
    tx_message.Data[2] = (unsigned char)(get_mode_flag >> 8);
    tx_message.Data[3] = (unsigned char)get_mode_flag;;
    tx_message.Data[4] = (unsigned char)(die_flag >> 8);
    tx_message.Data[5] = (unsigned char)die_flag;
    tx_message.Data[6] = (unsigned char)(speed_yaw >> 8);
    tx_message.Data[7] = (unsigned char)speed_yaw;
	
    
    CAN_Transmit(CANx,&tx_message);

}
void sendcan3(CAN_TypeDef *CANx, int16_t chassis_power_buffer,int16_t mains_power_chassis_output,int16_t chassis_power_limit,int16_t chassis_power)
{
	CanTxMsg tx_message;
    
    tx_message.StdId = 0x406;//send to gyro controll board
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (unsigned char)(chassis_power_buffer >> 8);
    tx_message.Data[1] = (unsigned char)chassis_power_buffer;
    tx_message.Data[2] = (unsigned char)(mains_power_chassis_output >> 8);
    tx_message.Data[3] = (unsigned char)mains_power_chassis_output;
    tx_message.Data[4] = (unsigned char)(chassis_power_limit >> 8);
    tx_message.Data[5] = (unsigned char)chassis_power_limit;
    tx_message.Data[6] = (unsigned char)(chassis_power >> 8);
    tx_message.Data[7] = (unsigned char)chassis_power;
	
    
    CAN_Transmit(CANx,&tx_message);

}



int abcdef;
void CAN1_TX_IRQHandler(void) //CAN TX
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{abcdef++;
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
    }
}

/*************************************************************************
                          CAN1_RX0_IRQHandler
描述：CAN1的接收中断函数
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{   
    CanRxMsg rx_message;	
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
        CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
	    CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
	    Can1ReceiveMsgProcess(&rx_message);
    }
}

