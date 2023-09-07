#include "stm32f4xx.h"                  // Device header
#include "can.h"
#include "led.h"
#include "delay.h"

//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
//Fpclk1的时钟在初始化的时候设置为42M,
//如果设置CAN1_Mode_Init(CAN_SJW_1tq, CAN_BS2_6tq, CAN_BS1_7tq, 3, CAN_Mode_LoopBack),
//则波特率为:42M/((6+7+1)*3)=1Mbps.
//返回值:0,初始化OK;
//    其他,初始化失败; 

void CAN1_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟

//引脚复用
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1

//初始化GPIO
	GPIO_InitTypeDef GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA11,PA12
//初始化CAN1
	CAN_InitTypeDef CAN_InitStructure;
	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定

	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;	 //模式设置 
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=CAN_BS1_7tq; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=3;  //分频系数(Fdiv)为brp+1	

	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 
//配置过滤器
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
//初始化CAN1(Receive)NVIC中断
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
	CanTxMsg TxMessage;

u8 CAN1_Send_Msg(u8* msg,int stdID)
{	
	u8 mbox;
	u16 i=0;
	TxMessage.StdId=stdID;	 // 标准标识符
	TxMessage.ExtId=0x12;	 // 设置扩展标示符（29位）
	TxMessage.IDE=CAN_ID_STD;		  // 使用扩展标识符
	TxMessage.RTR=CAN_RTR_DATA;		  // 消息类型为数据帧，一帧8位
	TxMessage.DLC=8;							 // 发送信息长度
	for(i=0;i<8;i++)
		TxMessage.Data[i]=msg[i];				 // 第一帧信息          
	mbox=CAN_Transmit(CAN1, &TxMessage);   
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	if(i>=0XFFF)	return 1;
		return 0;		
}

void CAN1_M3508_2006(int current)
{
	u8 M3508_2006[8];	
	M3508_2006[0] = (uint8_t)(current >> 8) ;
	M3508_2006[1] = (uint8_t)current ;
	M3508_2006[2] = (uint8_t)(current >> 8) ;
	M3508_2006[3] = (uint8_t)current ;
	M3508_2006[4] = (uint8_t)(current >> 8) ;
	M3508_2006[5] = (uint8_t)current ;
	M3508_2006[6] = (uint8_t)(current >> 8) ;
	M3508_2006[7] = (uint8_t)current ;
	CAN1_Send_Msg(M3508_2006,0x200);//设置标识符，ID1~4为0x200，5~8为0x1FF
}

void CAN1_GM6020(int current)
{
	u8 GM6020[8];
	GM6020[0] = (current >> 8) ;
	GM6020[1] = current ;
	GM6020[2] = (current >> 8) ;
	GM6020[3] = current ;
	GM6020[4] = (current >> 8) ;
	GM6020[5] = current ;
	GM6020[6] = (current >> 8) ;
	GM6020[7] = current ;
	CAN1_Send_Msg(GM6020,0x1FF);//设置标识符，ID1~4为0x1FF，5~7为0x2FF
}

void CAN1_GM6020_p(int current)
{
	u8 GM6020[8];
	GM6020[0] = (current >> 8) ;
	GM6020[1] = current ;
	GM6020[2] = (current >> 8) ;
	GM6020[3] = current ;
	GM6020[4] = (current >> 8) ;
	GM6020[5] = current ;
	CAN1_Send_Msg(GM6020,0x2FF);//设置标识符，ID1~4为0x1FF，5~7为0x2FF
}

void CAN1_MF9025(int32_t speed)
{
	u8 MF9025[8];
	MF9025[0] = 0xA1 ;
	MF9025[1] = 0x00 ;
	MF9025[2] = 0x00 ;
	MF9025[3] = 0x00 ;
	MF9025[4] = (uint8_t)speed ;
	MF9025[5] = (uint8_t)(speed >> 8);
	MF9025[6] = (uint8_t)(speed >> 16);
	MF9025[7] = (uint8_t)(speed >> 24);
//	MF9025[0] = 0x19 ;
//	MF9025[1] = 0x00 ;
//	MF9025[2] = 0x00 ;
//	MF9025[3] = 0x00 ;
//	MF9025[4] = 0x00 ;
//	MF9025[5] = 0x00;
//	MF9025[6] = 0x00;
//	MF9025[7] = 0x00;
	CAN1_Send_Msg(MF9025,0x141);//设置标识符,0x140+ID		(uint8_t)speed		*(uint8_t*)(&speed)
}

//中断服务函数			  
CanRxMsg rx_message;

void CAN1_RX0_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
    {

				
			CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);//|CAN_IT_FF0|CAN_IT_FOV0
      
			CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
		
			//电机编码器数据处理

	 		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
		
		CAN_ClearITPendingBit(CAN1, CAN_IT_EWG|CAN_IT_EPV|CAN_IT_BOF|CAN_IT_LEC|CAN_IT_ERR);
}


