/* Includes ------------------------------------------------------------------*/
#include "can.h"
/**
  ******************************************************************************
  * @file    can.c
  * @author  TC
  * @version V1.0.0
  * @date    07-September-2023
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Controller area network (CAN) peripheral:
  *           + Initialization and Configuration 
  *           + CAN Frames Transmission
  *           + CAN Frames Reception
  *           + Operation modes switch
  *           + Error management
  *           + Interrupts and flags
  *
@verbatim
 ===============================================================================
                        ##### How to use #####
 ===============================================================================
    [..]
      (#) Enable the CAN controller interface clock using 
          RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); for CAN1 
          and RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE); for CAN2
      -@- 如果您只使用CAN2，则必须启用CAN1时钟
       
      (#) CAN pins configuration
        (++) 启用CAN gpio时钟，使用如下功能:
							RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOx,ENABLE); 
        (++) 使用以下功能将涉及的CAN引脚连接到AF9
							GPIO_PinAFConfig(GPIOx, GPIO_PinSourcex, GPIO_AF_CANx);
        (++) 通过调用将这些CAN引脚配置为备用功能模式
							函数GPIO_Init();
							使用CAN_Init()和初始化和配置CAN
							CAN_FilterInit()函数。
			(#)使用CAN_Transmit()函数发送所需的CAN帧。

			(#)使用CAN_TransmitStatus()检查CAN帧的传输函数。

			(#)使用CAN_CancelTransmit()取消CAN帧的传输函数。
				 使用can_receive()函数接收一个CAN帧。

		  (#)使用CAN_FIFORelease()函数释放接收fifo。

			(#)返回等待接收帧的个数
					CAN_MessagePending()函数。
											 
      (#) 要控制CAN事件，可以使用以下两种方法之一:
        (++) 使用CAN_GetFlagStatus()函数检查CAN标志。 
        (++) 通过初始化阶段的CAN_ITConfig()函数和中断例程中的CAN_GetITStatus()函数使用CAN中断来检查事件是否发生。
             检查一个标志后，你应该使用CAN_ClearFlag()函数清除它。在检查中断事件后，您应该使用CAN_ClearITPendingBit()函数清除它。  

@endverbatim
           
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************  
  */
	
 
/* Variables_definination-----------------------------------------------------------------------------------------------*/
u8 CAN1_receive_buf[20];
u8 CAN2_receive_buf[20];
/*----------------------------------------------------------------------------------------------------------------------*/

/**
************************************************************************************************************************
* @Name     : CAN1_Mode_Init
* @brief    : This function initializes the CAN1 struct, the struct of CAN_Filter, and the NVIC configuration
* @param    : tbs2    tbs2:时间段2的时间单元.  Range from:CAN_BS2_1tq to CAN_BS2_8tq;
*	@param		:	tbs1		tbs1:时间段1的时间单元.  Range from:CAN_BS1_1tq to CAN_BS1_16tq;
*	@param		:	brp			brp :波特率分频器.       Range from:1 to 1024;                   tq=(brp)*tpclk1
* @param		: mode 		mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
* @retval   : 0,初始化OK;其他,初始化失败; 
* @Note     : 波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
*							Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
*							则波特率为:42M/((6+7+1)*6)=500Kbps
************************************************************************************************************************
**/
u8 CAN1_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

  	GPIO_InitTypeDef       GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE|CAN1_TX0_INT_ENABLE
   	NVIC_InitTypeDef       NVIC_InitStructure;
#endif
    //使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟	            

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA11,PA12
	
	  //引脚复用映射配置
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1
	  
		CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);
		
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=ENABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=DISABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~ CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;// Tbs2范围CAN_BS2_1tq ~ CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 
    
		
		//配置过滤器
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

#if CAN1_RX0_INT_ENABLE
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);	
#endif
#if CAN1_TX0_INT_ENABLE
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
	return 0;
}   
 
u8 CAN2_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

  	GPIO_InitTypeDef       GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN2_RX0_INT_ENABLE|CAN2_TX0_INT_ENABLE
   	NVIC_InitTypeDef       NVIC_InitStructure;
#endif
    //使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA时钟	            

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA11,PA12
	
	  //引脚复用映射配置
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOA11复用为CAN1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOA12复用为CAN1
	  
		CAN_DeInit(CAN2);
    CAN_StructInit(&CAN_InitStructure);
		
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=ENABLE ;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=DISABLE ;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN2, &CAN_InitStructure);   // 初始化CAN1 
    
		//配置过滤器
		CAN_FilterInitStructure.CAN_FilterNumber=14;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化

#if CAN2_RX0_INT_ENABLE
	    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
#if CAN2_TX0_INT_ENABLE
		NVIC_InitStructure.NVIC_IRQChannel = CAN2_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif	  
		CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		
	return 0;
}   



void CAN1_TX_IRQHandler(void) //CAN TX
{
  if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET)    //if transmit mailbox is empty 
  {
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);   
  }
}

void CAN2_TX_IRQHandler(void) //CAN TX
{
  if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET)    //if transmit mailbox is empty 
  {
	   CAN_ClearITPendingBit(CAN2,CAN_IT_TME);   
  }
}

#if CAN2_RX0_INT_ENABLE
void CAN2_RX0_IRQHandler(void)
{
		u8 i;
    CanRxMsg rx_message;
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
    {
       CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);//|CAN_IT_FF0|CAN_IT_FOV0
       CAN_Receive(CAN2, CAN_FIFO0, &rx_message);  
       //电机编码器数据处理
       CAN2_Receive_Msg(rx_message.Data);
			 for(i=0;i<8;i++)
				CAN2_receive_buf[i]=rx_message.Data[i];	
    }
}
#endif
#if CAN1_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数	
void CAN1_RX0_IRQHandler(void)
{	 	 
 		u8 i;
    CanRxMsg rx_message;		 
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
    {
       CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);//|CAN_IT_FF0|CAN_IT_FOV0
       CAN_Receive(CAN1, CAN_FIFO0, &rx_message);  
       //电机编码器数据处理
				for(i=0;i<8;i++)
				CAN1_receive_buf[i]=rx_message.Data[i];
    }

}
#endif


/**
************************************************************************************************************************
* @Name     : CAN1_Send_Msg
* @brief    : can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
* @param    : len     len:数据长度(最大为8)				  
* @retval   : 0,成功; 其他,失败;
* @Note     : none
************************************************************************************************************************
**/
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x251;	 // 标准标识符为0
  TxMessage.ExtId=0x12;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;							 // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;		
}


/**
************************************************************************************************************************
* @Name     : Can1_Receive_Msg
* @brief    : This function process the can message representing the encoder data received from the CAN2 bus.
* @param    : buf 数据缓存区;	 
* @retval   : 0,无数据被收到;其他,接收的数据长度;
* @Note     : can口接收数据查询
************************************************************************************************************************
**/
u8 CAN1_Receive_Msg(u8 *buf)
{		
		
 	u32 i;
	CanRxMsg rm1;

    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN1, CAN_FIFO0, &rm1);//读取数据	
	
		for(i=0;i<rm1.DLC;i++)
    buf[i]=rm1.Data[i]; 
	
	return rm1.Data[0];	
}
/**
************************************************************************************************************************
* @Name     : Can2_Receive_Msg
* @brief    : This function process the can message representing the encoder data received from the CAN2 bus.
* @param    : buf 数据缓存区;	 
* @retval   : 0,无数据被收到;其他,接收的数据长度;
* @Note     : can口接收数据查询
************************************************************************************************************************
**/
u8 CAN2_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg rm;
	rm.StdId=0x205;
	rm.RTR=0;
	rm.IDE=0;
    if(CAN_MessagePending(CAN2,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN2, CAN_FIFO0, &rm);//读取数据	
    for(i=0;i<rm.DLC;i++)
    buf[i]=rm.Data[i];  
	return rm.Data[0];	
}

/**
************************************************************************************************************************
* @Name     : set_M3508_info
* @brief    : This function sends encoder information for the M3508 via CAN1  
*							can发送一组数据(固定格式:ID为0X200,标准帧,数据帧)	
* @param    : velocity     the velocity of the M3508 motor.
* @param    : temperature  the temperature of the M3508 motor.
* @retval   : void
* @Note     : none
************************************************************************************************************************
**/
void set_M3508_info()
{
		CanTxMsg tm;					  //CAN的发送报文结构体
		tm.StdId=0x250;	
		tm.ExtId=0;
		tm.IDE=0;					
		tm.RTR=0;
		tm.DLC=8;
		tm.Data[0]=CAN2_receive_buf[2];	  //高八位转移低八位
		tm.Data[1]=CAN2_receive_buf[3];	  //十六位数据赋值给八位数据，保留低八位
		tm.Data[2]=CAN2_receive_buf[6];
		tm.Data[3]=0;
		tm.Data[4]=0;
		tm.Data[5]=0;
		tm.Data[6]=0;
		tm.Data[7]=0;
//		tm.Data[0]=(uint8_t)(velocity>>8);//高八位转移低八位
//		tm.Data[1]=(uint8_t)(velocity);	  //十六位数据赋值给八位数据，保留低八位
//		tm.Data[2]=(uint8_t)(temperature>>8);
//		tm.Data[3]=(uint8_t)(temperature);
//		tm.Data[4]=0;
//		tm.Data[5]=0;
//		tm.Data[6]=0;
//		tm.Data[7]=0;
		CAN_Transmit(CAN1 ,&tm);
}



void Set_Gimbal_Current1(CAN_TypeDef *CANx, int16_t ch_1_iq, int16_t ch_2_iq, int16_t ch_3_iq, int16_t ch_4_iq)
{
    CanTxMsg tx_message;    
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;

    tx_message.Data[0] = (unsigned char)(ch_1_iq >> 8);
    tx_message.Data[1] = (unsigned char)ch_1_iq;
    tx_message.Data[2] = (unsigned char)(ch_2_iq >> 8);
    tx_message.Data[3] = (unsigned char)ch_2_iq;
    tx_message.Data[4] = (unsigned char)(ch_3_iq >> 8);
    tx_message.Data[5] = (unsigned char)ch_3_iq;
    tx_message.Data[6] = (unsigned char)(ch_4_iq >> 8);
    tx_message.Data[7] = (unsigned char)ch_4_iq;
	
    CAN_Transmit(CANx,&tx_message);
}








