#include "main.h"
#include "string.h"

HI220_Stucture HI220_Data_From_Usart;
Hi220_Flags_t Hi220_Flags = {0};

static void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
    uint32_t crc = *currectCrc;
    uint32_t j;
    for (j=0; j < lengthInBytes; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    } 
    *currectCrc = crc;
}
char *p2 = 0;

u8 USART6_DMA_RX_BUF[USART6_RX_BUF_LENGTH] = {0};

void USART6_Configuration_For_Hi220()
{
    USART_InitTypeDef usart;
    GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
		DMA_InitTypeDef dma;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); 
	
		//串口1对应引脚复用映射
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOA9复用为USART1
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOA10复用为USART1
		
		//USART1端口配置
		gpio.GPIO_Pin 	= GPIO_Pin_6 | GPIO_Pin_7; //GPIOA9与GPIOA10
		gpio.GPIO_Mode 	= GPIO_Mode_AF;//复用功能
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
		gpio.GPIO_PuPd 	= GPIO_PuPd_NOPULL; //上拉
		GPIO_Init(GPIOC,&gpio); //初始化PA9，PA10

    USART_DeInit(USART6);
//    USART_StructInit(&usart);
    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART6, &usart);   


		USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
    
		DMA_DeInit(DMA2_Stream1);
//    DMA_StructInit(&dma);
    dma.DMA_Channel = DMA_Channel_5;
    dma.DMA_PeripheralBaseAddr	= (uint32_t)(&USART6->DR);
    dma.DMA_Memory0BaseAddr   	= (uint32_t)&USART6_DMA_RX_BUF[0];
    dma.DMA_DIR 			    = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize			= USART6_RX_BUF_LENGTH;//sizeof(USART1_DMA_RX_BUF);
    dma.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
    dma.DMA_Mode 				= DMA_Mode_Normal;
    dma.DMA_Priority 			= DMA_Priority_Medium;
    dma.DMA_FIFOMode 			= DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold 		= DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst 		= DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream1, &dma);
    DMA_Cmd(DMA2_Stream1, ENABLE);
		
    //配置Memory1,Memory0是第一个使用的Memory
		//使能双缓冲区模式时，将自动使能循环模式（DMA_SxCR 中的 CIRC 位的状态是“无
		//关”），并在每次事务结束时交换存储器指针。
//    DMA_DoubleBufferModeConfig(DMA2_Stream6, (uint32_t)&USART6_DMA_TX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
//    DMA_DoubleBufferModeCmd(DMA2_Stream6, ENABLE);
    
		nvic.NVIC_IRQChannel = USART6_IRQn;                          
		nvic.NVIC_IRQChannelPreemptionPriority = 3;   //pre-emption priority 
		nvic.NVIC_IRQChannelSubPriority = 3;		    //subpriority 
		nvic.NVIC_IRQChannelCmd = ENABLE;			
		NVIC_Init(&nvic);	
		USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
//		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);        //usart rx idle interrupt  enabled
//		nvic.NVIC_IRQChannel = DMA2_Stream2_IRQn ;
//    nvic.NVIC_IRQChannelPreemptionPriority =3;
//    nvic.NVIC_IRQChannelSubPriority = 0;
//    nvic.NVIC_IRQChannelCmd = ENABLE; 
//    NVIC_Init(&nvic); 
//		DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);
		
		
    USART_Cmd(USART6,ENABLE);
}

void USART6_IRQHandler(void)                	//串口1中断服务程序
{
	uint16_t CRCReceived = 0;            /* CRC value received from a frame */
	uint16_t CRCCalculated = 0;          /* CRC value caluated from a frame */
	u8 length=0;
	u8 Length_Data=0;
	u8 Length_Deal = 6;
	u8 *p;
	
	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)    //接收中断
	{
		(void)USART6->SR;
		(void)USART6->DR;
		DMA_Cmd(DMA2_Stream1, DISABLE); 
		DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);  //************************************
		length = USART6_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA2_Stream1);
		
		if(Hi220_Flags.Hi220_Flag_Reconfig.Flag_Reconfig != 0)
		{
			USART6_DMA_RX_BUF[USART6_RX_BUF_LENGTH-1] = '\0';
			if(Hi220_Flags.Hi220_Flag_Reconfig.Bits.BAUD_Reconfig)
			{
				p2 = strstr((char*)USART6_DMA_RX_BUF,"OK");
				if(p2 != 0)
					Hi220_Flags.Hi220_Flag_Configured.Bits.BAUD_Configured = 1;
			}
			if(Hi220_Flags.Hi220_Flag_Reconfig.Bits.Eout_Reconfig)
			{
				p2 = strstr((char*)USART6_DMA_RX_BUF,"OK");
				if(p2 != 0)
					Hi220_Flags.Hi220_Flag_Configured.Bits.Eout_Configured = 1;
			}
			if(Hi220_Flags.Hi220_Flag_Reconfig.Bits.ODR_Reconfig)
			{
				p2 = strstr((char*)USART6_DMA_RX_BUF,"OK");
				if(p2 != 0)
					Hi220_Flags.Hi220_Flag_Configured.Bits.ODR_Configured = 1;
			}
			if(Hi220_Flags.Hi220_Flag_Reconfig.Bits.SETPTL_Reconfig)
			{
				p2 = strstr((char*)USART6_DMA_RX_BUF,"new packet items");
				if(p2 != 0)
					Hi220_Flags.Hi220_Flag_Configured.Bits.SETPTL_Configured = 1;
			}
			memset(USART6_DMA_RX_BUF,0,sizeof(USART6_DMA_RX_BUF));
		}
		else
		{
			if(USART6_DMA_RX_BUF[0] == 0x5a && USART6_DMA_RX_BUF[1] == 0xa5 && (USART6_DMA_RX_BUF[3]<<8) + USART6_DMA_RX_BUF[2] + 6 == length && length >= 8)
			{					
				crc16_update(&CRCCalculated, &USART6_DMA_RX_BUF[0], 4);
				crc16_update(&CRCCalculated, &USART6_DMA_RX_BUF[6], (length-6));				
				CRCReceived = (USART6_DMA_RX_BUF[5]<<8) + USART6_DMA_RX_BUF[4];
				if(CRCCalculated == CRCReceived)
				{
					Length_Deal = 6;
					
					while(Length_Deal < length)
					{
						p = &USART6_DMA_RX_BUF[Length_Deal];
						switch(*p)
						{
							case 0x90:
								HI220_Data_From_Usart.User_ID = p[1];
								Length_Deal += (LENGTH_USER_ID_0x90 + 1);
								break;
							case 0xa0:
								HI220_Data_From_Usart.Acc_X = (p[2]<<8) +p[1];
								HI220_Data_From_Usart.Acc_Y = (p[4]<<8) +p[3];
								HI220_Data_From_Usart.Acc_Z = (p[6]<<8) +p[5];
								Length_Deal += (LENGTH_ACC_0xa0+1);
								break;
							case 0xa5:
								HI220_Data_From_Usart.Linear_Acc_X = (p[2]<<8) +p[1];
								HI220_Data_From_Usart.Linear_Acc_Y = (p[4]<<8) +p[3];
								HI220_Data_From_Usart.Linear_Acc_Z = (p[6]<<8) +p[5];
								Length_Deal += (LENGTH_LINEAR_ACC_0xa5 + 1);
								break;
							case 0xb0:
								HI220_Data_From_Usart.Ang_Velocity_X = (p[2]<<8) +p[1];
								HI220_Data_From_Usart.Ang_Velocity_Y = (p[4]<<8) +p[3];
								HI220_Data_From_Usart.Ang_Velocity_Z = (p[6]<<8) +p[5];
								Length_Deal += (LENGTH_ANG_VEL_0xb0+1); 
								break;
							case 0xc0:
								HI220_Data_From_Usart.Mag_X = (p[2]<<8) +p[1];
								HI220_Data_From_Usart.Mag_Y = (p[4]<<8) +p[3];
								HI220_Data_From_Usart.Mag_Z = (p[6]<<8) +p[5];
								Length_Deal += (LENGTH_MAG_0xc0+1); 
								break;
							case 0xd0:
								HI220_Data_From_Usart.Euler_Angle_Pitch_s16 = (p[2]<<8) +p[1];
								HI220_Data_From_Usart.Euler_Angle_Roll_s16  = (p[4]<<8) +p[3];
								HI220_Data_From_Usart.Euler_Angle_Yaw_s16   = (p[6]<<8) +p[5];
								HI220_Data_From_Usart.Euler_Angle_Pitch_s16_2_f = 0.01f * HI220_Data_From_Usart.Euler_Angle_Pitch_s16; //原始数据放大了100倍
								HI220_Data_From_Usart.Euler_Angle_Roll_s16_2_f = 0.01f * HI220_Data_From_Usart.Euler_Angle_Roll_s16;	 //原始数据放大了100倍
								HI220_Data_From_Usart.Euler_Angle_Yaw_s16_2_f = 0.1f * HI220_Data_From_Usart.Euler_Angle_Yaw_s16;			 //原始数据放大了10倍
								Length_Deal += (LENGTH_EULER_ANG_s16_0xd0+1); 
								break;
							case 0xd9:
								memcpy(&HI220_Data_From_Usart.Euler_Angle_Pitch.Euler_Angle_Pitch_u8, &p[1], 4);
								memcpy(&HI220_Data_From_Usart.Euler_Angle_Roll.Euler_Angle_Roll_u8, &p[5], 4);
								memcpy(&HI220_Data_From_Usart.Euler_Angle_Yaw.Euler_Angle_Yaw_u8, &p[9], 4);
								Length_Deal += (LENGTH_EULER_ANG_f_0xd9+1); 
								break;
							case 0xd1:
								memcpy(&HI220_Data_From_Usart.Quaternion_W.Quaternion_W_u8, &p[1], 4);
								memcpy(&HI220_Data_From_Usart.Quaternion_X.Quaternion_X_u8, &p[5], 4);
								memcpy(&HI220_Data_From_Usart.Quaternion_Y.Quaternion_Y_u8, &p[9], 4);
								memcpy(&HI220_Data_From_Usart.Quaternion_Z.Quaternion_Z_u8, &p[13], 4);
								Length_Deal += (LENGTH_QUATERNION_0xd1+1); 
								break;
							case 0xf0:
								Length_Deal += (LENGTH_AIR_PRESS_0xf0+1); 
								break;
							default:
								Length_Deal = length;
								break;					
							
						}
					}
				}
				
			}
			

		}
		DMA_SetCurrDataCounter(DMA2_Stream1,USART6_RX_BUF_LENGTH);
		DMA_Cmd(DMA2_Stream1, ENABLE);
  } 
} 

//void delay_1ms(unsigned int t)
//{
//	int i;
//	for( i=0;i<t;i++)
//	{
//		int a=42000; //at 168MHz 42000 is ok
//		while(a--);
//	}
//}


void Hi220_getYawPitchRoll() 
{  
	volatile static float Last_yaw_temp,Yaw_temp,Last_pitch_temp,Pitch_temp; //
	volatile static int Yaw_count,Pitch_count;


  yaw_Gyro = -HI220_Data_From_Usart.Ang_Velocity_Z * 0.1f;
	pitch_Gyro = HI220_Data_From_Usart.Ang_Velocity_X * 0.1f;
	
	Last_yaw_temp = Yaw_temp;
	Yaw_temp = -HI220_Data_From_Usart.Euler_Angle_Yaw_s16_2_f;; 
	if(Yaw_temp-Last_yaw_temp>=330)  
	{
		Yaw_count--;
	}
	else if (Yaw_temp-Last_yaw_temp<=-330)
	{
		Yaw_count++;
	}
	yaw_Angle = Yaw_temp + Yaw_count*360; 

	pitch_Angle = -HI220_Data_From_Usart.Euler_Angle_Roll_s16_2_f;   //*************************去负号*********************************
//	Last_pitch_temp = Pitch_temp;
//	Pitch_temp = HI220_Data_From_Usart.Euler_Angle_Pitch.Euler_Angle_Pitch_f;;  
//	if(Pitch_temp-Last_pitch_temp>=330)  
//	{
//		Pitch_count--;
//	}
//	else if (Pitch_temp-Last_pitch_temp<=-330)
//	{
//		Pitch_count++;
//	}
//	pitch_Angle = Pitch_temp + Pitch_count*360; 
	

	LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_IMU));   //feed(clear) the IMU ERROR Count
}


void Hi220_Reconfig_ODR(u8 *odr)
{
	while(!Hi220_Flags.Hi220_Flag_Configured.Bits.ODR_Configured)
	{
		
		printf("AT+ODR=%s\r\n",odr);
		//printf("AT+ODR=1000\r\n");
		Hi220_Flags.Hi220_Flag_Reconfig.Bits.ODR_Reconfig = 1;
		delay_ms(100);
	}
	Hi220_Flags.Hi220_Flag_Reconfig.Bits.ODR_Reconfig = 0;
}
void Hi220_Reconfig_BAUD(u8 *baud)
{
	while(!Hi220_Flags.Hi220_Flag_Configured.Bits.BAUD_Configured)
	{
		
		printf("AT+BAUD=%s\r\n",baud);
//		printf("AT+BAUD=115200\r\n");
		Hi220_Flags.Hi220_Flag_Reconfig.Bits.BAUD_Reconfig = 1;
		delay_ms(100);
	}
	Hi220_Flags.Hi220_Flag_Reconfig.Bits.BAUD_Reconfig = 0;
}

void Hi220_Reconfig_SETPTL(u8 *ptl)
{
	while(!Hi220_Flags.Hi220_Flag_Configured.Bits.SETPTL_Configured)
	{
		
		printf("AT+SETPTL=%s\r\n",ptl);
		Hi220_Flags.Hi220_Flag_Reconfig.Bits.SETPTL_Reconfig = 1;
		delay_ms(100);
	}
	Hi220_Flags.Hi220_Flag_Reconfig.Bits.SETPTL_Reconfig = 0;
}
void Hi220_Reconfig_EOUT(u8* eout)
{
	while(!Hi220_Flags.Hi220_Flag_Configured.Bits.Eout_Configured)
	{
		
		printf("AT+EOUT=%s\r\n",eout);
//		printf("AT+EOUT=0\r\n");
		Hi220_Flags.Hi220_Flag_Reconfig.Bits.Eout_Reconfig = 1;
		delay_ms(100);
	}
	Hi220_Flags.Hi220_Flag_Reconfig.Bits.Eout_Reconfig = 0;
}

void Hi220_Reset()
{
	printf("AT+RST\r\n");
}

void Hi220_Init()
{
	;
	Hi220_Reconfig_EOUT((u8*)"0");
	Hi220_Reconfig_BAUD((u8*)"115200");
	Hi220_Reconfig_ODR((u8*)"1000");

	Hi220_Reconfig_SETPTL((u8*)"A0,B0,C0,D0,D9");
	Hi220_Reset();	
}


