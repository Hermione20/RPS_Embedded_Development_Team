#include "main.h"


void Hardware_WDI_Configuration(void)
{
	GPIO_InitTypeDef gpio;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_0;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&gpio);
//	GPIO_SetBits(GPIOC,GPIO_Pin_0);
	GPIO_ResetBits(GPIOC,GPIO_Pin_0);
	
}

void Buzzer_Configuration(void)
{
	GPIO_InitTypeDef gpio;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_3;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&gpio);
	GPIO_ResetBits(GPIOC,GPIO_Pin_3);
}

void BSP_Init(void)
{

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	TIM6_Configuration();
	TIM6_Start(); 
 	Led_Configuration(); 
	GREEN_LED_ON();
	PWM_Configuration();
	PWM_Configuration1();
	UART4_Configuration();
	Laser_Configuration();	
//	BSP_UART5_InitConfig();
	/***************²â¾à************************/
//	MEASURE_DISTANCE_USART_Config();
//	MEASURE_DISTANCE_USART_DMA_Config();
//	delay_ms(10);
//	stop_measure();
//	delay_ms(10);
//  set_continue_measure_mode();
//	delay_ms(10);
//	start_measure();
	/*******************************************/
	
	
	USART3_Configuration_Send(); 
	 #if IMU == ICM20948

	   delay_ms(100);
	   spi1_init();
		 GPIO_ResetBits(GPIOC,GPIO_Pin_0);
		 ICM20948_init();	
		 #if GYRO_CALI == 1
		 IMU_Gyro_calibration(&IMU_Real_Data);	
	   #else
			IMU_Real_Data.GyroXOffset = GYRO_REAL_X_OFFSET;
			IMU_Real_Data.GyroYOffset = GYRO_REAL_Y_OFFSET;
			IMU_Real_Data.GyroZOffset = GYRO_REAL_Z_OFFSET;
		 #endif
	 #elif IMU == HI219
	 	USART6_Configuration_For_Hi220(); 
			#if HI219_FIRST_USED == 1
			Hi220_Init();
			#endif
	 #endif
		

	  delay_ms(100);
		CAN1_Configuration();           
		CAN2_Configuration();            
		USART1_Configuration(100000);  

	
}


