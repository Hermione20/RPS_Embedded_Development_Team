#include "main.h"
uint16_t  power1=0,power2=0;
uint32_t system_micrsecond;   //系统时间 单位ms
int OutData[4] = {0};   //虚拟示波器使用
void GyroCali(void);
void print_error(void);
int16_t temp1[6] = {0};
uint32_t Upload_Speed = 100;
int16_t SYS_START=0;
extern uint32_t time_tick_1ms;

extern float car_velocity;
extern power_limit_t power_limit;
#define upload_time (1000000/Upload_Speed)
void ICM20948_Gyro_calibration(void);

extern uint32_t time_tick_1ms;
extern u8 _Recognized_Flag;
extern float power_limit_rate;
float abbb[4];

float a,b,c,d;
int main(void)
{
  UART5_Configuration();
  ControtLoopTaskInit();   //app init23
  RemoteTaskInit();
  delay_ms(500);
  BSP_Init();
  system_micrsecond = Get_Time_Micros();
  RED_LED_ON();
  IWDG_Configuration();
  SYS_START = 1;   // 标志位 置1 进入控制中断6

  while(1)
    {   
//     USART_SendData(UART5,0x11);
//			Set_CM_Speed(CAN2, 2000,0,0,0);
//								  OutData[0]= CM1Encoder.filter_rate*100;
//	   					    OutData[1]= -CM2Encoder.filter_rate*100;
//								  OutData[2]= -CM3Encoder.filter_rate*100;
//	   					    OutData[3]= -CM4Encoder.filter_rate*100;
//			Set_Gimbal_Current1(CAN2,0,0,10000,0);
			
			    OutData[0] = judge_rece_mesg.power_heat_data.chassis_power*200;
//		     	OutData[1] = PowerSum*200;
//		    	OutData[2] = all_power*200;
//		    	OutData[3] = judge_rece_mesg.game_robot_state.chassis_power_limit*200;
			

//								Set_Gimbal_Current1(CAN2,abbb[0],abbb[1],abbb[2],abbb[3]);
//			OutData[0] =speed_yaw*5000;
			OutData[1] =GM2Encoder1.filter_rate*100;
			OutData[2] =GM3Encoder1.filter_rate*100;
			OutData[3] =GM4Encoder1.filter_rate*100;
                  OutPut_Data(OutData);
							
//									PWM4=a;
//									PWM5=b;
//									PWM6=c;
//									PWM7=d;
									
    }
}

