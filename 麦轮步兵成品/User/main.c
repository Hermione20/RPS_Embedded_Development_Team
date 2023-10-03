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

int main(void)
{

  ControtLoopTaskInit();   //app init23
  RemoteTaskInit();
  delay_ms(500);
  BSP_Init();
  system_micrsecond = Get_Time_Micros();
  RED_LED_ON();
  IWDG_Configuration();
  SYS_START = 1;   // 标志位 置1 进入控制中断6
  LASER_OFF();
  while(1)
    {

		  OutData[0]= yaw_Angle  *200;
			OutData[1]= gim.pid.yaw_angle_ref*200;
			OutData[2]= gim.pid.yaw_speed_ref*200;
			OutData[3]= gim.pid.yaw_speed_fdb*200;
								    
      OutPut_Data(OutData);
    }
}

