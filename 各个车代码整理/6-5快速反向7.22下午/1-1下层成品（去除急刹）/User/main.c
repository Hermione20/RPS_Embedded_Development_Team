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

  BSP_Init();


  while(1)
    {   

									
    }
}

