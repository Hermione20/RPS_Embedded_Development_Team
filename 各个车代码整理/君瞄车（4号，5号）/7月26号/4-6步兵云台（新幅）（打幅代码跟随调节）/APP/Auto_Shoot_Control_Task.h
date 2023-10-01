#ifndef _AUTO_SHOOT_CONTROL_TASK_H_
#define _AUTO_SHOOT_CONTROL_TASK_H_
#include "main.h"

/*   auto shoot   20190717   */
#define   GIMBAL_YAW_MID   720 + IMAGE_X_OFFET//IMAGE_X_OFFET
#define   GIMBAL_PIT_MID   540 + IMAGE_Y_OFFET


/*   small buf   20190717   */
//#define   GIMBAL_YAW_MID   640-30
//#define   GIMBAL_PIT_MID   360




typedef struct
{
  float x;
  float y;
  int16_t x1;
  int16_t y1;
  int16_t dis;
  uint8_t flag;
  uint8_t color;
  int16_t receNewDataFlag;
	int16_t id;
  uint8_t crc;
	float yaw_speed;
	float pitch_speed;
} location;


typedef enum
{
  unkown = 0,
  blue = 1,
  red  = 2,
} robot_color_e;

//extern Speed_Prediction_t Speed_Prediction;
extern robot_color_e robot_color ;
extern location new_location;
extern float xy_0_flag;
extern float xy_o_time;
//extern uint8_t color_set_flag;
extern uint8_t dataFromMF[7];		        //数据缓存
extern uint8_t dataFromMFReadyFlag; 		//妙算数据接收完成标志位
extern uint8_t auto_shoot_mode_set;
extern float yaw_buff;

extern float auto_clck;
extern float last_this_yaw_angle,last_this_pit_angle;
extern float fcount;
extern float usart_time;
void targetOffsetDataDeal(uint8_t  len, u8 *buf);
void send_protocol(float x,float y,int id);
void process_general_message(unsigned char* address, unsigned int length);

#endif

