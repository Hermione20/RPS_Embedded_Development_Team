#ifndef __AUTO_SHOOT_H
#define __AUTO_SHOOT_H
#include "public.h"
#include "send.pb-c.h"
#include "Recieve.pb-c.h"

/***********************************autoshoot****************************************/

typedef struct
{
  float x;                 //自瞄用yaw或大幅
  float y;                 //自瞄用pitch或大幅
  int16_t x1;              //yaw大幅
  int16_t y1;              //pitch大幅
  int16_t control_flag;    //是否进入自瞄模式的标志位（哨兵用）
  uint8_t flag;            //自瞄是否识别
  uint8_t xy_0_flag;       //大幅是否识别
  float xy_o_time;         //大幅标志位
  int16_t lost_cnt;
  float yaw_speed;
  float pitch_speed;
  float last_x;
  float last_y;
	u8 buff_kf_flag;
} location;

void vision_process_general_message(unsigned char* address, unsigned int length);
void send_protocol(float x, float y, float r, int id, float ammo_speed, int gimbal_mode, u8 *data);


extern location new_location;


#endif

