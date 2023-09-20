#ifndef __AUTO_SHOOT_H
#define __AUTO_SHOOT_H
#include "public.h"
#include "send.pb-c.h"
#include "Recieve.pb-c.h"

/***********************************autoshoot****************************************/

typedef struct
{
  float x;
  float y;
  int16_t x1;
  int16_t y1;
  int16_t dis;
  uint8_t flag;
  uint8_t xy_0_flag;
  uint8_t color;
  int16_t receNewDataFlag;
  int16_t id;
  uint8_t crc;
  float yaw_speed;
  float pitch_speed;
} location;

void vision_process_general_message(unsigned char* address, unsigned int length);
void send_protocol(float x, float y, float r, int id, float ammo_speed, int gimbal_mode, u8 *data);


extern location new_location;


#endif

