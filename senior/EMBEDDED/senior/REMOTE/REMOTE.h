#ifndef __REMOTE_H
#define __REMOTE_H
#include "public.h"




/***********************************Ò£¿ØÆ÷*********************************************/

typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	int8_t s1;
	int8_t s2;
}Remote;

typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;	

typedef	__packed struct
{
	uint16_t v;
	uint16_t last_v;
}Key;

typedef enum
{
  KEY_R_UP=0,
  KEY_R_DOWN=1,
 
} key_state_t;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctl_t;






void RemoteDataPrcess(uint8_t *pData);





extern RC_Ctl_t RC_CtrlData;





#endif
