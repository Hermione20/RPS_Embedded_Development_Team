#ifndef _CLIENT_H_
#define _CLIENT_H_
#include "main.h"

//颜色
#define UI_RB     0   //红蓝主色
#define UI_YELLOW 1
#define UI_GREEN  2
#define UI_ORANGE 3
#define UI_PURPLE 4
#define UI_PINK   5
#define UI_CYAN   6   //青色
#define UI_BLACK  7
#define UI_WHITE  8


typedef struct
{
  u8 wheel_error_flag;
	u8 shoot_error_flag;
	u8 shoot1_error_flag;
	u8 shoot2_error_flag;
	u8 poke_error_flag;
	u8 pitch_error_flag;
	u8 yaw_error_flag;
} Checkself_t;


extern u8  draw_cnt;
extern u8  draw_int;
void Client_send_handle(void);
void delete_Coverage(u8 coverage);
extern float pitch_remain;
extern float Yaw_remain;
extern double NX_time;
extern Checkself_t checkself;
extern int qwert;


#endif
