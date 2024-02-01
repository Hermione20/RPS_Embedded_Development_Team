#ifndef __OELDEY_H
#define __OELDEY_H
#include "public.h"














void Set_OIDelec_speed(CAN_TypeDef *CANx, int16_t id, uint8_t cmd, int32_t value);
void Set_OIDelec_heart(CAN_TypeDef *CANx, int16_t id, uint8_t cmd);
void OIDelec_EncoderTask(volatile Encoder *v, CanRxMsg * msg);


#endif
