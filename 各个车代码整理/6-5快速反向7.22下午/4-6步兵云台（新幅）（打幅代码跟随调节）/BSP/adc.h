#ifndef __ADC_H
#define __ADC_H	
#include "main.h"
#define CHANNAL_NUM 2 

void Adc_Init(void); 

extern u16 ADC_Buffer[CHANNAL_NUM];
#endif
