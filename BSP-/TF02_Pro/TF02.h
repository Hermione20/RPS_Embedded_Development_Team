#ifndef __TF02_H
#define __TF02_H	 

#include "main.h"

typedef struct
{
    float Dist;
	  uint16_t Strength;
	  uint16_t Temp;
}TF02_T;


extern TF02_T TF02;
extern void TF02_IdleCallback();


#endif
