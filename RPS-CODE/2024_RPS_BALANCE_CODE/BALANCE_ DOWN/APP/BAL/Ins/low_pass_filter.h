#ifndef __LOW_PASS_FILTER_H
#define __LOW_PASS_FILTER_H
#include "public.h"


typedef struct _lpf_first_order
{
    float fc;       // cut-off frequency
    float y_k1;     // last output
    float alpha;    // filter coefficient
    float ts;       // samping period
    float u_k1;     // last input
}Lpf1stObj;

float Lpf_1st_calcu(Lpf1stObj *filter, float u_k,float fc, float ts);



extern Lpf1stObj ACC_LPF;
extern Lpf1stObj LEFTLEG_LPF;
extern Lpf1stObj RIGHTLEG_LPF;




#endif
