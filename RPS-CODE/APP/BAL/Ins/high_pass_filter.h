#ifndef __HIGH_PASS_FILTER_H
#define __HIGH_PASS_FILTER_H
#include "public.h"


typedef struct _hpf_first_order
{
    float fc;       // cut-off frequency
    float y_k1;     // last output
    float alpha;    // filter coefficient
    float ts;       // samping period
    float u_k1;     // last input
}Hpf1stObj;

float hpf_1st_calcu(Hpf1stObj *filter, float u_k,float fc, float ts);





extern Hpf1stObj ACC_X_HIGHP;







#endif