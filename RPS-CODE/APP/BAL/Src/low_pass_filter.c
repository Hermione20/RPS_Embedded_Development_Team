#include "low_pass_filter.h"


Lpf1stObj ACC_LPF;
Lpf1stObj LEFTLEG_LPF;
Lpf1stObj RIGHTLEG_LPF;

float Lpf_1st_calcu(Lpf1stObj *filter, float u_k,float fc, float ts)
{
    filter->alpha = 1 / (1 + 1/(2 * PI * fc * ts) );
    filter->ts = ts;
    filter->fc = fc;
    float y_k = filter->alpha*u_k + (1-filter->alpha)*filter->y_k1;
    filter->y_k1 = y_k;
    filter->u_k1 = u_k;
    return y_k;
}

