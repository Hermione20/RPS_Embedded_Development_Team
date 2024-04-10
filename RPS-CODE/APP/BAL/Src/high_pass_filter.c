#include "high_pass_filter.h"

/*
 * @Author: luoqi 
 * @Date: 2022-11-29 09:44:24 
 * @Last Modified by: luoqi
 * @Last Modified time: 2022-11-29 09:53:35
 */
Hpf1stObj ACC_X_HIGHP;

float hpf_1st_calcu(Hpf1stObj *filter, float u_k, float fc, float ts)
{
    filter->alpha = 1 / (1 + 2 * PI * fc * ts );
    filter->ts = ts;
    filter->fc = fc;
    float y_k = filter->alpha * (u_k - filter->u_k1 + filter->y_k1);
    filter->y_k1 = y_k;
    filter->u_k1 = u_k;
    return y_k;
}




