/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_all_power_cal_mex.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Dec-2023 01:00:23
 */

#ifndef _CODER_ALL_POWER_CAL_MEX_H
#define _CODER_ALL_POWER_CAL_MEX_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS(void);

void unsafe_all_power_cal_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                      int32_T nrhs, const mxArray *prhs[5]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_all_power_cal_mex.h
 *
 * [EOF]
 */
