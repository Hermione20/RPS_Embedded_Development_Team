/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_all_power_cal_api.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Dec-2023 01:00:23
 */

#ifndef _CODER_ALL_POWER_CAL_API_H
#define _CODER_ALL_POWER_CAL_API_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
real_T all_power_cal(real_T T, real_T k1, real_T k2, real_T k3, real_T w);

void all_power_cal_api(const mxArray *const prhs[5], const mxArray **plhs);

void all_power_cal_atexit(void);

void all_power_cal_initialize(void);

void all_power_cal_terminate(void);

void all_power_cal_xil_shutdown(void);

void all_power_cal_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_all_power_cal_api.h
 *
 * [EOF]
 */
