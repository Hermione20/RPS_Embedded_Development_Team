/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_Vmax_cal_api.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Dec-2023 01:38:17
 */

#ifndef _CODER_VMAX_CAL_API_H
#define _CODER_VMAX_CAL_API_H

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
void Vmax_cal(real_T Kv, real_T Pmax, real_T bT_gain, real_T k1, real_T k2,
              real_T k3, real_T w, real_T Vmax[2]);

void Vmax_cal_api(const mxArray *const prhs[7], const mxArray **plhs);

void Vmax_cal_atexit(void);

void Vmax_cal_initialize(void);

void Vmax_cal_terminate(void);

void Vmax_cal_xil_shutdown(void);

void Vmax_cal_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_Vmax_cal_api.h
 *
 * [EOF]
 */
