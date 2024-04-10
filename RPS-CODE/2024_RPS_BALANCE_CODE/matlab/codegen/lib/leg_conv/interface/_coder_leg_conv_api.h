/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_leg_conv_api.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 21-Dec-2023 23:01:40
 */

#ifndef _CODER_LEG_CONV_API_H
#define _CODER_LEG_CONV_API_H

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
void leg_conv(real_T F, real_T Tp, real_T phi1, real_T phi4, real_T T[2]);

void leg_conv_api(const mxArray *const prhs[4], const mxArray **plhs);

void leg_conv_atexit(void);

void leg_conv_initialize(void);

void leg_conv_terminate(void);

void leg_conv_xil_shutdown(void);

void leg_conv_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_leg_conv_api.h
 *
 * [EOF]
 */
