/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_lqr_k_api.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 27-Mar-2024 03:41:41
 */

#ifndef _CODER_LQR_K_API_H
#define _CODER_LQR_K_API_H

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
void lqr_k(real_T L0, real_T K[12]);

void lqr_k_api(const mxArray *prhs, const mxArray **plhs);

void lqr_k_atexit(void);

void lqr_k_initialize(void);

void lqr_k_terminate(void);

void lqr_k_xil_shutdown(void);

void lqr_k_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_lqr_k_api.h
 *
 * [EOF]
 */
