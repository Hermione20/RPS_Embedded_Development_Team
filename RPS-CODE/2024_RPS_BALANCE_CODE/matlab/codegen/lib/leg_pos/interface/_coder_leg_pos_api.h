/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_leg_pos_api.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 21-Dec-2023 22:54:47
 */

#ifndef _CODER_LEG_POS_API_H
#define _CODER_LEG_POS_API_H

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
void leg_pos(real_T phi1, real_T phi4, real_T pos[2]);

void leg_pos_api(const mxArray *const prhs[2], const mxArray **plhs);

void leg_pos_atexit(void);

void leg_pos_initialize(void);

void leg_pos_terminate(void);

void leg_pos_xil_shutdown(void);

void leg_pos_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_leg_pos_api.h
 *
 * [EOF]
 */
