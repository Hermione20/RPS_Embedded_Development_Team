/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_leg_spd_api.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 21-Dec-2023 22:57:49
 */

#ifndef _CODER_LEG_SPD_API_H
#define _CODER_LEG_SPD_API_H

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
void leg_spd(real_T dphi1, real_T dphi4, real_T phi1, real_T phi4,
             real_T spd[2]);

void leg_spd_api(const mxArray *const prhs[4], const mxArray **plhs);

void leg_spd_atexit(void);

void leg_spd_initialize(void);

void leg_spd_terminate(void);

void leg_spd_xil_shutdown(void);

void leg_spd_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_leg_spd_api.h
 *
 * [EOF]
 */
