/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: leg_conv.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 21-Dec-2023 23:01:40
 */

/* Include Files */
#include "leg_conv.h"
#include <math.h>

/* Function Definitions */
/*
 * LEG_CONV
 *     T = LEG_CONV(F,Tp,PHI1,PHI4)
 *
 * Arguments    : double F
 *                double Tp
 *                double phi1
 *                double phi4
 *                double T[2]
 * Return Type  : void
 */
void leg_conv(double F, double Tp, double phi1, double phi4, double T[2])
{
  double t104;
  double t106_tmp;
  double t108_tmp;
  double t120_tmp;
  double t123_tmp;
  double t124_tmp;
  double t131_tmp;
  double t16_tmp;
  double t174;
  double t175;
  double t18_tmp;
  double t33_tmp;
  double t34_tmp;
  double t35_tmp;
  double t36_tmp;
  double t51_tmp;
  double t54_tmp;
  double t5_tmp;
  double t64_tmp;
  double t75_tmp;
  double t7_tmp;
  double t81_tmp;
  double t82_tmp;
  double t91_tmp;
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     2023-12-21 22:52:10 */
  t174 = cos(phi1);
  t5_tmp = cos(phi4);
  t175 = sin(phi1);
  t7_tmp = sin(phi4);
  t16_tmp = t174 * 0.15;
  t18_tmp = t175 * 0.15;
  t33_tmp = t174 * 0.0864;
  t34_tmp = t5_tmp * 0.0864;
  t35_tmp = t175 * 0.0864;
  t36_tmp = t7_tmp * 0.0864;
  t104 = t18_tmp - t7_tmp * 0.15;
  t51_tmp = (t5_tmp * 0.15 - t16_tmp) + 0.15;
  t54_tmp = t35_tmp - t36_tmp;
  t64_tmp = (t34_tmp - t33_tmp) + 0.0864;
  t75_tmp = t104 * t104 + t51_tmp * t51_tmp;
  t81_tmp = t174 * t104 * 0.3 + t175 * t51_tmp * 0.3;
  t82_tmp = t5_tmp * t104 * 0.3 + t7_tmp * t51_tmp * 0.3;
  t51_tmp = 1.0 / (t64_tmp + t75_tmp);
  t91_tmp = t51_tmp * t51_tmp;
  t104 = sqrt((t54_tmp * t54_tmp + t64_tmp * t64_tmp) - t75_tmp * t75_tmp);
  t106_tmp = 1.0 / t104;
  t108_tmp = (t36_tmp - t35_tmp) + t104;
  t120_tmp = atan(t51_tmp * t108_tmp) * 2.0;
  t123_tmp = cos(t120_tmp);
  t124_tmp = sin(t120_tmp);
  t131_tmp = 1.0 / (t91_tmp * (t108_tmp * t108_tmp) + 1.0);
  t120_tmp =
      (t35_tmp + t81_tmp) * t91_tmp * t108_tmp +
      t51_tmp *
          (t33_tmp - t106_tmp *
                         ((t174 * t54_tmp * 0.1728 + t175 * t64_tmp * 0.1728) -
                          t75_tmp * t81_tmp * 2.0) /
                         2.0);
  t33_tmp =
      (t36_tmp + t82_tmp) * t91_tmp * t108_tmp +
      t51_tmp * (t34_tmp -
                 t106_tmp *
                     ((t5_tmp * t54_tmp * 0.1728 + t7_tmp * t64_tmp * 0.1728) -
                      t75_tmp * t82_tmp * 2.0) /
                     2.0);
  t35_tmp = t18_tmp + t124_tmp * 0.288;
  t81_tmp = (t16_tmp + t123_tmp * 0.288) - 0.075;
  t104 = (-t16_tmp - t123_tmp * 0.288) + 0.075;
  t174 = t104 * t104;
  t175 = 1.0 / t104;
  t104 = t35_tmp * t35_tmp;
  t106_tmp = t123_tmp * t131_tmp;
  t91_tmp = t16_tmp - t106_tmp * t120_tmp * 0.576;
  t82_tmp = t124_tmp * t131_tmp;
  t120_tmp = t18_tmp - t82_tmp * t120_tmp * 0.576;
  t108_tmp = F * (1.0 / sqrt(t104 + t81_tmp * t81_tmp));
  t51_tmp = Tp * t174 * (1.0 / (t104 + t174));
  t104 = t35_tmp * (1.0 / t174);
  T[0] = t108_tmp * (t35_tmp * t91_tmp * 2.0 - t81_tmp * t120_tmp * 2.0) / 2.0 -
         t51_tmp * (t175 * t91_tmp - t104 * t120_tmp);
  T[1] = t108_tmp *
             (t106_tmp * t35_tmp * t33_tmp * 1.152 -
              t82_tmp * t81_tmp * t33_tmp * 1.152) /
             2.0 +
         t51_tmp * (t175 * (0.0 - t106_tmp * t33_tmp * 0.576) +
                    t104 * (t82_tmp * t33_tmp * 0.576));
}

/*
 * File trailer for leg_conv.c
 *
 * [EOF]
 */