/*
 * File: fk_with_ofst.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 09-May-2025 11:02:17
 */

#ifndef FK_WITH_OFST_H
#define FK_WITH_OFST_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void fk_with_ofst(double theta[7], int b_lt_or_rt, double a_arr[7],
                         double alpha_arr[7], double d_arr[7],
                         double theta_arr[7], double carte[3],
                         double eulVal[3], double *bet, int *LOrR, int *FOrB);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for fk_with_ofst.h
 *
 * [EOF]
 */
